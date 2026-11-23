/**
 * @file    buddy3_ir.c
 * @brief   Buddy 3: IR line following and sensor telemetry.
 *
 * File guide:
 *   Core functionality: sensor setup, update scheduling, line steering,
 *   junction handling, line-loss recovery and safety stops.
 *   Bonus features: encoder heading hold,
 *   obstacle-avoidance handoff and dashboard diagnostics.
 *   Compatibility: stage 0 three-way steering and its search sweep.
 *
 * The default stage 3 path calls buddy3_staged_step(), which uses the
 * separate buddy3_steering and buddy3_junction modules. The legacy stage 0
 * path calls buddy3_steer_three_way(). See buddy3_follow_line() for dispatch.
 *
 * Ported from the team's Pico SDK (CMake) version.  The algorithm is
 * unchanged; what changed:
 *   - ADC access goes through robot_hal.c.
 *   - State shared with other tasks is copied under a short interrupt
 *     lock (DI/EI) so readers never see a half-updated struct.
 *   - No libc/libm: small local helpers instead of memset/fmaxf/fminf.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy2_motion.h"
#include "buddy3_ir.h"
#include "robot_debug.h"
#include "pin_config.h"
#include "robot_hal.h"
#include "barcode_tuning.h"
#include "buddy3_steering.h"
#include "robot_steering_trace.h"
#include "robot_ir_stream.h"
#include "buddy3_junction.h"

_Static_assert(BUDDY3_SAMPLE_PERIOD_MS <= 255u,
               "IR stream interval must fit its wire field");

/* ------------------------------------------------------------------ *
 *  Core settings - sensor thresholds and layout
 * ------------------------------------------------------------------ */

#define BUDDY3_LINE_THRESHOLD     (350u)
#define BUDDY3_JUNCTION_THRESHOLD (800u)
#define BUDDY3_NORMALIZED_MAX     (1000)
#define BUDDY3_LINE_LOST_STOP_MS  (250u)
#define BUDDY3_SENSOR_COUNT       (3u)

/* ---- Sensor layout ---------------------------------------------------
 * The two FRONT sensors (ADC0, ADC1) sit on the black bracket; the third
 * (GPIO) is on the white arm beside the car and reads the barcodes.
 * Steering compares the two front sensors (position = right - left).  The
 * same rule works whether the line is wider than the sensor gap (ours:
 * both on black = centred) or narrower (both on white = centred).
 *   BUDDY3_LINE_BETWEEN_SENSORS 1 : this "compare the two" algorithm
 *   BUDDY3_LINE_BETWEEN_SENSORS 0 : the old CMake algorithm
 * If the car steers AWAY from the line instead of back onto it, swap the
 * two numbers below (sensor wires or motor wires are the other way round).
 */
#define BUDDY3_LINE_BETWEEN_SENSORS (1)
/* Confirmed 30 Sep with a hand test: ADC0 = left sensor, ADC1 = center.
 * ADC indices of the front sensors, from the car's point of view. */
#define BUDDY3_FRONT_RIGHT (0u)
#define BUDDY3_FRONT_LEFT  (1u)
#define BUDDY3_SIDE_SENSOR (2u) /* the barcode sensor on the side arm */
/* Stop after sustained black on the floor sensors. Leave time to drive off
 * the black start box. */
#define BUDDY3_ALL_BLACK_STOP_MS (3000u)
/* Our line is 18 mm, narrower than the gap between the front sensors, so
 * "both on floor" is NORMAL (line between them) - it only means "lost" if
 * it lasts long (see the line search below). */
#define BUDDY3_EDGE_THRESHOLD (500u) /* normalized: sensor is over black */
/* Front sensor pattern ("side"): bit 0 left black, bit 1 right black. */
#define BUDDY3_SIDE_NONE  (0u)
#define BUDDY3_SIDE_LEFT  (1u)
#define BUDDY3_SIDE_RIGHT (2u)
/* ------------------------------------------------------------------ *
 *  Legacy settings - stage 0 turns and line search
 * ------------------------------------------------------------------ */

/* Steering tested on the car (30 Sep): black under the CENTER sensor ("R")
 * -> left wheel faster (turn right); under the LEFT sensor -> right wheel
 * faster.  Both correct, so no swap is needed.
 * If one sensor stays on black this long, the turn is too weak (curve):
 * run the inner wheel slightly backwards for a tighter turn.
 * Disabled (7 Oct): the reversing inner wheel made the car spin in circles. */
#define BUDDY3_HARD_TURN_ENABLE    (0)
#define BUDDY3_HARD_TURN_MS        (600u)
#define BUDDY3_HARD_TURN_INNER_PWM (-5000)
/* Line search (11 Oct).  Both front sensors white for SEARCH_AFTER_MS after
 * the last touch = the line has slipped outside or ended.  The car then
 * SWEEPS on the spot, with a limited angle:
 *   1. turn up to SWEEP_DEG towards the side that saw the line last
 *      (left if no side has been seen yet),
 *   2. turn back the other way, up to SWEEP_DEG past the start heading,
 *   3. turn back to the start heading,
 *   4. still nothing: keep spinning on the spot (up to SPIN_MAX_DEG, about
 *      one full turn) until a sensor finds a black line,
 *   5. nothing found in a full turn: stop.
 * As soon as either front sensor sees black, normal following takes over.
 * Forward only: the heading when the search started is remembered.  Black
 * seen while the car points more than BEHIND_DEG away from it is the line
 * the car came along, so it is ignored - at a dead end the car stops
 * instead of turning round and driving back.
 * Angles come from the wheel encoders (calibrate the track width so a
 * Motion "Turn Left 90" really turns 90 deg); each step also has a time
 * limit (SWEEP_STEP_MS per SWEEP_DEG) in case the encoders miss ticks. */
#define BUDDY3_SEARCH_ENABLE        (1)
#define BUDDY3_SEARCH_BEHIND_DEG    (120.0f)
#define BUDDY3_SEARCH_AFTER_MS      (1000u)
#define BUDDY3_SEARCH_SWEEP_DEG     (60.0f)
#define BUDDY3_SEARCH_SWEEP_STEP_MS (1500u)
#define BUDDY3_SEARCH_SPIN_MAX_DEG  (400.0f)
#define BUDDY3_SEARCH_SPIN_MAX_MS   (10000u)
/* TEMPORARY TIME-ONLY SEARCH (easy to revert): each search step currently
 * ends on its time limit only, because the encoder tick calibration is not
 * reliable yet.  Set this to 1 after both wheel encoders and the track width
 * have been calibrated, to also end a step when the encoders say the target
 * angle has been turned. */
#define BUDDY3_SEARCH_USE_ENCODER_ANGLE (0)
/* With the search off: both front sensors white for this long after the
 * last touch = no line, so STOP (do not drive away).  Line following stays
 * on; the car moves again as soon as a front sensor sees black. */
#define BUDDY3_LOST_STOP_MS (1000u)
/* ------------------------------------------------------------------ *
 *  Steering support - PWM limits and encoder heading hold
 * ------------------------------------------------------------------ */

/* Lowest power for the inner wheel on a correction.  0 = it may stop but
 * never runs backwards, so a correction curves instead of pivoting. */
#define BUDDY3_INNER_MIN_PWM (0)
/* Keep an open-loop straight segment from gradually curving away while both
 * front sensors see the floor.  The same wheel-distance correction is used
 * while crossing a junction, but normal tracking needs it too: with a
 * straddling two-sensor layout, both-white cannot tell a centred line from a
 * completely missed line. */
#define BUDDY3_STRAIGHT_HEADING_PWM_PER_MM (120.0f)
/* The heading hold may slow a wheel by at most a third of its command. */
#define BUDDY3_HOLD_MAX_DIVISOR (3.0f)

/* Steering gains are stored per 1000 position units. */
#define BUDDY3_GAIN_SCALE (1000.0f)
/* g_test_last_phase value meaning "no phase printed yet". */
#define BUDDY3_PHASE_NONE     (255u)
#define BUDDY3_TEST_STAGE_MAX (3u)

#define BUDDY3_PI        (3.14159265f)
#define BUDDY3_HALF_TURN (180.0f)
#define BUDDY3_FULL_TURN (360.0f)

/* Line search sweep (see BUDDY3_SEARCH_*). */
#define BUDDY3_SEARCH_IDLE    (0u)
#define BUDDY3_SEARCH_FIRST   (1u) /* towards the last-seen side      */
#define BUDDY3_SEARCH_SECOND  (2u) /* back and past, the other way    */
#define BUDDY3_SEARCH_RETURN  (3u) /* back to the start heading       */
#define BUDDY3_SEARCH_SPIN    (4u) /* full spin on the spot           */
#define BUDDY3_SEARCH_GAVE_UP (5u) /* nothing in a full turn: stopped */

/* ------------------------------------------------------------------ *
 *  Shared state - sensor calibration and controller state
 * ------------------------------------------------------------------ */

/* Internal snapshot groups each sensor's raw and normalized readings. */
typedef struct
{
    ir_sensor_reading_t sensors[BUDDY3_SENSOR_COUNT];
    bool                black[BUDDY3_SENSOR_COUNT];
    bool                raw_tuning[BUDDY3_SENSOR_COUNT];
    int16_t             line_position;
    bool                b_line_seen;
    bool                b_junction_seen;
} ir_processing_state_t;

/* API PWM before Buddy 2's global speed limit and trim. Base is 60%;
 * main's 27 percentage-point curve power commands about 33%/87% on a
 * one-sided detection. Junction crossing uses the base and encoder hold. */
/* Sensor order: front right, front left, side barcode. */
static ir_calibration_params_t g_sensor_configs[BUDDY3_SENSOR_COUNT] = {
    {200u, 3000u, false}, /* right sensor */
    {200u, 1300u, false}, /* left sensor */
    {0u, 1u, (BARCODE_DIGITAL_BLACK_LEVEL == 0u)} /* DO level, not ADC */
};

static line_follow_configs_t g_steering_configs = {
    /* Default: 27 percentage-point curve power. */
    (27.0f * (float32_t) BUDDY2_PWM_MAX) / 100000.0f,
    2.5f, /* derivative gain: 2.5f = 2500 / 1000 */
    19660 /* base PWM: 19660 = 60 % duty (ROBOT_LINE_BASE_DEFAULT_PCT) */
};

static ir_processing_state_t g_state;
static volatile bool g_b_line_following = false;
/* Levels set by hand from the dashboard: written by the controller task in
 * one DI/EI step and applied by the IR task, which therefore never sees a
 * half-written request. */
static volatile uint8_t  g_level_sensor    = BUDDY3_SIDE_SENSOR;
static volatile uint16_t g_level_white     = 0u;
static volatile uint16_t g_level_black     = 0u;
static volatile bool     g_b_level_request = false;
/* Start uses bounded corner recovery. Stage 1 remains available for the
 * isolated, gentle-tracking test through the serial line_stage command. */
static uint8_t           g_test_stage = 3u;
static buddy3_steering_t g_test_steering;
static buddy3_junction_t g_junction;
static uint8_t           g_junction_last_phase = B3J_IDLE;
static uint8_t          g_test_last_phase       = BUDDY3_PHASE_NONE;
/* Records brief front-line observations while normal following is paused for
 * Buddy 5 avoidance.  The final return phase consumes this event. */
static volatile bool            g_b_line_seen_event  = false;
static uint32_t                 g_last_update_ms     = 0u;
static int16_t                  g_previous_position  = 0;
static uint32_t                 g_line_lost_since_ms = 0u;
static bool      g_b_reported_lost        = false;
static bool      g_b_seen_since_start     = false;
static bool      g_b_start_pending        = false;
static uint32_t  g_last_touch_ms          = 0u;
static uint8_t   g_last_touch_side        = 0u; /* 0 none, 1 L, 2 R, 3 both */
static uint32_t  g_all_black_since_ms     = 0u;
static bool      g_b_reported_black       = false;
static uint32_t  g_side_since_ms          = 0u; /* current L/R pattern start */
static uint8_t   g_last_single_side       = 0u; /* 1 L, 2 R: last seen alone */
static bool      g_b_straight_hold        = false;
static float32_t g_straight_hold_left_mm  = 0.0f;
static float32_t g_straight_hold_right_mm = 0.0f;
static uint8_t   g_trace_phase            = TRACE_LEGACY;

static uint8_t   g_search_step       = BUDDY3_SEARCH_IDLE;
static int8_t    g_search_dir        = 1;    /* +1 left, -1 right (step)  */
static uint32_t  g_search_step_ms    = 0u;   /* when the current step began */
static float32_t g_search_step_deg   = 0.0f; /* encoder angle when it began */
static float32_t g_search_origin_deg = 0.0f; /* heading when search began  */
static bool      g_b_reported_behind = false;

/* ------------------------------------------------------------------ *
 *  Internal function declarations
 * ------------------------------------------------------------------ */

static int16_t   buddy3_clamp_pwm (float32_t value);
static uint16_t  buddy3_normalize (uint16_t raw, uint16_t minimum,
                                   uint16_t maximum, bool b_line_is_dark);
static float32_t buddy3_encoder_angle_deg (void);
static void      buddy3_search_begin_step (uint8_t step, int8_t direction,
                                           uint32_t now_ms);
static void
buddy3_apply_straight_heading_hold (buddy3_steering_output_t * p_output,
                                    uint8_t                    side);
static void      buddy3_apply_levels (void);
static robot_steering_input_t buddy3_trace_input (void);
static void buddy3_drive_observed (int16_t left, int16_t right);
static void buddy3_stop_observed (bool b_disable);
static void buddy3_stop_with_input (const robot_steering_input_t * p_input);
static void buddy3_read_sensors (ir_processing_state_t * p_next,
                                 uint32_t                now_ms);
static void buddy3_follow_line (const ir_processing_state_t * p_next,
                                int16_t previous_position, uint32_t now_ms);
#if BUDDY3_LINE_BETWEEN_SENSORS
static void    buddy3_track_side (const ir_processing_state_t * p_next,
                                  uint32_t                      now_ms);
static uint8_t buddy3_ignore_line_behind (uint8_t side);
static void    buddy3_log_side (uint8_t side, uint32_t now_ms);
static bool    buddy3_dark_surface_stop (const ir_processing_state_t * p_next,
                                         uint32_t                      now_ms);
static void    buddy3_staged_step (uint32_t now_ms);
static void    buddy3_steer_three_way (uint32_t now_ms);
static void    buddy3_search_step (uint32_t now_ms, int32_t base,
                                   int32_t * p_left_pwm, int32_t * p_right_pwm);
#endif
static bool buddy3_lost_stop (const ir_processing_state_t * p_next,
                              uint32_t                      now_ms);

/* ------------------------------------------------------------------ *
 *  Core API - initialization, scheduling and line-follow controls
 * ------------------------------------------------------------------ */

void buddy3_init (void)
{
    ir_processing_state_t empty = {0};

    hal_adc_init();
    hal_adc_pin_init(IR_SENSOR_LEFT_PIN);
    hal_adc_pin_init(IR_SENSOR_CENTER_PIN);

    g_state             = empty;
    g_last_update_ms    = 0u;
    g_b_line_following  = false;
    g_b_line_seen_event = false;
}

void buddy3_set_line_speed (int16_t base_pwm, float32_t proportional_gain)
{
    UINT interrupt_state = 0u;

    /* Only the steering settings: the IR task changes the sensor levels
     * itself (white zero, levels from the dashboard), and writing back a
     * copy of them taken a moment earlier could undo such a change.  One
     * DI/EI step, so the IR task never steers with a new base and an old
     * gain. */
    DI(interrupt_state);
    g_steering_configs.base_pwm          = base_pwm;
    g_steering_configs.proportional_gain = proportional_gain;
    EI(interrupt_state);
}

void buddy3_get_calibration (buddy3_calibration_t * p_calibration)
{
    UINT     interrupt_state = 0u;
    uint32_t index           = 0u;

    if (NULL != p_calibration)
    {
        DI(interrupt_state);
        for (index = 0u; index < BUDDY3_SENSOR_COUNT; index++)
        {
            p_calibration->minimum[index] = g_sensor_configs[index].minimum;
            p_calibration->maximum[index] = g_sensor_configs[index].maximum;
        }
        /* All sensors share inversion while configured through the
         * compatibility API. */
        p_calibration->line_is_dark      = g_sensor_configs[0].line_is_dark;
        p_calibration->proportional_gain = g_steering_configs.proportional_gain;
        p_calibration->derivative_gain   = g_steering_configs.derivative_gain;
        p_calibration->base_pwm          = g_steering_configs.base_pwm;
        EI(interrupt_state);
    }
}

void buddy3_update (uint32_t now_ms)
{
    ir_processing_state_t next              = {0};
    int16_t               previous_position = 0;
    UINT                  interrupt_state   = 0u;

    if ((now_ms - g_last_update_ms) >= BUDDY3_SAMPLE_PERIOD_MS)
    {
        g_last_update_ms = now_ms;
        g_trace_phase    = TRACE_LEGACY;

        previous_position = g_previous_position;
        buddy3_read_sensors(&next, now_ms);
        buddy3_apply_levels();

        // The digital reader captures edges separately. Keep this level for IR diagnostics.
        const uint16_t raw = next.sensors[BUDDY3_SIDE_SENSOR].raw;
        next.black[BUDDY3_SIDE_SENSOR] = barcode_tuning_sample(raw, false);
#if defined(ROBOT_DIAGNOSTICS) && ROBOT_DIAGNOSTICS
        buddy2_encoder_position_t motion = {0};
        buddy2_get_encoder_position(&motion);
        const robot_ir_stream_sample_t observation = {
            .ts_ms = now_ms,
            .raw = raw,
            .normalized = next.sensors[BUDDY3_SIDE_SENSOR].normalized,
            .black = next.black[BUDDY3_SIDE_SENSOR],
            .left_x10 = robot_ir_stream_position(motion.left_distance_mm),
            .right_x10 = robot_ir_stream_position(motion.right_distance_mm),
            .minimum = g_sensor_configs[BUDDY3_SIDE_SENSOR].minimum,
            .maximum = g_sensor_configs[BUDDY3_SIDE_SENSOR].maximum,
            .inverted = g_sensor_configs[BUDDY3_SIDE_SENSOR].line_is_dark,
        };
        robot_ir_stream_record(&observation);
#endif

        DI(interrupt_state);
        g_state = next;
        if (next.b_line_seen)
        {
            g_b_line_seen_event = true;
        }
        EI(interrupt_state);

#if BUDDY3_LINE_BETWEEN_SENSORS
        buddy3_track_side(&next, now_ms);
#endif

        if (g_b_line_following)
        {
            buddy3_follow_line(&next, previous_position, now_ms);
        }
    }
}

void buddy3_start_line_follow (void)
{
    buddy3_junction_reset(&g_junction);
    g_junction_last_phase = B3J_IDLE;
    buddy3_steering_reset(&g_test_steering);
    g_test_last_phase    = BUDDY3_PHASE_NONE;
    g_b_reported_lost    = false;
    g_b_seen_since_start = false;
    g_b_start_pending    = true;
    g_line_lost_since_ms = 0u;
    g_last_single_side   = 0u;
    g_search_step        = BUDDY3_SEARCH_IDLE;
    g_side_since_ms      = g_last_update_ms;
    g_b_straight_hold    = false;
    g_b_line_following   = true;
}

void buddy3_stop_line_follow (void)
{
    buddy3_stop_with_input(NULL);
}

bool buddy3_line_following (void)
{
    return g_b_line_following;
}

void buddy3_set_test_stage (uint8_t stage)
{
    if (!g_b_line_following && (stage <= BUDDY3_TEST_STAGE_MAX))
    {
        g_test_stage = stage;
    }
}

uint8_t buddy3_test_stage (void)
{
    return g_test_stage;
}

/* ------------------------------------------------------------------ *
 *  Diagnostics API - sensor snapshots and console output
 * ------------------------------------------------------------------ */

void buddy3_get_state (buddy3_ir_state_t * p_state)
{
    UINT     interrupt_state = 0u;
    uint32_t index           = 0u;

    if (NULL != p_state)
    {
        DI(interrupt_state);
        /* Adapt the per-sensor storage to the existing public snapshot. */
        for (index = 0u; index < BUDDY3_SENSOR_COUNT; index++)
        {
            p_state->raw[index]        = g_state.sensors[index].raw;
            p_state->normalized[index] = g_state.sensors[index].normalized;
        }
        p_state->line_position   = g_state.line_position;
        p_state->line_seen       = g_state.b_line_seen;
        p_state->junction_seen   = g_state.b_junction_seen;
        EI(interrupt_state);
    }
}

void buddy3_print_debug (void)
{
#if BUDDY3_DEBUG
    buddy3_ir_state_t state = {0};

    /* The getter releases its interrupt lock before any console output. */
    buddy3_get_state(&state);
    tm_printf((UB *) "[buddy3] raw L=%u R=%u side=%u | norm L=%u R=%u side=%u "
                     "| ADC L=%u R=%u side=2\n",
              (UINT) state.raw[BUDDY3_FRONT_LEFT],
              (UINT) state.raw[BUDDY3_FRONT_RIGHT],
              (UINT) state.raw[BUDDY3_SIDE_SENSOR],
              (UINT) state.normalized[BUDDY3_FRONT_LEFT],
              (UINT) state.normalized[BUDDY3_FRONT_RIGHT],
              (UINT) state.normalized[BUDDY3_SIDE_SENSOR],
              (UINT) BUDDY3_FRONT_LEFT, (UINT) BUDDY3_FRONT_RIGHT);
#endif
}

/* ------------------------------------------------------------------ *
 *  Bonus API - line observations for obstacle-avoidance return
 * ------------------------------------------------------------------ */

void buddy3_clear_line_seen_event (void)
{
    UINT interrupt_state = 0u;

    DI(interrupt_state);
    g_b_line_seen_event = false;
    EI(interrupt_state);
}

bool buddy3_take_line_seen_event (void)
{
    UINT interrupt_state = 0u;
    bool b_seen          = false;

    DI(interrupt_state);
    b_seen              = g_b_line_seen_event;
    g_b_line_seen_event = false;
    EI(interrupt_state);

    return b_seen;
}

/* ------------------------------------------------------------------ *
 *  Sensor API - black/white status and calibration
 * ------------------------------------------------------------------ */

void buddy3_get_colors (bool * p_left_black, bool * p_right_black,
                        bool * p_side_black)
{
    UINT lock;
    DI(lock);
    if (p_left_black != NULL)
    {
        *p_left_black = g_state.black[BUDDY3_FRONT_LEFT];
    }
    if (p_right_black != NULL)
    {
        *p_right_black = g_state.black[BUDDY3_FRONT_RIGHT];
    }
    if (p_side_black != NULL)
    {
        *p_side_black = g_state.black[BUDDY3_SIDE_SENSOR];
    }
    EI(lock);
}

bool buddy3_set_ir_levels (uint8_t sensor, uint16_t white, uint16_t black)
{
    /* Dashboard numbering to the firmware's sensor index. */
    static const uint8_t index[] = {BUDDY3_SIDE_SENSOR, BUDDY3_FRONT_LEFT,
                                    BUDDY3_FRONT_RIGHT};
    UINT                 interrupt_state = 0u;
    bool                 b_ok =
        (sensor >= BUDDY3_LEVELS_LEFT) && (sensor <= BUDDY3_LEVELS_RIGHT) &&
        (black <= BUDDY3_RAW_MAX) &&
        (white < black) &&
        (((uint32_t) black - (uint32_t) white) >= BUDDY3_LEVEL_MIN_SPAN);

    if (b_ok)
    {
        /* The IR task has the higher priority: without the lock it could
         * run between these stores, while an earlier request is still
         * waiting, and apply the old levels to the new sensor. */
        DI(interrupt_state);
        g_level_sensor    = index[sensor];
        g_level_white     = white;
        g_level_black     = black;
        g_b_level_request = true;
        EI(interrupt_state);
    }

    return b_ok;
}

/* ------------------------------------------------------------------ *
 *  Shared helpers - PWM limits and analog normalization
 * ------------------------------------------------------------------ */

static int16_t buddy3_clamp_pwm (float32_t value)
{
    int16_t result = 0;

    if (value > (float32_t) BUDDY2_PWM_MAX)
    {
        result = (int16_t) BUDDY2_PWM_MAX;
    }
    else if (value < -(float32_t) BUDDY2_PWM_MAX)
    {
        result = (int16_t) (-BUDDY2_PWM_MAX);
    }
    else
    {
        result = (int16_t) value; /* Within +/-BUDDY2_PWM_MAX here. */
    }

    return result;
}

static uint16_t buddy3_normalize (uint16_t raw, uint16_t minimum,
                                  uint16_t maximum, bool b_line_is_dark)
{
    int32_t value = 0;

    if (maximum > minimum)
    {
        value = (((int32_t) raw - (int32_t) minimum) * BUDDY3_NORMALIZED_MAX) /
                ((int32_t) maximum - (int32_t) minimum);
        if (value < 0)
        {
            value = 0;
        }
        if (value > BUDDY3_NORMALIZED_MAX)
        {
            value = BUDDY3_NORMALIZED_MAX;
        }
        if (b_line_is_dark)
        {
            value = BUDDY3_NORMALIZED_MAX - value;
        }
    }

    /* Clamped to 0..BUDDY3_NORMALIZED_MAX above. */
    return (uint16_t) value;
}

/* ------------------------------------------------------------------ *
 *  Encoder helpers - heading measurement and legacy search setup
 * ------------------------------------------------------------------ */

/* Car heading from the wheel encoders, in degrees (only differences are
 * used, so the sign convention and any left/right encoder mix-up do not
 * matter: the sweep compares the SIZE of the turn). */
static float32_t buddy3_encoder_angle_deg (void)
{
    buddy2_motion_state_t motion      = {0};
    buddy2_calibration_t  calibration = {0};
    float32_t             angle_deg   = 0.0f;

    buddy2_get_motion_state(&motion);
    buddy2_get_calibration(&calibration);
    if (calibration.track_width_mm > 0.0f)
    {
        angle_deg = ((motion.right_distance_mm - motion.left_distance_mm) /
                     calibration.track_width_mm) *
                    (BUDDY3_HALF_TURN / BUDDY3_PI);
    }

    return angle_deg;
}

static void buddy3_search_begin_step (uint8_t step, int8_t direction,
                                      uint32_t now_ms)
{
    g_search_step     = step;
    g_search_dir      = direction;
    g_search_step_ms  = now_ms;
    g_search_step_deg = buddy3_encoder_angle_deg();
}

/* ------------------------------------------------------------------ *
 *  Bonus control - encoder heading hold on straight segments
 * ------------------------------------------------------------------ */

/* When the line is centred between the two front sensors, both report white.
 * Preserve the current heading using the encoders so unequal motors do not
 * silently carry the car off the line before either sensor can see an edge.
 * A one-sided line reading always takes priority and resets this hold. */
static void
buddy3_apply_straight_heading_hold (buddy3_steering_output_t * p_output,
                                    uint8_t                    side)
{
    buddy2_motion_state_t motion     = {0};
    float32_t             left       = 0.0f;
    float32_t             right      = 0.0f;
    float32_t             error      = 0.0f;
    float32_t             correction = 0.0f;
    int16_t               base       = 0;

    if ((NULL != p_output) && (B3_TRACK == p_output->phase) &&
        (BUDDY3_SIDE_NONE == side))
    {
        buddy2_get_motion_state(&motion);
    }

    if ((NULL == p_output) || (B3_TRACK != p_output->phase) ||
        (BUDDY3_SIDE_NONE != side) || motion.left_encoder_fault ||
        motion.right_encoder_fault)
    {
        g_b_straight_hold = false;
    }
    else if (!g_b_straight_hold)
    {
        g_straight_hold_left_mm  = motion.left_distance_mm;
        g_straight_hold_right_mm = motion.right_distance_mm;
        g_b_straight_hold        = true;
    }
    else
    {
        left  = motion.left_distance_mm - g_straight_hold_left_mm;
        right = motion.right_distance_mm - g_straight_hold_right_mm;
        if (!(left >= 0.0f) || !(right >= 0.0f))
        {
            /* A reset or reverse must establish a new forward reference. */
            g_straight_hold_left_mm  = motion.left_distance_mm;
            g_straight_hold_right_mm = motion.right_distance_mm;
        }
        else
        {
            error      = right - left;
            correction = ((error < 0.0f) ? -error : error) *
                         BUDDY3_STRAIGHT_HEADING_PWM_PER_MM;
            base       = (p_output->left < p_output->right) ? p_output->left
                                                            : p_output->right;
            if (correction > ((float32_t) base / BUDDY3_HOLD_MAX_DIVISOR))
            {
                correction = (float32_t) base / BUDDY3_HOLD_MAX_DIVISOR;
            }
            /* 0 <= correction <= base / 3, so it fits int16_t. */
            if (error > 0.0f)
            {
                p_output->right -= (int16_t) correction;
            }
            else if (error < 0.0f)
            {
                p_output->left -= (int16_t) correction;
            }
            else
            {
                /* Heading unchanged: nothing to correct. */
            }
        }
    }
}

/* Apply the pending front-sensor calibration at the next line step. */
static void buddy3_apply_levels (void)
{
    if (g_b_level_request)
    {
        g_b_level_request                        = false;
        g_sensor_configs[g_level_sensor].minimum = g_level_white;
        g_sensor_configs[g_level_sensor].maximum = g_level_black;
        tm_printf((UB *) "[ir] sensor %u levels white %u black %u\n",
                  (UINT) g_level_sensor, (UINT) g_level_white,
                  (UINT) g_level_black);
    }
}


/* ------------------------------------------------------------------ *
 *  Diagnostics helpers - steering evidence and motor requests
 * ------------------------------------------------------------------ */

static robot_steering_input_t buddy3_trace_input (void)
{
    static const uint8_t   front[2] = {BUDDY3_FRONT_LEFT, BUDDY3_FRONT_RIGHT};
    robot_steering_input_t input    = {0};
    UINT                   state    = 0u;
    uint32_t               slot     = 0u;

    DI(state);
    input.ts_ms     = g_last_update_ms;
    input.phase     = g_trace_phase;
    input.threshold = BUDDY3_EDGE_THRESHOLD;
    for (slot = 0u; slot < 2u; ++slot)
    {
        uint32_t sensor = front[slot];

        input.raw[slot]              = g_state.sensors[sensor].raw;
        input.norm[slot]             = g_state.sensors[sensor].normalized;
        input.calibration[slot * 2u] = g_sensor_configs[sensor].minimum;
        input.calibration[(slot * 2u) + 1u] = g_sensor_configs[sensor].maximum;
        if (g_sensor_configs[sensor].line_is_dark)
        {
            input.dark |= (uint8_t) (1u << slot); /* bit 0 or 1 */
        }
    }
    EI(state);

    return input;
}

static void buddy3_drive_observed (int16_t left, int16_t right)
{
    robot_steering_input_t input = buddy3_trace_input();

    buddy2_set_motor_pwm_observed(left, right, &input);
}

static void buddy3_stop_observed (bool b_disable)
{
    robot_steering_input_t input = buddy3_trace_input();

    if (b_disable)
    {
        buddy3_stop_with_input(&input);
    }
    else
    {
        buddy2_stop_observed(&input);
    }
}

static void buddy3_stop_with_input (const robot_steering_input_t * p_input)
{
    g_b_line_following = false;
    buddy3_junction_reset(&g_junction);
    g_b_straight_hold       = false;
    buddy2_stop_observed(p_input);
}

/* ------------------------------------------------------------------ *
 *  Core processing - ADC sampling and line/junction detection
 * ------------------------------------------------------------------ */

/* Read and normalise the sensors, then find the line. */
static void buddy3_read_sensors (ir_processing_state_t * p_next,
                                 uint32_t                now_ms)
{
    uint16_t raw[BUDDY3_SENSOR_COUNT];
    uint32_t index = 0u;
#if !BUDDY3_LINE_BETWEEN_SENSORS
    uint32_t weighted_total = 0u;
#endif

    /* Read the sensors first (slow part, no lock held). */
    raw[0] = hal_adc_read(IR_ADC_CHANNEL_LEFT);
    raw[1] = hal_adc_read(IR_ADC_CHANNEL_CENTER);
    raw[2] = hal_gpio_read(BARCODE_DIGITAL_PIN) ? 1u : 0u;

    *p_next = g_state;
    for (index = 0u; index < BUDDY3_SENSOR_COUNT; index++)
    {
        p_next->sensors[index].raw = raw[index];
        p_next->sensors[index].normalized =
            buddy3_normalize(raw[index], g_sensor_configs[index].minimum,
                             g_sensor_configs[index].maximum,
                             g_sensor_configs[index].line_is_dark);
    }

    /* Dashboard IDs: left=1 (ADC1), right=2 (ADC0). Preserve analog
     * readings for diagnostics; the classified snapshot drives steering. */
    for (index = 0u; index < 2u; ++index)
    {
        uint8_t sensor = (index == BUDDY3_FRONT_LEFT) ? 1u : 2u;
        barcode_tuning_state_t tuning;
        p_next->black[index] = ir_tuning_sample(sensor, raw[index],
            p_next->sensors[index].normalized >= BUDDY3_EDGE_THRESHOLD);
        ir_tuning_get(sensor, &tuning);
        p_next->raw_tuning[index] = tuning.enabled;
    }

#if BUDDY3_LINE_BETWEEN_SENSORS
    /* Front sensors STRADDLE the line (see the photo in docs/): normally both
     * see the floor.  When one of them touches the line, steer towards that
     * side so the line goes back between them.  position = right - left:
     * > 0 -> line under the right-front sensor -> turn right. */
    {
        int32_t left = p_next->raw_tuning[BUDDY3_FRONT_LEFT] ?
            (p_next->black[BUDDY3_FRONT_LEFT] ? 1000 : 0) :
            (int32_t) p_next->sensors[BUDDY3_FRONT_LEFT].normalized;
        int32_t right = p_next->raw_tuning[BUDDY3_FRONT_RIGHT] ?
            (p_next->black[BUDDY3_FRONT_RIGHT] ? 1000 : 0) :
            (int32_t) p_next->sensors[BUDDY3_FRONT_RIGHT].normalized;

        (void) now_ms;
        p_next->b_line_seen = (left >= (int32_t) BUDDY3_EDGE_THRESHOLD) ||
                              (right >= (int32_t) BUDDY3_EDGE_THRESHOLD);
        p_next->b_junction_seen =
            (left >= (int32_t) BUDDY3_JUNCTION_THRESHOLD) &&
            (right >= (int32_t) BUDDY3_JUNCTION_THRESHOLD);
        /* Both black = a crossing line (or the car was lifted): straight.
         * right - left is within +/-1000, so it fits int16_t. */
        p_next->line_position =
            p_next->b_junction_seen ? 0 : (int16_t) (right - left);
        g_previous_position = p_next->line_position;
    }
#else
    /* Original algorithm: the line runs UNDER the two front sensors. */
    weighted_total      = (uint32_t) p_next->sensors[0].normalized +
                          (uint32_t) p_next->sensors[1].normalized;
    p_next->b_line_seen = (weighted_total >= BUDDY3_LINE_THRESHOLD);
    p_next->b_junction_seen =
        (p_next->sensors[0].normalized >= BUDDY3_JUNCTION_THRESHOLD) &&
        (p_next->sensors[1].normalized >= BUDDY3_JUNCTION_THRESHOLD);
    if (p_next->b_line_seen)
    {
        int32_t weighted_position = ((int32_t) p_next->sensors[1].normalized -
                                     (int32_t) p_next->sensors[0].normalized) *
                                    1000;

        /* The weighted mean lies within +/-1000. */
        p_next->line_position =
            (int16_t) (weighted_position / (int32_t) weighted_total);
        g_previous_position  = p_next->line_position;
        g_line_lost_since_ms = 0u;
    }
    else
    {
        p_next->line_position = 0;
        if (0u == g_line_lost_since_ms)
        {
            g_line_lost_since_ms = now_ms;
        }
    }
#endif
}

/* ------------------------------------------------------------------ *
 *  Core control - safety checks and steering-path selection
 * ------------------------------------------------------------------ */

/* Decide the wheel commands while line following is on. */
static void buddy3_follow_line (const ir_processing_state_t * p_next,
                                int16_t previous_position, uint32_t now_ms)
{
    bool b_done = false;

#if BUDDY3_LINE_BETWEEN_SENSORS
    (void) previous_position;
    b_done = buddy3_dark_surface_stop(p_next, now_ms);
    if (!b_done && (0u != g_test_stage))
    {
        buddy3_staged_step(now_ms);
        b_done = true;
    }
#endif

    if (!b_done)
    {
        b_done = buddy3_lost_stop(p_next, now_ms);
    }

    if (!b_done)
    {
#if BUDDY3_LINE_BETWEEN_SENSORS
        buddy3_steer_three_way(now_ms);
#else
        float32_t derivative =
            (float32_t) p_next->line_position - (float32_t) previous_position;
        float32_t correction =
            (g_steering_configs.proportional_gain *
             (float32_t) p_next->line_position) +
            (g_steering_configs.derivative_gain * derivative);

        buddy3_drive_observed(
            buddy3_clamp_pwm((float32_t) g_steering_configs.base_pwm +
                             correction),
            buddy3_clamp_pwm((float32_t) g_steering_configs.base_pwm -
                             correction));
#endif
    }
}

#if BUDDY3_LINE_BETWEEN_SENSORS
/* ------------------------------------------------------------------ *
 *  Line history - sensor transitions and recovery direction
 * ------------------------------------------------------------------ */

/* Track which front sensors see the line, log changes, and maintain the
 * touch timers used by the search and stop rules. */
static void buddy3_track_side (const ir_processing_state_t * p_next,
                               uint32_t                      now_ms)
{
    /* Which front sensor(s) are on black right now: 1 left, 2 right. */
    uint8_t side =
        (uint8_t) ((p_next->black[BUDDY3_FRONT_LEFT]
                        ? BUDDY3_SIDE_LEFT
                        : 0u) |
                   (p_next->black[BUDDY3_FRONT_RIGHT]
                        ? BUDDY3_SIDE_RIGHT
                        : 0u));

    side = buddy3_ignore_line_behind(side);

    if (side != g_last_touch_side)
    {
        buddy3_log_side(side, now_ms);
        g_side_since_ms = now_ms;
    }
    g_last_touch_side = side;
    if ((BUDDY3_SIDE_LEFT == side) || (BUDDY3_SIDE_RIGHT == side))
    {
        g_last_single_side = side;
    }

    if (g_b_start_pending)
    {
        g_last_touch_ms   = now_ms; /* start the "no line" timer now */
        g_b_start_pending = false;
    }
    if (BUDDY3_SIDE_NONE != side)
    {
        g_last_touch_ms = now_ms;
        if ((g_b_reported_lost || (BUDDY3_SEARCH_IDLE != g_search_step)) &&
            g_b_line_following)
        {
            tm_printf((UB *) "[line] line found - driving\n");
        }
        g_b_reported_lost = false;
        g_search_step     = BUDDY3_SEARCH_IDLE;
    }
}

/* Forward only: during a search, black seen while the car points back the
 * way it came is the old line - ignore it and keep turning. */
static uint8_t buddy3_ignore_line_behind (uint8_t side)
{
#if BUDDY3_SEARCH_ENABLE
    if ((0u == g_test_stage) && (BUDDY3_SIDE_NONE != side) &&
        g_b_line_following && (BUDDY3_SEARCH_IDLE != g_search_step) &&
        (BUDDY3_SEARCH_GAVE_UP != g_search_step))
    {
        float32_t offset = buddy3_encoder_angle_deg() - g_search_origin_deg;

        while (offset > BUDDY3_HALF_TURN)
        {
            offset -= BUDDY3_FULL_TURN;
        }
        while (offset < -BUDDY3_HALF_TURN)
        {
            offset += BUDDY3_FULL_TURN;
        }
        if ((offset > BUDDY3_SEARCH_BEHIND_DEG) ||
            (offset < -BUDDY3_SEARCH_BEHIND_DEG))
        {
            if (!g_b_reported_behind)
            {
                /* The offset is within +/-180 degrees. */
                tm_printf(
                    (UB *) "[line] line at %d deg is behind us - ignored\n",
                    (INT) offset);
                g_b_reported_behind = true;
            }
            side = BUDDY3_SIDE_NONE;
        }
    }
#endif

    return side;
}

/* ------------------------------------------------------------------ *
 *  Diagnostics helpers - idle sensor transition logging
 * ------------------------------------------------------------------ */

/* Sensor check while idle: say which side sees the line, so the wiring can
 * be verified by hand (see docs).  While following, log every change with
 * what the car is told to do. */
static void buddy3_log_side (uint8_t side, uint32_t now_ms)
{
    static const char * const color[] = {"white", "BLACK"};
    static const char * const steer[] = {
        "straight", "steer LEFT", "steer RIGHT", "both black (junction check)"};
    const char * p_left     = color[side & 1u];
    const char * p_right    = color[(side >> 1u) & 1u];
    const char * p_decision = buddy3_junction_active(&g_junction)
                                  ? "junction straight (branch ignored)"
                                  : steer[side];

    if (g_b_line_following)
    {
        /* Compare with what the car really did to check the wiring. */
        tm_printf((UB *) "[line] %u.%02u s   front L=%s R=%s   -> %s\n",
                  (UINT) (now_ms / 1000u), (UINT) ((now_ms % 1000u) / 10u),
                  p_left, p_right, p_decision);
    }
    else
    {
        tm_printf((UB *) "[sensor] front L=%s R=%s\n", p_left, p_right);
    }
}

/* ------------------------------------------------------------------ *
 *  Safety - sustained all-black stop
 * ------------------------------------------------------------------ */

/* Lifted or dark surface: the floor sensors read black. */
static bool buddy3_dark_surface_stop (const ir_processing_state_t * p_next,
                                      uint32_t                      now_ms)
{
    bool b_stopped = false;

    /* GP28 reads bars, so only the front sensors indicate floor in this build. */
    if (p_next->black[BUDDY3_FRONT_LEFT] && p_next->black[BUDDY3_FRONT_RIGHT])
    {
        if (0u == g_all_black_since_ms)
        {
            g_all_black_since_ms = now_ms;
        }
        if ((now_ms - g_all_black_since_ms) > BUDDY3_ALL_BLACK_STOP_MS)
        {
            g_trace_phase = TRACE_DARK;
            buddy3_stop_observed(false);
            if (!g_b_reported_black)
            {
                tm_printf((UB *) "[line] floor sensors black - lifted or dark "
                                 "surface? stopped\n");
                g_b_reported_black = true;
            }
            b_stopped = true;
        }
    }
    else
    {
        g_all_black_since_ms = 0u;
        g_b_reported_black   = false;
    }

    return b_stopped;
}

/* ------------------------------------------------------------------ *
 *  Core control - stages 1 to 3, and junctions
 * ------------------------------------------------------------------ */

/* Staged steering (stage 1..3) with junction handling. */
static void buddy3_staged_step (uint32_t now_ms)
{
    static const char * const phases[] = {"straight", "micro", "corner",
                                          "seek",     "align", "both-black",
                                          "stopped"};
    static const char * const names[]  = {"idle", "confirm", "straight",
                                          "cleared", "stopped"};
    buddy2_motion_state_t     motion   = {0};
    buddy3_junction_output_t  junction = {0};
    buddy3_steering_output_t  output   = {0};
    bool b_was_crossing                = buddy3_junction_active(&g_junction);
    /* A long curve may legitimately lead straight into a junction. The
     * junction's own both-black debounce then owns that crossing; only a
     * genuine line-loss recovery or an already stopped run blocks entry. */
    bool b_entry_allowed =
        !g_test_steering.recovering && !g_test_steering.stopped;

    buddy2_get_motion_state(&motion);
    junction = buddy3_junction_step(
        &g_junction, g_last_touch_side, now_ms, motion.left_distance_mm,
        motion.right_distance_mm,
        !motion.left_encoder_fault && !motion.right_encoder_fault,
        b_entry_allowed, g_steering_configs.base_pwm);
    if (junction.phase != g_junction_last_phase)
    {
        tm_printf((UB *) "[junction] %s L=%d R=%d\n", names[junction.phase],
                  (INT) junction.left, (INT) junction.right);
        g_junction_last_phase = junction.phase;
    }

    if (junction.handled)
    {
        g_trace_phase = junction.stopped ? TRACE_STOP : TRACE_CROSS;
        /* Discard the old correction: do not turn toward a side branch. */
        buddy3_steering_reset(&g_test_steering);
        g_test_last_phase = BUDDY3_PHASE_NONE;
        /* Junction crossing owns its own encoder heading reference. */
        g_b_straight_hold = false;
        if (junction.stopped)
        {
            buddy3_stop_observed(true);
        }
        else
        {
            buddy3_drive_observed(junction.left, junction.right);
        }
    }
    else
    {
        if (b_was_crossing)
        {
            buddy3_steering_reset(&g_test_steering);
            g_test_last_phase = BUDDY3_PHASE_NONE;
        }
        output = buddy3_steering_step(
            &g_test_steering, g_test_stage, g_last_touch_side, now_ms,
            g_steering_configs.base_pwm,
            buddy3_clamp_pwm(g_steering_configs.proportional_gain *
                             BUDDY3_GAIN_SCALE));
        buddy3_apply_straight_heading_hold(&output, g_last_touch_side);
        if (output.phase != g_test_last_phase)
        {
            tm_printf((UB *) "[steer] stage=%u %s L=%d R=%d\n",
                      (UINT) g_test_stage, phases[output.phase],
                      (INT) output.left, (INT) output.right);
            g_test_last_phase = output.phase;
        }
        g_trace_phase = output.phase;
        if (B3_STOP == output.phase)
        {
            buddy3_stop_observed(true);
        }
        else
        {
            buddy3_drive_observed(output.left, output.right);
        }
    }
}
#endif

/* ------------------------------------------------------------------ *
 *  Safety - line-loss stop and legacy search completion
 * ------------------------------------------------------------------ */

/* Stop while no line is found.  Returns true when the car was stopped. */
static bool buddy3_lost_stop (const ir_processing_state_t * p_next,
                              uint32_t                      now_ms)
{
    bool b_stopped = false;

#if BUDDY3_LINE_BETWEEN_SENSORS
    (void) p_next;
#if BUDDY3_SEARCH_ENABLE
    /* Sweep and full spin found nothing.  Stay stopped (line following
     * stays on) until a front sensor sees black again. */
    (void) now_ms;
    if (BUDDY3_SEARCH_GAVE_UP == g_search_step)
    {
        buddy3_stop_observed(false);
        if (!g_b_reported_lost)
        {
            tm_printf((UB *) "[line] full spin found no line ahead (dead end?) "
                             "- stopped\n");
            g_b_reported_lost = true;
        }
        b_stopped = true;
    }
#else
    if ((now_ms - g_last_touch_ms) > BUDDY3_LOST_STOP_MS)
    {
        buddy3_stop_observed(false);
        if (!g_b_reported_lost)
        {
            tm_printf((UB *) "[line] no line seen for %u ms - stopped\n",
                      (UINT) BUDDY3_LOST_STOP_MS);
            g_b_reported_lost = true;
        }
        b_stopped = true;
    }
#endif
#else
    /* Line must be seen at least once before driving, then may be lost for
     * up to BUDDY3_LINE_LOST_STOP_MS. */
    if (!p_next->b_line_seen &&
        (!g_b_seen_since_start ||
         ((now_ms - g_line_lost_since_ms) > BUDDY3_LINE_LOST_STOP_MS)))
    {
        buddy3_stop_observed(false);
        if (!g_b_reported_lost)
        {
            tm_printf((UB *) "[line] no line under the sensors (L=%u C=%u raw) "
                             "- waiting\n",
                      (UINT) p_next->sensors[0].raw,
                      (UINT) p_next->sensors[1].raw);
            g_b_reported_lost = true;
        }
        b_stopped = true;
    }
    else
    {
        g_b_seen_since_start = true;
        if (g_b_reported_lost)
        {
            tm_printf((UB *) "[line] line found - driving\n");
            g_b_reported_lost = false;
        }
    }
#endif

    return b_stopped;
}

#if BUDDY3_LINE_BETWEEN_SENSORS
/* ------------------------------------------------------------------ *
 *  Legacy control - stage 0 three-way steering
 * ------------------------------------------------------------------ */

/* Thresholded analog readings -> three-way steering (the derivative is not
 * used here: the position only jumps between -1000, 0 and +1000).
 *   one sensor black  : turn towards it (harder if it stays black)
 *   both white, recent: straight (line is between the sensors)
 *   both white, long  : search sweep, then full spin (see above)
 *   both black        : straight (crossing / start box) */
static void buddy3_steer_three_way (uint32_t now_ms)
{
    int32_t base = (int32_t) g_steering_configs.base_pwm;
    /* The gain scaled per 1000 position units is a PWM difference. */
    int32_t turn =
        (int32_t) (g_steering_configs.proportional_gain * BUDDY3_GAIN_SCALE);
    int32_t  left_pwm  = base;
    int32_t  right_pwm = base;
    uint8_t  side      = g_last_touch_side;
    uint32_t held_ms   = now_ms - g_side_since_ms;

    if ((BUDDY3_SIDE_LEFT == side) || (BUDDY3_SIDE_RIGHT == side))
    {
        int32_t outer = base + turn;
        int32_t inner = base - turn;

        if (inner < BUDDY3_INNER_MIN_PWM)
        {
            inner = BUDDY3_INNER_MIN_PWM;
        }
#if BUDDY3_HARD_TURN_ENABLE
        if (held_ms > BUDDY3_HARD_TURN_MS)
        {
            inner = BUDDY3_HARD_TURN_INNER_PWM;
        }
#else
        (void) held_ms;
#endif

        if (BUDDY3_SIDE_RIGHT == side) /* line under the right sensor */
        {
            left_pwm  = outer;
            right_pwm = inner;
        }
        else /* line under the left sensor: turn left */
        {
            left_pwm  = inner;
            right_pwm = outer;
        }
    }
    else if (BUDDY3_SEARCH_ENABLE && (BUDDY3_SIDE_NONE == side) &&
             ((BUDDY3_SEARCH_IDLE != g_search_step) ||
              ((now_ms - g_last_touch_ms) > BUDDY3_SEARCH_AFTER_MS)))
    {
        buddy3_search_step(now_ms, base, &left_pwm, &right_pwm);
    }
    else
    {
        /* Straight. */
    }

    if (BUDDY3_SEARCH_IDLE != g_search_step)
    {
        g_trace_phase = TRACE_SEEK;
    }
    buddy3_drive_observed(buddy3_clamp_pwm((float32_t) left_pwm),
                          buddy3_clamp_pwm((float32_t) right_pwm));
}

/* ------------------------------------------------------------------ *
 *  Legacy recovery - stage 0 sweep and full-spin search
 * ------------------------------------------------------------------ */

/* Advance the line search and give the spin-on-the-spot wheel commands. */
static void buddy3_search_step (uint32_t now_ms, int32_t base,
                                int32_t * p_left_pwm, int32_t * p_right_pwm)
{
    uint32_t limit_ms    = BUDDY3_SEARCH_SWEEP_STEP_MS;
    bool     b_step_done = false;
#if BUDDY3_SEARCH_USE_ENCODER_ANGLE
    float32_t target_deg = BUDDY3_SEARCH_SWEEP_DEG;
    float32_t turned     = 0.0f;
#endif

    if (BUDDY3_SEARCH_IDLE == g_search_step)
    {
        /* Right first only if the right sensor saw the line last. */
        buddy3_search_begin_step(
            BUDDY3_SEARCH_FIRST,
            (BUDDY3_SIDE_RIGHT == g_last_single_side) ? -1 : 1, now_ms);
        g_search_origin_deg = g_search_step_deg;
        g_b_reported_behind = false;
        tm_printf((UB *) "[line] line lost - sweeping %s first (max %u deg)\n",
                  (g_search_dir > 0) ? "LEFT" : "RIGHT",
                  (UINT) BUDDY3_SEARCH_SWEEP_DEG);
    }
    if (BUDDY3_SEARCH_SECOND == g_search_step)
    {
        /* From one side all the way across to the other. */
        limit_ms = 2u * BUDDY3_SEARCH_SWEEP_STEP_MS;
    }
    else if (BUDDY3_SEARCH_SPIN == g_search_step)
    {
        limit_ms = BUDDY3_SEARCH_SPIN_MAX_MS;
    }
    else
    {
        /* First and return steps use the single-sweep limit. */
    }

    b_step_done = ((now_ms - g_search_step_ms) >= limit_ms);
#if BUDDY3_SEARCH_USE_ENCODER_ANGLE
    if (BUDDY3_SEARCH_SECOND == g_search_step)
    {
        target_deg = 2.0f * BUDDY3_SEARCH_SWEEP_DEG;
    }
    else if (BUDDY3_SEARCH_SPIN == g_search_step)
    {
        target_deg = BUDDY3_SEARCH_SPIN_MAX_DEG;
    }
    else
    {
        /* One sweep angle. */
    }
    turned = buddy3_encoder_angle_deg() - g_search_step_deg;
    if (turned < 0.0f)
    {
        turned = -turned;
    }
    b_step_done = b_step_done || (turned >= target_deg);
#endif

    if (b_step_done)
    {
        if (BUDDY3_SEARCH_FIRST == g_search_step)
        {
            /* g_search_dir is +/-1, so its negation fits int8_t. */
            buddy3_search_begin_step(BUDDY3_SEARCH_SECOND,
                                     (int8_t) (-g_search_dir), now_ms);
        }
        else if (BUDDY3_SEARCH_SECOND == g_search_step)
        {
            buddy3_search_begin_step(BUDDY3_SEARCH_RETURN,
                                     (int8_t) (-g_search_dir), now_ms);
        }
        else if (BUDDY3_SEARCH_RETURN == g_search_step)
        {
            /* Keep turning the same way: one smooth full spin. */
            buddy3_search_begin_step(BUDDY3_SEARCH_SPIN, g_search_dir, now_ms);
            tm_printf((UB *) "[line] nothing within %u deg - spinning to "
                             "find the line\n",
                      (UINT) BUDDY3_SEARCH_SWEEP_DEG);
        }
        else
        {
            g_search_step = BUDDY3_SEARCH_GAVE_UP;
        }
    }

    if (BUDDY3_SEARCH_GAVE_UP == g_search_step)
    {
        *p_left_pwm  = 0;
        *p_right_pwm = 0;
    }
    else
    {
        /* Spin on the spot: +1 = left (right wheel forward). */
        *p_left_pwm  = -(int32_t) g_search_dir * base;
        *p_right_pwm = (int32_t) g_search_dir * base;
    }
}
#endif

/*** end of file ***/

