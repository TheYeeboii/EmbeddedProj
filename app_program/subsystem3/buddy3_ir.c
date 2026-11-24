/**
 * @file    buddy3_ir.c
 * @brief   Buddy 3: IR line following and barcode decoding.
 *
 * File guide:
 *   Core functionality: sensor setup, update scheduling, line steering,
 *   junction handling, line-loss recovery and safety stops.
 *   Bonus features: barcode reading/navigation, encoder heading hold,
 *   obstacle-avoidance handoff and dashboard diagnostics.
 *
 * Line following runs buddy3_staged_step(), which uses the separate
 * buddy3_steering and buddy3_junction modules (corner holding and bounded
 * corner recovery).  See buddy3_follow_line().
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
#include "pin_config.h"
#include "robot_hal.h"
#include "barcode_tuning.h"
#include "barcode_slowdown.h"
#include "buddy3_steering.h"
#include "robot_steering_trace.h"
#include "robot_barcode_trace.h"
#include "robot_ir_stream.h"
#include "buddy3_barcode.h"
#include "buddy3_junction.h"

_Static_assert((BUDDY3_BARCODE_PERIOD_MS > 0u) &&
               (BUDDY3_SAMPLE_PERIOD_MS >= BUDDY3_BARCODE_PERIOD_MS) &&
               (BUDDY3_SAMPLE_PERIOD_MS % BUDDY3_BARCODE_PERIOD_MS == 0u),
               "Front interval must be a multiple of barcode interval");
_Static_assert(BUDDY3_SAMPLE_PERIOD_MS <= 255u,
               "IR stream interval must fit its wire field");

/* ------------------------------------------------------------------ *
 *  Core settings - sensor thresholds and layout
 * ------------------------------------------------------------------ */

#define BUDDY3_LINE_THRESHOLD     (350u)
#define BUDDY3_JUNCTION_THRESHOLD (800u)
#define BUDDY3_NORMALIZED_MAX     (1000)
#define BUDDY3_SENSOR_COUNT       (3u)
#define BUDDY3_PI                 (3.14159265f)

/* ---- Sensor layout ---------------------------------------------------
 * The two FRONT sensors (ADC0, ADC1) sit on the black bracket; the third
 * (ADC2) is on the white arm beside the car and reads the barcodes.
 * Steering compares the two front sensors (position = right - left).  The
 * same rule works whether the line is wider than the sensor gap or narrower
 * (ours: both on white = centred).
 * If the car steers AWAY from the line instead of back onto it, swap the
 * two numbers below (sensor wires or motor wires are the other way round).
 */
/* Confirmed 30 Sep with a hand test: ADC0 = left sensor, ADC1 = center.
 * ADC indices of the front sensors, from the car's point of view. */
#define BUDDY3_FRONT_RIGHT (0u)
#define BUDDY3_FRONT_LEFT  (1u)
#define BUDDY3_SIDE_SENSOR (2u) /* the barcode sensor on the side arm */
/* All three sensors on black for this long = the car was lifted or put on
 * a dark surface (the side sensor is normally on white): stop.  Longer than
 * the time needed to drive off the black start box. */
#define BUDDY3_ALL_BLACK_STOP_MS (3000u)
/* End of the line: the line is narrower than the sensor gap, so both sensors
 * on white looks the same whether the line is centred or gone.  While
 * following, the rover drifts and touches an edge every few centimetres; if
 * neither front sensor has touched the line for this much wheel travel, the
 * line has ended (or been lost): stop instead of driving off.  If an encoder
 * has failed, travel cannot be measured, so the time limit is used. */
#define BUDDY3_NO_LINE_STOP_MM (300.0f)
#define BUDDY3_NO_LINE_STOP_MS (3000u)
/* Our line is 18 mm, narrower than the gap between the front sensors, so
 * "both on floor" is NORMAL (line between them). */
#define BUDDY3_EDGE_THRESHOLD (500u) /* normalized: sensor is over black */
/* Front sensor pattern ("side"): bit 0 left black, bit 1 right black. */
#define BUDDY3_SIDE_NONE  (0u)
#define BUDDY3_SIDE_LEFT  (1u)
#define BUDDY3_SIDE_RIGHT (2u)
/* ------------------------------------------------------------------ *
 *  Steering support - PWM limits and encoder heading hold
 * ------------------------------------------------------------------ */

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


/* ------------------------------------------------------------------ *
 *  Bonus settings - barcode sampling and classification
 * ------------------------------------------------------------------ */

/* Barcode reader: hysteresis on the side sensor and the largest heading
 * change a longitudinal barcode tolerates (radians). Narrow printed bars blur
 * under the sensor spot: the faintest peaks near raw 1050, about 24 % of the
 * side calibration, while white paper stays below raw 500 (8 %). Black
 * starts at 21 % (raw ~960) and white returns at 14 % (raw ~700). Manual
 * driving gets 20 degrees instead of 10: the encoders disagree by a few
 * percent even when the car drives straight. */
/* The dashboard draws these two thresholds from its own copies: change
 * dashboard/rover-wire.js irStream(), ir-stream-model.js and
 * ir-stream-view.js with them. */
#define BUDDY3_BARCODE_BLACK_ON    (210u)
#define BUDDY3_BARCODE_WHITE_ON    (140u)
#define BUDDY3_BARCODE_MAX_YAW_RAD (0.3491f)
#define BUDDY3_BARCODE_MANUAL_YAW_RAD (0.3491f)
#define BUDDY3_BARCODE_STEP_FRACTION  (0.25f)
/* Most encoder travel buddy3_smooth_travel() holds back, in guard steps
 * (3 mm with the default 3 mm element). */
#define BUDDY3_BARCODE_BACKLOG_STEPS (4.0f)
/* A barcode element must span at least four encoder ticks. */
#define BUDDY3_BARCODE_TICKS_PER_ELEMENT (4.0f)
#define BUDDY3_BARCODE_ELEMENT_MAX_MM    (1000u)
#define BUDDY3_BARCODE_OFFSET_MAX_MM     (500)

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
    bool                b_barcode_ready;
    uint8_t             barcode_command;
} ir_processing_state_t;

/* API PWM before Buddy 2's global speed limit and trim. Base is 60%;
 * main's 27 percentage-point curve power commands about 33%/87% on a
 * one-sided detection. Junction crossing uses the base and encoder hold. */
/* Sensor order: front right, front left, side barcode. */
static ir_calibration_params_t g_sensor_configs[BUDDY3_SENSOR_COUNT] = {
    {200u, 3000u, false}, /* right sensor */
    {200u, 1300u, false}, /* left sensor */
    /* Side: measured over the printed barcode, white paper reads ~200 and a
     * wide bar ~3800 raw. */
    {0u, 1u, (BARCODE_DIGITAL_BLACK_LEVEL == 0u)} /* DO level, not ADC */
};

static line_follow_configs_t g_steering_configs = {
    /* Default: 27 percentage-point curve power. */
    (27.0f * (float32_t) BUDDY2_PWM_MAX) / 100000.0f,
    2.5f, /* derivative gain: 2.5f = 2500 / 1000 */
    19660 /* base PWM: 19660 = 60 % duty (ROBOT_LINE_BASE_DEFAULT_PCT) */
};

static ir_processing_state_t g_state;
static barcode_slowdown_t g_barcode_slowdown;
static volatile bool         g_b_line_following = false;
/* Manual-drive barcode reading: the controller task writes the request (one
 * byte, single writer); only the IR task acts on it and keeps
 * g_b_manual_scan, so the reader is never reset from another task. */
static volatile bool g_b_manual_scan_request = false;
/* Levels set by hand from the dashboard: written by the controller task in
 * one DI/EI step and applied by the IR task, which therefore never sees a
 * half-written request. */
static volatile uint8_t  g_level_sensor    = BUDDY3_SIDE_SENSOR;
static volatile uint16_t g_level_white     = 0u;
static volatile uint16_t g_level_black     = 0u;
static volatile bool     g_b_level_request = false;
static bool              g_b_manual_scan   = false;
static buddy3_steering_t g_test_steering;
static buddy3_junction_t g_junction;
static uint8_t           g_junction_last_phase = B3J_IDLE;
/* Turn a barcode asked for at the next junction (written by the controller,
 * read by the IR task), and the turn handed back once the junction is
 * reached (written by the IR task, taken by the controller). One byte each. */
static volatile uint8_t g_junction_turn         = BUDDY3_TURN_NONE;
static volatile uint8_t g_junction_turn_reached = BUDDY3_TURN_NONE;
static uint8_t          g_test_last_phase       = BUDDY3_PHASE_NONE;
/* Records brief front-line observations while normal following is paused for
 * Buddy 5 avoidance.  The final return phase consumes this event. */
static volatile bool            g_b_line_seen_event  = false;
static uint32_t                 g_last_update_ms     = 0u;
static int16_t                  g_previous_position  = 0;
static buddy3_barcode_decoder_t g_barcode_decoder;
static buddy3_barcode_config_t  g_barcode_config = {
    BUDDY3_BARCODE_MIN_ELEMENT_MM};
static float32_t g_barcode_offset           = BUDDY3_BARCODE_OFFSET_LEFT_MM;
static bool      g_b_barcode_calibrated     = false;
static bool      g_b_barcode_odometry_valid = false;
static float32_t g_barcode_last_left        = 0.0f;
static float32_t g_barcode_last_right       = 0.0f;
static float32_t g_barcode_yaw              = 0.0f;
/* Encoder travel not yet given to the reader (buddy3_smooth_travel). */
static float32_t              g_barcode_backlog = 0.0f;
static buddy3_barcode_event_t g_pending_barcode;
static uint32_t               g_barcode_dropped = 0u;
static bool                   g_b_barcode_black = false;
static bool      g_b_barcode_ready        = false;
static uint8_t   g_last_touch_side        = 0u; /* 0 none, 1 L, 2 R, 3 both */
static uint32_t  g_all_black_since_ms     = 0u;
static bool      g_b_reported_black       = false;
/* Wheel travel (mean of both wheels) and time at the last front-sensor touch,
 * for the end-of-line stop.  false = take a new reference. */
static bool      g_b_touch_valid          = false;
static float32_t g_touch_mm               = 0.0f;
static uint32_t  g_touch_ms               = 0u;
static bool      g_b_straight_hold        = false;
static float32_t g_straight_hold_left_mm  = 0.0f;
static float32_t g_straight_hold_right_mm = 0.0f;
static uint8_t   g_trace_phase            = TRACE_LEGACY;

/* ------------------------------------------------------------------ *
 *  Internal function declarations
 * ------------------------------------------------------------------ */

static int16_t   buddy3_clamp_pwm (float32_t value);
static uint16_t  buddy3_normalize (uint16_t raw, uint16_t minimum,
                                   uint16_t maximum, bool b_line_is_dark);
static void      buddy3_apply_manual_scan (uint32_t now_ms);
static void      buddy3_apply_levels (void);
static void      buddy3_trace_configure (uint16_t min_element_mm,
                                         int16_t  offset_left_mm);
static void      buddy3_sample_side (uint32_t now_ms);
static float32_t buddy3_smooth_travel (float32_t travel);
static void      buddy3_decode_barcode (uint16_t sensor_value, uint16_t raw,
                                        uint32_t now_ms, bool b_line_step);
static bool      buddy3_barcode_turned (void);
static void      buddy3_colour_reset (void);
static bool
buddy3_barcode_permitted (const buddy2_encoder_position_t * p_motion,
                          float32_t left_step, float32_t right_step);
static uint8_t
buddy3_barcode_blocked (const buddy2_encoder_position_t * p_motion,
                        float32_t left_step, float32_t right_step,
                        float32_t travel, bool b_permitted);
static robot_steering_input_t buddy3_trace_input (void);
static void buddy3_drive_observed (int16_t left, int16_t right);
static void buddy3_stop_observed (bool b_disable);
static void buddy3_stop_with_input (const robot_steering_input_t * p_input);
static void buddy3_read_sensors (ir_processing_state_t * p_next,
                                 uint32_t                now_ms);
static void buddy3_follow_line (const ir_processing_state_t * p_next,
                                int16_t previous_position, uint32_t now_ms);
static void    buddy3_track_side (const ir_processing_state_t * p_next,
                                  uint32_t                      now_ms);
static void    buddy3_log_side (uint8_t side, uint32_t now_ms);
static bool    buddy3_dark_surface_stop (const ir_processing_state_t * p_next,
                                         uint32_t                      now_ms);
static bool    buddy3_end_of_line_stop (uint32_t now_ms);
static void    buddy3_staged_step (uint32_t now_ms);

/* ------------------------------------------------------------------ *
 *  Core API - initialization, scheduling and line-follow controls
 * ------------------------------------------------------------------ */

void buddy3_init (void)
{
    ir_processing_state_t empty = {0};

    hal_adc_init();
    hal_adc_pin_init(IR_SENSOR_LEFT_PIN);
    hal_adc_pin_init(IR_SENSOR_CENTER_PIN);
    hal_gpio_input_pullup(BARCODE_DIGITAL_PIN);

    g_barcode_slowdown = (barcode_slowdown_t) {0};
    buddy2_set_barcode_slowdown(false);
    g_state             = empty;
    g_last_update_ms    = 0u;
    g_b_line_following  = false;
    g_b_line_seen_event = false;
    /* The defaults are small whole millimetres, so they fit the types. */
    (void) buddy3_configure_barcode((uint16_t) BUDDY3_BARCODE_MIN_ELEMENT_MM,
                                    (int16_t) BUDDY3_BARCODE_OFFSET_LEFT_MM);
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
    barcode_tuning_state_t side_tuning       = {0};
    int16_t               previous_position = 0;
    UINT                  interrupt_state   = 0u;

    if ((now_ms - g_last_update_ms) < BUDDY3_SAMPLE_PERIOD_MS)
    {
        /* Between line steps: only the side sensor, for the barcode. */
        buddy3_sample_side(now_ms);
    }
    else
    {
        g_last_update_ms = now_ms;
        g_trace_phase    = TRACE_LEGACY;

        previous_position = g_previous_position;
        buddy3_read_sensors(&next, now_ms);

        /* The outrigger sensor (ADC2) reads the barcodes beside the track. */
        buddy3_apply_levels();
        buddy3_apply_manual_scan(now_ms);
        buddy3_decode_barcode(next.sensors[BUDDY3_SIDE_SENSOR].normalized,
                              next.sensors[BUDDY3_SIDE_SENSOR].raw, now_ms,
                              true);
        barcode_tuning_get(&side_tuning);
        next.raw_tuning[BUDDY3_SIDE_SENSOR] = side_tuning.enabled;
        next.black[BUDDY3_SIDE_SENSOR] = side_tuning.enabled ?
            side_tuning.black :
            (next.sensors[BUDDY3_SIDE_SENSOR].normalized >= BUDDY3_EDGE_THRESHOLD);
        next.b_barcode_ready = g_b_barcode_ready;
        /* A decoded symbol is an uppercase ASCII letter. */
        next.barcode_command =
            (g_b_barcode_ready && (B3BC_VALID == g_pending_barcode.kind))
                ? (uint8_t) g_pending_barcode.decoded.symbol
                : 0u;

        DI(interrupt_state);
        g_state = next;
        if (next.b_line_seen)
        {
            g_b_line_seen_event = true;
        }
        EI(interrupt_state);

        buddy3_track_side(&next, now_ms);

        if (g_b_line_following)
        {
            buddy3_follow_line(&next, previous_position, now_ms);
        }
    }
}

void buddy3_start_line_follow (void)
{
    robot_barcode_trace_cancel(g_last_update_ms);
    buddy3_junction_reset(&g_junction);
    g_junction_last_phase = B3J_IDLE;
    buddy3_colour_reset();
    /* A turn belongs to the run that read its barcode. */
    g_junction_turn         = BUDDY3_TURN_NONE;
    g_junction_turn_reached = BUDDY3_TURN_NONE;
    buddy3_barcode_reset(&g_barcode_decoder);
    g_b_barcode_odometry_valid = false;
    g_b_barcode_ready          = false;
    g_state.b_barcode_ready    = false;
    buddy3_steering_reset(&g_test_steering);
    g_test_last_phase    = BUDDY3_PHASE_NONE;
    g_b_straight_hold    = false;
    g_b_touch_valid      = false; /* the start counts as a touch */
    g_b_line_following   = true;
}

void buddy3_stop_line_follow (void)
{
    buddy3_stop_with_input(NULL);
}

void buddy3_set_manual_barcode (bool b_enabled)
{
    g_b_manual_scan_request = b_enabled;
}

bool buddy3_line_following (void)
{
    return g_b_line_following;
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
        p_state->barcode_ready   = g_state.b_barcode_ready;
        p_state->barcode_command = g_state.barcode_command;
        EI(interrupt_state);
    }
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
    buddy3_ir_state_t state = {0};

    buddy3_get_state(&state);
    if ((NULL != p_left_black) && (NULL != p_right_black) &&
        (NULL != p_side_black))
    {
        UINT lock;
        DI(lock);
        *p_left_black = g_state.black[BUDDY3_FRONT_LEFT];
        *p_right_black = g_state.black[BUDDY3_FRONT_RIGHT];
        *p_side_black = g_state.black[BUDDY3_SIDE_SENSOR];
        EI(lock);
    }
}

bool buddy3_set_ir_levels (uint8_t sensor, uint16_t white, uint16_t black)
{
    /* Dashboard numbering to the firmware's sensor index. */
    static const uint8_t index[] = {BUDDY3_SIDE_SENSOR, BUDDY3_FRONT_LEFT,
                                    BUDDY3_FRONT_RIGHT};
    UINT                 interrupt_state = 0u;
    bool                 b_ok =
        (sensor <= BUDDY3_LEVELS_RIGHT) && (black <= BUDDY3_RAW_MAX) &&
        (white < black) &&
        (((uint32_t) black - (uint32_t) white) >= BUDDY3_LEVEL_MIN_SPAN) &&
        (sensor != BUDDY3_LEVELS_BARCODE); /* barcode sensor is digital */

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
 *  Bonus API - barcode navigation and decoder configuration
 * ------------------------------------------------------------------ */

void buddy3_set_junction_turn (uint8_t turn)
{
    if ((BUDDY3_TURN_NONE == turn) || (BUDDY3_TURN_LEFT == turn) ||
        (BUDDY3_TURN_RIGHT == turn) || (BUDDY3_TURN_UTURN == turn))
    {
        g_junction_turn = turn; /* One byte store. */
    }
}

bool buddy3_take_junction_turn (uint8_t * p_turn)
{
    UINT interrupt_state = 0u;
    bool b_taken         = false;

    if (NULL != p_turn)
    {
        DI(interrupt_state);
        if (BUDDY3_TURN_NONE != g_junction_turn_reached)
        {
            *p_turn                 = g_junction_turn_reached;
            g_junction_turn_reached = BUDDY3_TURN_NONE;
            b_taken                 = true;
        }
        EI(interrupt_state);
    }

    return b_taken;
}

bool buddy3_take_barcode (buddy3_barcode_event_t * p_event)
{
    UINT interrupt_state = 0u;
    bool b_taken         = false;

    if (NULL != p_event)
    {
        DI(interrupt_state);
        if (g_b_barcode_ready)
        {
            *p_event                = g_pending_barcode;
            p_event->dropped        = g_barcode_dropped;
            g_b_barcode_ready       = false;
            g_state.b_barcode_ready = false;
            b_taken                 = true;
        }
        EI(interrupt_state);
    }

    return b_taken;
}

bool buddy3_configure_barcode (uint16_t min_element_mm, int16_t offset_left_mm)
{
    buddy2_calibration_t geometry     = {0};
    float32_t            left_tick    = 0.0f;
    float32_t            right_tick   = 0.0f;
    float32_t            minimum_cell = 0.0f;
    bool                 b_ok         = false;

    if (!g_b_line_following)
    {
        robot_barcode_trace_cancel(g_last_update_ms);
        robot_barcode_trace_enable(false);
        g_b_barcode_calibrated = false;
        buddy3_barcode_reset(&g_barcode_decoder);
        g_b_barcode_ready          = false;
        g_state.b_barcode_ready    = false;
        g_b_barcode_odometry_valid = false;
        buddy2_get_calibration(&geometry);

        if (0u == min_element_mm)
        {
            tm_printf((UB *) "[barcode] disabled: configure minimum resolvable "
                             "element and sensor offset\n");
        }
        else if (!(geometry.wheel_diameter_mm > 0.0f) ||
                 !(geometry.track_width_mm > 0.0f) ||
                 !(geometry.left_ticks_per_wheel_rev > 0.0f) ||
                 !(geometry.right_ticks_per_wheel_rev > 0.0f))
        {
            /* Buddy 2 geometry is not usable: stay disabled. */
        }
        else
        {
            left_tick    = (BUDDY3_PI * geometry.wheel_diameter_mm) /
                           geometry.left_ticks_per_wheel_rev;
            right_tick   = (BUDDY3_PI * geometry.wheel_diameter_mm) /
                           geometry.right_ticks_per_wheel_rev;
            minimum_cell = BUDDY3_BARCODE_TICKS_PER_ELEMENT *
                           ((left_tick > right_tick) ? left_tick : right_tick);
            if (((float32_t) min_element_mm < minimum_cell) ||
                (min_element_mm > BUDDY3_BARCODE_ELEMENT_MAX_MM) ||
                (offset_left_mm < -BUDDY3_BARCODE_OFFSET_MAX_MM) ||
                (offset_left_mm > BUDDY3_BARCODE_OFFSET_MAX_MM))
            {
                /* minimum_cell is a few millimetres, so it fits UINT. */
                tm_printf((UB *) "[barcode] rejected: minimum element needs "
                                 ">=%u mm at current encoder resolution; "
                                 "offset -500..500\n",
                          (UINT) (minimum_cell + 0.999f));
            }
            else
            {
                g_barcode_config.min_element_mm = (float32_t) min_element_mm;
                g_barcode_offset                = (float32_t) offset_left_mm;
                g_b_barcode_calibrated          = true;
                buddy3_trace_configure(min_element_mm, offset_left_mm);
                tm_printf((UB *) "[barcode] configured minimum-element=%u mm "
                                 "offset-left=%d mm (adaptive Code39)\n",
                          (UINT) min_element_mm, (INT) offset_left_mm);
                b_ok = true;
            }
        }
    }

    return b_ok;
}

bool buddy3_barcode_calibrated (void)
{
    return g_b_barcode_calibrated;
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

/* ------------------------------------------------------------------ *
 *  Bonus processing - barcode calibration, sampling and decoding
 * ------------------------------------------------------------------ */

/* Give the barcode trace the reader's present settings. */
static void buddy3_trace_configure (uint16_t min_element_mm,
                                    int16_t  offset_left_mm)
{
    buddy2_calibration_t geometry = {0};

    buddy2_get_calibration(&geometry);
    {
        int32_t trace_config[BARCODE_CONFIG_COUNT] = {
            g_sensor_configs[BUDDY3_SIDE_SENSOR].minimum,
            g_sensor_configs[BUDDY3_SIDE_SENSOR].maximum,
            g_sensor_configs[BUDDY3_SIDE_SENSOR].line_is_dark,
            (int32_t) BUDDY3_BARCODE_WHITE_ON,
            (int32_t) BUDDY3_BARCODE_BLACK_ON,
            (int32_t) min_element_mm * 100,
            (int32_t) offset_left_mm * 10,
            robot_barcode_scaled(geometry.track_width_mm, 10.0f),
            robot_barcode_scaled(geometry.wheel_diameter_mm, 10.0f),
            robot_barcode_scaled(geometry.left_ticks_per_wheel_rev, 10.0f),
            robot_barcode_scaled(geometry.right_ticks_per_wheel_rev, 10.0f)};

        robot_barcode_trace_configure(trace_config);
    }
}

/* Apply sensor levels set from the dashboard (IR task, every line step).
 * Skipped while a barcode is being read. */
static void buddy3_apply_levels (void)
{
    if (g_b_level_request &&
        ((BUDDY3_SIDE_SENSOR != g_level_sensor) || !g_barcode_decoder.active))
    {
        g_b_level_request                        = false;
        g_sensor_configs[g_level_sensor].minimum = g_level_white;
        g_sensor_configs[g_level_sensor].maximum = g_level_black;
        tm_printf((UB *) "[ir] sensor %u levels white %u black %u\n",
                  (UINT) g_level_sensor, (UINT) g_level_white,
                  (UINT) g_level_black);
    }
}

/* A manual scan starting or ending restarts the reader, here in the IR task
 * (before decoding), so a half-read barcode from before is never mixed in. */
static void buddy3_apply_manual_scan (uint32_t now_ms)
{
    if (g_b_manual_scan_request != g_b_manual_scan)
    {
        g_b_manual_scan = g_b_manual_scan_request;
        robot_barcode_trace_cancel(now_ms);
        buddy3_barcode_reset(&g_barcode_decoder);
        buddy3_colour_reset();
        g_b_barcode_odometry_valid = false;
    }
}

/* Read only the side sensor and feed the reader (between line steps). */
static void buddy3_sample_side (uint32_t now_ms)
{
    uint16_t raw = hal_gpio_read(BARCODE_DIGITAL_PIN) ? 1u : 0u;

    buddy3_apply_manual_scan(now_ms);
    buddy3_decode_barcode(
        buddy3_normalize(raw, g_sensor_configs[BUDDY3_SIDE_SENSOR].minimum,
                         g_sensor_configs[BUDDY3_SIDE_SENSOR].maximum,
                         g_sensor_configs[BUDDY3_SIDE_SENSOR].line_is_dark),
        raw, now_ms, false);
}

/* Spread encoder ticks evenly over the samples.  Each wheel moves in
 * 0.53 mm ticks, so with 2 ms samples the averaged travel jumps between
 * 0, 0.26, 0.53 and 0.79 mm even at a steady 0.5 mm per sample, and a single
 * 0.79 mm sample would trip the reader's 0.75 mm spacing guard.  Travel above
 * the guard is carried into the next samples instead; the total distance is
 * unchanged, so bar widths are too.  If the robot really is too fast the
 * backlog keeps growing, and past BUDDY3_BARCODE_BACKLOG_STEPS guard-sized
 * steps it is passed on whole, so the guard still stops the scan. */
static float32_t buddy3_smooth_travel (float32_t travel)
{
    float32_t limit =
        g_barcode_config.min_element_mm * BUDDY3_BARCODE_STEP_FRACTION;
    float32_t fed = travel;

    if (!(travel >= 0.0f) || !(limit > 0.0f))
    {
        /* Reverse, NaN or unconfigured: no smoothing, the reader decides. */
        g_barcode_backlog = 0.0f;
    }
    else
    {
        g_barcode_backlog += travel;
        if (g_barcode_backlog > (limit * BUDDY3_BARCODE_BACKLOG_STEPS))
        {
            fed = g_barcode_backlog; /* Really too fast. */
        }
        else
        {
            fed = (g_barcode_backlog < limit) ? g_barcode_backlog : limit;
        }
        g_barcode_backlog -= fed;
    }

    return fed;
}

/* Called every BUDDY3_BARCODE_PERIOD_MS (2 ms); bar boundaries are measured
 * in encoder distance, so the sampling rate only limits the speed.  The IR
 * stream (rover/ir) keeps one row per 10 ms line step (b_line_step). */
static void buddy3_decode_barcode (uint16_t sensor_value, uint16_t raw,
                                   uint32_t now_ms, bool b_line_step)
{
    buddy2_encoder_position_t   motion       = {0};
    buddy2_calibration_t        geometry     = {0};
    float32_t                   left_step    = 0.0f;
    float32_t                   right_step   = 0.0f;
    float32_t                   travel       = 0.0f;
    buddy3_barcode_event_t      event        = {0};
    robot_barcode_observation_t observation  = {0};
    bool                        b_was_active = g_barcode_decoder.active;
    float32_t                   unfinished =
        g_barcode_decoder.position - g_barcode_decoder.edge_position;
    bool b_emitted       = false;
    bool b_permitted     = false;
    UINT interrupt_state = 0u;

    buddy2_get_encoder_position(&motion);
    buddy2_get_calibration(&geometry);
    if (!g_b_barcode_odometry_valid)
    {
        g_barcode_last_left        = motion.left_distance_mm;
        g_barcode_last_right       = motion.right_distance_mm;
        g_b_barcode_odometry_valid = true;
        g_barcode_backlog          = 0.0f;
    }
    left_step            = motion.left_distance_mm - g_barcode_last_left;
    right_step           = motion.right_distance_mm - g_barcode_last_right;
    g_barcode_last_left  = motion.left_distance_mm;
    g_barcode_last_right = motion.right_distance_mm;
    if (!g_barcode_decoder.active)
    {
        g_barcode_yaw = 0.0f;
    }
    if (geometry.track_width_mm > 0.0f)
    {
        g_barcode_yaw += (right_step - left_step) / geometry.track_width_mm;
    }
    b_permitted = buddy3_barcode_permitted(&motion, left_step, right_step);
    travel      = buddy3_smooth_travel(buddy3_barcode_travel(
        left_step, right_step, geometry.track_width_mm, g_barcode_offset));

    g_b_barcode_black = barcode_tuning_sample(raw, g_b_barcode_black);
    {
        bool was_slow = g_barcode_slowdown.active;
        bool slow = barcode_slowdown_update(&g_barcode_slowdown,
                                             g_b_barcode_black, now_ms);
        if (slow != was_slow)
        {
            buddy2_set_barcode_slowdown(slow);
            tm_printf((UB *) "[barcode] %s\n",
                slow ? "black: temporary speed 60%" :
                       "white over 3 seconds: normal speed restored");
        }
    }

    observation.ts_ms      = now_ms;
    observation.raw        = raw;
    observation.normalized = sensor_value;
    observation.left_x10 = robot_barcode_scaled(motion.left_distance_mm, 10.0f);
    observation.right_x10 =
        robot_barcode_scaled(motion.right_distance_mm, 10.0f);
    observation.black       = g_b_barcode_black ? 1u : 0u;
    observation.travel_x100 = robot_barcode_scaled(travel, 100.0f);
    observation.blocked = buddy3_barcode_blocked(&motion, left_step, right_step,
                                                 travel, b_permitted);
    if (travel > 0.0f)
    {
        unfinished += travel;
    }
    if (b_line_step)
    {
        robot_ir_stream_record(
            &observation, g_sensor_configs[BUDDY3_SIDE_SENSOR].minimum,
            g_sensor_configs[BUDDY3_SIDE_SENSOR].maximum,
            g_sensor_configs[BUDDY3_SIDE_SENSOR].line_is_dark);
    }
    b_emitted = buddy3_barcode_step(&g_barcode_decoder, &g_barcode_config,
                                    g_b_barcode_black, travel, now_ms,
                                    b_permitted, &event);
    robot_barcode_trace_observe(&g_barcode_decoder, &observation, b_was_active,
                                unfinished, b_emitted ? &event : NULL);
    if (b_emitted)
    {
        DI(interrupt_state);
        /* Keep the first unread event; do not overwrite it with a later
         * frame. */
        if (!g_b_barcode_ready)
        {
            g_pending_barcode = event;
            g_b_barcode_ready = true;
        }
        else
        {
            ++g_barcode_dropped;
        }
        EI(interrupt_state);
    }
}

/* Start the jump detector afresh, e.g. after the levels change: the next
 * reading is judged by level alone and becomes the first peak or valley. */
static void buddy3_colour_reset (void)
{
    g_b_barcode_black = false;
}

/* True once the heading has changed too much for a longitudinal barcode. */
static bool buddy3_barcode_turned (void)
{
    float32_t limit = BUDDY3_BARCODE_MAX_YAW_RAD;

    if (!g_b_line_following)
    {
        limit = BUDDY3_BARCODE_MANUAL_YAW_RAD;
    }

    return ((g_barcode_yaw > limit) || (g_barcode_yaw < -limit));
}

/* The reader may only run while tracking straight-ish along the line, or
 * while driven straight forward by hand, with healthy encoders, forward
 * travel and a small heading change. */
static bool
buddy3_barcode_permitted (const buddy2_encoder_position_t * p_motion,
                          float32_t left_step, float32_t right_step)
{
    bool b_tracking =
        g_b_line_following && !buddy3_junction_active(&g_junction) &&
        ((B3_TRACK == g_test_last_phase) || (B3_MICRO == g_test_last_phase));

    return ((b_tracking || g_b_manual_scan) && g_b_barcode_calibrated &&
            !p_motion->left_encoder_fault && !p_motion->right_encoder_fault &&
            /* Large turns, reverse and recovery invalidate a barcode. */
            !(left_step < 0.0f) && !(right_step < 0.0f) &&
            !buddy3_barcode_turned());
}

/* Why the reader is suspended, as BARCODE_BLOCKED_* bits. */
static uint8_t
buddy3_barcode_blocked (const buddy2_encoder_position_t * p_motion,
                        float32_t left_step, float32_t right_step,
                        float32_t travel, bool b_permitted)
{
    uint8_t blocked = 0u;

    if ((!g_b_line_following && !g_b_manual_scan) || !g_b_barcode_calibrated)
    {
        blocked |= BARCODE_BLOCKED_STOPPED;
    }
    if (p_motion->left_encoder_fault || p_motion->right_encoder_fault)
    {
        blocked |= BARCODE_BLOCKED_ENCODER;
    }
    if (buddy3_barcode_turned())
    {
        blocked |= BARCODE_BLOCKED_TURN;
    }
    if ((left_step < 0.0f) || (right_step < 0.0f) || (travel < 0.0f))
    {
        blocked |= BARCODE_BLOCKED_REVERSE;
    }
    if (b_permitted && (travel > (g_barcode_config.min_element_mm *
                                  BUDDY3_BARCODE_STEP_FRACTION)))
    {
        blocked |= BARCODE_BLOCKED_SAMPLE_GAP;
    }
    /* Junction and recovery phases only matter while line following. */
    if (g_b_line_following &&
        (buddy3_junction_active(&g_junction) ||
         ((B3_TRACK != g_test_last_phase) && (B3_MICRO != g_test_last_phase))))
    {
        blocked |= BARCODE_BLOCKED_JUNCTION;
    }

    return blocked;
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
    robot_barcode_trace_cancel(g_last_update_ms);
    g_b_line_following = false;
    buddy3_junction_reset(&g_junction);
    buddy3_barcode_reset(&g_barcode_decoder);
    g_b_barcode_ready       = false;
    g_state.b_barcode_ready = false;
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

    /* Read the sensors first (slow part, no lock held). */
    raw[0] = hal_adc_read(IR_ADC_CHANNEL_LEFT);
    raw[1] = hal_adc_read(IR_ADC_CHANNEL_CENTER);
    raw[2] = hal_gpio_read(BARCODE_DIGITAL_PIN) ? 1u : 0u; /* digital DO */

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
}

/* ------------------------------------------------------------------ *
 *  Core control - safety checks and steering-path selection
 * ------------------------------------------------------------------ */

/* Decide the wheel commands while line following is on. */
static void buddy3_follow_line (const ir_processing_state_t * p_next,
                                int16_t previous_position, uint32_t now_ms)
{
    (void) previous_position;
    if (!buddy3_dark_surface_stop(p_next, now_ms) &&
        !buddy3_end_of_line_stop(now_ms))
    {
        buddy3_staged_step(now_ms);
    }
}

/* End of the line (see BUDDY3_NO_LINE_STOP_MM).  Returns true when the car
 * was stopped; line following is switched off, so a new start is needed. */
static bool buddy3_end_of_line_stop (uint32_t now_ms)
{
    buddy2_motion_state_t motion    = {0};
    float32_t             travelled = 0.0f;
    bool                  b_encoder_fault;
    bool                  b_end     = false;

    buddy2_get_motion_state(&motion);
    travelled       = (motion.left_distance_mm + motion.right_distance_mm) / 2.0f;
    b_encoder_fault = motion.left_encoder_fault || motion.right_encoder_fault;

    /* New reference on a touch, at the start, after a reset or reverse, and
     * while a junction crossing or corner recovery (both bounded by their
     * own limits) is in charge. */
    if (!g_b_touch_valid || (BUDDY3_SIDE_NONE != g_last_touch_side) ||
        (travelled < g_touch_mm) || buddy3_junction_active(&g_junction) ||
        g_test_steering.recovering)
    {
        g_touch_mm      = travelled;
        g_touch_ms      = now_ms;
        g_b_touch_valid = true;
    }
    else if (b_encoder_fault)
    {
        b_end = ((now_ms - g_touch_ms) > BUDDY3_NO_LINE_STOP_MS);
    }
    else
    {
        b_end = ((travelled - g_touch_mm) > BUDDY3_NO_LINE_STOP_MM);
    }

    if (b_end)
    {
        g_trace_phase = TRACE_STOP;
        buddy3_stop_observed(true);
        tm_printf((UB *) "[line] no line for %u mm - end of line, stopped\n",
                  (UINT) BUDDY3_NO_LINE_STOP_MM);
    }

    return b_end;
}

/* ------------------------------------------------------------------ *
 *  Line history - sensor transitions and recovery direction
 * ------------------------------------------------------------------ */

/* Track which front sensors see the line and log changes. */
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

    if (side != g_last_touch_side)
    {
        buddy3_log_side(side, now_ms);
    }
    g_last_touch_side = side;
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

/* Lifted / dark surface: every sensor reads "black".  Returns true when the
 * car was stopped for it. */
static bool buddy3_dark_surface_stop (const ir_processing_state_t * p_next,
                                      uint32_t                      now_ms)
{
    bool b_stopped = false;

    if (p_next->black[0] && p_next->black[1] && p_next->black[2])
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
                tm_printf((UB *) "[line] all sensors black - lifted or dark "
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
 *  Core control - steering, junctions and barcode turn handoff
 * ------------------------------------------------------------------ */

/* Steering with corner holding, corner recovery and junction handling. */
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

    if (junction.handled && !junction.stopped &&
        (B3J_CROSS == junction.phase) && (BUDDY3_TURN_NONE != g_junction_turn))
    {
        /* A barcode asked for a turn here: stop on the junction and hand
         * the turn to the controller instead of crossing straight. */
        g_junction_turn_reached = g_junction_turn;
        g_junction_turn         = BUDDY3_TURN_NONE;
        g_trace_phase           = TRACE_STOP;
        tm_printf((UB *) "[junction] barcode turn %s\n",
                  (BUDDY3_TURN_LEFT == g_junction_turn_reached)
                      ? "left"
                      : ((BUDDY3_TURN_RIGHT == g_junction_turn_reached)
                             ? "right"
                             : "U-turn"));
        buddy3_stop_observed(true);
    }
    else if (junction.handled)
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
            &g_test_steering, g_last_touch_side, now_ms,
            g_steering_configs.base_pwm,
            buddy3_clamp_pwm(g_steering_configs.proportional_gain *
                             BUDDY3_GAIN_SCALE));
        buddy3_apply_straight_heading_hold(&output, g_last_touch_side);
        if (output.phase != g_test_last_phase)
        {
            tm_printf((UB *) "[steer] %s L=%d R=%d\n", phases[output.phase],
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

/*** end of file ***/

