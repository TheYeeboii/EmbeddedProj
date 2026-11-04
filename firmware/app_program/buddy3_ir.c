/**
 * @file    buddy3_ir.c
 * @brief   Buddy 3: IR line following and barcode decoding.
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
#include "buddy3_steering.h"
#include "robot_steering_trace.h"
#include "robot_barcode_trace.h"
#include "buddy3_barcode.h"
#include "buddy3_junction.h"

#define BUDDY3_LINE_THRESHOLD       (350u)
#define BUDDY3_JUNCTION_THRESHOLD   (800u)
#define BUDDY3_NORMALIZED_MAX       (1000)
#define BUDDY3_LINE_LOST_STOP_MS    (250u)
#define BUDDY3_SENSOR_COUNT         (3u)

/* ---- Sensor layout ---------------------------------------------------
 * The two FRONT sensors (ADC0, ADC1) sit on the black bracket; the third
 * (ADC2) is on the white arm beside the car and reads the barcodes.
 * Steering compares the two front sensors (position = right - left).  The
 * same rule works whether the line is wider than the sensor gap (ours:
 * both on black = centred) or narrower (both on white = centred).
 *   BUDDY3_LINE_BETWEEN_SENSORS 1 : this "compare the two" algorithm
 *   BUDDY3_LINE_BETWEEN_SENSORS 0 : the old CMake algorithm
 * If the car steers AWAY from the line instead of back onto it, swap the
 * two numbers below (sensor wires or motor wires are the other way round).
 */
#define BUDDY3_LINE_BETWEEN_SENSORS (1)
/* Confirmed 30 Sep with a hand test: ADC0 = left sensor, ADC1 = center. */
#define BUDDY3_FRONT_RIGHT           (0u)   /* ADC index of the right-front sensor (from CAR's POV) */
#define BUDDY3_FRONT_LEFT          (1u)   /* ADC index of the left-front sensor (from CAR's POV) */
/* All three sensors on black for this long = the car was lifted or put on
 * a dark surface (the side sensor is normally on white): stop. */
#define BUDDY3_ALL_BLACK_STOP_MS    (3000u)   /* > time to drive off the black start box */
/* Our line is 18 mm, narrower than the gap between the front sensors, so
 * "both on floor" is NORMAL (line between them) - it only means "lost" if
 * it lasts long (see the line search below). */
#define BUDDY3_EDGE_THRESHOLD       (500u) /* normalized: sensor is over black */
/* Steering tested on the car (30 Sep): black under the CENTER sensor ("R")
 * -> left wheel faster (turn right); under the LEFT sensor -> right wheel
 * faster.  Both correct, so no swap is needed.
 * If one sensor stays on black this long, the turn is too weak (curve):
 * run the inner wheel slightly backwards for a tighter turn.
 * Disabled (7 Oct): the reversing inner wheel made the car spin in circles. */
#define BUDDY3_HARD_TURN_ENABLE     (0)
#define BUDDY3_HARD_TURN_MS         (600u)
#define BUDDY3_HARD_TURN_INNER_PWM  (-5000)
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
/* With the search off: both front sensors white for this long after the
 * last touch = no line, so STOP (do not drive away).  Line following stays
 * on; the car moves again as soon as a front sensor sees black. */
#define BUDDY3_LOST_STOP_MS         (1000u)
/* Lowest power for the inner wheel on a correction.  0 = it may stop but
 * never runs backwards, so a correction curves instead of pivoting. */
#define BUDDY3_INNER_MIN_PWM        (0)
/* Keep an open-loop straight segment from gradually curving away while both
 * front sensors see the floor.  The same wheel-distance correction is used
 * while crossing a junction, but normal tracking needs it too: with a
 * straddling two-sensor layout, both-white cannot tell a centred line from a
 * completely missed line. */
#define BUDDY3_STRAIGHT_HEADING_PWM_PER_MM  (120.0f)

/* API PWM before Buddy 2's global speed limit and trim. Base is 35%;
 * main's 27 percentage-point curve power commands about 8%/62% on a
 * one-sided detection. Junction crossing uses the base and encoder hold. */
/* Sensor order: front left, front right, side barcode. */
static ir_calibration_params_t g_sensor_configs[BUDDY3_SENSOR_COUNT] =
{
    { 200u, 3000u, false }, // right sensor
    { 200u, 1300u, false }, // left sensor
    { 2000u, 2500u, false } // side sensor
};

static line_follow_configs_t g_steering_configs =
{
    (27.0f * (float)BUDDY2_PWM_MAX) / 100000.0f, // default: 27 percentage-point curve power
    2.5f, // derivative gain: 2.5f = 2500 / 1000
    11469 // base PWM: 11469 = 35 % duty
};

/* Internal snapshot groups each sensor's raw and normalized readings together. */
typedef struct
{
    ir_sensor_reading_t sensors[BUDDY3_SENSOR_COUNT];
    int16_t line_position;
    bool line_seen;
    bool junction_seen;
    bool barcode_ready;
    uint8_t barcode_command;
} ir_processing_state_t;

static ir_processing_state_t g_state;
static volatile bool     g_line_following     = false;
/* Start uses bounded corner recovery. Stage 1 remains available for the
 * isolated, gentle-tracking test through the serial line_stage command. */
static uint8_t g_test_stage = 3u;
static buddy3_steering_t g_test_steering;
static buddy3_junction_t g_junction;
static uint8_t g_junction_last_phase = B3J_IDLE;
static uint8_t g_test_last_phase = 255u;
/* Records brief front-line observations while normal following is paused for
 * Buddy 5 avoidance.  The final return phase consumes this event. */
static volatile bool     g_line_seen_event    = false;
static uint32_t          g_last_update_ms     = 0u;
static int16_t           g_previous_position  = 0;
static uint32_t          g_line_lost_since_ms = 0u;
static buddy3_barcode_decoder_t g_barcode_decoder;
static buddy3_barcode_config_t g_barcode_config = { BUDDY3_BARCODE_MIN_ELEMENT_MM };
static float g_barcode_offset = BUDDY3_BARCODE_OFFSET_LEFT_MM;
static bool g_barcode_calibrated = false;
static bool g_barcode_odometry_valid = false;
static float g_barcode_last_left, g_barcode_last_right, g_barcode_yaw;
static buddy3_barcode_event_t g_pending_barcode;
static uint32_t g_barcode_dropped;
static bool g_barcode_black;
static bool              g_barcode_ready      = false;
static bool              g_reported_lost      = false;
static bool              g_seen_since_start   = false;
static bool              g_start_pending      = false;
static uint32_t          g_last_touch_ms      = 0u;
static uint8_t           g_last_touch_side    = 0u;   /* 0 none, 1 L, 2 R, 3 both */
static uint32_t          g_all_black_since_ms = 0u;
static bool              g_reported_black     = false;
static uint32_t          g_side_since_ms      = 0u;   /* when the current L/R pattern began */
static uint8_t           g_last_single_side   = 0u;   /* 1 L, 2 R: last side seen alone */
static bool              g_straight_hold_active = false;
static float             g_straight_hold_left_mm;
static float             g_straight_hold_right_mm;

/* Line search sweep (see BUDDY3_SEARCH_*). */
#define BUDDY3_SEARCH_IDLE          (0u)
#define BUDDY3_SEARCH_FIRST         (1u)   /* towards the last-seen side      */
#define BUDDY3_SEARCH_SECOND        (2u)   /* back and past, the other way    */
#define BUDDY3_SEARCH_RETURN        (3u)   /* back to the start heading       */
#define BUDDY3_SEARCH_SPIN          (4u)   /* full spin on the spot           */
#define BUDDY3_SEARCH_GAVE_UP       (5u)   /* nothing in a full turn: stopped */
static uint8_t           g_search_step        = BUDDY3_SEARCH_IDLE;
static int8_t            g_search_dir         = 1;    /* +1 left, -1 right (current step) */
static uint32_t          g_search_step_ms     = 0u;   /* when the current step began      */
static float             g_search_step_deg    = 0.0f; /* encoder angle when it began      */
static float             g_search_origin_deg  = 0.0f; /* travel heading when search began */
static bool              g_reported_behind    = false;

static int16_t buddy3_clamp_pwm(float value)
{
    if (value > (float)BUDDY2_PWM_MAX)
    {
        return (int16_t)BUDDY2_PWM_MAX;
    }
    if (value < -(float)BUDDY2_PWM_MAX)
    {
        return (int16_t)(-BUDDY2_PWM_MAX);
    }
    return (int16_t)value;
}

static uint16_t buddy3_normalize(uint16_t raw, uint16_t minimum,
                                 uint16_t maximum, bool line_is_dark)
{
    int32_t value;

    if (maximum <= minimum)
    {
        return 0u;
    }
    value = (((int32_t)raw - (int32_t)minimum) * BUDDY3_NORMALIZED_MAX) /
            ((int32_t)maximum - (int32_t)minimum);
    if (value < 0)
    {
        value = 0;
    }
    if (value > BUDDY3_NORMALIZED_MAX)
    {
        value = BUDDY3_NORMALIZED_MAX;
    }
    if (line_is_dark)
    {
        value = BUDDY3_NORMALIZED_MAX - value;
    }
    return (uint16_t)value;
}

/* Car heading from the wheel encoders, in degrees (only differences are
 * used, so the sign convention and any left/right encoder mix-up do not
 * matter: the sweep compares the SIZE of the turn). */
static float buddy3_encoder_angle_deg(void)
{
    buddy2_motion_state_t motion;
    buddy2_calibration_t  calibration;

    buddy2_get_motion_state(&motion);
    buddy2_get_calibration(&calibration);
    if (calibration.track_width_mm <= 0.0f)
    {
        return 0.0f;
    }
    return ((motion.right_distance_mm - motion.left_distance_mm) /
            calibration.track_width_mm) * (180.0f / 3.14159265f);
}

static void buddy3_search_begin_step(uint8_t step, int8_t direction, uint32_t now_ms)
{
    g_search_step     = step;
    g_search_dir      = direction;
    g_search_step_ms  = now_ms;
    g_search_step_deg = buddy3_encoder_angle_deg();
}

/* When the line is centred between the two front sensors, both report white.
 * Preserve the current heading using the encoders so unequal motors do not
 * silently carry the car off the line before either sensor can see an edge.
 * A one-sided line reading always takes priority and resets this hold. */
static void buddy3_apply_straight_heading_hold(buddy3_steering_output_t *p_output,
                                                uint8_t side)
{
    buddy2_motion_state_t motion = {0};
    float left;
    float right;
    float error;
    float correction;
    int16_t base;

    if ((NULL == p_output) || (B3_TRACK != p_output->phase) || (0u != side))
    {
        g_straight_hold_active = false;
        return;
    }

    buddy2_get_motion_state(&motion);
    if (motion.left_encoder_fault || motion.right_encoder_fault)
    {
        g_straight_hold_active = false;
        return;
    }
    if (!g_straight_hold_active)
    {
        g_straight_hold_left_mm  = motion.left_distance_mm;
        g_straight_hold_right_mm = motion.right_distance_mm;
        g_straight_hold_active   = true;
        return;
    }

    left  = motion.left_distance_mm  - g_straight_hold_left_mm;
    right = motion.right_distance_mm - g_straight_hold_right_mm;
    if (!(left >= 0.0f) || !(right >= 0.0f))
    {
        /* A reset or reverse must establish a new forward reference. */
        g_straight_hold_left_mm  = motion.left_distance_mm;
        g_straight_hold_right_mm = motion.right_distance_mm;
        return;
    }

    error = right - left;
    correction = (error < 0.0f ? -error : error) * BUDDY3_STRAIGHT_HEADING_PWM_PER_MM;
    base = (p_output->left < p_output->right) ? p_output->left : p_output->right;
    if (correction > (float)base / 3.0f)
    {
        correction = (float)base / 3.0f;
    }
    if (error > 0.0f)
    {
        p_output->right -= (int16_t)correction;
    }
    else if (error < 0.0f)
    {
        p_output->left -= (int16_t)correction;
    }
}

/* Sampling remains 10 ms; bit boundaries are measured in encoder distance. */
static void buddy3_decode_barcode(uint16_t sensor_value, uint16_t raw, uint32_t now_ms)
{
    buddy2_motion_state_t motion = {0};
    buddy2_calibration_t geometry;
    float dl, dr, travel, yaw;
    buddy3_barcode_event_t event;
    robot_barcode_observation_t observation = {0};
    bool was_active = g_barcode_decoder.active;
    float unfinished = g_barcode_decoder.position - g_barcode_decoder.edge_position;
    bool emitted;
    UINT interrupt_state;
    bool permitted = g_line_following && !buddy3_junction_active(&g_junction) &&
                     g_barcode_calibrated &&
                     (g_test_last_phase == B3_TRACK || g_test_last_phase == B3_MICRO);
    buddy2_get_motion_state(&motion);
    buddy2_get_calibration(&geometry);
    if (motion.left_encoder_fault || motion.right_encoder_fault) permitted = false;
    if (!g_barcode_odometry_valid) {
        g_barcode_last_left = motion.left_distance_mm;
        g_barcode_last_right = motion.right_distance_mm;
        g_barcode_odometry_valid = true;
    }
    dl = motion.left_distance_mm - g_barcode_last_left;
    dr = motion.right_distance_mm - g_barcode_last_right;
    g_barcode_last_left = motion.left_distance_mm;
    g_barcode_last_right = motion.right_distance_mm;
    yaw = geometry.track_width_mm > 0.0f ? (dr - dl) / geometry.track_width_mm : 0.0f;
    if (!g_barcode_decoder.active) g_barcode_yaw = 0.0f;
    g_barcode_yaw += yaw;
    /* Large turns, reverse and recovery invalidate a longitudinal barcode. */
    if (dl < 0.0f || dr < 0.0f ||
        g_barcode_yaw > 0.1745f || g_barcode_yaw < -0.1745f) permitted = false;
    travel = buddy3_barcode_travel(dl, dr, geometry.track_width_mm, g_barcode_offset);
    /* Hysteresis avoids adding transitions near a noisy threshold. */
    if (sensor_value >= 600u) g_barcode_black = true;
    else if (sensor_value <= 400u) g_barcode_black = false;
    observation.ts_ms = now_ms;
    observation.raw = raw; observation.normalized = sensor_value;
    observation.left_x10 = robot_barcode_scaled(motion.left_distance_mm,10);
    observation.right_x10 = robot_barcode_scaled(motion.right_distance_mm,10);
    observation.black = g_barcode_black;
    if (!g_line_following || !g_barcode_calibrated) observation.blocked |= 1;
    if (motion.left_encoder_fault || motion.right_encoder_fault) observation.blocked |= 2;
    if (g_barcode_yaw > 0.1745f || g_barcode_yaw < -0.1745f) observation.blocked |= 4;
    if (dl < 0 || dr < 0) observation.blocked |= 8;
    if (buddy3_junction_active(&g_junction) ||
        (g_test_last_phase != B3_TRACK && g_test_last_phase != B3_MICRO)) observation.blocked |= 16;
    if (travel > 0) unfinished += travel;
    emitted = buddy3_barcode_step(&g_barcode_decoder, &g_barcode_config,
              g_barcode_black, travel, now_ms, permitted, &event);
    robot_barcode_trace_observe(&g_barcode_decoder,&observation,was_active,unfinished,
                               emitted ? &event : NULL);
    if (emitted) {
        DI(interrupt_state);
        /* Keep the first unread event; do not overwrite it with a later frame. */
        if (!g_barcode_ready) {
            g_pending_barcode = event;
            g_barcode_ready = true;
        } else {
            ++g_barcode_dropped;
        }
        EI(interrupt_state);
    }
}

void buddy3_init(void)
{
    ir_processing_state_t empty = { 0 };

    hal_adc_init();
    hal_adc_pin_init(IR_SENSOR_LEFT_PIN);
    hal_adc_pin_init(IR_SENSOR_CENTER_PIN);
    hal_adc_pin_init(IR_SENSOR_RIGHT_PIN);

    g_state            = empty;
    g_last_update_ms   = 0u;
    g_line_following   = false;
    g_line_seen_event  = false;
    (void)buddy3_configure_barcode((uint16_t)BUDDY3_BARCODE_MIN_ELEMENT_MM,
                                  (int16_t)BUDDY3_BARCODE_OFFSET_LEFT_MM);
}

void buddy3_set_calibration(const buddy3_calibration_t *p_calibration)
{
    UINT interrupt_state;
    uint32_t index;

    if (NULL == p_calibration)
    {
        return;
    }
    DI(interrupt_state);
    /* Keep both groups in one protected update for legacy callers. */
    for (index = 0u; index < BUDDY3_SENSOR_COUNT; index++)
    {
        g_sensor_configs[index].minimum = p_calibration->minimum[index];
        g_sensor_configs[index].maximum = p_calibration->maximum[index];
        /* The compatibility API applies one inversion setting to every sensor. */
        g_sensor_configs[index].line_is_dark = p_calibration->line_is_dark;
    }
    g_steering_configs.proportional_gain = p_calibration->proportional_gain;
    g_steering_configs.derivative_gain = p_calibration->derivative_gain;
    g_steering_configs.base_pwm = p_calibration->base_pwm;
    EI(interrupt_state);
}

void buddy3_get_calibration(buddy3_calibration_t *p_calibration)
{
    UINT interrupt_state;
    uint32_t index;

    if (NULL != p_calibration)
    {
        DI(interrupt_state);
        for (index = 0u; index < BUDDY3_SENSOR_COUNT; index++)
        {
            p_calibration->minimum[index] = g_sensor_configs[index].minimum;
            p_calibration->maximum[index] = g_sensor_configs[index].maximum;
        }
        /* All sensors share inversion while configured through the compatibility API. */
        p_calibration->line_is_dark = g_sensor_configs[0].line_is_dark;
        p_calibration->proportional_gain = g_steering_configs.proportional_gain;
        p_calibration->derivative_gain = g_steering_configs.derivative_gain;
        p_calibration->base_pwm = g_steering_configs.base_pwm;
        EI(interrupt_state);
    }
}

static uint8_t g_trace_phase = TRACE_LEGACY;
static void buddy3_stop_with_input(const robot_steering_input_t *input);
static robot_steering_input_t buddy3_trace_input(void)
{
    robot_steering_input_t input = {0};
    UINT state;
    unsigned i;
    DI(state);
    input.ts_ms = g_last_update_ms;
    input.phase = g_trace_phase;
    input.threshold = BUDDY3_EDGE_THRESHOLD;
    for (i = 0; i < 2; ++i) {
        unsigned sensor = i == 0 ? BUDDY3_FRONT_LEFT : BUDDY3_FRONT_RIGHT;
        input.raw[i] = g_state.sensors[sensor].raw;
        input.norm[i] = g_state.sensors[sensor].normalized;
        input.calibration[i * 2] = g_sensor_configs[sensor].minimum;
        input.calibration[i * 2 + 1] = g_sensor_configs[sensor].maximum;
        if (g_sensor_configs[sensor].line_is_dark) input.dark |= (uint8_t)(1u << i);
    }
    EI(state);
    return input;
}
static void buddy3_drive_observed(int16_t left, int16_t right)
{
    robot_steering_input_t input = buddy3_trace_input();
    buddy2_set_motor_pwm_observed(left, right, &input);
}
static void buddy3_stop_observed(bool disable)
{
    robot_steering_input_t input = buddy3_trace_input();
    if (disable) buddy3_stop_with_input(&input);
    else buddy2_stop_observed(&input);
}

void buddy3_update(uint32_t now_ms)
{
    ir_processing_state_t next;
    uint16_t          raw[BUDDY3_SENSOR_COUNT];
    uint32_t          index;
#if !BUDDY3_LINE_BETWEEN_SENSORS
    uint32_t          weighted_total;
#endif
    int16_t           previous_position;
    UINT              interrupt_state;

    if ((now_ms - g_last_update_ms) < BUDDY3_SAMPLE_PERIOD_MS)
    {
        return;
    }
    g_last_update_ms = now_ms;
    g_trace_phase = TRACE_LEGACY;

    /* Read the sensors first (slow part, no lock held). */
    raw[0] = hal_adc_read(IR_ADC_CHANNEL_LEFT);
    raw[1] = hal_adc_read(IR_ADC_CHANNEL_CENTER);
    raw[2] = hal_adc_read(IR_ADC_CHANNEL_RIGHT);

    next = g_state;
    for (index = 0u; index < BUDDY3_SENSOR_COUNT; index++)
    {
        next.sensors[index].raw        = raw[index];
        next.sensors[index].normalized = buddy3_normalize(raw[index],
                                                  g_sensor_configs[index].minimum,
                                                  g_sensor_configs[index].maximum,
                                                  g_sensor_configs[index].line_is_dark);
    }

    previous_position = g_previous_position;

#if BUDDY3_LINE_BETWEEN_SENSORS
    /* Front sensors STRADDLE the line (see the photo in docs/): normally both
     * see the floor.  When one of them touches the line, steer towards that
     * side so the line goes back between them.  position = right - left:
     * > 0 -> line under the right-front sensor -> turn right. */
    {
        int32_t left  = (int32_t)next.sensors[BUDDY3_FRONT_LEFT].normalized;
        int32_t right = (int32_t)next.sensors[BUDDY3_FRONT_RIGHT].normalized;

        next.line_seen     = (left >= (int32_t)BUDDY3_EDGE_THRESHOLD) ||
                             (right >= (int32_t)BUDDY3_EDGE_THRESHOLD);
        next.junction_seen = (left >= (int32_t)BUDDY3_JUNCTION_THRESHOLD) &&
                             (right >= (int32_t)BUDDY3_JUNCTION_THRESHOLD);
        /* Both black = a crossing line (or the car was lifted): go straight. */
        next.line_position = next.junction_seen ? 0 : (int16_t)(right - left);
        g_previous_position = next.line_position;
    }
#else
    /* Original algorithm: the line runs UNDER the two front sensors. */
    weighted_total     = (uint32_t)next.sensors[0].normalized + (uint32_t)next.sensors[1].normalized;
    next.line_seen     = (weighted_total >= BUDDY3_LINE_THRESHOLD);
    next.junction_seen = (next.sensors[0].normalized >= BUDDY3_JUNCTION_THRESHOLD) &&
                         (next.sensors[1].normalized >= BUDDY3_JUNCTION_THRESHOLD);
    if (next.line_seen)
    {
        int32_t weighted_position = ((int32_t)next.sensors[1].normalized -
                                     (int32_t)next.sensors[0].normalized) * 1000;

        next.line_position   = (int16_t)(weighted_position / (int32_t)weighted_total);
        g_previous_position  = next.line_position;
        g_line_lost_since_ms = 0u;
    }
    else
    {
        next.line_position = 0;
        if (0u == g_line_lost_since_ms)
        {
            g_line_lost_since_ms = now_ms;
        }
    }
#endif

    /* The outrigger sensor (ADC2) reads the barcodes beside the track. */
    buddy3_decode_barcode(next.sensors[2].normalized, next.sensors[2].raw, now_ms);
    next.barcode_ready   = g_barcode_ready;
    next.barcode_command = g_barcode_ready && g_pending_barcode.kind == B3BC_VALID ?
                           (uint8_t)g_pending_barcode.decoded.symbol : 0u;

    DI(interrupt_state);
    g_state = next;
    if (next.line_seen)
    {
        g_line_seen_event = true;
    }
    EI(interrupt_state);

#if BUDDY3_LINE_BETWEEN_SENSORS
    {
        /* Which front sensor(s) are on black right now: 1 left, 2 right. */
        uint8_t side = (uint8_t)
            (((next.sensors[BUDDY3_FRONT_LEFT].normalized  >= BUDDY3_EDGE_THRESHOLD) ? 1u : 0u) |
             ((next.sensors[BUDDY3_FRONT_RIGHT].normalized >= BUDDY3_EDGE_THRESHOLD) ? 2u : 0u));

#if BUDDY3_SEARCH_ENABLE
        /* Forward only: during a search, black seen while the car points
         * back the way it came is the old line - ignore it and keep turning. */
        if ((g_test_stage == 0u) && (0u != side) && g_line_following &&
            (BUDDY3_SEARCH_IDLE != g_search_step) && (BUDDY3_SEARCH_GAVE_UP != g_search_step))
        {
            float offset = buddy3_encoder_angle_deg() - g_search_origin_deg;

            while (offset > 180.0f)
            {
                offset -= 360.0f;
            }
            while (offset < -180.0f)
            {
                offset += 360.0f;
            }
            if ((offset > BUDDY3_SEARCH_BEHIND_DEG) || (offset < -BUDDY3_SEARCH_BEHIND_DEG))
            {
                if (!g_reported_behind)
                {
                    tm_printf((UB *)"[line] line at %d deg is behind us - ignored\n", (INT)offset);
                    g_reported_behind = true;
                }
                side = 0u;
            }
        }
#endif

        /* Sensor check while idle: say which side sees the line, so the
         * wiring can be verified by hand (see docs). */
        if (side != g_last_touch_side)
        {
            static const char *const color[] = { "white", "BLACK" };
            static const char *const steer[] = { "straight", "steer LEFT", "steer RIGHT", "both black (junction check)" };
            const char *p_left  = color[side & 1u];
            const char *p_right = color[(side >> 1) & 1u];
            const char *decision = buddy3_junction_active(&g_junction) ?
                                   "junction straight (branch ignored)" : steer[side];

            if (g_line_following)
            {
                /* Run log: every change, with what the car is told to do.
                 * Compare with what it really did to check the wiring. */
                tm_printf((UB *)"[line] %u.%02u s   front L=%s R=%s   -> %s\n",
                          (UINT)(now_ms / 1000u), (UINT)((now_ms % 1000u) / 10u),
                          p_left, p_right, decision);
            }
            else
            {
                tm_printf((UB *)"[sensor] front L=%s R=%s\n", p_left, p_right);
            }
        }
        if (side != g_last_touch_side)
        {
            g_side_since_ms = now_ms;
        }
        g_last_touch_side = side;
        if ((1u == side) || (2u == side))
        {
            g_last_single_side = side;
        }

        if (g_start_pending)
        {
            g_last_touch_ms = now_ms;   /* start the "no line" timer now */
            g_start_pending = false;
        }
        if (0u != side)
        {
            g_last_touch_ms = now_ms;
            if ((g_reported_lost || (BUDDY3_SEARCH_IDLE != g_search_step)) && g_line_following)
            {
                tm_printf((UB *)"[line] line found - driving\n");
            }
            g_reported_lost = false;
            g_search_step   = BUDDY3_SEARCH_IDLE;
        }
    }
#endif

    if (!g_line_following)
    {
        return;
    }

#if BUDDY3_LINE_BETWEEN_SENSORS
    /* Lifted / dark surface: every sensor reads "black". */
    if ((next.sensors[0].normalized >= BUDDY3_EDGE_THRESHOLD) &&
        (next.sensors[1].normalized >= BUDDY3_EDGE_THRESHOLD) &&
        (next.sensors[2].normalized >= BUDDY3_EDGE_THRESHOLD))
    {
        if (0u == g_all_black_since_ms)
        {
            g_all_black_since_ms = now_ms;
        }
        if ((now_ms - g_all_black_since_ms) > BUDDY3_ALL_BLACK_STOP_MS)
        {
            g_trace_phase = TRACE_DARK;
            buddy3_stop_observed(false);
            if (!g_reported_black)
            {
                tm_printf((UB *)"[line] all sensors black - lifted or dark surface? stopped\n");
                g_reported_black = true;
            }
            return;
        }
    }
    else
    {
        g_all_black_since_ms = 0u;
        g_reported_black     = false;
    }

    if (g_test_stage != 0u)
    {
        buddy2_motion_state_t motion = {0};
        buddy3_junction_output_t junction;
        bool was_crossing = buddy3_junction_active(&g_junction);
        /* A long curve may legitimately lead straight into a junction. The
         * junction's own both-black debounce then owns that crossing; only a
         * genuine line-loss recovery or an already stopped run blocks entry. */
        bool entry_allowed = !g_test_steering.recovering && !g_test_steering.stopped;
        buddy3_steering_output_t output;
        static const char *const phases[] = {
            "straight", "micro", "corner", "seek", "align", "both-black", "stopped"
        };
        buddy2_get_motion_state(&motion);
        junction = buddy3_junction_step(&g_junction, g_last_touch_side, now_ms,
                        motion.left_distance_mm, motion.right_distance_mm,
                        !motion.left_encoder_fault && !motion.right_encoder_fault,
                        entry_allowed, g_steering_configs.base_pwm);
        if (junction.phase != g_junction_last_phase) {
            static const char *const names[] = {"idle", "confirm", "straight", "cleared", "stopped"};
            tm_printf((UB *)"[junction] %s L=%d R=%d\n",
                      names[junction.phase], (INT)junction.left, (INT)junction.right);
            g_junction_last_phase = junction.phase;
        }
        if (junction.handled) {
            g_trace_phase = junction.stopped ? TRACE_STOP : TRACE_CROSS;
            /* Discard the old correction: do not turn toward a side branch. */
            buddy3_steering_reset(&g_test_steering);
            g_test_last_phase = 255u;
            /* Junction crossing owns its own encoder heading reference. */
            g_straight_hold_active = false;
            if (junction.stopped) buddy3_stop_observed(true);
            else buddy3_drive_observed(junction.left, junction.right);
            return;
        }
        if (was_crossing) {
            buddy3_steering_reset(&g_test_steering);
            g_test_last_phase = 255u;
        }
        output = buddy3_steering_step(&g_test_steering, g_test_stage,
                    g_last_touch_side, now_ms, g_steering_configs.base_pwm,
                    buddy3_clamp_pwm(g_steering_configs.proportional_gain * 1000.0f));
        buddy3_apply_straight_heading_hold(&output, g_last_touch_side);
        if (output.phase != g_test_last_phase)
        {
            tm_printf((UB *)"[steer] stage=%u %s L=%d R=%d\n",
                      (UINT)g_test_stage, phases[output.phase],
                      (INT)output.left, (INT)output.right);
            g_test_last_phase = output.phase;
        }
        g_trace_phase = output.phase;
        if (output.phase == B3_STOP)
        {
            buddy3_stop_observed(true);
        }
        else
        {
            buddy3_drive_observed(output.left, output.right);
        }
        return;
    }

#if BUDDY3_SEARCH_ENABLE
    /* Sweep and full spin found nothing.  Stay stopped (line
     * following stays on) until a front sensor sees black again. */
    if (BUDDY3_SEARCH_GAVE_UP == g_search_step)
    {
        buddy3_stop_observed(false);
        if (!g_reported_lost)
        {
            tm_printf((UB *)"[line] full spin found no line ahead (dead end?) - stopped\n");
            g_reported_lost = true;
        }
        return;
    }
#else
    if ((now_ms - g_last_touch_ms) > BUDDY3_LOST_STOP_MS)
    {
        buddy3_stop_observed(false);
        if (!g_reported_lost)
        {
            tm_printf((UB *)"[line] no line seen for %u ms - stopped\n",
                      (UINT)BUDDY3_LOST_STOP_MS);
            g_reported_lost = true;
        }
        return;
    }
#endif
#else
    /* Line must be seen at least once before driving, then may be lost for
     * up to BUDDY3_LINE_LOST_STOP_MS. */
    if (!next.line_seen && (!g_seen_since_start ||
        ((now_ms - g_line_lost_since_ms) > BUDDY3_LINE_LOST_STOP_MS)))
    {
        buddy3_stop_observed(false);
        if (!g_reported_lost)
        {
            tm_printf((UB *)"[line] no line under the sensors (L=%u C=%u raw) - waiting\n",
                      (UINT)next.sensors[0].raw, (UINT)next.sensors[1].raw);
            g_reported_lost = true;
        }
        return;
    }
    g_seen_since_start = true;
    if (g_reported_lost)
    {
        tm_printf((UB *)"[line] line found - driving\n");
        g_reported_lost = false;
    }
#endif

#if BUDDY3_LINE_BETWEEN_SENSORS
    /* Digital sensors -> simple three-way steering (the derivative term is
     * not used here: the position only jumps between -1000, 0 and +1000).
     *   one sensor black  : turn towards it (harder if it stays black)
     *   both white, recent: straight (line is between the sensors)
     *   both white, long  : search sweep, then full spin (see above)
     *   both black        : straight (crossing / start box) */
    (void)previous_position;
    {
        int32_t  base       = (int32_t)g_steering_configs.base_pwm;
        int32_t  turn       = (int32_t)(g_steering_configs.proportional_gain * 1000.0f);
        int32_t  left_pwm   = base;
        int32_t  right_pwm  = base;
        uint8_t  side       = g_last_touch_side;
        uint32_t held_ms    = now_ms - g_side_since_ms;

        if ((1u == side) || (2u == side))
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
            (void)held_ms;
#endif

            if (2u == side)            /* line under the center sensor: turn right */
            {
                left_pwm  = outer;
                right_pwm = inner;
            }
            else                       /* line under the left sensor: turn left */
            {
                left_pwm  = inner;
                right_pwm = outer;
            }
        }
        else if (BUDDY3_SEARCH_ENABLE && (0u == side) &&
                 ((BUDDY3_SEARCH_IDLE != g_search_step) ||
                  ((now_ms - g_last_touch_ms) > BUDDY3_SEARCH_AFTER_MS)))
        {
            uint32_t limit_ms = BUDDY3_SEARCH_SWEEP_STEP_MS;

            if (BUDDY3_SEARCH_IDLE == g_search_step)
            {
                /* Right first only if the right sensor saw the line last. */
                buddy3_search_begin_step(BUDDY3_SEARCH_FIRST,
                                         (2u == g_last_single_side) ? -1 : 1, now_ms);
                g_search_origin_deg = g_search_step_deg;
                g_reported_behind   = false;
                tm_printf((UB *)"[line] line lost - sweeping %s first (max %u deg)\n",
                          (g_search_dir > 0) ? "LEFT" : "RIGHT",
                          (UINT)BUDDY3_SEARCH_SWEEP_DEG);
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

            /* TEMPORARY TIME-ONLY SEARCH (easy to revert)
             *
             * The original encoder-angle completion below is disabled because
             * the current encoder tick calibration is not reliable.  Restore
             * this block and remove the time-only condition below after both
             * wheel encoders and track width have been calibrated.
             *
             * float    turned;
             * float    target_deg = BUDDY3_SEARCH_SWEEP_DEG;
             *
             * if (BUDDY3_SEARCH_SECOND == g_search_step)
             * {
             *     target_deg = 2.0f * BUDDY3_SEARCH_SWEEP_DEG;
             * }
             * else if (BUDDY3_SEARCH_SPIN == g_search_step)
             * {
             *     target_deg = BUDDY3_SEARCH_SPIN_MAX_DEG;
             * }
             *
             * turned = buddy3_encoder_angle_deg() - g_search_step_deg;
             * if (turned < 0.0f)
             * {
             *     turned = -turned;
             * }
             * if ((turned >= target_deg) ||
             *     ((now_ms - g_search_step_ms) >= limit_ms))
             */
            if ((now_ms - g_search_step_ms) >= limit_ms)
            {
                if (BUDDY3_SEARCH_FIRST == g_search_step)
                {
                    buddy3_search_begin_step(BUDDY3_SEARCH_SECOND, (int8_t)(-g_search_dir), now_ms);
                }
                else if (BUDDY3_SEARCH_SECOND == g_search_step)
                {
                    buddy3_search_begin_step(BUDDY3_SEARCH_RETURN, (int8_t)(-g_search_dir), now_ms);
                }
                else if (BUDDY3_SEARCH_RETURN == g_search_step)
                {
                    /* Keep turning the same way: one smooth full spin. */
                    buddy3_search_begin_step(BUDDY3_SEARCH_SPIN, g_search_dir, now_ms);
                    tm_printf((UB *)"[line] nothing within %u deg - spinning to find the line\n",
                              (UINT)BUDDY3_SEARCH_SWEEP_DEG);
                }
                else
                {
                    g_search_step = BUDDY3_SEARCH_GAVE_UP;
                }
            }

            if (BUDDY3_SEARCH_GAVE_UP == g_search_step)
            {
                left_pwm  = 0;
                right_pwm = 0;
            }
            else
            {
                /* Spin on the spot: +1 = left (right wheel forward). */
                left_pwm  = -(int32_t)g_search_dir * base;
                right_pwm =  (int32_t)g_search_dir * base;
            }
        }
        else
        {
            /* straight */
        }
        if (g_search_step != BUDDY3_SEARCH_IDLE) g_trace_phase = TRACE_SEEK;
        buddy3_drive_observed(buddy3_clamp_pwm((float)left_pwm),
                             buddy3_clamp_pwm((float)right_pwm));
    }
#else
    {
        float derivative = (float)next.line_position - (float)previous_position;
        float correction = (g_steering_configs.proportional_gain * (float)next.line_position) +
                           (g_steering_configs.derivative_gain * derivative);

        buddy3_drive_observed(buddy3_clamp_pwm((float)g_steering_configs.base_pwm + correction),
                             buddy3_clamp_pwm((float)g_steering_configs.base_pwm - correction));
    }
#endif
}

void buddy3_start_line_follow(void)
{
    robot_barcode_trace_cancel(g_last_update_ms);
    buddy3_junction_reset(&g_junction);
    g_junction_last_phase = B3J_IDLE;
    buddy3_barcode_reset(&g_barcode_decoder);
    g_barcode_odometry_valid = false;
    g_barcode_ready = false;
    g_state.barcode_ready = false;
    buddy3_steering_reset(&g_test_steering);
    g_test_last_phase = 255u;
    g_reported_lost      = false;
    g_seen_since_start   = false;
    g_start_pending      = true;
    g_line_lost_since_ms = 0u;
    g_last_single_side   = 0u;
    g_search_step        = BUDDY3_SEARCH_IDLE;
    g_side_since_ms      = g_last_update_ms;
    g_straight_hold_active = false;
    g_line_following     = true;
}

void buddy3_stop_line_follow(void)
{
    buddy3_stop_with_input(NULL);
}
static void buddy3_stop_with_input(const robot_steering_input_t *input)
{
    robot_barcode_trace_cancel(g_last_update_ms);
    g_line_following = false;
    buddy3_junction_reset(&g_junction);
    buddy3_barcode_reset(&g_barcode_decoder);
    g_barcode_ready = false;
    g_state.barcode_ready = false;
    g_straight_hold_active = false;
    buddy2_stop_observed(input);
}

bool buddy3_line_following(void)
{
    return g_line_following;
}

void buddy3_set_test_stage(uint8_t stage)
{
    if (!g_line_following && stage <= 3u)
    {
        g_test_stage = stage;
    }
}

uint8_t buddy3_test_stage(void)
{
    return g_test_stage;
}

void buddy3_get_state(buddy3_ir_state_t *p_state)
{
    UINT interrupt_state;
    uint32_t index;

    if (NULL != p_state)
    {
        DI(interrupt_state);
        /* Adapt the per-sensor storage to the existing public snapshot. */
        for (index = 0u; index < BUDDY3_SENSOR_COUNT; index++)
        {
            p_state->raw[index] = g_state.sensors[index].raw;
            p_state->normalized[index] = g_state.sensors[index].normalized;
        }
        p_state->line_position = g_state.line_position;
        p_state->line_seen = g_state.line_seen;
        p_state->junction_seen = g_state.junction_seen;
        p_state->barcode_ready = g_state.barcode_ready;
        p_state->barcode_command = g_state.barcode_command;
        EI(interrupt_state);
    }
}

void buddy3_print_debug(void)
{
#if BUDDY3_DEBUG
    buddy3_ir_state_t state;

    /* The getter releases its interrupt lock before any console output. */
    buddy3_get_state(&state);
    tm_printf((UB *)"[buddy3] raw L=%u R=%u side=%u | norm L=%u R=%u side=%u | ADC L=%u R=%u side=2\n",
              (UINT)state.raw[BUDDY3_FRONT_LEFT],
              (UINT)state.raw[BUDDY3_FRONT_RIGHT],
              (UINT)state.raw[2],
              (UINT)state.normalized[BUDDY3_FRONT_LEFT],
              (UINT)state.normalized[BUDDY3_FRONT_RIGHT],
              (UINT)state.normalized[2],
              (UINT)BUDDY3_FRONT_LEFT, (UINT)BUDDY3_FRONT_RIGHT);
#endif
}

void buddy3_clear_line_seen_event(void)
{
    UINT interrupt_state;

    DI(interrupt_state);
    g_line_seen_event = false;
    EI(interrupt_state);
}

bool buddy3_take_line_seen_event(void)
{
    UINT interrupt_state;
    bool result;

    DI(interrupt_state);
    result = g_line_seen_event;
    g_line_seen_event = false;
    EI(interrupt_state);
    return result;
}

void buddy3_get_colors(bool *p_left_black, bool *p_right_black, bool *p_side_black)
{
    buddy3_ir_state_t state;

    buddy3_get_state(&state);
    if ((NULL != p_left_black) && (NULL != p_right_black) && (NULL != p_side_black))
    {
        *p_left_black  = (state.normalized[BUDDY3_FRONT_LEFT]  >= BUDDY3_EDGE_THRESHOLD);
        *p_right_black = (state.normalized[BUDDY3_FRONT_RIGHT] >= BUDDY3_EDGE_THRESHOLD);
        *p_side_black  = (state.normalized[2] >= BUDDY3_EDGE_THRESHOLD);
    }
}

bool buddy3_take_barcode(buddy3_barcode_event_t *p_event)
{
    UINT interrupt_state;
    bool taken = false;

    if (NULL == p_event)
    {
        return false;
    }
    DI(interrupt_state);
    if (g_barcode_ready)
    {
        *p_event              = g_pending_barcode;
        p_event->dropped       = g_barcode_dropped;
        g_barcode_ready       = false;
        g_state.barcode_ready = false;
        taken                 = true;
    }
    EI(interrupt_state);

    return taken;
}

bool buddy3_configure_barcode(uint16_t min_element_mm, int16_t offset_left_mm)
{
    buddy2_calibration_t geometry;
    float left_tick, right_tick, minimum_cell;
    if (g_line_following) return false;
    robot_barcode_trace_cancel(g_last_update_ms);
    robot_barcode_trace_enable(false);
    g_barcode_calibrated = false;
    buddy3_barcode_reset(&g_barcode_decoder);
    g_barcode_ready = false;
    g_state.barcode_ready = false;
    g_barcode_odometry_valid = false;
    if (min_element_mm == 0u) {
        tm_printf((UB *)"[barcode] disabled: configure minimum resolvable element and sensor offset\n");
        return false;
    }
#if defined(ENCODER_LEFT_MIRROR_RIGHT) && ENCODER_LEFT_MIRROR_RIGHT
    tm_printf((UB *)"[barcode] disabled: left encoder mirrors right; repair/calibrate both encoders\n");
    return false;
#endif
    buddy2_get_calibration(&geometry);
    if (!(geometry.wheel_diameter_mm > 0.0f) || !(geometry.track_width_mm > 0.0f) ||
        !(geometry.left_ticks_per_wheel_rev > 0.0f) ||
        !(geometry.right_ticks_per_wheel_rev > 0.0f)) return false;
    left_tick = 3.14159265f * geometry.wheel_diameter_mm / geometry.left_ticks_per_wheel_rev;
    right_tick = 3.14159265f * geometry.wheel_diameter_mm / geometry.right_ticks_per_wheel_rev;
    minimum_cell = 4.0f * (left_tick > right_tick ? left_tick : right_tick);
    if ((float)min_element_mm < minimum_cell || min_element_mm > 1000u ||
        offset_left_mm < -500 || offset_left_mm > 500) {
        tm_printf((UB *)"[barcode] rejected: minimum element needs >=%u mm at current encoder resolution; offset -500..500\n",
                  (UINT)(minimum_cell + 0.999f));
        return false;
    }
    g_barcode_config.min_element_mm = (float)min_element_mm;
    g_barcode_offset = (float)offset_left_mm;
    g_barcode_calibrated = true;
    {
        int32_t trace_config[11] = {
            g_sensor_configs[2].minimum, g_sensor_configs[2].maximum,
            g_sensor_configs[2].line_is_dark, 400, 600,
            (int32_t)min_element_mm * 100, (int32_t)offset_left_mm * 10,
            robot_barcode_scaled(geometry.track_width_mm,10),
            robot_barcode_scaled(geometry.wheel_diameter_mm,10),
            robot_barcode_scaled(geometry.left_ticks_per_wheel_rev,10),
            robot_barcode_scaled(geometry.right_ticks_per_wheel_rev,10)
        };
        robot_barcode_trace_configure(trace_config);
    }
    tm_printf((UB *)"[barcode] configured minimum-element=%u mm offset-left=%d mm (adaptive Code39)\n",
              (UINT)min_element_mm, (INT)offset_left_mm);
    return true;
}

bool buddy3_barcode_calibrated(void)
{
    return g_barcode_calibrated;
}
