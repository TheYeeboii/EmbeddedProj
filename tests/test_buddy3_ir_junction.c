#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "../firmware/app_program/buddy3_ir.c"
#include "data/barcode_a_passes.h"

static uint16_t              g_adc[3];
static buddy2_motion_state_t g_fake_motion;
static int16_t               g_left_pwm;
static int16_t               g_right_pwm;
static bool g_test_barcode_slowdown;
void buddy2_set_barcode_slowdown (bool active)
{
    g_test_barcode_slowdown = active;
}
int                          tm_printf (const unsigned char * fmt, ...)
{
    (void) fmt;
    return 0;
}
void hal_adc_init (void)
{
}
void hal_adc_pin_init (uint32_t pin)
{
#if BARCODE_DIGITAL_INPUT
    assert(pin != IR_SENSOR_RIGHT_PIN);
#else
    (void) pin;
#endif
}
uint16_t hal_adc_read (uint32_t channel)
{
#if BARCODE_DIGITAL_INPUT
    assert(channel != IR_ADC_CHANNEL_RIGHT);
#endif
    return g_adc[channel];
}
void buddy2_get_motion_state (buddy2_motion_state_t * p_state)
{
    *p_state = g_fake_motion;
}
void buddy2_get_encoder_position (buddy2_encoder_position_t * p_position)
{
    p_position->left_distance_mm    = g_fake_motion.left_distance_mm;
    p_position->right_distance_mm   = g_fake_motion.right_distance_mm;
    p_position->left_encoder_fault  = g_fake_motion.left_encoder_fault;
    p_position->right_encoder_fault = g_fake_motion.right_encoder_fault;
}
void buddy2_get_calibration (buddy2_calibration_t * p_cal)
{
    buddy2_calibration_t geometry = {65, 150, 387, 387, 32767, 0};
    *p_cal                        = geometry;
}
void buddy2_set_motor_pwm (int16_t left, int16_t right)
{
    g_left_pwm  = left;
    g_right_pwm = right;
}
void buddy2_stop (void)
{
    g_left_pwm  = 0;
    g_right_pwm = 0;
}
static robot_steering_input_t g_decision_input;
void buddy2_set_motor_pwm_observed (int16_t left, int16_t right,
                                    const struct robot_steering_input * input)
{
    assert(input);
    g_decision_input = *input;
    buddy2_set_motor_pwm(left, right);
}
void buddy2_stop_observed (const struct robot_steering_input * input)
{
    if (input)
    {
        g_decision_input = *input;
    }
    buddy2_stop();
}
static void observe (unsigned side, uint32_t now, float32_t left_mm,
                     float32_t right_mm)
{
    /* Express inputs as black/white, independent of the active calibration. */
    g_adc[0] = (0u != (side & 2u)) ? g_sensor_configs[0].maximum
                                   : g_sensor_configs[0].minimum;
    g_adc[1] = (0u != (side & 1u)) ? g_sensor_configs[1].maximum
                                   : g_sensor_configs[1].minimum;
    g_adc[2] =
        g_sensor_configs[2].minimum; /* Side barcode sensor remains white. */
    g_fake_motion.left_distance_mm  = left_mm;
    g_fake_motion.right_distance_mm = right_mm;
    buddy3_update(now);
}
static void reset (void)
{
    buddy2_motion_state_t empty = {0};
    g_fake_motion               = empty;
    buddy3_init();
    buddy3_set_test_stage(1);
    buddy3_start_line_follow();
}
#if !BARCODE_DIGITAL_INPUT
/* Raw IR is observable before enabling the decoder, while stationary. */
static void test_raw_stream (void)
{
    robot_ir_stream_batch_t stream = {0};

    buddy3_init();
    g_adc[0] = g_sensor_configs[0].minimum;
    g_adc[1] = g_sensor_configs[1].minimum;
    g_adc[2] = g_sensor_configs[2].maximum;
    buddy3_update(10);
    g_adc[2] = g_sensor_configs[2].minimum;
    buddy3_update(20);
    assert(robot_ir_stream_peek(&stream) && 2 == stream.count);
    assert(1 == stream.rows[0].black && 0 == stream.rows[1].black);
    assert(0 == stream.rows[0].left_x10 && 0 == stream.rows[0].right_x10);
    /* Decoding is configured by default (3 mm elements) but stays idle while
     * the robot is neither line following nor driven forward by hand. */
    assert(buddy3_barcode_calibrated() && !g_barcode_decoder.active);
    robot_ir_stream_ack(&stream);
}

/* Encoder model for scan_barcode(): 0 = exact distance, otherwise each wheel
 * reports whole ticks of this many mm (65 mm wheel, 387 ticks/rev = 0.528),
 * the right wheel 0.4 tick out of step with the left, as on the robot. */
static float32_t g_tick_mm = 0.0f;
/* Sample timing jitter: odd samples land this fraction of a step late, so
 * consecutive samples cover (1 - j) and (1 + j) steps, as when the 2 ms task
 * is delayed by others. */
static float32_t g_jitter = 0.0f;
/* Narrow element and gap width of the scanned barcode (wide = 3x). */
static float32_t g_narrow_mm = 4.0f;
/* Stop for this long once the sensor reaches 100 mm (inside the barcode),
 * as when W is released and pressed again; 0 = drive straight through. */
static uint32_t g_pause_ms = 0u;
/* Side sensor raw levels for black and white; 0 = calibration max/min. */
static uint16_t g_black_raw = 0u;
static uint16_t g_white_raw = 0u;

static float32_t encoder_mm (float32_t pos, float32_t phase)
{
    float32_t ticks = 0.0f;

    if (g_tick_mm > 0.0f)
    {
        ticks = (float32_t) (int32_t) ((pos / g_tick_mm) + phase);
        pos   = (ticks - phase) * g_tick_mm;
    }
    return (pos < 0.0f) ? 0.0f : pos;
}

/* How the robot moves over the barcode in scan_barcode(). */
#define SCAN_LINE_FOLLOW (0u)
#define SCAN_MANUAL      (1u)
#define SCAN_NEITHER     (2u)

/* Drive over "*F*" in Code 39 under the side sensor and count the reads:
 * buddy3_update() every step_ms while the sensor moves step_mm.  Exercises
 * real ADC normalization, the encoder adapter and event handoff. */
static unsigned scan_barcode (unsigned how, uint32_t step_ms, float32_t step_mm)
{
    const char * patterns[] = {"010010100", "001011000", "010010100"};
    float32_t    boundaries[30];
    unsigned     reads = 0u;

    boundaries[0] = 60;
    for (unsigned index = 0; index < 29; ++index)
    {
        /* Gaps and narrow elements are g_narrow_mm, wide elements 3x. */
        float32_t width = g_narrow_mm;

        if ((9u != index) && (19u != index) &&
            ('1' == patterns[index / 10u][index % 10u]))
        {
            width = 3.0f * g_narrow_mm;
        }
        boundaries[index + 1] = boundaries[index] + width;
    }
    buddy3_init();
    g_fake_motion.left_distance_mm  = 0.0f;
    g_fake_motion.right_distance_mm = 0.0f;
    assert(
        !buddy3_configure_barcode(2, 0)); /* Actual encoder resolution gate. */
    assert(buddy3_configure_barcode(3, 0));
    buddy3_set_manual_barcode(SCAN_MANUAL == how);
    if (SCAN_LINE_FOLLOW == how)
    {
        buddy3_start_line_follow();
    }
    uint32_t paused_ms = 0u;

    for (unsigned index = 1; ((float32_t) index * step_mm) < 350.0f; ++index)
    {
        float32_t pos =
            ((float32_t) index + ((0u != (index & 1u)) ? g_jitter : 0.0f)) *
            step_mm;
        bool b_black = false;
        for (unsigned element = 0; element < 29; ++element)
        {
            if (pos >= boundaries[element] && pos < boundaries[element + 1])
            {
                b_black = (0u == (element % 2u));
            }
        }
        g_adc[0] = g_sensor_configs[0].minimum;
        g_adc[1] = g_sensor_configs[1].minimum;
        g_adc[2] = b_black
                       ? ((0u != g_black_raw) ? g_black_raw
                                              : g_sensor_configs[2].maximum)
                       : ((0u != g_white_raw) ? g_white_raw
                                              : g_sensor_configs[2].minimum);
        g_fake_motion.left_distance_mm  = encoder_mm(pos, 0.0f);
        g_fake_motion.right_distance_mm = encoder_mm(pos, 0.4f);
        buddy3_update((index * step_ms) + paused_ms);
        if ((0u == paused_ms) && (0u != g_pause_ms) && (pos >= 100.0f))
        {
            /* Robot stopped: same position, time moves on. */
            for (; paused_ms < g_pause_ms; paused_ms += step_ms)
            {
                buddy3_update((index * step_ms) + paused_ms + step_ms);
            }
        }
        buddy3_barcode_event_t event = {0};

        if (buddy3_take_barcode(&event))
        {
            /* A rejected attempt (e.g. stalled) is not a read; a valid one
             * must be exactly "F" forwards. */
            if (getenv("SCAN_DEBUG"))
            {
                printf("event kind %d status %d\n", (int) event.kind,
                       (int) event.format_status);
            }
            if (B3BC_VALID == event.kind)
            {
                assert('F' == event.decoded.symbol);
                assert(!event.decoded.reversed && !event.dropped);
                ++reads;
            }
        }
    }
    assert((SCAN_LINE_FOLLOW == how) == buddy3_line_following());
    buddy3_set_manual_barcode(false);

    return reads;
}

static void test_barcode_handoff (void)
{
    /* 50 mm/s, sampled every 10 ms (line steps only). */
    assert(1u == scan_barcode(SCAN_LINE_FOLLOW, 10u, 0.5f));
    /* Driven straight forward by hand, without line following. */
    assert(1u == scan_barcode(SCAN_MANUAL, 10u, 0.5f));
    /* Neither: the reader stays suspended and reads nothing. */
    assert(0u == scan_barcode(SCAN_NEITHER, 10u, 0.5f));
    /* The W preset measured on the robot: 2.37 mm per 10 ms (237 mm/s).
     * Sampled only every 10 ms that is over the 0.75 mm limit and refused;
     * sampled every BUDDY3_BARCODE_PERIOD_MS (2 ms, 0.47 mm) it reads. */
    assert(0u == scan_barcode(SCAN_MANUAL, 10u, 2.37f));
    assert(1u == scan_barcode(SCAN_MANUAL, BUDDY3_BARCODE_PERIOD_MS, 0.474f));
    assert(1u ==
           scan_barcode(SCAN_LINE_FOLLOW, BUDDY3_BARCODE_PERIOD_MS, 0.474f));
    /* As measured on the robot: whole encoder ticks, 3 mm narrow bars like
     * the printed barcodes, 0.52 mm per 2 ms on average with 20 % timing
     * jitter.  Some samples then measure 0.79 mm, which used to stop the
     * scan ("sample gap"); the smoothing keeps it going and it reads. */
    g_tick_mm   = 0.52766f;
    g_narrow_mm = 3.0f;
    g_jitter    = 0.2f;
    assert(1u == scan_barcode(SCAN_MANUAL, BUDDY3_BARCODE_PERIOD_MS, 0.52f));
    /* W released mid-barcode for 0.5 s, then pressed again: still reads.
     * A pause longer than the reader's 1.5 s stall limit abandons it. */
    g_pause_ms = 500u;
    assert(1u == scan_barcode(SCAN_MANUAL, BUDDY3_BARCODE_PERIOD_MS, 0.52f));
    g_pause_ms = 2000u;
    assert(0u == scan_barcode(SCAN_MANUAL, BUDDY3_BARCODE_PERIOD_MS, 0.52f));
    g_pause_ms = 0u;
    /* Really too fast (1.2 mm per sample) is still refused. */
    assert(0u == scan_barcode(SCAN_MANUAL, BUDDY3_BARCODE_PERIOD_MS, 1.2f));
    /* A faint peak above the black threshold reads from clear white,
     * even when the rise is smaller than the relative jump threshold. */
    g_black_raw = 1050u;
    g_white_raw = 200u;
    assert(1u == scan_barcode(SCAN_MANUAL, BUDDY3_BARCODE_PERIOD_MS, 0.52f));
    /* A peak below the black threshold still needs adjusted calibration. */
    g_black_raw = 900u;
    g_white_raw = 500u;
    assert(0u == scan_barcode(SCAN_MANUAL, BUDDY3_BARCODE_PERIOD_MS, 0.52f));
    /* ...until the dashboard levels are set to them (white 500, black 1100). */
    g_sensor_configs[2].minimum = 500u;
    g_sensor_configs[2].maximum = 1100u;
    assert(1u == scan_barcode(SCAN_MANUAL, BUDDY3_BARCODE_PERIOD_MS, 0.52f));
    g_sensor_configs[2].minimum = 200u;
    g_sensor_configs[2].maximum = 3800u;
    g_black_raw                 = 0u;
    g_white_raw                 = 0u;
    g_tick_mm                   = 0.0f;
    g_narrow_mm                 = 4.0f;
    g_jitter                    = 0.0f;
}

/* White-level zero: the side sensor's present reading becomes white (0),
 * unless it is too close to black to be paper. */
static void test_white_zero (void)
{
    buddy3_ir_state_t state   = {0};
    uint16_t          maximum = 0u;

    buddy3_init();
    maximum  = g_sensor_configs[2].maximum;
    g_adc[0] = g_sensor_configs[0].minimum;
    g_adc[1] = g_sensor_configs[1].minimum;
    g_adc[2] = 1740u; /* A dull white surface, as measured on the robot. */
    buddy3_update(10u);
    buddy3_get_state(&state);
    assert(state.normalized[2] > 0u);
    buddy3_zero_barcode_white();
    for (uint32_t step = 2u; step <= 20u; step++)
    {
        buddy3_update(step * 10u);
    }
    buddy3_get_state(&state);
    assert(1740u == g_sensor_configs[2].minimum && 0u == state.normalized[2]);
    assert(maximum == g_sensor_configs[2].maximum);
    /* On a black bar: no change. */
    g_adc[2] = (uint16_t) (maximum - 100u);
    buddy3_zero_barcode_white();
    for (uint32_t step = 21u; step <= 40u; step++)
    {
        buddy3_update(step * 10u);
    }
    assert(1740u == g_sensor_configs[2].minimum && !g_b_zero_request);
    /* Levels set by hand: applied at the next line step, and a manual
     * setting cancels a pending automatic zero. */
    assert(!buddy3_set_ir_levels(BUDDY3_LEVELS_BARCODE, 3000u, 3100u));
    assert(!buddy3_set_ir_levels(BUDDY3_LEVELS_BARCODE, 500u, 4096u));
    assert(!buddy3_set_ir_levels(3u, 200u, 1300u)); /* No such sensor. */
    /* Front line sensors: dashboard left/right reach the right entries. */
    assert(buddy3_set_ir_levels(BUDDY3_LEVELS_LEFT, 250u, 1500u));
    buddy3_update(480u);
    assert(250u == g_sensor_configs[BUDDY3_FRONT_LEFT].minimum &&
           1500u == g_sensor_configs[BUDDY3_FRONT_LEFT].maximum);
    assert(buddy3_set_ir_levels(BUDDY3_LEVELS_RIGHT, 300u, 2800u));
    buddy3_update(490u);
    assert(300u == g_sensor_configs[BUDDY3_FRONT_RIGHT].minimum &&
           2800u == g_sensor_configs[BUDDY3_FRONT_RIGHT].maximum);
    g_sensor_configs[BUDDY3_FRONT_LEFT].minimum  = 200u;
    g_sensor_configs[BUDDY3_FRONT_LEFT].maximum  = 1300u;
    g_sensor_configs[BUDDY3_FRONT_RIGHT].minimum = 200u;
    g_sensor_configs[BUDDY3_FRONT_RIGHT].maximum = 3000u;
    assert(buddy3_set_ir_levels(BUDDY3_LEVELS_BARCODE, 1300u, 3900u));
    buddy3_zero_barcode_white();
    g_adc[2] = 1300u;
    buddy3_update(500u);
    assert(1300u == g_sensor_configs[2].minimum &&
           3900u == g_sensor_configs[2].maximum && !g_b_zero_request);
    buddy3_get_state(&state);
    assert(0u == state.normalized[2]);
    g_adc[2] = 2600u; /* Halfway: 500/1000. */
    buddy3_update(510u);
    buddy3_get_state(&state);
    assert(500u == state.normalized[2]);
    g_sensor_configs[2].minimum = 200u; /* Back to the default for the rest. */
    g_sensor_configs[2].maximum = maximum;
}

/* Real passes over a printed *A*: the blurred sensor never reaches white
 * between two thick bars, so fixed thresholds saw one long black.  The jump
 * detector must recover the Code 39 narrow/wide pattern (wide > 10 mm). */
static void replay_pass (const uint16_t (*p_pass)[2], size_t count)
{
    /* Start marker read backwards, gap, then the first 8 of reversed A. */
    static const char expected[]             = "nnWnWnnWnnWnnWnnnn";
    char              seen[sizeof(expected)] = {0};
    size_t            runs                   = 0u;
    bool              b_was                  = false;
    float32_t         start                  = 0.0f;

    g_sensor_configs[2].minimum = 1274u;
    g_sensor_configs[2].maximum = 3800u;
    buddy3_colour_reset();
    for (size_t index = 0u; index < count; index++)
    {
        float32_t position_mm = (float32_t) p_pass[index][0] / 10.0f;
        bool      b_black     = buddy3_barcode_colour(
            buddy3_normalize(p_pass[index][1], 1274u, 3800u, false));

        if (b_black != b_was)
        {
            if (b_was || (runs > 0u))
            {
                if (runs < (sizeof(expected) - 1u))
                {
                    seen[runs] = ((position_mm - start) > 10.0f) ? 'W' : 'n';
                }
                runs++;
            }
            start = position_mm;
            b_was = b_black;
        }
    }
    /* Each recording holds about a second of samples: at least the start
     * marker, the gap and the first few elements of the letter. */
    assert(runs >= 15u);
    assert(0 == strncmp(seen, expected,
                        (runs < (sizeof(expected) - 1u))
                            ? runs
                            : (sizeof(expected) - 1u)));
    g_sensor_configs[2].minimum = 200u;
    g_sensor_configs[2].maximum = 3800u;
    buddy3_colour_reset();
}

static void test_real_barcode_passes (void)
{
    replay_pass(g_replay_a5, sizeof(g_replay_a5) / sizeof(g_replay_a5[0]));
    replay_pass(g_replay_a3, sizeof(g_replay_a3) / sizeof(g_replay_a3[0]));
}
#endif

/* A barcode turn request stops line following on the junction and hands
 * the turn over once; without one the junction is crossed straight. */
static void test_junction_turn_handoff (void)
{
    uint8_t turn = BUDDY3_TURN_NONE;

    reset();
    buddy3_set_junction_turn(7u); /* Unknown value: ignored. */
    buddy3_set_junction_turn(BUDDY3_TURN_LEFT);
    observe(0, 10, 0, 0);
    assert(!buddy3_take_junction_turn(&turn)); /* Not at a junction yet. */
    observe(3, 30, 2, 2);
    assert(buddy3_line_following() && g_left_pwm > 0); /* Debouncing. */
    observe(3, 60, 5, 5);
    assert(!buddy3_line_following() && 0 == g_left_pwm && 0 == g_right_pwm);
    assert(buddy3_take_junction_turn(&turn) && BUDDY3_TURN_LEFT == turn);
    assert(!buddy3_take_junction_turn(&turn)); /* Taken once. */

    /* Starting line following clears an old request. */
    buddy3_set_junction_turn(BUDDY3_TURN_RIGHT);
    reset();
    observe(0, 10, 0, 0);
    observe(3, 30, 2, 2);
    observe(3, 60, 5, 5);
    assert(buddy3_line_following() && B3J_CROSS == g_junction.phase);
    assert(!buddy3_take_junction_turn(&turn));
}

/* Encoder hold, edge overrides, junction crossing and the fault stops. */
static void test_steering_and_junctions (void)
{
    buddy3_init();
    assert(buddy3_test_stage() ==
           3u); /* Dashboard/GP21 Start gets recovery by default. */
    reset();
    observe(0, 10, 0, 0);
    observe(0, 20, 1, 3);
    assert(g_left_pwm > g_right_pwm); /* Keep a centred, both-white straight
                                         segment straight. */
    observe(1, 30, 4, 6);
    assert(g_left_pwm <
           g_right_pwm); /* A line edge immediately overrides encoder hold. */
    assert(30 == g_decision_input.ts_ms &&
           TRACE_MICRO == g_decision_input.phase);
    assert(g_decision_input.norm[0] >= g_decision_input.threshold &&
           g_decision_input.norm[1] < g_decision_input.threshold);
    reset();
    g_test_steering.corner_active = true;
    observe(3, 10, 0, 0);
    assert(B3J_CONFIRM ==
           g_junction.phase); /* A long curve may enter a real junction. */
    reset();
    observe(0, 10, 0, 0);
    assert(g_left_pwm == g_right_pwm && g_left_pwm > 0);
    observe(1, 20, 1, 1);
    assert(g_left_pwm < g_right_pwm);
    observe(3, 30, 2, 2);
    assert(g_left_pwm == g_right_pwm && g_left_pwm > 0);
    observe(3, 60, 5, 5);
    assert(B3J_CROSS == g_junction.phase);
    observe(1, 70, 8, 8);
    assert(g_left_pwm == g_right_pwm && buddy3_line_following());
    observe(0, 80, 20, 20);
    assert(B3J_CROSS == g_junction.phase);
    observe(0, 90, 30, 30);
    assert(B3J_CLEAR == g_junction.phase);
    observe(1, 100, 31, 31);
    assert(g_left_pwm < g_right_pwm); /* Ordinary steering resumes. */
    observe(0, 16000, 32, 32);
    assert(buddy3_line_following() && g_left_pwm == g_right_pwm);
    reset();
    observe(3, 10, 0, 0);
    g_fake_motion.left_encoder_fault = true;
    observe(3, 20, 0, 0);
    assert(!buddy3_line_following() && 0 == g_left_pwm && 0 == g_right_pwm);
    observe(0, 30, 1, 1);
    assert(!buddy3_line_following() && 0 == g_left_pwm);
    reset();
    observe(3, 10, 0, 0);
    observe(3, 40, 0, 0);
    observe(3, 2510, 0, 0);
    assert(!buddy3_line_following() && 0 == g_left_pwm && 0 == g_right_pwm);
}

static void test_front_raw_tuning (void)
{
    ir_processing_state_t next = {0};
    assert(ir_tuning_request(1u, 220u, 260u, false));
    assert(ir_tuning_request(2u, 900u, 1000u, true));
    g_adc[BUDDY3_FRONT_LEFT] = 270u;
    g_adc[BUDDY3_FRONT_RIGHT] = 1050u;
    buddy3_read_sensors(&next, 6000u);
    assert(next.black[BUDDY3_FRONT_LEFT]);
    assert(!next.black[BUDDY3_FRONT_RIGHT]);
    assert(next.sensors[BUDDY3_FRONT_LEFT].normalized < BUDDY3_EDGE_THRESHOLD);
    assert(next.b_line_seen && next.line_position == -1000);
    buddy3_track_side(&next, 6000u);
    assert(g_last_touch_side == BUDDY3_SIDE_LEFT);
    g_adc[BUDDY3_FRONT_LEFT] = 240u;
    buddy3_read_sensors(&next, 6010u);
    assert(next.black[BUDDY3_FRONT_LEFT]); /* Grey retains black. */
    g_adc[BUDDY3_FRONT_LEFT] = 210u;
    g_adc[BUDDY3_FRONT_RIGHT] = 850u;
    buddy3_read_sensors(&next, 6020u);
    assert(!next.black[BUDDY3_FRONT_LEFT] && next.black[BUDDY3_FRONT_RIGHT]);
    buddy3_track_side(&next, 6020u);
    assert(g_last_touch_side == BUDDY3_SIDE_RIGHT);
    assert(ir_tuning_request(1u, 0u, 0u, false));
    assert(ir_tuning_request(2u, 0u, 0u, false));
    buddy3_read_sensors(&next, 6030u);
}

int main (void)
{
#if !BARCODE_DIGITAL_INPUT
    /* The power-up white zero is tested on its own, last: the other tests
     * start from the default calibration. */
    g_b_zero_request = false;
    test_raw_stream();
    test_barcode_handoff();
    test_white_zero();
    test_real_barcode_passes();
#endif
    test_steering_and_junctions();
    test_junction_turn_handoff();
    test_front_raw_tuning();
    puts("PASS: real IR integration reads barcodes line following and by hand "
         "(2 ms side sampling at 237 mm/s), "
         "overrides prior turn, hands barcode turns over at the junction, "
         "ignores junction "
         "branches, resumes micro steering, stops on encoder faults/stall, no "
         "15s cutoff");
    return 0;
}

/*** end of file ***/

