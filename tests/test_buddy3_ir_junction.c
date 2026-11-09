#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include "../firmware/app_program/buddy3_ir.c"

static uint16_t              g_adc[3];
static buddy2_motion_state_t g_fake_motion;
static int16_t               g_left_pwm;
static int16_t               g_right_pwm;
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
    (void) pin;
}
uint16_t hal_adc_read (uint32_t channel)
{
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
        g_adc[2] =
            b_black ? g_sensor_configs[2].maximum : g_sensor_configs[2].minimum;
        g_fake_motion.left_distance_mm  = encoder_mm(pos, 0.0f);
        g_fake_motion.right_distance_mm = encoder_mm(pos, 0.4f);
        buddy3_update(index * step_ms);
        buddy3_barcode_event_t event = {0};

        if (buddy3_take_barcode(&event))
        {
            assert(B3BC_VALID == event.kind && event.decoded.symbol == 'F');
            assert(!event.decoded.reversed && !event.dropped);
            ++reads;
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
    /* Really too fast (1.2 mm per sample) is still refused. */
    assert(0u == scan_barcode(SCAN_MANUAL, BUDDY3_BARCODE_PERIOD_MS, 1.2f));
    g_tick_mm   = 0.0f;
    g_narrow_mm = 4.0f;
    g_jitter    = 0.0f;
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

int main (void)
{
    test_raw_stream();
    test_barcode_handoff();
    test_steering_and_junctions();
    puts("PASS: real IR integration reads barcodes line following and by hand "
         "(2 ms side sampling at 237 mm/s), "
         "overrides prior turn, ignores junction "
         "branches, resumes micro steering, stops on encoder faults/stall, no "
         "15s cutoff");
    return 0;
}

/*** end of file ***/

