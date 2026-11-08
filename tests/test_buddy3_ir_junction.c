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
    assert(!buddy3_barcode_calibrated() && !g_barcode_decoder.active);
    robot_ir_stream_ack(&stream);
}

/* Exercise real ADC normalization, encoder adapter and event handoff. */
static void test_barcode_handoff (void)
{
    const char * patterns[] = {"010010100", "001011000", "010010100"};
    float32_t    boundaries[30];
    unsigned     reads = 0u;

    boundaries[0] = 60;
    for (unsigned index = 0; index < 29; ++index)
    {
        /* Gaps and narrow elements are 4 mm, wide elements 12 mm. */
        float32_t width = 4.0f;

        if ((9u != index) && (19u != index) &&
            ('1' == patterns[index / 10u][index % 10u]))
        {
            width = 12.0f;
        }
        boundaries[index + 1] = boundaries[index] + width;
    }
    buddy3_init();
    assert(
        !buddy3_configure_barcode(2, 0)); /* Actual encoder resolution gate. */
    assert(buddy3_configure_barcode(3, 0));
    buddy3_start_line_follow();
    for (unsigned index = 1; index < 700; ++index)
    {
        float32_t pos     = (float32_t) index * 0.5f;
        bool      b_black = false;
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
        g_fake_motion.left_distance_mm  = pos;
        g_fake_motion.right_distance_mm = pos;
        buddy3_update(index * 10);
        buddy3_barcode_event_t event = {0};

        if (buddy3_take_barcode(&event))
        {
            assert(B3BC_VALID == event.kind && event.decoded.symbol == 'F');
            assert(!event.decoded.reversed && !event.dropped);
            ++reads;
        }
    }
    assert(1 == reads && buddy3_line_following());
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
    puts("PASS: real IR integration overrides prior turn, ignores junction "
         "branches, resumes micro steering, stops on encoder faults/stall, no "
         "15s cutoff");
    return 0;
}

/*** end of file ***/

