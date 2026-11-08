#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include "../firmware/app_program/buddy3_ir.c"

static uint16_t              adc[3];
static buddy2_motion_state_t fake_motion;
static int16_t               left_pwm, right_pwm;
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
    return adc[channel];
}
void buddy2_get_motion_state (buddy2_motion_state_t * s)
{
    *s = fake_motion;
}
void buddy2_get_encoder_position (buddy2_encoder_position_t * s)
{
    s->left_distance_mm    = fake_motion.left_distance_mm;
    s->right_distance_mm   = fake_motion.right_distance_mm;
    s->left_encoder_fault  = fake_motion.left_encoder_fault;
    s->right_encoder_fault = fake_motion.right_encoder_fault;
}
void buddy2_get_calibration (buddy2_calibration_t * c)
{
    buddy2_calibration_t geometry = {65, 150, 387, 387, 32767, 0};
    *c                            = geometry;
}
void buddy2_set_motor_pwm (int16_t l, int16_t r)
{
    left_pwm  = l;
    right_pwm = r;
}
void buddy2_stop (void)
{
    left_pwm  = 0;
    right_pwm = 0;
}
static robot_steering_input_t decision_input;
void buddy2_set_motor_pwm_observed (int16_t l, int16_t r,
                                    const struct robot_steering_input * input)
{
    assert(input);
    decision_input = *input;
    buddy2_set_motor_pwm(l, r);
}
void buddy2_stop_observed (const struct robot_steering_input * input)
{
    if (input)
    {
        decision_input = *input;
    }
    buddy2_stop();
}
static void observe (unsigned side, uint32_t now, float32_t l, float32_t r)
{
    /* Express inputs as black/white, independent of the active calibration. */
    adc[0] =
        (side & 2) ? g_sensor_configs[0].maximum : g_sensor_configs[0].minimum;
    adc[1] =
        (side & 1) ? g_sensor_configs[1].maximum : g_sensor_configs[1].minimum;
    adc[2] =
        g_sensor_configs[2].minimum; /* Side barcode sensor remains white. */
    fake_motion.left_distance_mm  = l;
    fake_motion.right_distance_mm = r;
    buddy3_update(now);
}
static void reset (void)
{
    buddy2_motion_state_t empty = {0};
    fake_motion                 = empty;
    buddy3_init();
    buddy3_set_test_stage(1);
    buddy3_start_line_follow();
}
int main (void)
{
    /* Raw IR is observable before enabling the decoder, while stationary. */
    buddy3_init();
    adc[0] = g_sensor_configs[0].minimum;
    adc[1] = g_sensor_configs[1].minimum;
    adc[2] = g_sensor_configs[2].maximum;
    buddy3_update(10);
    adc[2] = g_sensor_configs[2].minimum;
    buddy3_update(20);
    robot_ir_stream_batch_t stream;
    assert(robot_ir_stream_peek(&stream) && 2 == stream.count);
    assert(1 == stream.rows[0].black && 0 == stream.rows[1].black);
    assert(0 == stream.rows[0].left_x10 && 0 == stream.rows[0].right_x10);
    assert(!buddy3_barcode_calibrated() && !g_barcode_decoder.active);
    robot_ir_stream_ack(&stream);
    /* Exercise real ADC normalization, encoder adapter and event handoff. */
    const char * patterns[] = {"010010100", "001011000", "010010100"};
    float32_t    boundaries[30];
    boundaries[0] = 60;
    for (unsigned i = 0; i < 29; ++i)
    {
        float32_t width   = (9 == i || 19 == i)
                                ? 4
                                : (patterns[i / 10][i % 10] == '1' ? 12 : 4);
        boundaries[i + 1] = boundaries[i] + width;
    }
    buddy3_init();
    assert(
        !buddy3_configure_barcode(2, 0)); /* Actual encoder resolution gate. */
    assert(buddy3_configure_barcode(3, 0));
    buddy3_start_line_follow();
    unsigned reads = 0;
    for (unsigned i = 1; i < 700; ++i)
    {
        float32_t pos   = (float32_t) i * 0.5f;
        bool      black = false;
        for (unsigned e = 0; e < 29; ++e)
        {
            if (pos >= boundaries[e] && pos < boundaries[e + 1])
            {
                black = e % 2 == 0;
            }
        }
        adc[0] = g_sensor_configs[0].minimum;
        adc[1] = g_sensor_configs[1].minimum;
        adc[2] =
            black ? g_sensor_configs[2].maximum : g_sensor_configs[2].minimum;
        fake_motion.left_distance_mm = fake_motion.right_distance_mm = pos;
        buddy3_update(i * 10);
        buddy3_barcode_event_t event;
        if (buddy3_take_barcode(&event))
        {
            assert(B3BC_VALID == event.kind && event.decoded.symbol == 'F');
            assert(!event.decoded.reversed && !event.dropped);
            ++reads;
        }
    }
    assert(1 == reads && buddy3_line_following());
    buddy3_init();
    assert(buddy3_test_stage() ==
           3u); /* Dashboard/GP21 Start gets recovery by default. */
    reset();
    observe(0, 10, 0, 0);
    observe(0, 20, 1, 3);
    assert(
        left_pwm >
        right_pwm); /* Keep a centred, both-white straight segment straight. */
    observe(1, 30, 4, 6);
    assert(left_pwm <
           right_pwm); /* A line edge immediately overrides encoder hold. */
    assert(30 == decision_input.ts_ms && TRACE_MICRO == decision_input.phase);
    assert(decision_input.norm[0] >= decision_input.threshold &&
           decision_input.norm[1] < decision_input.threshold);
    reset();
    g_test_steering.corner_active = true;
    observe(3, 10, 0, 0);
    assert(B3J_CONFIRM ==
           g_junction.phase); /* A long curve may enter a real junction. */
    reset();
    observe(0, 10, 0, 0);
    assert(left_pwm == right_pwm && left_pwm > 0);
    observe(1, 20, 1, 1);
    assert(left_pwm < right_pwm);
    observe(3, 30, 2, 2);
    assert(left_pwm == right_pwm && left_pwm > 0);
    observe(3, 60, 5, 5);
    assert(B3J_CROSS == g_junction.phase);
    observe(1, 70, 8, 8);
    assert(left_pwm == right_pwm && buddy3_line_following());
    observe(0, 80, 20, 20);
    assert(B3J_CROSS == g_junction.phase);
    observe(0, 90, 30, 30);
    assert(B3J_CLEAR == g_junction.phase);
    observe(1, 100, 31, 31);
    assert(left_pwm < right_pwm); /* Ordinary steering resumes. */
    observe(0, 16000, 32, 32);
    assert(buddy3_line_following() && left_pwm == right_pwm);
    reset();
    observe(3, 10, 0, 0);
    fake_motion.left_encoder_fault = true;
    observe(3, 20, 0, 0);
    assert(!buddy3_line_following() && 0 == left_pwm && 0 == right_pwm);
    observe(0, 30, 1, 1);
    assert(!buddy3_line_following() && 0 == left_pwm);
    reset();
    observe(3, 10, 0, 0);
    observe(3, 40, 0, 0);
    observe(3, 2510, 0, 0);
    assert(!buddy3_line_following() && 0 == left_pwm && 0 == right_pwm);
    puts("PASS: real IR integration overrides prior turn, ignores junction "
         "branches, resumes micro steering, stops on encoder faults/stall, no "
         "15s cutoff");
    return 0;
}

/*** end of file ***/

