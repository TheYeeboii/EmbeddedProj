#include <assert.h>
#include <stdlib.h>
#include <string.h>
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
    assert(pin == IR_SENSOR_LEFT_PIN || pin == IR_SENSOR_CENTER_PIN);
}
uint16_t hal_adc_read (uint32_t channel)
{
    assert(channel == IR_ADC_CHANNEL_LEFT || channel == IR_ADC_CHANNEL_CENTER);
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
    g_adc[2] = 0u; /* GP28 remains white. */
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

bool hal_gpio_read (uint32_t pin)
{
    assert(pin == BARCODE_DIGITAL_PIN);
    return g_adc[2] != 0u;
}

static void test_barcode_pin_is_only_telemetry (void)
{
    buddy3_ir_state_t state;
    buddy3_init();
    g_adc[0] = g_adc[1] = 200u;
    g_adc[2] = 1u;
    buddy3_start_line_follow();
    buddy3_update(BUDDY3_SAMPLE_PERIOD_MS);
    buddy3_get_state(&state);
    assert(state.raw[2] == 1u && state.normalized[2] == 1000u);
    assert(g_left_pwm > 0 && g_right_pwm > 0);

    g_state.black[BUDDY3_FRONT_LEFT] = true;
    g_state.black[BUDDY3_FRONT_RIGHT] = true;
    assert(!buddy3_dark_surface_stop(&g_state, 1000u));
    g_state.black[BUDDY3_SIDE_SENSOR] = false;
    assert(buddy3_dark_surface_stop(&g_state, 4001u));
    g_state.black[BUDDY3_FRONT_LEFT] = false;
    g_state.black[BUDDY3_SIDE_SENSOR] = true;
    assert(!buddy3_dark_surface_stop(&g_state, 4011u));
}

int main (void)
{
    test_steering_and_junctions();
    test_front_raw_tuning();
    test_barcode_pin_is_only_telemetry();
    puts("PASS: front IR tuning, steering, junction stops and GP28 telemetry without barcode motor control");
    return 0;
}
