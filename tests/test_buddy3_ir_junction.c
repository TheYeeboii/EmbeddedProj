#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include "../firmware/app_program/buddy3_ir.c"

static uint16_t adc[3];
static buddy2_motion_state_t fake_motion;
static int16_t left_pwm,right_pwm;
int tm_printf(const unsigned char *fmt,...) { (void)fmt; return 0; }
void hal_adc_init(void) {}
void hal_adc_pin_init(uint32_t pin) { (void)pin; }
uint16_t hal_adc_read(uint32_t channel) { return adc[channel]; }
void buddy2_get_motion_state(buddy2_motion_state_t *s) { *s=fake_motion; }
void buddy2_get_calibration(buddy2_calibration_t *c) {
    buddy2_calibration_t geometry={65,150,387,387,32767,0}; *c=geometry;
}
void buddy2_set_motor_pwm(int16_t l,int16_t r) { left_pwm=l; right_pwm=r; }
void buddy2_stop(void) { left_pwm=0; right_pwm=0; }
static robot_steering_input_t decision_input;
void buddy2_set_motor_pwm_observed(int16_t l,int16_t r,const struct robot_steering_input *input) {
    assert(input); decision_input=*input; buddy2_set_motor_pwm(l,r);
}
void buddy2_stop_observed(const struct robot_steering_input *input) {
    if (input) decision_input=*input;
    buddy2_stop();
}
static void observe(unsigned side,uint32_t now,float l,float r) {
    /* Express inputs as black/white, independent of the active calibration. */
    adc[0]=(side&2)?g_sensor_configs[0].maximum:g_sensor_configs[0].minimum;
    adc[1]=(side&1)?g_sensor_configs[1].maximum:g_sensor_configs[1].minimum;
    adc[2]=g_sensor_configs[2].minimum; /* Side barcode sensor remains white. */
    fake_motion.left_distance_mm=l; fake_motion.right_distance_mm=r;
    buddy3_update(now);
}
static void reset(void) {
    buddy2_motion_state_t empty={0}; fake_motion=empty;
    buddy3_init(); buddy3_set_test_stage(1); buddy3_start_line_follow();
}
int main(void) {
    buddy3_init();
    assert(buddy3_test_stage()==3u); /* Dashboard/GP21 Start gets recovery by default. */
    reset();
    observe(0,10,0,0);
    observe(0,20,1,3);
    assert(left_pwm>right_pwm); /* Keep a centred, both-white straight segment straight. */
    observe(1,30,4,6);
    assert(left_pwm<right_pwm); /* A line edge immediately overrides encoder hold. */
    assert(decision_input.ts_ms==30 && decision_input.phase==TRACE_MICRO);
    assert(decision_input.norm[0]>=decision_input.threshold && decision_input.norm[1]<decision_input.threshold);
    reset();
    g_test_steering.corner_active=true;
    observe(3,10,0,0);
    assert(g_junction.phase==B3J_CONFIRM); /* A long curve may enter a real junction. */
    reset();
    observe(0,10,0,0); assert(left_pwm==right_pwm&&left_pwm>0);
    observe(1,20,1,1); assert(left_pwm<right_pwm);
    observe(3,30,2,2); assert(left_pwm==right_pwm&&left_pwm>0);
    observe(3,60,5,5); assert(g_junction.phase==B3J_CROSS);
    observe(1,70,8,8); assert(left_pwm==right_pwm&&buddy3_line_following());
    observe(0,80,20,20); assert(g_junction.phase==B3J_CROSS);
    observe(0,90,30,30); assert(g_junction.phase==B3J_CLEAR);
    observe(1,100,31,31); assert(left_pwm<right_pwm); /* Ordinary steering resumes. */
    observe(0,16000,32,32); assert(buddy3_line_following()&&left_pwm==right_pwm);
    reset(); observe(3,10,0,0);
    fake_motion.left_encoder_fault=true; observe(3,20,0,0);
    assert(!buddy3_line_following()&&left_pwm==0&&right_pwm==0);
    observe(0,30,1,1); assert(!buddy3_line_following()&&left_pwm==0);
    reset(); observe(3,10,0,0); observe(3,40,0,0); observe(3,2510,0,0);
    assert(!buddy3_line_following()&&left_pwm==0&&right_pwm==0);
    puts("PASS: real IR integration overrides prior turn, ignores junction branches, resumes micro steering, stops on encoder faults/stall, no 15s cutoff");
    return 0;
}
