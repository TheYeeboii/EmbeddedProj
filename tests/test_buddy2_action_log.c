#include <assert.h>
#include <stdio.h>
#include "../firmware/app_program/robot_action_log.c"
#include "../firmware/app_program/buddy2_motion.c"

static uint64_t fake_us;
int tk_cre_mtx(const T_CMTX *c) { (void)c; return 1; }
int tk_loc_mtx(ID id,int timeout) { (void)id;(void)timeout;return 0; }
int tk_unl_mtx(ID id) { (void)id;return 0; }
uint64_t hal_time_us(void) { return fake_us; }
uint32_t app_now_ms(void) { return (uint32_t)(fake_us / 1000u); }
void hal_pwm_release_reset(void) {}
void hal_pwm_pin_init(uint32_t pin,uint32_t wrap) { (void)pin;(void)wrap; }
void hal_pwm_set_level(uint32_t pin,uint32_t level) { (void)pin;(void)level; }
void hal_gpio_input_pullup(uint32_t pin) { (void)pin; }
int32_t gpio_irq_register(uint32_t pin,uint32_t events,gpio_irq_cb_t cb) { (void)pin;(void)events;(void)cb;return 0; }
static void drain(void) { robot_action_event_t e; while(robot_action_peek(&e))robot_action_ack(e.seq); }
int main(void) {
    buddy2_calibration_t c;
    robot_action_event_t e;
    assert(buddy2_motion_init()==0);
    buddy2_get_calibration(&c);
    assert(c.left_ticks_per_wheel_rev==387.0f&&c.right_ticks_per_wheel_rev==387.0f);
    buddy2_set_trim(50,100);
    fake_us=10000; buddy2_set_motor_pwm(12000,12000);
    assert(g_left_applied_pwm==2400&&g_right_applied_pwm==4800);
    robot_steering_event_t trace;
    while(robot_steering_peek(&trace))robot_steering_ack(trace.seq);
    fake_us=20000;
    robot_steering_input_t input={0}; input.ts_ms=20; input.norm[0]=900;
    input.threshold=550; input.phase=TRACE_MICRO;
    buddy2_set_motor_pwm_observed(10000,20000,&input);
    assert(robot_steering_peek(&trace) && trace.has_input && trace.input.norm[0]==900);
    assert(trace.requested[0]==10000 && trace.applied[0]==2000 && trace.applied[1]==8000);
    buddy2_set_motor_pwm(12000,12000);
    drain(); /* Clear transitions deliberately introduced by the observed command. */
    assert(!robot_action_peek(&e)); /* Unequal trim alone is not a turn. */
    fake_us=20000; buddy2_set_motor_pwm(8000,12000);
    assert(robot_action_peek(&e)&&e.action==ROBOT_ACTION_TURN_LEFT&&e.ts_ms==20);
    assert(e.left_pwm==8000&&e.right_pwm==12000);
    robot_action_ack(e.seq);
    buddy2_stop();
    assert(robot_action_peek(&e)&&e.action==ROBOT_ACTION_STOP_LEFT);
    drain();
    fake_us=30000; assert(buddy2_turn_right(90,12000));
    assert(robot_action_peek(&e)&&e.action==ROBOT_ACTION_TURN_RIGHT&&e.source==ROBOT_ACTION_MOTION);
    drain();
    g_left_ticks=250; g_right_ticks=250;
    fake_us=40000; buddy2_update(40);
    assert(!buddy2_motion_active());
    assert(robot_action_peek(&e)&&e.action==ROBOT_ACTION_STOP_RIGHT);
    drain();
    assert(buddy2_move_forward(100,12000));
    fake_us=60000; buddy2_update(60);
    assert(!robot_action_peek(&e)); /* Straight-move PID does not invent turn events. */
    buddy2_stop();
    assert(buddy2_turn_left(90,12000)); drain();
    fake_us=80000; buddy2_update(80);
    fake_us=900000; buddy2_update(900);
    assert(!buddy2_motion_active());
    assert(g_state.left_encoder_fault||g_state.right_encoder_fault);
    assert(robot_action_peek(&e)&&e.action==ROBOT_ACTION_STOP_LEFT);
    assert(g_left_applied_pwm==0&&g_right_applied_pwm==0);
    puts("PASS: real Buddy 2 hooks capture accepted steering, preserve trim/calibration, ignore straight PID, and log completion/fault stops");
    return 0;
}
