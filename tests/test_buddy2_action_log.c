#include <assert.h>
#include <stdio.h>
#include "../firmware/app_program/robot_action_log.c"
#include "../firmware/app_program/buddy2_motion.c"

static uint64_t g_fake_us;
int             tk_cre_mtx (const T_CMTX * motion_cal)
{
    (void) motion_cal;
    return 1;
}
int tk_loc_mtx (ID id, int timeout)
{
    (void) id;
    (void) timeout;
    return 0;
}
int tk_unl_mtx (ID id)
{
    (void) id;
    return 0;
}
uint32_t app_now_ms (void)
{
    return (uint32_t) (g_fake_us / 1000u);
}
void hal_pwm_release_reset (void)
{
}
void hal_pwm_pin_init (uint32_t pin, uint32_t wrap)
{
    (void) pin;
    (void) wrap;
}
void hal_pwm_set_level (uint32_t pin, uint32_t level)
{
    (void) pin;
    (void) level;
}
void hal_gpio_input_pullup (uint32_t pin)
{
    (void) pin;
}
int32_t gpio_irq_register (uint32_t pin, uint32_t events, gpio_irq_cb_t cb)
{
    (void) pin;
    (void) events;
    (void) cb;
    return 0;
}
static void drain (void)
{
    robot_action_event_t action_event;
    while (robot_action_peek(&action_event))
    {
        robot_action_ack(action_event.seq);
    }
}
/* Steering, trim, motion and fault-stop hooks feed the action log. */
static void test_action_hooks (void)
{
    buddy2_calibration_t motion_cal   = {0};
    robot_action_event_t action_event = {0};
    assert(0 == buddy2_motion_init());
    buddy2_get_calibration(&motion_cal);
    assert(387.0f == motion_cal.left_ticks_per_wheel_rev &&
           387.0f == motion_cal.right_ticks_per_wheel_rev);
    buddy2_set_trim(50, 100);
    /* Fixed here, not the boot default, so the expected PWMs below hold. */
    assert(buddy2_set_speed_limit(40));
    g_fake_us = 10000;
    buddy2_set_motor_pwm(12000, 12000);
    assert(2400 == g_left_applied_pwm && 4800 == g_right_applied_pwm);
    robot_steering_event_t trace;
    while (robot_steering_peek(&trace))
    {
        robot_steering_ack(trace.seq);
    }
    g_fake_us                    = 20000;
    robot_steering_input_t input = {0};
    input.ts_ms                  = 20;
    input.norm[0]                = 900;
    input.threshold              = 550;
    input.phase                  = TRACE_MICRO;
    buddy2_set_motor_pwm_observed(10000, 20000, &input);
    assert(robot_steering_peek(&trace) && trace.has_input &&
           900 == trace.input.norm[0]);
    assert(10000 == trace.requested[0] && 2000 == trace.applied[0] &&
           8000 == trace.applied[1]);
    buddy2_set_motor_pwm(12000, 12000);
    drain(); /* Clear transitions deliberately introduced by the observed
                command. */
    assert(!robot_action_peek(
        &action_event)); /* Unequal trim alone is not a turn. */
    g_fake_us = 20000;
    buddy2_set_motor_pwm(8000, 12000);
    assert(robot_action_peek(&action_event) &&
           ROBOT_ACTION_TURN_LEFT == action_event.action &&
           20 == action_event.ts_ms);
    assert(8000 == action_event.left_pwm && 12000 == action_event.right_pwm);
    robot_action_ack(action_event.seq);
    buddy2_stop();
    assert(robot_action_peek(&action_event) &&
           ROBOT_ACTION_STOP_LEFT == action_event.action);
    drain();
    g_fake_us = 30000;
    assert(buddy2_turn_right(90, 12000));
    assert(robot_action_peek(&action_event) &&
           ROBOT_ACTION_TURN_RIGHT == action_event.action &&
           ROBOT_ACTION_MOTION == action_event.source);
    drain();
    g_left_ticks  = 250;
    g_right_ticks = 250;
    g_fake_us     = 40000;
    buddy2_update(40);
    assert(!buddy2_motion_active());
    assert(robot_action_peek(&action_event) &&
           ROBOT_ACTION_STOP_RIGHT == action_event.action);
    drain();
    assert(buddy2_move_forward(100, 12000));
    g_fake_us = 60000;
    buddy2_update(60);
    assert(!robot_action_peek(
        &action_event)); /* Straight-move PID does not invent turn events. */
    buddy2_stop();
    assert(buddy2_turn_left(90, 12000));
    drain();
    g_fake_us = 80000;
    buddy2_update(80);
    g_fake_us = 900000;
    buddy2_update(900);
    assert(!buddy2_motion_active());
    assert(g_state.left_encoder_fault || g_state.right_encoder_fault);
    assert(robot_action_peek(&action_event) &&
           ROBOT_ACTION_STOP_LEFT == action_event.action);
    assert(0 == g_left_applied_pwm && 0 == g_right_applied_pwm);
}

/* An encoder position read between motor updates includes intervening ticks. */
static void test_encoder_position (void)
{
    buddy2_motion_state_t state;
    buddy2_encoder_position_t position;

    buddy2_set_motor_pwm(10000, 10000);
    buddy2_update(7000u);
    ++g_left_ticks;
    buddy2_get_encoder_position(&position);
    buddy2_get_motion_state(&state);
    assert(position.left_distance_mm > state.left_distance_mm);
    assert(state.timestamp_ms == 7000u);
}

/* A late motor update must not skip the next on-time update. */
static void test_update_jitter (void)
{
    buddy2_motion_state_t state = {0};

    buddy2_stop();
    buddy2_set_motor_pwm(10000, 10000);
    buddy2_update(8000u); /* On time. */
    buddy2_update(8021u); /* Held back 1 ms. */
    g_left_ticks += 10;
    buddy2_update(8040u); /* On time again: 19 ms after the late one. */
    buddy2_get_motion_state(&state);
    assert(8040u == state.timestamp_ms);
    /* 10 ticks of 0.528 mm in 19 ms is about 278 mm/s. */
    assert((state.left_speed_mm_s > 276.0f) &&
           (state.left_speed_mm_s < 279.0f));
    buddy2_update(8041u); /* Queued wake-up straight after: skipped. */
    buddy2_get_motion_state(&state);
    assert(8040u == state.timestamp_ms);
    buddy2_update(8060u);
    buddy2_get_motion_state(&state);
    assert(8060u == state.timestamp_ms);
}

int main (void)
{
    test_action_hooks();
    test_encoder_position();
    test_update_jitter();
    puts("PASS: a late wake-up does not cost the next 20 ms control step; a "
         "queued one is skipped");
    puts("PASS: real Buddy 2 hooks capture accepted steering, preserve "
         "trim/calibration, ignore straight PID, and log completion/fault "
         "stops");
    return 0;
}

/*** end of file ***/

