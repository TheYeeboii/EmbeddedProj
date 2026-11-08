#include <assert.h>
#include <stdio.h>
#include "buddy3_barcode.h"
#include "../firmware/app_program/robot_action_log.c"
#include "../firmware/app_program/buddy2_motion.c"

static uint64_t fake_us;
int             tk_cre_mtx (const T_CMTX * c)
{
    (void) c;
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
uint64_t hal_time_us (void)
{
    return fake_us;
}
uint32_t app_now_ms (void)
{
    return (uint32_t) (fake_us / 1000u);
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
    robot_action_event_t e;
    while (robot_action_peek(&e))
    {
        robot_action_ack(e.seq);
    }
}
int main (void)
{
    buddy2_calibration_t c;
    robot_action_event_t e;
    assert(buddy2_motion_init() == 0);
    buddy2_get_calibration(&c);
    assert(387.0f == c.left_ticks_per_wheel_rev &&
           387.0f == c.right_ticks_per_wheel_rev);
    buddy2_set_trim(50, 100);
    fake_us = 10000;
    buddy2_set_motor_pwm(12000, 12000);
    assert(2400 == g_left_applied_pwm && 4800 == g_right_applied_pwm);
    robot_steering_event_t trace;
    while (robot_steering_peek(&trace))
    {
        robot_steering_ack(trace.seq);
    }
    fake_us                      = 20000;
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
    assert(!robot_action_peek(&e)); /* Unequal trim alone is not a turn. */
    fake_us = 20000;
    buddy2_set_motor_pwm(8000, 12000);
    assert(robot_action_peek(&e) && ROBOT_ACTION_TURN_LEFT == e.action &&
           20 == e.ts_ms);
    assert(8000 == e.left_pwm && 12000 == e.right_pwm);
    robot_action_ack(e.seq);
    buddy2_stop();
    assert(robot_action_peek(&e) && ROBOT_ACTION_STOP_LEFT == e.action);
    drain();
    fake_us = 30000;
    assert(buddy2_turn_right(90, 12000));
    assert(robot_action_peek(&e) && ROBOT_ACTION_TURN_RIGHT == e.action &&
           ROBOT_ACTION_MOTION == e.source);
    drain();
    g_left_ticks  = 250;
    g_right_ticks = 250;
    fake_us       = 40000;
    buddy2_update(40);
    assert(!buddy2_motion_active());
    assert(robot_action_peek(&e) && ROBOT_ACTION_STOP_RIGHT == e.action);
    drain();
    assert(buddy2_move_forward(100, 12000));
    fake_us = 60000;
    buddy2_update(60);
    assert(!robot_action_peek(
        &e)); /* Straight-move PID does not invent turn events. */
    buddy2_stop();
    assert(buddy2_turn_left(90, 12000));
    drain();
    fake_us = 80000;
    buddy2_update(80);
    fake_us = 900000;
    buddy2_update(900);
    assert(!buddy2_motion_active());
    assert(g_state.left_encoder_fault || g_state.right_encoder_fault);
    assert(robot_action_peek(&e) && ROBOT_ACTION_STOP_LEFT == e.action);
    assert(0 == g_left_applied_pwm && 0 == g_right_applied_pwm);
    /* Reproduce the actual 20 ms motor / 10 ms IR cadence. Cached odometry
     * jumps by two ticks and cannot even accumulate the white lead-in with
     * minimum=3 mm. Fresh snapshots recover each real intervening tick. */
    buddy2_stop();
    buddy2_reset_encoders();
    buddy2_set_motor_pwm(10000, 10000);
    buddy3_barcode_decoder_t cached = {0}, fresh = {0};
    buddy3_barcode_config_t  config          = {3};
    float32_t                previous_cached = 0, previous_fresh = 0;
    unsigned                 cached_reads = 0, fresh_reads = 0;
    const char *             groups[] = {"010010100", "001011000", "010010100"};
    float32_t                boundaries[30];
    boundaries[0] = 60;
    for (unsigned i = 0; i < 29; ++i)
    {
        boundaries[i + 1] =
            boundaries[i] + ((9 == i || 19 == i)             ? 4
                             : groups[i / 10][i % 10] == '1' ? 12
                                                             : 4);
    }
    for (unsigned tick = 1; tick <= 650; ++tick)
    {
        uint32_t now = 1000 + tick * 10;
        fake_us      = (uint64_t) now * 1000;
        ++g_left_ticks;
        ++g_right_ticks;
        if (tick % 2 == 0)
        {
            buddy2_update(now);
        }
        buddy2_motion_state_t     old;
        buddy2_encoder_position_t position = {0};
        buddy2_get_motion_state(&old);
        buddy2_get_encoder_position(&position);
        assert(old.left_ticks ==
               g_state.left_ticks); // Snapshot does not advance control state.
        assert(position.left_distance_mm >= old.left_distance_mm);
        if (tick % 2)
        {
            assert(position.left_distance_mm > old.left_distance_mm);
        }
        bool black = false;
        for (unsigned i = 0; i < 29; ++i)
        {
            if (position.left_distance_mm >= boundaries[i] &&
                position.left_distance_mm < boundaries[i + 1])
            {
                black = i % 2 == 0;
            }
        }
        buddy3_barcode_event_t event;
        if (buddy3_barcode_step(&cached, &config, black,
                                old.left_distance_mm - previous_cached, now,
                                true, &event) &&
            B3BC_VALID == event.kind)
        {
            ++cached_reads;
        }
        if (buddy3_barcode_step(&fresh, &config, black,
                                position.left_distance_mm - previous_fresh, now,
                                true, &event) &&
            B3BC_VALID == event.kind)
        {
            assert(event.decoded.symbol == 'F');
            ++fresh_reads;
        }
        previous_cached = old.left_distance_mm;
        previous_fresh  = position.left_distance_mm;
    }
    assert(0 == cached_reads && 1 == fresh_reads);
    puts("PASS: real 20ms/10ms encoder cadence: cached reader fails; fresh "
         "snapshot decodes F without changing motor control");
    puts("PASS: real Buddy 2 hooks capture accepted steering, preserve "
         "trim/calibration, ignore straight PID, and log completion/fault "
         "stops");
    return 0;
}

/*** end of file ***/

