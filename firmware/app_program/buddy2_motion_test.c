/**
 * @file    buddy2_motion_test.c
 * @brief   Buddy 2: button-started motion test (for calibration).
 *
 * Press GP20: the car drives forward, turns left, turns right and reverses,
 * printing encoder ticks / distance / speed.  Compare with what the car
 * really did and record it in docs/BUDDY2_CALIBRATION.md.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "app_main.h"
#include "buddy2_motion.h"
#include "buddy2_motion_test.h"
#include "pin_config.h"
#include "robot_hal.h"

/* 20000 is about 61 % duty; the control law holds 65 % of that (about 40 %
 * duty) while driving. */
#define TEST_PWM                (20000)
#define TEST_DISTANCE_MM        (500.0f)
#define TEST_TURN_DEG           (90.0f)
#define TEST_STEP_TIMEOUT_MS    (10000u)
#define TEST_PRINT_PERIOD_MS    (200u)
#define TEST_START_DELAY_MS     (3500)

/* ------------------------------------------------------------------ *
 *  Motion test task
 * ------------------------------------------------------------------ */

static void test_print_state(const char *p_label)
{
    buddy2_motion_state_t state;

    buddy2_get_motion_state(&state);
    tm_printf((UB *)"[test %-5s] ticks L=%d R=%d | distance L=%d R=%d mm | speed L=%d R=%d mm/s\n",
              p_label,
              (INT)state.left_ticks, (INT)state.right_ticks,
              (INT)state.left_distance_mm, (INT)state.right_distance_mm,
              (INT)state.left_speed_mm_s, (INT)state.right_speed_mm_s);
}

/** Wait for a started move to finish, printing progress. */
static void test_wait_for_move(const char *p_label)
{
    uint32_t start_ms = app_now_ms();

    while (buddy2_motion_active())
    {
        test_print_state(p_label);
        if ((app_now_ms() - start_ms) > TEST_STEP_TIMEOUT_MS)
        {
            buddy2_stop();
            tm_printf((UB *)"[%s] TIMEOUT - stopped. Are the encoders connected?\n", p_label);
            break;
        }
        (void)tk_dly_tsk(TEST_PRINT_PERIOD_MS);
    }
    (void)tk_dly_tsk(500);              /* let the car settle */
    test_print_state(p_label);
}

static bool test_button_pressed(void)
{
    return !hal_gpio_read(BUTTON_TEST_PIN);   /* active low */
}

void buddy2_motion_test_task(INT stacd, void *p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    /* Let the start-up banner print first. */
    (void)tk_dly_tsk(TEST_START_DELAY_MS);

    while (1)
    {
        tm_printf((UB *)"\n[test] Press GP20 to run the motion test: forward %d mm, left %d deg, "
                        "right %d deg, backward %d mm (PWM %d).\n",
                  (INT)TEST_DISTANCE_MM, (INT)TEST_TURN_DEG,
                  (INT)TEST_TURN_DEG, (INT)TEST_DISTANCE_MM, (INT)TEST_PWM);

        while (!test_button_pressed())
        {
            (void)tk_dly_tsk(20);
        }
        tm_printf((UB *)"Starting in 1 s - hands off!\n");
        (void)tk_dly_tsk(1000);

        /* Odometry is zeroed before each step, so each printout shows
         * that step on its own. */
        buddy2_reset_encoders();
        (void)buddy2_move_forward(TEST_DISTANCE_MM, TEST_PWM);
        test_wait_for_move("fwd");

        buddy2_reset_encoders();
        (void)buddy2_turn_left(TEST_TURN_DEG, TEST_PWM);
        test_wait_for_move("left");

        buddy2_reset_encoders();
        (void)buddy2_turn_right(TEST_TURN_DEG, TEST_PWM);
        test_wait_for_move("right");

        buddy2_reset_encoders();
        (void)buddy2_move_backward(TEST_DISTANCE_MM, TEST_PWM);
        test_wait_for_move("back");

        tm_printf((UB *)"Test done. Measure what the car actually did and compare.\n");
    }
}
