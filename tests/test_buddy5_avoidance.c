#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdarg.h>
int tk_dly_tsk (unsigned int delay_ms);
#include "buddy5_avoidance.c"

static jmp_buf               g_done;
static unsigned              g_now;
static unsigned              g_cycles;
static unsigned              g_stage;
static unsigned              g_status;
static unsigned              g_progress;
static unsigned              g_angle;
static unsigned              g_scenario;
static int                   g_left;
static int                   g_right;
static bool                  g_b_active;
static bool                  g_b_choose_left;
static bool                  g_b_aborted;
static bool                  g_b_reverse_active;
static unsigned              g_reverses;
static buddy4_imu_state_t    g_imu;
static buddy2_motion_state_t g_motion;
uint32_t                     app_now_ms (void)
{
    return g_now;
}
int tm_printf (const unsigned char * fmt, ...)
{
    (void) fmt;
    return 0;
}
void buddy4_get_state (buddy4_imu_state_t * p_state)
{
    *p_state = g_imu;
}
void buddy2_get_motion_state (buddy2_motion_state_t * p_state)
{
    *p_state = g_motion;
}
void buddy2_set_motor_pwm (int16_t left, int16_t right)
{
    g_left  = left;
    g_right = right;
}
void buddy2_stop (void)
{
    g_left = g_right   = 0;
    g_b_reverse_active = false;
}
bool buddy2_move_backward (float32_t distance_mm, int16_t max_pwm)
{
    assert(APP_AVOID_REVERSE_DISTANCE_MM == distance_mm);
    assert(APP_AVOID_REVERSE_PWM == max_pwm);
    ++g_reverses;
    g_left = g_right   = -max_pwm;
    g_b_reverse_active = true;
    return true;
}
bool buddy2_motion_active (void)
{
    return g_b_reverse_active;
}
void buddy5_set_servo_angle (uint8_t angle_deg)
{
    g_angle = angle_deg;
}
uint32_t buddy5_measure_cm (uint32_t now_ms)
{
    (void) now_ms;
    return g_b_active ? 100 : 10;
}
uint32_t buddy5_sample_with_result_cm (buddy5_ping_result_t * p_result)
{
    *p_result = BUDDY5_PING_VALID;
    if ((3 == g_scenario && 135 == g_angle) ||
        (4 == g_scenario && 45 == g_angle))
    {
        *p_result = BUDDY5_PING_TIMEOUT_RISE;
        return 0;
    }
    return ((45 == g_angle) != g_b_choose_left) ? 40 : 10;
}
bool robot_controller_begin_obstacle_avoidance (void)
{
    g_b_active = true;
    return true;
}
bool robot_controller_obstacle_avoidance_active (void)
{
    return g_b_active;
}
void robot_controller_finish_obstacle_avoidance (void)
{
    g_b_active = false;
}
void robot_controller_abort_obstacle_avoidance (void)
{
    g_b_active  = false;
    g_b_aborted = true;
}
bool buddy3_take_line_seen_event (void)
{
    return false;
}
void buddy3_clear_line_seen_event (void)
{
}
void robot_telemetry_clear_avoidance_scan (void)
{
}
void robot_telemetry_set_avoidance_scan_values (uint32_t left_cm,
                                                uint32_t right_cm)
{
    (void) left_cm;
    (void) right_cm;
}
void robot_telemetry_set_avoidance_scan (uint32_t left_cm, uint32_t right_cm,
                                         bool b_left)
{
    (void) left_cm;
    (void) right_cm;
    assert(b_left == g_b_choose_left);
}
void robot_telemetry_set_avoidance_turn_pwm (int16_t left, int16_t right)
{
    (void) left;
    (void) right;
}
void robot_telemetry_set_avoidance_status (uint8_t status, int16_t heading_x10,
                                           uint16_t target_x10)
{
    (void) heading_x10;
    (void) target_x10;
    g_status = status;
}
void robot_telemetry_set_avoidance_forward_progress (uint16_t progress_mm,
                                                     uint16_t target_mm)
{
    (void) target_mm;
    g_progress = progress_mm;
}

int tk_dly_tsk (unsigned int delay_ms)
{
    g_now += delay_ms;
    assert(++g_cycles < 100);
    if (ROBOT_AVOID_STATUS_REVERSING == g_status)
    {
        assert(g_b_reverse_active && 1 == g_reverses);
        assert(g_left == -APP_AVOID_REVERSE_PWM &&
               g_right == -APP_AVOID_REVERSE_PWM);
        g_b_reverse_active = false;
        return 0;
    }
    if (g_scenario >= 3 && ROBOT_AVOID_STATUS_BLOCKED == g_status)
    {
        assert(g_b_aborted && !g_b_active && 0 == g_left && 0 == g_right);
        longjmp(g_done, 1);
    }
    if (ROBOT_AVOID_STATUS_TURNING == g_status && 0 == g_stage)
    {
        assert(g_b_choose_left ? g_left < g_right : g_left > g_right);
        /* Start=350 degrees. A wrong-way 40-degree change cannot finish. */
        g_imu.heading_deg_x10 = g_b_choose_left ? 300 : 3100;
        g_stage               = 1;
    }
    else if (1 == g_stage)
    {
        assert(ROBOT_AVOID_STATUS_TURNING == g_status);
        /* The obstacle-side monitor sees 10 cm, so turn-out extends to 40 deg.
         */
        g_imu.heading_deg_x10 = g_b_choose_left ? 3100 : 300;
        g_stage               = 2;
    }
    else if (2 == g_stage)
    {
        if (ROBOT_AVOID_STATUS_TURNING == g_status)
        {
            assert(0 == g_left &&
                   0 == g_right); /* Servo re-centring before forward. */
        }
        else
        {
            assert(ROBOT_AVOID_STATUS_FORWARD == g_status && 9000 == g_left &&
                   9000 == g_right);
            g_imu.heading_deg_x10 += g_b_choose_left ? -44 : 44;
            g_motion.left_distance_mm  = 100;
            g_motion.right_distance_mm = 200;
            if (1 == g_scenario)
            {
                g_motion.left_encoder_fault = true;
            }
            if (2 == g_scenario)
            {
                g_motion.right_encoder_fault = true;
            }
            g_stage = 3;
        }
    }
    else if (3 == g_stage)
    {
        if (g_scenario)
        {
            assert(g_b_aborted && !g_b_active && 0 == g_left && 0 == g_right);
            longjmp(g_done, 1);
        }
        assert(ROBOT_AVOID_STATUS_FORWARD == g_status && 150 == g_progress);
        assert(g_b_choose_left ? (10320 == g_left && 7680 == g_right)
                               : (7680 == g_left && 10320 == g_right));
        g_motion.left_distance_mm  = 270;
        g_motion.right_distance_mm = 290;
        g_stage                    = 4;
    }
    else if (4 == g_stage)
    {
        assert(ROBOT_AVOID_STATUS_RECOVERING == g_status && 280 == g_progress);
        g_stage = 5;
    }
    else if (5 == g_stage)
    {
        assert(ROBOT_AVOID_STATUS_RECOVERING == g_status);
        assert(g_b_choose_left ? g_left > g_right : g_left < g_right);
        g_imu.heading_deg_x10 = 3500;
        g_stage               = 6;
    }
    else if (6 == g_stage)
    {
        assert(ROBOT_AVOID_STATUS_FORWARD == g_status && 9000 == g_left &&
               9000 == g_right);
        longjmp(g_done, 1);
    }
    return 0;
}
int main (void)
{
    assert(app_heading_change_x10(3590, 10) == 20);
    assert(app_heading_change_x10(10, 3590) == -20);
    assert(app_heading_change_x10(3600, 0) == 0);
    assert(!app_scan_angle_is_left(45));
    assert(app_scan_angle_is_left(135));
    /* A left turn watches physical right (45); a right turn watches physical
     * left (135). */
    assert(app_outward_monitor_angle(true) == 45);
    assert(app_outward_monitor_angle(false) == 135);
    for (g_scenario = 0; g_scenario < 5; g_scenario++)
    {
        for (unsigned side = 0; side < 2; side++)
        {
            g_b_choose_left = side != 0;
            g_b_active = g_b_aborted = g_b_reverse_active = false;
            g_reverses                                    = 0;
            g_now = g_cycles = g_stage = g_status = g_progress = 0;
            g_left = g_right      = 0;
            g_imu                 = (buddy4_imu_state_t) {0};
            g_imu.mag_ok          = true;
            g_imu.heading_deg_x10 = 3500;
            g_motion              = (buddy2_motion_state_t) {0};
            if (!setjmp(g_done))
            {
                buddy5_avoidance_task(0, NULL);
            }
        }
    }
    puts("PASS: both turns, wrong-way rejection, 0/360 wrap, forward "
         "correction, two encoders, recovery, individual encoder faults, "
         "invalid left/right scan");
    return 0;
}

/*** end of file ***/

