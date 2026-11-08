#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdarg.h>
int tk_dly_tsk (unsigned int ms);
#include "buddy5_avoidance.c"

static jmp_buf            done;
static unsigned           now, cycles, stage, status, progress, angle, scenario;
static int                left, right;
static bool               active, choose_left, aborted, reverse_active;
static unsigned           reverses;
static buddy4_imu_state_t imu;
static buddy2_motion_state_t motion;
uint32_t                     app_now_ms (void)
{
    return now;
}
int tm_printf (const unsigned char * fmt, ...)
{
    (void) fmt;
    return 0;
}
void buddy4_get_state (buddy4_imu_state_t * s)
{
    *s = imu;
}
void buddy2_get_motion_state (buddy2_motion_state_t * s)
{
    *s = motion;
}
void buddy2_set_motor_pwm (int16_t l, int16_t r)
{
    left  = l;
    right = r;
}
void buddy2_stop (void)
{
    left = right   = 0;
    reverse_active = false;
}
bool buddy2_move_backward (float32_t distance_mm, int16_t max_pwm)
{
    assert(APP_AVOID_REVERSE_DISTANCE_MM == distance_mm);
    assert(APP_AVOID_REVERSE_PWM == max_pwm);
    ++reverses;
    left = right   = -max_pwm;
    reverse_active = true;
    return true;
}
bool buddy2_motion_active (void)
{
    return reverse_active;
}
void buddy5_set_servo_angle (uint8_t a)
{
    angle = a;
}
uint32_t buddy5_measure_cm (uint32_t t)
{
    (void) t;
    return active ? 100 : 10;
}
uint32_t buddy5_sample_with_result_cm (buddy5_ping_result_t * r)
{
    *r = BUDDY5_PING_VALID;
    if ((3 == scenario && 135 == angle) || (4 == scenario && 45 == angle))
    {
        *r = BUDDY5_PING_TIMEOUT_RISE;
        return 0;
    }
    return ((45 == angle) != choose_left) ? 40 : 10;
}
bool robot_controller_begin_obstacle_avoidance (void)
{
    active = true;
    return true;
}
bool robot_controller_obstacle_avoidance_active (void)
{
    return active;
}
void robot_controller_finish_obstacle_avoidance (void)
{
    active = false;
}
void robot_controller_abort_obstacle_avoidance (void)
{
    active  = false;
    aborted = true;
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
void robot_telemetry_set_avoidance_scan_values (uint32_t l, uint32_t r)
{
    (void) l;
    (void) r;
}
void robot_telemetry_set_avoidance_scan (uint32_t l, uint32_t r, bool b)
{
    (void) l;
    (void) r;
    assert(b == choose_left);
}
void robot_telemetry_set_avoidance_turn_pwm (int16_t l, int16_t r)
{
    (void) l;
    (void) r;
}
void robot_telemetry_set_avoidance_status (uint8_t s, int16_t h, uint16_t t)
{
    (void) h;
    (void) t;
    status = s;
}
void robot_telemetry_set_avoidance_forward_progress (uint16_t p, uint16_t t)
{
    (void) t;
    progress = p;
}

int tk_dly_tsk (unsigned int ms)
{
    now += ms;
    assert(++cycles < 100);
    if (ROBOT_AVOID_STATUS_REVERSING == status)
    {
        assert(reverse_active && 1 == reverses);
        assert(left == -APP_AVOID_REVERSE_PWM &&
               right == -APP_AVOID_REVERSE_PWM);
        reverse_active = false;
        return 0;
    }
    if (scenario >= 3 && ROBOT_AVOID_STATUS_BLOCKED == status)
    {
        assert(aborted && !active && 0 == left && 0 == right);
        longjmp(done, 1);
    }
    if (ROBOT_AVOID_STATUS_TURNING == status && 0 == stage)
    {
        assert(choose_left ? left < right : left > right);
        /* Start=350 degrees. A wrong-way 40-degree change cannot finish. */
        imu.heading_deg_x10 = choose_left ? 300 : 3100;
        stage               = 1;
    }
    else if (1 == stage)
    {
        assert(ROBOT_AVOID_STATUS_TURNING == status);
        /* The obstacle-side monitor sees 10 cm, so turn-out extends to 40 deg.
         */
        imu.heading_deg_x10 = choose_left ? 3100 : 300;
        stage               = 2;
    }
    else if (2 == stage)
    {
        if (ROBOT_AVOID_STATUS_TURNING == status)
        {
            assert(0 == left &&
                   0 == right); /* Servo re-centring before forward. */
        }
        else
        {
            assert(ROBOT_AVOID_STATUS_FORWARD == status && 9000 == left &&
                   9000 == right);
            imu.heading_deg_x10 += choose_left ? -44 : 44;
            motion.left_distance_mm  = 100;
            motion.right_distance_mm = 200;
            if (1 == scenario)
            {
                motion.left_encoder_fault = true;
            }
            if (2 == scenario)
            {
                motion.right_encoder_fault = true;
            }
            stage = 3;
        }
    }
    else if (3 == stage)
    {
        if (scenario)
        {
            assert(aborted && !active && 0 == left && 0 == right);
            longjmp(done, 1);
        }
        assert(ROBOT_AVOID_STATUS_FORWARD == status && 150 == progress);
        assert(choose_left ? (10320 == left && 7680 == right)
                           : (7680 == left && 10320 == right));
        motion.left_distance_mm  = 270;
        motion.right_distance_mm = 290;
        stage                    = 4;
    }
    else if (4 == stage)
    {
        assert(ROBOT_AVOID_STATUS_RECOVERING == status && 280 == progress);
        stage = 5;
    }
    else if (5 == stage)
    {
        assert(ROBOT_AVOID_STATUS_RECOVERING == status);
        assert(choose_left ? left > right : left < right);
        imu.heading_deg_x10 = 3500;
        stage               = 6;
    }
    else if (6 == stage)
    {
        assert(ROBOT_AVOID_STATUS_FORWARD == status && 9000 == left &&
               9000 == right);
        longjmp(done, 1);
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
    for (scenario = 0; scenario < 5; scenario++)
    {
        for (unsigned side = 0; side < 2; side++)
        {
            choose_left = side != 0;
            active = aborted = reverse_active = false;
            reverses                          = 0;
            now = cycles = stage = status = progress = 0;
            left = right        = 0;
            imu                 = (buddy4_imu_state_t) {0};
            imu.mag_ok          = true;
            imu.heading_deg_x10 = 3500;
            motion              = (buddy2_motion_state_t) {0};
            if (!setjmp(done))
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

