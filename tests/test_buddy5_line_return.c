#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
int tk_dly_tsk (unsigned int delay_ms);
#include "buddy5_avoidance.c"

static unsigned              g_now;
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
bool buddy2_set_motor_pwm_at_speed (int16_t left, int16_t right,
                                    uint8_t speed_pct)
{
    (void) speed_pct;
    buddy2_set_motor_pwm(left, right);
    return true;
}
bool buddy2_move_backward_at_speed (float32_t distance_mm, int16_t max_pwm,
                                    uint8_t speed_pct)
{
    (void) speed_pct;
    return buddy2_move_backward(distance_mm, max_pwm);
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
static bool g_b_line_seen;
bool        buddy3_take_line_seen_event (void)
{
    bool b_seen   = g_b_line_seen;
    g_b_line_seen = false;
    return b_seen;
}
void buddy3_clear_line_seen_event (void)
{
    g_b_line_seen = false;
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
    return 0;
}

/* A fresh avoidance run in the given state, with a good IMU heading. */
static app_avoid_ctx_t start (app_avoid_state_t state)
{
    app_avoid_ctx_t ctx = {0};

    g_b_active  = true;
    g_b_aborted = g_b_reverse_active = g_b_line_seen = false;
    g_now                                            = 1000u;
    g_left = g_right            = 1234;
    g_imu                       = (buddy4_imu_state_t) {0};
    g_imu.mag_ok                = true;
    g_imu.heading_deg_x10       = 900;
    g_motion                    = (buddy2_motion_state_t) {0};
    ctx.state                   = state;
    ctx.stall_state             = state;
    ctx.state_started_ms        = g_now;
    ctx.stall_window_started_ms = g_now;
    ctx.arc_start_heading_x10   = 0;
    ctx.forward_target_mm       = 250.0f;
    return ctx;
}

/* Handed back to line following (finished, not aborted), motors stopped. */
static bool handed_back (void)
{
    return !g_b_active && !g_b_aborted && (0 == g_left) && (0 == g_right);
}

/* Aborted: stopped, and line following is not restarted. */
static bool stopped (void)
{
    return !g_b_active && g_b_aborted && (0 == g_left) && (0 == g_right);
}

/* Turning back to the start heading: the line ends the run; a timeout or a
 * lost heading stops it. */
static void test_heading_recovery (void)
{
    app_avoid_ctx_t ctx;

    /* Line crossed while turning back to the start heading: take it. */
    ctx           = start(APP_AVOID_WAIT_RECOVER_HEADING);
    g_b_line_seen = true;
    app_recover_heading(&ctx, g_now);
    assert(handed_back() && APP_AVOID_WATCH_FORWARD == ctx.state);

    /* No line while recovering: keep turning, still avoiding. */
    ctx = start(APP_AVOID_WAIT_RECOVER_HEADING);
    app_recover_heading(&ctx, g_now);
    assert(g_b_active && !g_b_aborted && 0 != g_left + g_right);

    /* Heading recovery that times out stops; it does not resume blind. */
    ctx = start(APP_AVOID_WAIT_RECOVER_HEADING);
    /* The wheels turned (no stall), but the heading never came back. */
    ctx.stall_window_started_ms = g_now + APP_AVOID_RECOVER_TIMEOUT_MS;
    app_recover_heading(&ctx, g_now + APP_AVOID_RECOVER_TIMEOUT_MS);
    assert(stopped() && ROBOT_AVOID_STATUS_TIMEOUT == g_status);

    /* Lost IMU heading while recovering stops. */
    ctx          = start(APP_AVOID_WAIT_RECOVER_HEADING);
    g_imu.mag_ok = false;
    app_recover_heading(&ctx, g_now);
    assert(stopped());
}

/* Forward legs: the line ends the straight leg and the inward diagonal, but
 * not the outward one; no line, or a lost heading, stops the run. */
static void test_forward_legs (void)
{
    app_avoid_ctx_t ctx;

    /* Line crossed on the straight leg after recovery: take it. */
    ctx             = start(APP_AVOID_WAIT_FORWARD_DISTANCE);
    ctx.forward_leg = APP_AVOID_FORWARD_STRAIGHT;
    g_b_line_seen   = true;
    app_forward_distance(&ctx, g_now);
    assert(handed_back());

    /* Line on the inward diagonal: take it (unchanged behaviour). */
    ctx             = start(APP_AVOID_WAIT_FORWARD_DISTANCE);
    ctx.forward_leg = APP_AVOID_FORWARD_INWARD_DIAGONAL;
    g_b_line_seen   = true;
    app_forward_distance(&ctx, g_now);
    assert(handed_back());

    /* The outward diagonal starts on the original line: ignore it there. */
    ctx             = start(APP_AVOID_WAIT_FORWARD_DISTANCE);
    ctx.forward_leg = APP_AVOID_FORWARD_OUTWARD_DIAGONAL;
    g_b_line_seen   = true;
    app_forward_distance(&ctx, g_now);
    assert(g_b_active && !g_b_aborted && g_b_line_seen);

    /* Inward diagonal finished without the line: stop, never hand an
     * off-line car to line following (both white looks centred). */
    ctx             = start(APP_AVOID_WAIT_FORWARD_DISTANCE);
    ctx.forward_leg = APP_AVOID_FORWARD_INWARD_DIAGONAL;
    app_forward_leg_done(&ctx, g_now, 0);
    assert(stopped() && ROBOT_AVOID_STATUS_TIMEOUT == g_status);

    /* Lost IMU heading on a forward leg stops. */
    ctx             = start(APP_AVOID_WAIT_FORWARD_DISTANCE);
    ctx.forward_leg = APP_AVOID_FORWARD_STRAIGHT;
    g_imu.mag_ok    = false;
    app_forward_distance(&ctx, g_now);
    assert(stopped());

    /* Leaving the outward diagonal clears black seen on the old line. */
    ctx             = start(APP_AVOID_WAIT_FORWARD_DISTANCE);
    ctx.forward_leg = APP_AVOID_FORWARD_OUTWARD_DIAGONAL;
    g_b_line_seen   = true;
    app_forward_leg_done(&ctx, g_now, 0);
    assert(APP_AVOID_WAIT_RECOVER_HEADING == ctx.state && !g_b_line_seen &&
           g_b_active);
}

/* The sideways offset from the line, and the last diagonal sized from it. */
static void test_route_offset (void)
{
    app_avoid_ctx_t ctx;

    /* Built-in sine (no math.h in the firmware file): close to the real one. */
    for (int tenths = -1800; tenths <= 1800; tenths += 50)
    {
        assert(fabsf(app_sine((int16_t) tenths) -
                     sinf((float32_t) tenths * 0.0017453293f)) < 0.002f);
    }

    /* Sideways offset from the line: 100 mm driven 30 degrees right of the
     * line is 50 mm to the right; reversing along the line adds nothing. */
    ctx                   = start(APP_AVOID_WAIT_FORWARD_DISTANCE);
    g_imu.heading_deg_x10 = 3500; /* Line heading, near the 0/360 wrap. */
    app_route_begin(&ctx);
    g_motion.left_distance_mm = g_motion.right_distance_mm = -150.0f;
    app_route_track(&ctx);
    assert(ctx.b_route_tracked && fabsf(ctx.route_lateral_mm) < 0.5f);
    g_imu.heading_deg_x10     = 200; /* 30 degrees right, through the wrap. */
    g_motion.left_distance_mm = g_motion.right_distance_mm = -50.0f;
    app_route_track(&ctx);
    assert(fabsf(ctx.route_lateral_mm - 50.0f) < 0.5f);

    /* Last diagonal, 30 degrees left of the line: 100 mm cancels 50 mm, but
     * it is never shorter than the mirrored outward leg; plus the margin. */
    ctx.paired_diagonal_distance_mm = 150.0f;
    assert(fabsf(app_return_distance_mm(&ctx, 3200) -
                 (150.0f + APP_AVOID_RETURN_MARGIN_MM)) < 0.5f);
    /* A wider swing than planned (300 mm out) needs 600 mm back, capped. */
    ctx.route_lateral_mm = 300.0f;
    assert(fabsf(app_return_distance_mm(&ctx, 3200) - APP_AVOID_RETURN_MAX_MM) <
           0.5f);
    ctx.route_lateral_mm = 200.0f;
    assert(fabsf(app_return_distance_mm(&ctx, 3200) -
                 (400.0f + APP_AVOID_RETURN_MARGIN_MM)) < 0.5f);
    /* Heading away from the line, or nearly parallel: mirrored leg + margin. */
    assert(fabsf(app_return_distance_mm(&ctx, 200) -
                 (150.0f + APP_AVOID_RETURN_MARGIN_MM)) < 0.5f);
    assert(fabsf(app_return_distance_mm(&ctx, 3450) -
                 (150.0f + APP_AVOID_RETURN_MARGIN_MM)) < 0.5f);
    /* Same on the left side (offset negative, heading back to the right). */
    ctx.route_lateral_mm = -200.0f;
    assert(fabsf(app_return_distance_mm(&ctx, 200) -
                 (400.0f + APP_AVOID_RETURN_MARGIN_MM)) < 0.5f);
    /* Lost IMU heading: the offset is no longer trusted. */
    g_imu.mag_ok = false;
    app_route_track(&ctx);
    assert(!ctx.b_route_tracked);
    assert(fabsf(app_return_distance_mm(&ctx, 200) -
                 (150.0f + APP_AVOID_RETURN_MARGIN_MM)) < 0.5f);
}

int main (void)
{
    test_heading_recovery();
    test_forward_legs();
    test_route_offset();
    puts("PASS: avoidance returns to the line when recovery, the straight "
         "leg or the inward diagonal crosses it; ignores the original line "
         "on the outward leg; stops (no blind line following) on no line, "
         "IMU loss and timeouts; sizes the last diagonal from the measured "
         "sideways offset");
    return 0;
}

/*** end of file ***/

