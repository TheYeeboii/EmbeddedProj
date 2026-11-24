/* Buddy 5 avoidance: the pieces of the route back to the line, one at a
 * time.  The whole route runs in test_buddy5_avoidance.c; this checks each
 * rule directly:
 *   - black is ignored until the leg back toward the line, then it starts
 *     the align leg, and a leg that never finds the line stops;
 *   - the side-sonar pass logic (alongside, ended, tail; never seen);
 *   - pivots stop on target or when they overshoot, slow near the end;
 *   - short compass drop-outs are ridden through, long ones stop;
 *   - edge and sidestep sizing from a sweep, with the beam correction.
 */
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
int tk_dly_tsk (unsigned int delay_ms);
#include "buddy5_avoidance.c"

static unsigned              g_now;
static unsigned              g_status;
static unsigned              g_angle;
static int                   g_left;
static int                   g_right;
static bool                  g_b_active;
static bool                  g_b_aborted;
static bool                  g_b_line_seen;
static bool                  g_b_front_black; /* front IR on black now */
static uint32_t              g_side_cm; /* side ping; 0 = no echo */
static uint32_t              g_front_cm;
static buddy4_imu_state_t    g_imu;
static buddy2_motion_state_t g_motion;

uint32_t app_now_ms (void)
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
void buddy2_get_calibration (buddy2_calibration_t * p_calibration)
{
    buddy2_calibration_t empty = {0};

    *p_calibration                = empty;
    p_calibration->track_width_mm = 150.0f;
}
void buddy2_stop (void)
{
    g_left = g_right = 0;
}
bool buddy2_set_motor_pwm_at_speed (int16_t left, int16_t right,
                                    uint8_t speed_pct)
{
    assert(APP_AVOID_SPEED_PCT == speed_pct);
    g_left  = left;
    g_right = right;
    return true;
}
bool buddy2_move_backward_at_speed (float32_t distance_mm, int16_t max_pwm,
                                    uint8_t speed_pct)
{
    (void) distance_mm;
    (void) max_pwm;
    (void) speed_pct;
    return true;
}
bool buddy2_motion_active (void)
{
    return false;
}
void buddy5_set_servo_angle (uint8_t angle_deg)
{
    g_angle = angle_deg;
}
uint32_t buddy5_measure_cm (uint32_t now_ms)
{
    (void) now_ms;
    return g_front_cm;
}
uint32_t buddy5_sample_with_result_cm (buddy5_ping_result_t * p_result)
{
    *p_result = (BUDDY5_NO_ECHO == g_side_cm) ? BUDDY5_PING_TIMEOUT_RISE
                                              : BUDDY5_PING_VALID;
    return g_side_cm;
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
void buddy3_get_colors (bool * p_left_black, bool * p_right_black,
                        bool * p_side_black)
{
    *p_left_black  = g_b_front_black;
    *p_right_black = false;
    *p_side_black  = false;
}
bool buddy3_take_line_seen_event (void)
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
    (void) b_left;
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
void robot_telemetry_set_obstacle_size (uint32_t face_mm, uint32_t width_mm,
                                        bool b_partial)
{
    (void) face_mm;
    (void) width_mm;
    (void) b_partial;
}
void robot_telemetry_set_avoidance_forward_progress (uint16_t progress_mm,
                                                     uint16_t target_mm)
{
    (void) progress_mm;
    (void) target_mm;
}
int tk_dly_tsk (unsigned int delay_ms)
{
    g_now += delay_ms;
    return 0;
}

/* A run going round on the right of a line heading 90.0 degrees, with a
 * good compass; sidestep 200 mm, obstacle face 250 mm ahead. */
static app_avoid_ctx_t start (void)
{
    app_avoid_ctx_t ctx = {0};

    g_b_active  = true;
    g_b_aborted = g_b_line_seen = g_b_front_black = false;
    g_now                       = 1000u;
    g_left = g_right            = 1234;
    g_side_cm                   = BUDDY5_NO_ECHO;
    g_front_cm                  = BUDDY5_NO_ECHO;
    g_imu                       = (buddy4_imu_state_t) {0};
    g_imu.mag_ok                = true;
    g_imu.heading_deg_x10       = 900;
    g_motion                    = (buddy2_motion_state_t) {0};
    ctx.b_go_right              = true;
    ctx.line_heading_x10        = 900;
    ctx.sidestep_mm             = 200.0f;
    ctx.face_mm                 = 250.0f;
    ctx.pos_y_mm                = 200.0f; /* out beside the obstacle */
    ctx.pass_gap_mm             = 130.0f;
    ctx.b_head_valid            = true;
    ctx.head_x10                = 900.0f;
    return ctx;
}

/* Put the rover on a heading: compass and tracked heading agree. */
static void set_heading (app_avoid_ctx_t * p_ctx, int16_t heading_x10)
{
    g_imu.heading_deg_x10 = heading_x10;
    p_ctx->head_x10       = (float32_t) heading_x10;
}

/* Drive both wheels forward by mm and run one 100 ms step. */
static void drive (app_avoid_ctx_t * p_ctx, float32_t mm)
{
    g_motion.left_distance_mm += mm;
    g_motion.right_distance_mm += mm;
    g_now += 100u;
    app_step(p_ctx, g_now);
}

/* Black on the way out and past the obstacle is the line beside it: it does
 * not end the run.  Only the leg back toward the line takes it, and then
 * drives the sensor-to-axle distance before turning along the line. */
static void test_line_only_on_return (void)
{
    app_avoid_ctx_t ctx = start();

    ctx.pos_y_mm = 0.0f; /* starting on the line */
    app_start_leg(&ctx, g_now, APP_LEG_SIDESTEP, 1800, 200.0f);
    g_b_line_seen = true;
    drive(&ctx, 50.0f);
    assert((APP_AVOID_WAIT_LEG == ctx.state) && (APP_LEG_SIDESTEP == ctx.leg));
    assert(g_b_active && (g_left > 0));

    app_start_leg(&ctx, g_now, APP_LEG_PASS, 900, APP_PASS_MAX_MM);
    g_b_line_seen = true;
    drive(&ctx, 50.0f);
    assert((APP_LEG_PASS == ctx.leg) && g_b_active);
    assert(ROBOT_AVOID_STATUS_PASSING == g_status);

    /* The pivot toward the line clears the old event first. */
    set_heading(&ctx, 0);
    app_start_pivot(&ctx, g_now, 0, APP_AFTER_PIVOT_RETURN);
    g_b_line_seen = true;
    drive(&ctx, 0.0f);
    assert((APP_AVOID_WAIT_LEG == ctx.state) && (APP_LEG_RETURN == ctx.leg));
    assert(!g_b_line_seen);
    assert(APP_RETURN_MAX_MM == ctx.leg_target_mm);
    drive(&ctx, 40.0f);
    assert((APP_LEG_RETURN == ctx.leg) &&
           (ROBOT_AVOID_STATUS_REJOINING == g_status));

    /* Now black is the line beyond the obstacle: crossed (black, then white
     * 10 mm on), so the align leg drives the rest of the 100 mm. */
    g_b_line_seen = true;
    drive(&ctx, 40.0f);
    assert(APP_LEG_RETURN == ctx.leg); /* on black: crossing or branch? */
    drive(&ctx, 10.0f);
    assert((APP_LEG_ALIGN == ctx.leg) &&
           (fabsf(ctx.leg_target_mm - (APP_ALIGN_FORWARD_MM - 10.0f)) < 0.5f));
    drive(&ctx, 60.0f);
    assert(APP_LEG_ALIGN == ctx.leg);
    drive(&ctx, 50.0f);
    assert((APP_AVOID_WAIT_PIVOT == ctx.state) &&
           (900 == ctx.pivot_target_x10) &&
           (APP_AFTER_PIVOT_FINISH == ctx.after_pivot));
    /* The pivot back along the line turns right (clockwise), a front
     * sensor touches the line on the way ... */
    drive(&ctx, 0.0f);
    assert((g_left > 0) && (g_right < 0));
    g_b_front_black = true;
    drive(&ctx, 0.0f);
    g_b_front_black = false;
    /* ... and on the line's heading the run hands back. */
    set_heading(&ctx, 895);
    drive(&ctx, 0.0f);
    assert(!g_b_active && !g_b_aborted && (0 == g_left) && (0 == g_right));
    assert(ROBOT_AVOID_STATUS_COMPLETE == g_status);
    assert(APP_ULTRASONIC_ANGLE_DEG == g_angle);
}

/* The way back that never finds the line, with no position to trust, stops
 * instead of wandering. */
static void test_line_not_found (void)
{
    app_avoid_ctx_t ctx = start();

    ctx.pos_y_mm = 2000.0f; /* far out: the position never says "there" */
    app_start_leg(&ctx, g_now, APP_LEG_RETURN, 0, 300.0f);
    drive(&ctx, 150.0f);
    assert(g_b_active);
    drive(&ctx, 160.0f);
    assert(!g_b_active && g_b_aborted && (0 == g_left));
    assert(ROBOT_AVOID_STATUS_TIMEOUT == g_status);
}

/* Idea B: the tracked position.  Driving at the compass heading moves it
 * along and across the line; on the way back it says where the line is, the
 * IR confirms it (and corrects the position), or after a short way more the
 * position is trusted. */
static void test_tracked_position (void)
{
    app_avoid_ctx_t ctx = start();

    /* Tracking from here, on the line, line heading 90.0. */
    ctx.pos_y_mm       = 0.0f;
    ctx.b_tracking     = true;
    ctx.track_left_mm  = g_motion.left_distance_mm;
    ctx.track_right_mm = g_motion.right_distance_mm;

    /* 200 mm at 180.0 (90 degrees right of the line): y +200, x 0. */
    set_heading(&ctx, 1800);
    app_start_leg(&ctx, g_now, APP_LEG_SIDESTEP, 1800, 200.0f);
    drive(&ctx, 100.0f);
    assert(fabsf(ctx.pos_y_mm - 100.0f) < 1.0f && fabsf(ctx.pos_x_mm) < 1.0f);
    assert(APP_LEG_SIDESTEP == ctx.leg);
    drive(&ctx, 100.0f);
    /* The sidestep ends on the position, not just the distance. */
    assert(fabsf(ctx.pos_y_mm - 200.0f) < 1.0f);
    assert(APP_AVOID_WAIT_PIVOT == ctx.state);

    /* 100 mm at 120.0 (30 degrees off the line): x +86.6, y +50. */
    ctx.state = APP_AVOID_WAIT_LEG;
    set_heading(&ctx, 1200);
    drive(&ctx, 100.0f);
    assert(fabsf(ctx.pos_x_mm - 86.6f) < 1.0f);
    assert(fabsf(ctx.pos_y_mm - 250.0f) < 1.0f);

    /* Back toward the line (heading 0.0): the IR misses it.  When the
     * position puts the sensors on it, drive APP_RETURN_CONFIRM_MM more,
     * then trust the position and align the rest of the way. */
    ctx.pos_y_mm = 160.0f;
    set_heading(&ctx, 0);
    app_start_leg(&ctx, g_now, APP_LEG_RETURN, 0, 500.0f);
    drive(&ctx, 40.0f); /* y 120 */
    assert((APP_LEG_RETURN == ctx.leg) && !ctx.b_return_trusting);
    drive(&ctx, 30.0f); /* y 90: sensors should be on the line */
    assert((APP_LEG_RETURN == ctx.leg) && ctx.b_return_trusting);
    drive(&ctx, 30.0f); /* y 60 */
    assert(APP_LEG_RETURN == ctx.leg);
    drive(&ctx, 40.0f); /* y 20, 70 mm since, but only 140 mm back */
    assert(APP_LEG_RETURN == ctx.leg); /* at least APP_RETURN_MIN_MM first */
    drive(&ctx, 100.0f);
    assert(APP_LEG_RETURN == ctx.leg);
    drive(&ctx, 60.0f); /* 300 mm: now trust it (past the line: align 0) */
    assert(APP_LEG_ALIGN == ctx.leg);
    assert(ctx.leg_target_mm < 1.0f);

    /* With the IR on the line, the position is corrected to one
     * sensor-to-axle distance from it, and the align leg drives that. */
    ctx.pos_y_mm          = 200.0f; /* drifted a little */
    app_start_leg(&ctx, g_now, APP_LEG_RETURN, 0, 500.0f);
    g_b_line_seen = true;
    drive(&ctx, 10.0f);
    drive(&ctx, 10.0f); /* white again: crossed it */
    assert((APP_LEG_ALIGN == ctx.leg) &&
           (fabsf(ctx.leg_target_mm - (APP_ALIGN_FORWARD_MM - 10.0f)) < 0.5f));
    assert(fabsf(ctx.pos_y_mm - (APP_ALIGN_FORWARD_MM - 10.0f)) < 1.0f);
    assert(fabsf(app_sine(900) - 1.0f) < 0.002f);
    assert(fabsf(app_cosine(-600) - 0.5f) < 0.002f);
    assert(fabsf(app_cosine(1800) + 1.0f) < 0.002f);
}

/* Turning or driving back toward the line, the front sonar keeps watching:
 * the obstacle in front means it is not past it yet, so it turns back along
 * the line, drives further and tries again, at most three times. */
static void test_turn_back_retry (void)
{
    app_avoid_ctx_t ctx = start(); /* round the right: back toward is left */

    /* Turning back toward the line (heading 0), the box is 18 cm ahead. */
    app_start_pivot(&ctx, g_now, 0, APP_AFTER_PIVOT_RETURN);
    g_front_cm = 18u;
    drive(&ctx, 0.0f);
    assert(g_b_active && (1u == ctx.pass_retries));
    assert((APP_AVOID_WAIT_PIVOT == ctx.state) && (900 == ctx.pivot_target_x10) &&
           (APP_AFTER_PIVOT_PASS_MORE == ctx.after_pivot));
    /* Back on the line's heading: drive further along it ... */
    g_front_cm = BUDDY5_NO_ECHO;
    drive(&ctx, 0.0f);
    assert((APP_AVOID_WAIT_LEG == ctx.state) && (APP_LEG_PASS_MORE == ctx.leg) &&
           (APP_PASS_RETRY_MM == ctx.leg_target_mm));
    drive(&ctx, 100.0f);
    assert(APP_LEG_PASS_MORE == ctx.leg);
    assert(ROBOT_AVOID_STATUS_PASSING == g_status);
    drive(&ctx, 100.0f);
    /* ... then turn back toward the line again. */
    assert((APP_AVOID_WAIT_PIVOT == ctx.state) && (0 == ctx.pivot_target_x10) &&
           (APP_AFTER_PIVOT_RETURN == ctx.after_pivot));

    /* On the way back to the line the box ahead (20 cm) does the same. */
    set_heading(&ctx, 0);
    app_start_leg(&ctx, g_now, APP_LEG_RETURN, 0, 400.0f);
    g_front_cm = 20u;
    drive(&ctx, 20.0f);
    assert((2u == ctx.pass_retries) &&
           (APP_AFTER_PIVOT_PASS_MORE == ctx.after_pivot));

    /* Farther than that is not the box: keep going. */
    app_start_leg(&ctx, g_now, APP_LEG_RETURN, 0, 400.0f);
    g_front_cm = 40u;
    drive(&ctx, 20.0f);
    assert((APP_LEG_RETURN == ctx.leg) && (2u == ctx.pass_retries));

    /* A third retry is allowed, a fourth stops the run. */
    g_front_cm = 15u;
    drive(&ctx, 20.0f);
    assert(g_b_active && (3u == ctx.pass_retries));
    app_start_leg(&ctx, g_now, APP_LEG_RETURN, 0, 400.0f);
    drive(&ctx, 20.0f);
    assert(!g_b_active && g_b_aborted &&
           (ROBOT_AVOID_STATUS_BLOCKED == g_status));
}

/* After the last pivot along the line, if no front sensor has touched it,
 * the rover sweeps 35 degrees toward where it should be, then the other
 * way, and hands back the moment a sensor finds it; never found, it stops
 * instead of driving off. */
static void test_line_search (void)
{
    app_avoid_ctx_t ctx = start(); /* round the right: line is to the left */

    /* Last pivot done (on the line's heading), nothing touched. */
    app_start_pivot(&ctx, g_now, 900, APP_AFTER_PIVOT_FINISH);
    drive(&ctx, 0.0f);
    assert((APP_AVOID_WAIT_PIVOT == ctx.state) &&
           (APP_AFTER_PIVOT_SEARCH_OTHER == ctx.after_pivot));
    assert(app_heading_offset_x10(900, -APP_SEARCH_SWEEP_X10) ==
           ctx.pivot_target_x10); /* left first */
    drive(&ctx, 0.0f);
    assert((g_left < 0) && (g_right > 0) && g_b_active);
    /* The sensor finds the line part way: hand back at once. */
    set_heading(&ctx, 800);
    g_b_front_black = true;
    drive(&ctx, 0.0f);
    assert(!g_b_active && !g_b_aborted && (0 == g_left));
    assert(ROBOT_AVOID_STATUS_COMPLETE == g_status);

    /* Not found either way: stops. */
    ctx             = start();
    g_b_front_black = false;
    app_start_pivot(&ctx, g_now, 900, APP_AFTER_PIVOT_FINISH);
    drive(&ctx, 0.0f);                      /* -> sweep left */
    set_heading(&ctx, (int16_t) (900 - APP_SEARCH_SWEEP_X10));
    drive(&ctx, 0.0f);                      /* left done -> sweep right */
    assert(APP_AFTER_PIVOT_SEARCH_FAIL == ctx.after_pivot);
    assert(app_heading_offset_x10(900, APP_SEARCH_SWEEP_X10) ==
           ctx.pivot_target_x10);
    set_heading(&ctx, (int16_t) (900 + APP_SEARCH_SWEEP_X10));
    drive(&ctx, 0.0f);
    assert(!g_b_active && g_b_aborted &&
           (ROBOT_AVOID_STATUS_TIMEOUT == g_status));
}

/* On the way back, a line the rover crosses (black that ends within
 * APP_RETURN_CROSS_MAX_MM) is the original line.  Black that goes on longer
 * is a line running the rover's way, such as a junction's branch beside the
 * box: ignored, wherever the tracked position says the rover is. */
static void test_branch_line_ignored (void)
{
    app_avoid_ctx_t ctx = start();

    ctx.pos_y_mm = 200.0f;
    set_heading(&ctx, 0); /* toward the line */
    app_start_leg(&ctx, g_now, APP_LEG_RETURN, 0, 800.0f);
    g_b_front_black = true; /* onto the branch, and along it */
    for (unsigned step = 0u; step < 8u; step++)
    {
        drive(&ctx, 10.0f);
    }
    assert((APP_LEG_RETURN == ctx.leg) && ctx.b_return_along);
    g_b_front_black = false; /* off the end of the branch: still not it */
    drive(&ctx, 10.0f);
    assert((APP_LEG_RETURN == ctx.leg) && !ctx.b_return_black);
    drive(&ctx, 30.0f);
    /* Then the original line: crossed in 20 mm. */
    g_b_front_black = true;
    drive(&ctx, 10.0f);
    drive(&ctx, 10.0f);
    assert(APP_LEG_RETURN == ctx.leg);
    g_b_front_black = false;
    drive(&ctx, 10.0f);
    assert(APP_LEG_ALIGN == ctx.leg);
    assert(fabsf(ctx.leg_target_mm - (APP_ALIGN_FORWARD_MM - 20.0f)) < 0.5f);

    /* A crossing while the tracked position says the original line is
     * still well ahead is another line on the mat: keep going. */
    ctx          = start();
    ctx.pos_y_mm = 450.0f;
    set_heading(&ctx, 0);
    app_start_leg(&ctx, g_now, APP_LEG_RETURN, 0, 800.0f);
    g_b_front_black = true;
    drive(&ctx, 10.0f);
    g_b_front_black = false;
    drive(&ctx, 10.0f);
    assert((APP_LEG_RETURN == ctx.leg) && !ctx.b_return_black);
}

/* Idea A: keep the gap while passing.  Nearer than planned steers away from
 * the obstacle, farther steers back in, limited; nothing near steers
 * straight. */
static void test_gap_keeping (void)
{
    app_avoid_ctx_t ctx = start(); /* round the right: obstacle on the left */

    ctx.pos_y_mm    = 200.0f;
    ctx.pass_gap_mm = 130.0f;
    app_start_leg(&ctx, g_now, APP_LEG_PASS, 900, APP_PASS_MAX_MM);
    ctx.b_pass_alongside = true;
    g_side_cm = 11u; /* 103 mm across: 27 mm too close -> away = right */
    (void) app_pass_update(&ctx, 300.0f);
    assert((ctx.pass_steer_x10 > 60) && (ctx.pass_steer_x10 < 100));
    g_side_cm = 17u; /* 160 mm: 30 mm too far -> back in = left */
    (void) app_pass_update(&ctx, 310.0f);
    assert((ctx.pass_steer_x10 < -70) && (ctx.pass_steer_x10 > -110));
    g_side_cm = 2u; /* very close: limited */
    (void) app_pass_update(&ctx, 320.0f);
    assert(APP_PASS_STEER_MAX_X10 == ctx.pass_steer_x10);
    g_side_cm = BUDDY5_NO_ECHO; /* nothing there: straight */
    (void) app_pass_update(&ctx, 330.0f);
    assert(0 == ctx.pass_steer_x10);

    /* Round the left the directions swap. */
    ctx.b_go_right = false;
    ctx.pos_y_mm   = -200.0f;
    g_side_cm      = 11u;
    (void) app_pass_update(&ctx, 340.0f);
    assert((ctx.pass_steer_x10 < -60) && (ctx.pass_steer_x10 > -100));

    /* Turned 20 degrees away from it, the same echo is farther to the
     * side (the sonar looks square-on): 110 mm, not 103 mm, so less
     * steering away -- no runaway. */
    ctx            = start();
    ctx.pass_gap_mm = 130.0f;
    app_start_leg(&ctx, g_now, APP_LEG_PASS, 900, APP_PASS_MAX_MM);
    ctx.b_pass_alongside = true;
    set_heading(&ctx, 1100); /* round the right: right is away */
    g_side_cm = 11u;
    (void) app_pass_update(&ctx, 300.0f);
    assert((ctx.pass_steer_x10 > 40) && (ctx.pass_steer_x10 < 70));

    /* The held heading bends by the offset: 90.0 + 5.0 to the right. */
    ctx            = start();
    ctx.pos_y_mm   = 200.0f;
    ctx.pass_gap_mm = 130.0f;
    app_start_leg(&ctx, g_now, APP_LEG_PASS, 900, APP_PASS_MAX_MM);
    ctx.b_pass_alongside = true;
    g_side_cm            = 11u;
    drive(&ctx, 20.0f);
    assert(g_left > g_right); /* steering right, away from it */
}

/* Passing: wait for the obstacle alongside, then for it to end, then the
 * tail.  Far readings (beyond the line) are not the obstacle. */
static void test_pass_logic (void)
{
    app_avoid_ctx_t ctx = start();

    app_start_leg(&ctx, g_now, APP_LEG_PASS, 900, APP_PASS_MAX_MM);
    g_side_cm = 60u; /* beyond the original line: not the obstacle */
    assert(!app_pass_update(&ctx, 100.0f));
    assert(!app_pass_update(&ctx, 120.0f));
    assert(!ctx.b_pass_alongside);
    g_side_cm = 12u; /* alongside */
    assert(!app_pass_update(&ctx, 260.0f));
    assert(!ctx.b_pass_alongside); /* one reading is not enough */
    assert(!app_pass_update(&ctx, 270.0f));
    assert(ctx.b_pass_alongside);
    g_side_cm = BUDDY5_NO_ECHO; /* its end */
    assert(!app_pass_update(&ctx, 400.0f));
    g_side_cm = 12u; /* a gap of one reading does not end it */
    assert(!app_pass_update(&ctx, 410.0f));
    g_side_cm = BUDDY5_NO_ECHO;
    assert(!app_pass_update(&ctx, 420.0f));
    assert(!app_pass_update(&ctx, 430.0f));
    assert(!app_pass_update(&ctx, 440.0f));
    assert(!app_pass_update(&ctx, 450.0f));
    assert(!ctx.b_pass_cleared); /* four clear readings are not enough */
    assert(!app_pass_update(&ctx, 460.0f));
    assert(ctx.b_pass_cleared && (460.0f == ctx.pass_cleared_mm));
    assert(!app_pass_update(&ctx, 460.0f + APP_PASS_TAIL_MM - 1.0f));
    assert(app_pass_update(&ctx, 460.0f + APP_PASS_TAIL_MM));

    /* Clear readings too soon after the face (missed echoes): not ended
     * until it is at least APP_PASS_MIN_DEPTH_MM past the face. */
    ctx = start();
    app_start_leg(&ctx, g_now, APP_LEG_PASS, 900, APP_PASS_MAX_MM);
    g_side_cm = 12u;
    (void) app_pass_update(&ctx, 240.0f);
    (void) app_pass_update(&ctx, 250.0f);
    assert(ctx.b_pass_alongside);
    g_side_cm = BUDDY5_NO_ECHO;
    for (float32_t mm = 260.0f; mm < (ctx.face_mm + APP_PASS_MIN_DEPTH_MM);
         mm += 10.0f)
    {
        (void) app_pass_update(&ctx, mm);
        assert(!ctx.b_pass_cleared);
    }
    (void) app_pass_update(&ctx, ctx.face_mm + APP_PASS_MIN_DEPTH_MM);
    assert(ctx.b_pass_cleared);

    /* Never seen alongside: assume a typical depth past the face. */
    ctx = start();
    app_start_leg(&ctx, g_now, APP_LEG_PASS, 900, APP_PASS_MAX_MM);
    assert(!app_pass_update(&ctx, ctx.face_mm + APP_PASS_UNSEEN_MM - 1.0f));
    assert(!app_pass_update(&ctx, ctx.face_mm + APP_PASS_UNSEEN_MM));
    assert(ctx.b_pass_cleared &&
           ((ctx.face_mm + APP_PASS_ASSUMED_DEPTH_MM) == ctx.pass_cleared_mm));
    assert(app_pass_update(&ctx, ctx.face_mm + APP_PASS_ASSUMED_DEPTH_MM +
                                     APP_PASS_TAIL_MM));

    /* Alongside for ever: a wall, not an obstacle; stops. */
    ctx = start();
    app_start_leg(&ctx, g_now, APP_LEG_PASS, 900, 300.0f);
    g_side_cm = 12u;
    drive(&ctx, 100.0f);
    drive(&ctx, 100.0f);
    assert(g_b_active);
    drive(&ctx, 110.0f);
    assert(!g_b_active && g_b_aborted &&
           (ROBOT_AVOID_STATUS_BLOCKED == g_status));
}

/* Pivots: the direction follows the shorter way, slow near the end, stop on
 * target or when they overshoot. */
static void test_pivots (void)
{
    app_avoid_ctx_t ctx = start();

    /* 90.0 -> 180.0 degrees is a right turn, fast at first. */
    app_start_pivot(&ctx, g_now, 1800, APP_AFTER_PIVOT_SIDESTEP);
    drive(&ctx, 0.0f);
    assert((APP_PIVOT_FAST_PWM == g_left) && (-APP_PIVOT_FAST_PWM == g_right));
    set_heading(&ctx, 1650); /* 15 degrees to go: slow */
    drive(&ctx, 0.0f);
    assert((APP_PIVOT_SLOW_PWM == g_left) && (-APP_PIVOT_SLOW_PWM == g_right));
    set_heading(&ctx, 1760); /* within 5 degrees: on to the sidestep */
    drive(&ctx, 0.0f);
    assert((APP_AVOID_WAIT_LEG == ctx.state) && (APP_LEG_SIDESTEP == ctx.leg));
    assert(1800 == ctx.leg_heading_x10);

    /* An overshoot past the target also stops it (no swinging back). */
    ctx = start();
    app_start_pivot(&ctx, g_now, 0, APP_AFTER_PIVOT_SIDESTEP);
    drive(&ctx, 0.0f);
    assert((g_left < 0) && (g_right > 0)); /* left turn */
    set_heading(&ctx, 300);
    drive(&ctx, 0.0f);
    assert(APP_AVOID_WAIT_PIVOT == ctx.state);
    set_heading(&ctx, 3500); /* 10 degrees past 0 */
    drive(&ctx, 0.0f);
    assert(APP_AVOID_WAIT_LEG == ctx.state);

    /* A pivot that does not get there times out and stops. */
    /* The compass says it is not turning, so the wheels' turn is taken as
     * slip (each reading is within the glitch gate of the estimate). */
    ctx = start();
    app_start_pivot(&ctx, g_now, 1800, APP_AFTER_PIVOT_SIDESTEP);
    for (unsigned step = 0u; (step < 70u) && g_b_active; step++)
    {
        /* The wheels move (not stuck), but the compass barely does. */
        g_motion.left_distance_mm += 20.0f;
        g_motion.right_distance_mm -= 20.0f;
        g_now += 100u;
        app_step(&ctx, g_now);
    }
    assert(!g_b_active && g_b_aborted &&
           (ROBOT_AVOID_STATUS_TIMEOUT == g_status));
}

/* Turn the wheels on the spot by an angle the rover really turns
 * (tenths of a degree; positive is right), allowing for the slip. */
static void pivot_wheels (float32_t turn_x10)
{
    float32_t arc_mm = ((turn_x10 / APP_X10_PER_RAD) * 150.0f * 0.5f) /
                       APP_PIVOT_WHEEL_SLIP;

    g_motion.left_distance_mm += arc_mm;
    g_motion.right_distance_mm -= arc_mm;
}

/* The heading: the compass, carried on by the encoders while it is out, a
 * jump far from the encoders' estimate ignored as a glitch unless it
 * persists, and a start at 0 with no compass at all. */
static void test_heading_bridge (void)
{
    app_avoid_ctx_t ctx     = start();
    int16_t         heading = 0;

    ctx.b_head_valid = false;
    app_heading_start(&ctx);
    assert(app_heading(&ctx, g_now, &heading) && (900 == heading));

    /* Compass out: a 30-degree right pivot on the wheels still counts. */
    g_imu.mag_ok = false;
    pivot_wheels(300.0f);
    app_heading_update(&ctx);
    assert(app_heading(&ctx, g_now, &heading) && (abs(heading - 1200) <= 2));

    /* Straight driving does not turn it. */
    g_motion.left_distance_mm += 100.0f;
    g_motion.right_distance_mm += 100.0f;
    app_heading_update(&ctx);
    assert(app_heading(&ctx, g_now, &heading) && (abs(heading - 1200) <= 2));

    /* Compass back and close to the estimate: taken as it is. */
    g_imu.mag_ok          = true;
    g_imu.heading_deg_x10 = 1180;
    app_heading_update(&ctx);
    assert(app_heading(&ctx, g_now, &heading) && (1180 == heading));

    /* One wild reading (motor current): ignored. */
    g_imu.heading_deg_x10 = 2900;
    app_heading_update(&ctx);
    assert(app_heading(&ctx, g_now, &heading) && (1180 == heading));
    g_imu.heading_deg_x10 = 1185;
    app_heading_update(&ctx);
    assert(app_heading(&ctx, g_now, &heading) && (1185 == heading));

    /* But if the compass keeps saying it, it is believed. */
    g_imu.heading_deg_x10 = 2000;
    for (uint32_t index = 0u; index < APP_COMPASS_AGREE_COUNT; index++)
    {
        app_heading_update(&ctx);
        assert(app_heading(&ctx, g_now, &heading) && (1185 == heading));
    }
    app_heading_update(&ctx);
    assert(app_heading(&ctx, g_now, &heading) && (2000 == heading));

    /* No compass at the start: turns are relative, so it starts at 0. */
    g_imu.mag_ok     = false;
    ctx.b_head_valid = false;
    app_heading_start(&ctx);
    assert(app_heading(&ctx, g_now, &heading) && (0 == heading));

    /* A whole route leg without the compass keeps going (no stop). */
    ctx          = start();
    g_imu.mag_ok = false;
    app_start_leg(&ctx, g_now, APP_LEG_PASS, 900, APP_PASS_MAX_MM);
    for (unsigned step = 0u; step < 20u; step++)
    {
        drive(&ctx, 20.0f);
    }
    assert(g_b_active && (APP_LEG_PASS == ctx.leg));
}

/* Edges and sidesteps from a sweep: a 200 mm wide face 250 mm ahead.  The
 * last echo on each side is one 5-degree step outside the true edge. */
static void test_edges (void)
{
    uint32_t        sweep[APP_SWEEP_POINTS] = {0};
    app_side_plan_t right                   = {0};
    app_side_plan_t left                    = {0};

    /* On the face: forward 250 mm, so distance 250 / cos(angle).  The face
     * reaches 100 mm each side (21.8 degrees); the beam echoes to 25. */
    for (uint8_t step = 0u; step <= 5u; step++)
    {
        uint32_t cm = (uint32_t) ((25.0f / g_sweep_cos[step]) + 0.5f);

        sweep[APP_SWEEP_CENTRE_INDEX - step] = cm;
        sweep[APP_SWEEP_CENTRE_INDEX + step] = cm;
    }
    app_plan_side(sweep, 250.0f, true, &right);
    app_plan_side(sweep, 250.0f, false, &left);
    assert(right.b_edge_found && left.b_edge_found);
    /* 276 mm at 20 degrees: 94 mm, close to the true 100. */
    assert((right.edge_mm > 85.0f) && (right.edge_mm < 110.0f));
    /* Round the right puts the arm side (left, 75 mm) next to it. */
    assert(fabsf(left.sidestep_mm - (left.edge_mm + APP_ROVER_RIGHT_EXTENT_MM +
                                     APP_AVOID_SIDE_MARGIN_MM)) < 0.01f);
    assert(fabsf(right.sidestep_mm -
                 (right.edge_mm + APP_ROVER_LEFT_EXTENT_MM +
                  APP_AVOID_SIDE_MARGIN_MM)) < 0.01f);
    assert(right.b_usable && left.b_usable);

    /* Something beside the rover, nearer than the face (as on the track:
     * 45-degree readings of 23-30 cm with the box 20-24 cm ahead), is not
     * part of the box: the width stays the box's own. */
    for (uint8_t step = 6u; step <= 10u; step++)
    {
        sweep[APP_SWEEP_CENTRE_INDEX - step] = 20u; /* 20 cm, off to one side */
    }
    app_plan_side(sweep, 250.0f, true, &right);
    assert(right.b_edge_found && (right.edge_mm < 110.0f));
    assert(!right.b_usable); /* but that side is in the way */

    /* A flat face that does not end within 60 degrees: unusable. */
    for (uint8_t step = 1u; step <= APP_SWEEP_CENTRE_INDEX; step++)
    {
        sweep[APP_SWEEP_CENTRE_INDEX - step] =
            (uint32_t) ((25.0f / g_sweep_cos[step]) + 0.5f);
    }
    app_plan_side(sweep, 250.0f, true, &right);
    assert(!right.b_edge_found && !right.b_usable);
}

int main (void)
{
    test_line_only_on_return();
    test_line_not_found();
    test_tracked_position();
    test_gap_keeping();
    test_turn_back_retry();
    test_branch_line_ignored();
    test_line_search();
    test_pass_logic();
    test_pivots();
    test_heading_bridge();
    test_edges();
    puts("PASS: avoidance back to the line: black only counts on the way "
         "back, then aligns and hands back; no line stops; tracked position "
         "(encoders + compass) ends the sidestep, says where the line is, is "
         "corrected by the IR or trusted; gap keeping while passing; "
         "obstacle still ahead when turning back -> drive further, retry; "
         "black that lasts (a branch) or a crossing far from the tracked line is "
         "ignored; gap from the real angle to the box; "
         "side-sonar pass "
         "(alongside, ended, tail, unseen, wall); pivots slow, stop on "
         "target or overshoot, time out; heading from the compass bridged "
         "by the encoders with a glitch gate; edges with beam correction, "
         "two-sided face test and sidesteps");
    return 0;
}

/*** end of file ***/
