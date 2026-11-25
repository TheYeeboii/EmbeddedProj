/* Buddy 5 avoidance: the rules of the way back to the line, one at a time.
 * The whole route runs in test_buddy5_avoidance.c; this checks directly:
 *   - black too early on the way back is ignored, a crossing starts the
 *     align leg, then the last pivot hands back to line following;
 *   - long black (a branch) is ignored;
 *   - no line within the leg's length stops the rover;
 *   - the box in front while going back: drive further, at most 3 times;
 *   - no line under the sensors after the last pivot: search both ways;
 *   - the encoder heading, with pivot scrub.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
int tk_dly_tsk (unsigned int delay_ms);
#include "buddy5_avoidance.c"

#define WHEEL_MM_PER_PWM (0.002f) /* wheel travel per step per PWM unit */

static uint32_t              g_now;
static uint8_t               g_status;
static uint8_t               g_angle;
static int16_t               g_left;
static int16_t               g_right;
static bool                  g_b_active;
static bool                  g_b_aborted;
static bool                  g_b_finished;
static bool                  g_b_black; /* front IR on black now */
static bool                  g_b_seen;  /* latched black event   */
static uint32_t              g_front_cm;
static buddy2_motion_state_t g_motion;

uint32_t app_now_ms (void)
{
    return g_now;
}
int tm_printf (const unsigned char * p_format, ...)
{
    (void) p_format;
    return 0;
}
int tk_dly_tsk (unsigned int delay_ms)
{
    g_now += delay_ms;
    return 0;
}
void buddy4_get_state (buddy4_imu_state_t * p_state)
{
    buddy4_imu_state_t empty = {0};

    *p_state = empty;
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
    g_left  = 0;
    g_right = 0;
}
bool buddy2_move_backward_at_speed (float32_t distance_mm, int16_t max_pwm,
                                    uint8_t speed_pct)
{
    (void) distance_mm;
    (void) max_pwm;
    (void) speed_pct;
    return true;
}
bool buddy2_set_motor_pwm_at_speed (int16_t left, int16_t right,
                                    uint8_t speed_pct)
{
    assert(APP_AVOID_SPEED_PCT == speed_pct);
    g_left  = left;
    g_right = right;
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
    *p_result = BUDDY5_PING_TIMEOUT_RISE;
    return BUDDY5_NO_ECHO;
}
bool robot_controller_begin_obstacle_avoidance (void)
{
    return false;
}
bool robot_controller_obstacle_avoidance_active (void)
{
    return g_b_active;
}
void robot_controller_finish_obstacle_avoidance (void)
{
    g_b_active   = false;
    g_b_finished = true;
}
void robot_controller_abort_obstacle_avoidance (void)
{
    g_b_active  = false;
    g_b_aborted = true;
}
void buddy3_get_colors (bool * p_left_black, bool * p_right_black,
                        bool * p_side_black)
{
    *p_left_black  = g_b_black;
    *p_right_black = false;
    *p_side_black  = false;
}
bool buddy3_take_line_seen_event (void)
{
    bool b_seen = g_b_seen;

    g_b_seen = false;
    return b_seen;
}
void buddy3_clear_line_seen_event (void)
{
    g_b_seen = false;
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
void robot_telemetry_set_avoidance_status (uint8_t status, int16_t change_x10,
                                           uint16_t target_x10)
{
    (void) change_x10;
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

/* ---------------- Helpers ---------------- */

/* Two distances or angles agree to well under a millimetre or degree. */
static bool test_near (float32_t expected, float32_t actual)
{
    return fabsf(expected - actual) < 0.01f;
}

/* One 100 ms step: the wheels move by the PWM last commanded. */
static void step (app_avoid_ctx_t * p_ctx)
{
    g_motion.left_distance_mm += (float32_t) g_left * WHEEL_MM_PER_PWM;
    g_motion.right_distance_mm += (float32_t) g_right * WHEEL_MM_PER_PWM;
    g_now += 100u;
    app_step(p_ctx, g_now);
}

/* Going round on the left, driving back toward the line (heading +90). */
static app_avoid_ctx_t start_return (void)
{
    app_avoid_ctx_t       ctx    = {0};
    buddy2_motion_state_t motion = {0};

    g_now        = 1000u;
    g_motion     = motion;
    g_status     = 0u;
    g_angle      = 0u;
    g_left       = 0;
    g_right      = 0;
    g_b_active   = true;
    g_b_aborted  = false;
    g_b_finished = false;
    g_b_black    = false;
    g_b_seen     = false;
    g_front_cm   = BUDDY5_NO_ECHO;

    ctx.b_go_right       = false;
    ctx.sidestep_mm      = APP_SIDESTEP_MM;
    ctx.heading_deg      = 90.0f;
    ctx.stall_state      = APP_AVOID_LEG;
    ctx.stall_started_ms = g_now;
    app_start_leg(&ctx, g_now, APP_LEG_RETURN, 90.0f,
                  APP_SIDESTEP_MM + APP_RETURN_EXTRA_MM);
    step(&ctx); /* first drive command */

    return ctx;
}

/* Drive the leg on until its progress reaches at least mm. */
static float32_t progress (const app_avoid_ctx_t * p_ctx)
{
    return 0.5f * ((g_motion.left_distance_mm - p_ctx->leg_left_mm) +
                   (g_motion.right_distance_mm - p_ctx->leg_right_mm));
}
static void drive_to (app_avoid_ctx_t * p_ctx, float32_t mm)
{
    while ((APP_AVOID_LEG == p_ctx->state) && (progress(p_ctx) < mm))
    {
        step(p_ctx);
    }
}

/* ---------------- Tests ---------------- */

static void test_crossing (void)
{
    app_avoid_ctx_t ctx = start_return();

    /* Black 60 mm in is far too early: another line on the mat. */
    drive_to(&ctx, 60.0f);
    g_b_black = true;
    step(&ctx);
    assert((APP_LEG_RETURN == ctx.leg) && !ctx.b_black);
    g_b_black = false;

    /* Black in the window, white again within 50 mm: the line. */
    drive_to(&ctx, APP_SIDESTEP_MM - APP_RETURN_EARLY_MM + 50.0f);
    g_b_black = true;
    step(&ctx);
    assert((APP_LEG_RETURN == ctx.leg) && ctx.b_black);
    step(&ctx); /* still on the tape */
    assert(APP_LEG_RETURN == ctx.leg);
    g_b_black = false;
    step(&ctx); /* white */
    assert((APP_AVOID_LEG == ctx.state) && (APP_LEG_ALIGN == ctx.leg));
    assert((ctx.leg_target_mm > (APP_ALIGN_FORWARD_MM - 50.0f)) &&
           (ctx.leg_target_mm < (APP_ALIGN_FORWARD_MM - 20.0f)));
    step(&ctx);
    assert(ROBOT_AVOID_STATUS_REJOINING == g_status);

    /* Align, then pivot back along the line (left turn: right wheel ahead).
     * Touching the line during the turn is not enough: no sensor on black
     * at the end, so it searches straight away. */
    drive_to(&ctx, ctx.leg_target_mm);
    step(&ctx);
    assert((APP_AVOID_PIVOT == ctx.state) && (APP_NEXT_FINISH == ctx.next) &&
           test_near(0.0f, ctx.pivot_target_deg));
    step(&ctx);
    assert((g_left < 0) && (g_right > 0));
    g_b_seen = true; /* crossed it mid-turn */
    while ((APP_AVOID_PIVOT == ctx.state) && (APP_NEXT_FINISH == ctx.next))
    {
        step(&ctx);
    }
    assert(!g_b_finished && (APP_AVOID_PIVOT == ctx.state) &&
           (APP_NEXT_SEARCH_OTHER == ctx.next));

    /* The search finds it: hand back. */
    step(&ctx);
    g_b_black = true;
    step(&ctx);
    assert(g_b_finished && !g_b_aborted && !g_b_active);
    assert(ROBOT_AVOID_STATUS_COMPLETE == g_status);
    assert((0 == g_left) && (0 == g_right));
    assert(APP_ULTRASONIC_ANGLE_DEG == g_angle);
    assert(fabsf(ctx.heading_deg) <= APP_SEARCH_SWEEP_DEG + 15.0f);
}

/* A front sensor on black when the last turn ends: hand back at once. */
static void test_on_line_after_turn (void)
{
    app_avoid_ctx_t ctx = start_return();

    app_start_pivot(&ctx, g_now, 0.0f, APP_NEXT_FINISH);
    g_b_black = true;
    while (APP_AVOID_PIVOT == ctx.state)
    {
        step(&ctx);
    }
    assert(g_b_finished && (ROBOT_AVOID_STATUS_COMPLETE == g_status));
}

static void test_branch_ignored (void)
{
    app_avoid_ctx_t ctx = start_return();

    drive_to(&ctx, APP_SIDESTEP_MM - APP_RETURN_EARLY_MM + 50.0f);
    g_b_black = true;
    step(&ctx);
    drive_to(&ctx, ctx.black_mm + APP_RETURN_CROSS_MAX_MM + 10.0f);
    step(&ctx);
    assert((APP_LEG_RETURN == ctx.leg) && ctx.b_black_long);
    g_b_black = false;
    step(&ctx);
    assert((APP_LEG_RETURN == ctx.leg) && !ctx.b_black);

    /* The real crossing after it still counts. */
    g_b_black = true;
    step(&ctx);
    g_b_black = false;
    step(&ctx);
    assert(APP_LEG_ALIGN == ctx.leg);
}

static void test_line_not_found (void)
{
    app_avoid_ctx_t ctx = start_return();

    drive_to(&ctx, APP_SIDESTEP_MM + APP_RETURN_EXTRA_MM + 1.0f);
    step(&ctx);
    assert(g_b_aborted && !g_b_finished && (0 == g_left));
    assert(ROBOT_AVOID_STATUS_TIMEOUT == g_status);
}

static void test_still_beside (void)
{
    app_avoid_ctx_t ctx = start_return();

    for (uint8_t retry = 1u; retry <= APP_PASS_MAX_RETRIES; retry++)
    {
        g_front_cm = 10u;
        step(&ctx);
        assert(g_b_active && (retry == ctx.pass_retries));
        assert((APP_AVOID_PIVOT == ctx.state) &&
               (APP_NEXT_PASS_MORE == ctx.next) &&
               test_near(0.0f, ctx.pivot_target_deg));
        g_front_cm = BUDDY5_NO_ECHO;
        while (APP_AVOID_PIVOT == ctx.state)
        {
            step(&ctx);
        }
        assert((APP_LEG_PASS_MORE == ctx.leg) &&
               test_near(APP_PASS_RETRY_MM, ctx.leg_target_mm));
        drive_to(&ctx, APP_PASS_RETRY_MM);
        step(&ctx);
        assert((APP_AVOID_PIVOT == ctx.state) &&
               (APP_NEXT_RETURN == ctx.next) &&
               test_near(90.0f, ctx.pivot_target_deg));
        while (APP_AVOID_PIVOT == ctx.state)
        {
            step(&ctx);
        }
        assert(APP_LEG_RETURN == ctx.leg);
    }
    g_front_cm = 10u;
    step(&ctx);
    assert(g_b_aborted && (ROBOT_AVOID_STATUS_BLOCKED == g_status));
}

static void test_search (void)
{
    app_avoid_ctx_t ctx = start_return();

    /* The last pivot never touched the line: search toward it (+35, the way
     * the rover was going) first, then the other way, then stop. */
    app_start_pivot(&ctx, g_now, 0.0f, APP_NEXT_FINISH);
    while (APP_AVOID_PIVOT == ctx.state && (APP_NEXT_FINISH == ctx.next))
    {
        step(&ctx);
    }
    assert((APP_NEXT_SEARCH_OTHER == ctx.next) &&
           test_near(APP_SEARCH_SWEEP_DEG, ctx.pivot_target_deg));
    while (APP_AVOID_PIVOT == ctx.state && (APP_NEXT_SEARCH_OTHER == ctx.next))
    {
        step(&ctx);
    }
    assert((APP_NEXT_SEARCH_FAIL == ctx.next) &&
           test_near(-APP_SEARCH_SWEEP_DEG, ctx.pivot_target_deg));
    while (APP_AVOID_PIVOT == ctx.state)
    {
        step(&ctx);
    }
    assert(g_b_aborted && (ROBOT_AVOID_STATUS_TIMEOUT == g_status));

    /* Found during the search: hands back at once. */
    ctx = start_return();
    app_start_pivot(&ctx, g_now, 0.0f, APP_NEXT_FINISH);
    while (APP_AVOID_PIVOT == ctx.state && (APP_NEXT_FINISH == ctx.next))
    {
        step(&ctx);
    }
    step(&ctx);
    g_b_black = true;
    step(&ctx);
    assert(g_b_finished && (ROBOT_AVOID_STATUS_COMPLETE == g_status));
}

static void test_encoder_heading (void)
{
    app_avoid_ctx_t ctx = {0};

    g_motion.left_distance_mm  = 0.0f;
    g_motion.right_distance_mm = 0.0f;
    /* Pivot right: 200 mm wheel difference on 150 mm, less scrub. */
    g_motion.left_distance_mm  = 100.0f;
    g_motion.right_distance_mm = -100.0f;
    app_heading_update(&ctx);
    assert(fabsf(ctx.heading_deg - (200.0f / 150.0f * APP_DEG_PER_RAD *
                                    APP_PIVOT_WHEEL_SLIP)) < 0.01f);
    /* Driving forward with the right wheel 10 mm behind: no scrub. */
    ctx.heading_deg = 0.0f;
    g_motion.left_distance_mm += 100.0f;
    g_motion.right_distance_mm += 90.0f;
    app_heading_update(&ctx);
    assert(fabsf(ctx.heading_deg - (10.0f / 150.0f * APP_DEG_PER_RAD)) < 0.01f);
}

int main (void)
{
    test_crossing();
    test_on_line_after_turn();
    test_branch_ignored();
    test_line_not_found();
    test_still_beside();
    test_search();
    test_encoder_heading();
    puts("PASS: avoidance back to the line: early black ignored, a crossing "
         "aligns, the last pivot hands back only with a sensor on the line, "
         "else searches at once; a branch is ignored; no line "
         "stops; the box ahead going back -> drive further, at most 3 times; "
         "search both ways after the last pivot; encoder heading with pivot "
         "scrub");
    return 0;
}

/*** end of file ***/

