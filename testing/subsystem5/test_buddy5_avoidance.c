/* Buddy 5 obstacle avoidance: the whole route, end to end.
 *
 * The real state machine (buddy5_avoidance.c, included below) runs one
 * 100 ms step at a time against a small simulated world: two wheels, a
 * compass, the scanning ultrasonic sensor and rectangular obstacles.  The
 * line the rover follows is the x axis; y grows to the rover's right, and so
 * does the heading (as on the compass).  Distances are millimetres.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
int tk_dly_tsk (unsigned int delay_ms);
#include "buddy5_avoidance.c"

#define SIM_STEP_MS       (100u)
#define SIM_MAX_STEPS     (600u)
#define SIM_TRACK_MM      (150.0f)
#define SIM_PWM_REF       (9000.0f) /* this PWM drives a wheel ... */
#define SIM_MM_PER_STEP   (25.0f)   /* ... this far per step (250 mm/s) */
#define SIM_APPROACH_MM   (20.0f)   /* line following before the obstacle */
#define SIM_REVERSE_MM    (20.0f)   /* a backward move, per step */
#define SIM_LINE_HALF_MM  (15.0f)   /* front sensors see the line */
#define SIM_SONAR_MAX_MM  (4000.0f) /* farther is "no echo" */
#define SIM_MM_PER_CM     (10.0f)
#define SIM_START_X10     (900) /* compass heading along the line */
#define SIM_X10_PER_RAD   (572.957795f)
#define SIM_RAD_PER_DEG   (0.0174532925f)
#define SIM_FULL_TURN_X10 (3600.0f)
#define SIM_MAX_BOXES     (3u)
#define SIM_MAX_REVERSES  (8u)
#define SIM_TINY          (0.000001f)

/** One rectangular obstacle, sides along the axes. */
typedef struct
{
    float32_t x_min;
    float32_t x_max;
    float32_t y_min;
    float32_t y_max;
} sim_box_t;

/** Fault injected once the route reaches a given point. */
typedef enum
{
    SIM_INJECT_NONE = 0,
    SIM_INJECT_IMU_ON_DIAGONAL,
    SIM_INJECT_ENCODER_ON_STRAIGHT,
    SIM_INJECT_STUCK_ON_ARC
} sim_inject_t;

static sim_box_t             g_boxes[SIM_MAX_BOXES];
static uint32_t              g_box_count;
static float32_t             g_pos_x;
static float32_t             g_pos_y;
static float32_t             g_heading_rad;
static uint32_t              g_now;
static int16_t               g_left_pwm;
static int16_t               g_right_pwm;
static float32_t             g_reverse_left_mm;
static float32_t             g_reverses_mm[SIM_MAX_REVERSES];
static uint32_t              g_reverse_count;
static uint8_t               g_servo_deg;
static uint32_t              g_status_seen;
static sim_inject_t          g_inject;
static buddy2_motion_state_t g_motion;
static bool                  g_b_following;
static bool                  g_b_active;
static bool                  g_b_finished;
static bool                  g_b_aborted;
static bool                  g_b_decision_left;
static bool                  g_b_line_seen;
static bool                  g_b_imu_lost;
static bool                  g_b_stuck;

/* ---------------- Simulated sensors ---------------- */

/* Distance along a ray to one axis-aligned slab, narrowing [*p_near, *p_far].
 * Returns false when the ray misses it. */
static bool sim_slab (float32_t start, float32_t step, float32_t low,
                      float32_t high, float32_t * p_near, float32_t * p_far)
{
    bool      b_hit  = true;
    float32_t t_low  = 0.0f;
    float32_t t_high = 0.0f;
    float32_t swap   = 0.0f;

    if (fabsf(step) < SIM_TINY)
    {
        b_hit = (start >= low) && (start <= high);
    }
    else
    {
        t_low  = (low - start) / step;
        t_high = (high - start) / step;
        if (t_low > t_high)
        {
            swap   = t_low;
            t_low  = t_high;
            t_high = swap;
        }
        *p_near = (t_low > *p_near) ? t_low : *p_near;
        *p_far  = (t_high < *p_far) ? t_high : *p_far;
        b_hit   = (*p_near <= *p_far);
    }

    return b_hit;
}

/* What the sonar reads at its present servo angle (90 = ahead, 45 = rover
 * right), in mm; above SIM_SONAR_MAX_MM means no echo. */
static float32_t sim_sonar_mm (void)
{
    float32_t angle =
        g_heading_rad + ((90.0f - (float32_t) g_servo_deg) * SIM_RAD_PER_DEG);
    float32_t step_x  = cosf(angle);
    float32_t step_y  = sinf(angle);
    float32_t nearest = SIM_SONAR_MAX_MM + 1.0f;

    for (uint32_t index = 0u; index < g_box_count; index++)
    {
        float32_t t_near = 0.0f;
        float32_t t_far  = SIM_SONAR_MAX_MM;

        if (sim_slab(g_pos_x, step_x, g_boxes[index].x_min,
                     g_boxes[index].x_max, &t_near, &t_far) &&
            sim_slab(g_pos_y, step_y, g_boxes[index].y_min,
                     g_boxes[index].y_max, &t_near, &t_far) &&
            (t_near < nearest))
        {
            nearest = t_near;
        }
    }

    return nearest;
}

/* ---------------- Stubs for the rest of the robot ---------------- */

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
    float32_t          heading =
        (float32_t) SIM_START_X10 + (g_heading_rad * SIM_X10_PER_RAD);

    while (heading < 0.0f)
    {
        heading += SIM_FULL_TURN_X10;
    }
    while (heading >= SIM_FULL_TURN_X10)
    {
        heading -= SIM_FULL_TURN_X10;
    }
    *p_state                 = empty;
    p_state->heading_deg_x10 = (int16_t) heading;
    p_state->mag_ok          = g_b_imu_lost ? 0u : 1u;
}
void buddy2_get_motion_state (buddy2_motion_state_t * p_state)
{
    *p_state = g_motion;
}
void buddy2_stop (void)
{
    g_left_pwm        = 0;
    g_right_pwm       = 0;
    g_reverse_left_mm = 0.0f;
}
bool buddy2_move_backward_at_speed (float32_t distance_mm, int16_t max_pwm,
                                    uint8_t speed_pct)
{
    (void) max_pwm;
    (void) speed_pct;
    assert(g_reverse_count < SIM_MAX_REVERSES);
    g_reverses_mm[g_reverse_count] = distance_mm;
    g_reverse_count++;
    g_left_pwm        = 0;
    g_right_pwm       = 0;
    g_reverse_left_mm = distance_mm;
    return true;
}
bool buddy2_set_motor_pwm_at_speed (int16_t left, int16_t right,
                                    uint8_t speed_pct)
{
    /* Every avoidance move runs at the fixed avoidance speed. */
    assert(APP_AVOID_SPEED_PCT == speed_pct);
    g_left_pwm  = left;
    g_right_pwm = right;
    return true;
}
bool buddy2_motion_active (void)
{
    return (g_reverse_left_mm > 0.0f);
}
void buddy5_set_servo_angle (uint8_t angle_deg)
{
    g_servo_deg = angle_deg;
}
uint32_t buddy5_measure_cm (uint32_t now_ms)
{
    float32_t distance_mm = sim_sonar_mm();

    (void) now_ms;
    return (distance_mm > SIM_SONAR_MAX_MM)
               ? BUDDY5_NO_ECHO
               : (uint32_t) (distance_mm / SIM_MM_PER_CM);
}
uint32_t buddy5_sample_with_result_cm (buddy5_ping_result_t * p_result)
{
    uint32_t distance_cm = buddy5_measure_cm(g_now);

    *p_result = (BUDDY5_NO_ECHO == distance_cm) ? BUDDY5_PING_TIMEOUT_FALL
                                                : BUDDY5_PING_VALID;
    return distance_cm;
}
bool robot_controller_begin_obstacle_avoidance (void)
{
    bool b_begun = g_b_following;

    if (b_begun)
    {
        g_b_following = false;
        g_b_active    = true;
    }
    return b_begun;
}
bool robot_controller_obstacle_avoidance_active (void)
{
    return g_b_active;
}
void robot_controller_finish_obstacle_avoidance (void)
{
    g_b_active    = false;
    g_b_finished  = true;
    g_b_following = true; /* The run came from line following. */
}
void robot_controller_abort_obstacle_avoidance (void)
{
    g_b_active  = false;
    g_b_aborted = true;
    buddy2_stop();
}
bool buddy3_take_line_seen_event (void)
{
    bool b_seen = g_b_line_seen;

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
    g_b_decision_left = b_left;
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
    g_status_seen |= (1u << status);
}
/* Last obstacle size reported to the dashboard. */
static uint32_t g_obstacle_face_mm;
static uint32_t g_obstacle_width_mm;
static bool     g_b_obstacle_partial;
void robot_telemetry_set_obstacle_size (uint32_t face_mm, uint32_t width_mm,
                                        bool b_partial)
{
    g_obstacle_face_mm   = face_mm;
    g_obstacle_width_mm  = width_mm;
    g_b_obstacle_partial = b_partial;
}
void robot_telemetry_set_avoidance_forward_progress (uint16_t progress_mm,
                                                     uint16_t target_mm)
{
    (void) progress_mm;
    (void) target_mm;
}

/* ---------------- The simulated world ---------------- */

/* Move the rover for one step: a backward move, the motor PWMs, or (before
 * avoidance starts) line following straight ahead. */
static void sim_move (void)
{
    float32_t left_mm  = 0.0f;
    float32_t right_mm = 0.0f;
    float32_t turn_rad = 0.0f;
    float32_t middle   = 0.0f;

    if (g_reverse_left_mm > 0.0f)
    {
        left_mm  = -SIM_REVERSE_MM;
        right_mm = -SIM_REVERSE_MM;
        g_reverse_left_mm -= SIM_REVERSE_MM;
    }
    else if (g_b_following)
    {
        left_mm  = SIM_APPROACH_MM;
        right_mm = SIM_APPROACH_MM;
    }
    else if (!g_b_stuck)
    {
        left_mm  = ((float32_t) g_left_pwm * SIM_MM_PER_STEP) / SIM_PWM_REF;
        right_mm = ((float32_t) g_right_pwm * SIM_MM_PER_STEP) / SIM_PWM_REF;
    }
    else
    {
        /* Pushing against something: the wheels do not turn. */
    }

    turn_rad = (left_mm - right_mm) / SIM_TRACK_MM;
    middle   = g_heading_rad + (turn_rad * 0.5f);
    g_pos_x += ((left_mm + right_mm) * 0.5f) * cosf(middle);
    g_pos_y += ((left_mm + right_mm) * 0.5f) * sinf(middle);
    g_heading_rad += turn_rad;
    g_motion.left_distance_mm += left_mm;
    g_motion.right_distance_mm += right_mm;
    if (fabsf(g_pos_y) < SIM_LINE_HALF_MM)
    {
        g_b_line_seen = true;
    }
    for (uint32_t index = 0u; index < g_box_count; index++)
    {
        /* The rover's centre never drives into an obstacle. */
        assert(!((g_pos_x > g_boxes[index].x_min) &&
                 (g_pos_x < g_boxes[index].x_max) &&
                 (g_pos_y > g_boxes[index].y_min) &&
                 (g_pos_y < g_boxes[index].y_max)));
    }
}

/* Inject the scenario's fault once the route reaches its point. */
static void sim_inject (const app_avoid_ctx_t * p_ctx)
{
    bool b_forward = (APP_AVOID_WAIT_FORWARD_DISTANCE == p_ctx->state);

    if ((SIM_INJECT_IMU_ON_DIAGONAL == g_inject) && b_forward &&
        (APP_AVOID_FORWARD_OUTWARD_DIAGONAL == p_ctx->forward_leg))
    {
        g_b_imu_lost = true;
    }
    if ((SIM_INJECT_ENCODER_ON_STRAIGHT == g_inject) && b_forward &&
        (APP_AVOID_FORWARD_STRAIGHT == p_ctx->forward_leg))
    {
        g_motion.left_encoder_fault = true;
    }
    if ((SIM_INJECT_STUCK_ON_ARC == g_inject) &&
        (APP_AVOID_WAIT_TURN_ARC == p_ctx->state))
    {
        g_b_stuck = true;
    }
}

/* A fresh world with the given obstacles: the rover line follows from the
 * origin, and the context starts the way buddy5_avoidance_task() sets it. */
static app_avoid_ctx_t sim_start (const sim_box_t * p_boxes, uint32_t count,
                                  sim_inject_t inject)
{
    app_avoid_ctx_t       ctx    = {0};
    buddy2_motion_state_t motion = {0};

    assert(count <= SIM_MAX_BOXES);
    memcpy(g_boxes, p_boxes, count * sizeof(sim_box_t));
    g_box_count       = count;
    g_pos_x           = 0.0f;
    g_pos_y           = 0.0f;
    g_heading_rad     = 0.0f;
    g_now             = 1000u;
    g_left_pwm        = 0;
    g_right_pwm       = 0;
    g_reverse_left_mm = 0.0f;
    g_reverse_count   = 0u;
    g_servo_deg       = APP_ULTRASONIC_ANGLE_DEG;
    g_status_seen     = 0u;
    g_inject          = inject;
    g_motion          = motion;
    g_b_following     = true;
    g_b_active        = false;
    g_b_finished      = false;
    g_b_aborted       = false;
    g_b_decision_left = false;
    g_b_line_seen     = false;
    g_b_imu_lost      = false;
    g_b_stuck         = false;

    ctx.state                       = APP_AVOID_WATCH_FORWARD;
    ctx.outward_arc_target_x10      = APP_TURN_TARGET_X10;
    ctx.return_arc_target_x10       = APP_TURN_TARGET_X10;
    ctx.forward_target_mm           = APP_AVOID_DIAGONAL_MIN_MM;
    ctx.planned_diagonal_mm         = APP_AVOID_DIAGONAL_MIN_MM;
    ctx.paired_diagonal_distance_mm = APP_AVOID_DIAGONAL_MIN_MM;
    ctx.forward_leg                 = APP_AVOID_FORWARD_OUTWARD_DIAGONAL;
    app_sweep_point_reset(&ctx.sweep_point);

    return ctx;
}

/* Run 100 ms steps until avoidance hands back to line following or stops. */
static void sim_run (app_avoid_ctx_t * p_ctx)
{
    uint32_t steps = 0u;

    for (steps = 0u; (steps < SIM_MAX_STEPS) && !g_b_finished && !g_b_aborted;
         steps++)
    {
        sim_inject(p_ctx);
        app_step(p_ctx, g_now);
        sim_move();
        g_now += SIM_STEP_MS;
    }
    assert(steps < SIM_MAX_STEPS);
    assert(APP_AVOID_WATCH_FORWARD == p_ctx->state);
    assert(APP_ULTRASONIC_ANGLE_DEG == g_servo_deg);
}

static bool status_seen (uint8_t status)
{
    return (0u != (g_status_seen & (1u << status)));
}

/* ---------------- Scenarios ---------------- */

/* An obstacle reaching further right than left: back away 80 mm, sweep, go
 * round on the left (30 degrees), recover, pass it and turn back in until
 * the front sensors find the line again. */
static void test_left_route (void)
{
    static const sim_box_t box = {500.0f, 600.0f, -40.0f, 150.0f};
    app_avoid_ctx_t        ctx = sim_start(&box, 1u, SIM_INJECT_NONE);

    sim_run(&ctx);
    assert(g_b_finished && !g_b_aborted && g_b_following);
    assert(g_b_decision_left);
    assert((1u == g_reverse_count) &&
           (APP_AVOID_REVERSE_DISTANCE_MM == g_reverses_mm[0]));
    assert(status_seen(ROBOT_AVOID_STATUS_SCANNING) &&
           status_seen(ROBOT_AVOID_STATUS_TURNING) &&
           status_seen(ROBOT_AVOID_STATUS_FORWARD) &&
           status_seen(ROBOT_AVOID_STATUS_RECOVERING) &&
           status_seen(ROBOT_AVOID_STATUS_COMPLETE));
    assert(g_pos_x > box.x_max); /* Past the obstacle. */
    /* The width (190 mm) is reported with both edges found.  The simulated
     * sonar is a thin ray, so each edge can be up to one 15-degree sweep
     * step inside the true edge (the real beam is about one step wide). */
    assert((g_obstacle_face_mm > 0u) && !g_b_obstacle_partial);
    assert((g_obstacle_width_mm > 76u) && (g_obstacle_width_mm <= 190u));
    /* Back on the line: crossed it within the last step. */
    assert(fabsf(g_pos_y) < (SIM_LINE_HALF_MM + SIM_MM_PER_STEP));
}

/* The mirror image: round on the right, which leads with the barcode arm,
 * so the rover backs away 100 mm more and turns out 60 degrees. */
static void test_right_route (void)
{
    static const sim_box_t box = {500.0f, 600.0f, -150.0f, 40.0f};
    app_avoid_ctx_t        ctx = sim_start(&box, 1u, SIM_INJECT_NONE);

    sim_run(&ctx);
    assert(g_b_finished && !g_b_aborted && !g_b_decision_left);
    assert((2u == g_reverse_count) &&
           (APP_AVOID_REVERSE_DISTANCE_MM == g_reverses_mm[0]) &&
           (APP_AVOID_ARM_SIDE_EXTRA_REVERSE_MM == g_reverses_mm[1]));
    assert(ctx.outward_arc_target_x10 >= APP_ARM_SIDE_TURN_TARGET_X10);
    assert(g_pos_x > box.x_max);
    assert(fabsf(g_pos_y) < (SIM_LINE_HALF_MM + SIM_MM_PER_STEP));
}

/* Walls close on both sides of a wide obstacle: no edge is found and the
 * fixed fallback route has under 20 cm each side, so the rover stays put. */
static void test_blocked (void)
{
    static const sim_box_t boxes[3] = {{500.0f, 600.0f, -2000.0f, 2000.0f},
                                       {0.0f, 2000.0f, -400.0f, -120.0f},
                                       {0.0f, 2000.0f, 120.0f, 400.0f}};
    app_avoid_ctx_t        ctx      = sim_start(boxes, 3u, SIM_INJECT_NONE);

    sim_run(&ctx);
    assert(g_b_aborted && !g_b_finished);
    assert(status_seen(ROBOT_AVOID_STATUS_BLOCKED));
    assert(!status_seen(ROBOT_AVOID_STATUS_TURNING));
    assert(fabsf(g_heading_rad) < SIM_TINY); /* It never turned. */
}

/* Faults part way round: each stops the rover instead of guessing. */
static void test_faults (void)
{
    static const sim_box_t box = {500.0f, 600.0f, -40.0f, 150.0f};
    app_avoid_ctx_t        ctx = {0};

    /* Compass lost on the outward diagonal. */
    ctx = sim_start(&box, 1u, SIM_INJECT_IMU_ON_DIAGONAL);
    sim_run(&ctx);
    assert(g_b_aborted && !g_b_finished);
    assert(status_seen(ROBOT_AVOID_STATUS_IMU_LOST));

    /* An encoder fault on the straight leg (after its first second). */
    ctx = sim_start(&box, 1u, SIM_INJECT_ENCODER_ON_STRAIGHT);
    sim_run(&ctx);
    assert(g_b_aborted && !g_b_finished);
    assert(status_seen(ROBOT_AVOID_STATUS_BLOCKED));

    /* Wheels blocked in the turn-out: back away and sweep again, three
     * times, then stop. */
    ctx = sim_start(&box, 1u, SIM_INJECT_STUCK_ON_ARC);
    sim_run(&ctx);
    assert(g_b_aborted && !g_b_finished);
    assert(APP_AVOID_MAX_STALL_RETRIES == ctx.stall_retries);
    assert(status_seen(ROBOT_AVOID_STATUS_BLOCKED));
}

/* Heading helpers: shortest change through the 0/360 wrap, and which way
 * each scan angle looks. */
static void test_helpers (void)
{
    assert(20 == app_heading_change_x10(3590, 10));
    assert(-20 == app_heading_change_x10(10, 3590));
    assert(0 == app_heading_change_x10(3600, 0));
    assert(!app_scan_angle_is_left(45));
    assert(app_scan_angle_is_left(135));
    /* A left turn watches the rover's right (45); a right turn its left. */
    assert(45 == app_outward_monitor_angle(true));
    assert(135 == app_outward_monitor_angle(false));
}

int main (void)
{
    test_helpers();
    test_left_route();
    test_right_route();
    test_blocked();
    test_faults();
    puts("PASS: avoidance route end to end in a simulated world: sweep, "
         "left (30 deg) and right (extra 100 mm reverse, 60 deg) routes back "
         "to the line, blocked fallback, IMU loss, encoder fault and three "
         "stall rescans stop");
    return 0;
}

/*** end of file ***/

