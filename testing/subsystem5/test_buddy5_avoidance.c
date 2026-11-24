/* Buddy 5 obstacle avoidance: the whole route, end to end.
 *
 * The real state machine (buddy5_avoidance.c, included below) runs one
 * 100 ms step at a time against a small simulated world: two wheels, a
 * compass, the scanning ultrasonic sensor, the two front IR sensors and
 * rectangular obstacles.  The line the rover follows is the x axis; y grows
 * to the rover's right, and so does the heading (as on the compass).
 * Distances are millimetres.
 *
 * The rover's position is its axle centre.  The sonar and the front IR
 * sensors are 100 mm ahead of it, so (as on the track) the sensors reach
 * the line beside an obstacle that sits on it.  The sonar beam is three rays
 * 7 degrees apart, so it sees corners a little early, like the real one.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
int tk_dly_tsk (unsigned int delay_ms);
#include "buddy5_avoidance.c"

#define SIM_STEP_MS        (100u)
#define SIM_MAX_STEPS      (900u)
#define SIM_TRACK_MM       (150.0f)
/* 5400 effective PWM (9000 at 60 %) drives a wheel 13 mm per 100 ms, as
 * measured on the rover's straight legs. */
#define SIM_MM_PER_PWM     (13.0f / 5400.0f)
/* On the spot the wheels scrub: the rover turns this fraction of what the
 * wheel travel says (the compass sees the real turn). */
#define SIM_PIVOT_SCRUB    (0.8f)
#define SIM_APPROACH_MM    (20.0f) /* line following before the obstacle */
#define SIM_REVERSE_MM     (20.0f) /* a backward move, per step */
#define SIM_LINE_HALF_MM   (9.0f)  /* 18 mm line */
#define SIM_SENSOR_FWD_MM  (100.0f)
#define SIM_SENSOR_SIDE_MM (15.0f)
#define SIM_SONAR_MAX_MM   (4000.0f) /* farther is "no echo" */
#define SIM_BEAM_HALF_RAD  (0.12f)   /* about 7 degrees */
#define SIM_MM_PER_CM      (10.0f)
#define SIM_START_X10      (900) /* compass heading along the line */
#define SIM_X10_PER_RAD    (572.957795f)
#define SIM_RAD_PER_DEG    (0.0174532925f)
#define SIM_FULL_TURN_X10  (3600.0f)
#define SIM_MAX_BOXES      (3u)
#define SIM_MAX_REVERSES   (8u)
#define SIM_TINY           (0.000001f)
/* Rover body around the axle centre: front, rear, left and right reach
 * (the barcode arm makes the left wider). */
#define SIM_BODY_FRONT_MM (120.0f)
#define SIM_BODY_REAR_MM  (80.0f)
#define SIM_BODY_LEFT_MM  (75.0f)
#define SIM_BODY_RIGHT_MM (55.0f)

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
    SIM_INJECT_IMU_BLIP_ON_PASS,   /* 300 ms drop-out: rides through it  */
    SIM_INJECT_IMU_LOST_ON_PASS,   /* gone for good: stops               */
    SIM_INJECT_ENCODER_ON_SIDESTEP,
    SIM_INJECT_STUCK_ON_PIVOT,
    SIM_INJECT_SIDE_SONAR_DEAF,    /* never sees the box while passing   */
    SIM_INJECT_IR_DEAF             /* front IR never sees the line       */
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
static uint32_t              g_inject_until_ms;
static buddy2_motion_state_t g_motion;
static bool                  g_b_following;
static bool                  g_b_active;
static bool                  g_b_finished;
static bool                  g_b_aborted;
static bool                  g_b_decision_left;
static bool                  g_b_line_seen;
static bool                  g_b_imu_lost;
static bool                  g_b_stuck;
static bool                  g_b_side_deaf;

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

/* Nearest box along one ray from the sonar, in mm. */
static float32_t sim_ray_mm (float32_t angle)
{
    float32_t sonar_x = g_pos_x + (SIM_SENSOR_FWD_MM * cosf(g_heading_rad));
    float32_t sonar_y = g_pos_y + (SIM_SENSOR_FWD_MM * sinf(g_heading_rad));
    float32_t step_x  = cosf(angle);
    float32_t step_y  = sinf(angle);
    float32_t nearest = SIM_SONAR_MAX_MM + 1.0f;

    for (uint32_t index = 0u; index < g_box_count; index++)
    {
        float32_t t_near = 0.0f;
        float32_t t_far  = SIM_SONAR_MAX_MM;

        if (sim_slab(sonar_x, step_x, g_boxes[index].x_min,
                     g_boxes[index].x_max, &t_near, &t_far) &&
            sim_slab(sonar_y, step_y, g_boxes[index].y_min,
                     g_boxes[index].y_max, &t_near, &t_far) &&
            (t_near < nearest))
        {
            nearest = t_near;
        }
    }

    return nearest;
}

/* What the sonar reads at its present servo angle (90 = ahead, 45 = rover
 * right): the nearest of three rays across the beam. */
static float32_t sim_sonar_mm (void)
{
    float32_t angle =
        g_heading_rad + ((90.0f - (float32_t) g_servo_deg) * SIM_RAD_PER_DEG);
    float32_t nearest = sim_ray_mm(angle);
    float32_t ray     = sim_ray_mm(angle - SIM_BEAM_HALF_RAD);

    nearest = (ray < nearest) ? ray : nearest;
    ray     = sim_ray_mm(angle + SIM_BEAM_HALF_RAD);
    nearest = (ray < nearest) ? ray : nearest;

    return nearest;
}

/* A point of the rover body, given relative to the axle (forward, right). */
static void sim_body_point (float32_t forward, float32_t right,
                            float32_t * p_x, float32_t * p_y)
{
    *p_x = g_pos_x + (forward * cosf(g_heading_rad)) -
           (right * sinf(g_heading_rad));
    *p_y = g_pos_y + (forward * sinf(g_heading_rad)) +
           (right * cosf(g_heading_rad));
}

/* No part of the rover body is inside an obstacle. */
static void sim_assert_clear (void)
{
    static const float32_t forward[3] = {SIM_BODY_FRONT_MM, 0.0f,
                                         -SIM_BODY_REAR_MM};
    static const float32_t right[3]   = {-SIM_BODY_LEFT_MM, 0.0f,
                                         SIM_BODY_RIGHT_MM};

    for (uint32_t f = 0u; f < 3u; f++)
    {
        for (uint32_t r = 0u; r < 3u; r++)
        {
            float32_t x = 0.0f;
            float32_t y = 0.0f;

            sim_body_point(forward[f], right[r], &x, &y);
            for (uint32_t index = 0u; index < g_box_count; index++)
            {
                assert(!((x > g_boxes[index].x_min) &&
                         (x < g_boxes[index].x_max) &&
                         (y > g_boxes[index].y_min) &&
                         (y < g_boxes[index].y_max)));
            }
        }
    }
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
void buddy2_get_calibration (buddy2_calibration_t * p_calibration)
{
    buddy2_calibration_t empty = {0};

    *p_calibration                = empty;
    p_calibration->track_width_mm = SIM_TRACK_MM;
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
    g_left_pwm  = (int16_t) (((int32_t) left * speed_pct) / 100);
    g_right_pwm = (int16_t) (((int32_t) right * speed_pct) / 100);
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
    bool b_side = (APP_ULTRASONIC_ANGLE_DEG != g_servo_deg) &&
                  ((g_servo_deg < APP_SWEEP_FIRST_DEG) ||
                   (g_servo_deg > app_sweep_angle(APP_SWEEP_POINTS - 1u)));

    if (g_b_side_deaf && b_side)
    {
        distance_cm = BUDDY5_NO_ECHO;
    }
    *p_result = (BUDDY5_NO_ECHO == distance_cm) ? BUDDY5_PING_TIMEOUT_RISE
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
/* Front sensors over the line right now (as Buddy 3 reports them). */
void buddy3_get_colors (bool * p_left_black, bool * p_right_black,
                        bool * p_side_black)
{
    float32_t x = 0.0f;
    float32_t y = 0.0f;

    sim_body_point(SIM_SENSOR_FWD_MM, -SIM_SENSOR_SIDE_MM, &x, &y);
    *p_left_black = (fabsf(y) < SIM_LINE_HALF_MM) &&
                    (SIM_INJECT_IR_DEAF != g_inject);
    sim_body_point(SIM_SENSOR_FWD_MM, SIM_SENSOR_SIDE_MM, &x, &y);
    *p_right_black = (fabsf(y) < SIM_LINE_HALF_MM) &&
                     (SIM_INJECT_IR_DEAF != g_inject);
    *p_side_black = false;
    (void) x;
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
    float32_t real_l   = 0.0f;
    float32_t real_r   = 0.0f;
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
        left_mm  = (float32_t) g_left_pwm * SIM_MM_PER_PWM;
        right_mm = (float32_t) g_right_pwm * SIM_MM_PER_PWM;
    }
    else
    {
        /* Pushing against something: the wheels do not turn. */
    }

    /* Wheels turning opposite ways scrub: the encoders count the wheel
     * travel, the rover really moves less. */
    real_l = left_mm;
    real_r = right_mm;
    if ((left_mm * right_mm) < 0.0f)
    {
        real_l *= SIM_PIVOT_SCRUB;
        real_r *= SIM_PIVOT_SCRUB;
    }
    /* Right wheel ahead turns left: the compass heading falls. */
    turn_rad = (real_l - real_r) / SIM_TRACK_MM;
    middle   = g_heading_rad + (turn_rad * 0.5f);
    g_pos_x += ((real_l + real_r) * 0.5f) * cosf(middle);
    g_pos_y += ((real_l + real_r) * 0.5f) * sinf(middle);
    g_heading_rad += turn_rad;
    g_motion.left_distance_mm += left_mm;
    g_motion.right_distance_mm += right_mm;

    /* Either front sensor over the line latches Buddy 3's event. */
    for (int32_t side = -1; side <= 1; side += 2)
    {
        float32_t x = 0.0f;
        float32_t y = 0.0f;

        sim_body_point(SIM_SENSOR_FWD_MM, (float32_t) side * SIM_SENSOR_SIDE_MM,
                       &x, &y);
        (void) x;
        if ((fabsf(y) < SIM_LINE_HALF_MM) && (SIM_INJECT_IR_DEAF != g_inject))
        {
            g_b_line_seen = true;
        }
    }
    sim_assert_clear();
}

/* Inject the scenario's fault once the route reaches its point. */
static void sim_inject (const app_avoid_ctx_t * p_ctx)
{
    bool b_leg  = (APP_AVOID_WAIT_LEG == p_ctx->state);
    bool b_pass = b_leg && (APP_LEG_PASS == p_ctx->leg);

    if ((SIM_INJECT_IMU_BLIP_ON_PASS == g_inject) && b_pass &&
        (0u == g_inject_until_ms))
    {
        g_b_imu_lost      = true;
        g_inject_until_ms = g_now + 300u;
    }
    if ((0u != g_inject_until_ms) && (g_now >= g_inject_until_ms))
    {
        g_b_imu_lost = false;
    }
    if ((SIM_INJECT_IMU_LOST_ON_PASS == g_inject) && b_pass)
    {
        g_b_imu_lost = true;
    }
    if ((SIM_INJECT_ENCODER_ON_SIDESTEP == g_inject) && b_leg &&
        (APP_LEG_SIDESTEP == p_ctx->leg))
    {
        g_motion.left_encoder_fault = true;
    }
    if ((SIM_INJECT_STUCK_ON_PIVOT == g_inject) &&
        (APP_AVOID_WAIT_PIVOT == p_ctx->state))
    {
        g_b_stuck = true;
    }
    g_b_side_deaf = (SIM_INJECT_SIDE_SONAR_DEAF == g_inject);
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
    g_inject_until_ms = 0u;
    g_motion          = motion;
    g_b_following     = true;
    g_b_active        = false;
    g_b_finished      = false;
    g_b_aborted       = false;
    g_b_decision_left = false;
    g_b_line_seen     = false;
    g_b_imu_lost      = false;
    g_b_stuck         = false;
    g_b_side_deaf     = false;

    ctx.state = APP_AVOID_WATCH_FORWARD;
    app_sweep_point_reset(&ctx.sweep_point);

    return ctx;
}

/* Run 100 ms steps until avoidance hands back to line following or stops. */
static void sim_run (app_avoid_ctx_t * p_ctx)
{
    uint32_t steps = 0u;

    bool     b_trace = (NULL != getenv("SIM_TRACE"));
    uint32_t last    = 0xFFFFFFFFu;

    for (steps = 0u; (steps < SIM_MAX_STEPS) && !g_b_finished && !g_b_aborted;
         steps++)
    {
        sim_inject(p_ctx);
        app_step(p_ctx, g_now);
        sim_move();
        g_now += SIM_STEP_MS;
        /* SIM_TRACE=1: print the pose at every change of state or leg. */
        if (b_trace && (((uint32_t) p_ctx->state * 16u + p_ctx->leg) != last))
        {
            last = (uint32_t) p_ctx->state * 16u + p_ctx->leg;
            printf("  t=%5u state=%u leg=%u x=%6.0f y=%5.0f hdg=%6.1f "
                   "servo=%u\n",
                   g_now, (unsigned) p_ctx->state, (unsigned) p_ctx->leg,
                   g_pos_x, g_pos_y, g_heading_rad / SIM_RAD_PER_DEG,
                   (unsigned) g_servo_deg);
            (void) fflush(stdout);
        }
    }
    if (b_trace)
    {
        printf("  end: x=%.0f y=%.0f hdg=%.1f finished=%d aborted=%d "
               "width=%u face=%u\n",
               g_pos_x, g_pos_y, g_heading_rad / SIM_RAD_PER_DEG,
               g_b_finished, g_b_aborted, g_obstacle_width_mm,
               g_obstacle_face_mm);
    }
    assert(steps < SIM_MAX_STEPS);
    assert(APP_AVOID_WATCH_FORWARD == p_ctx->state);
    assert(APP_ULTRASONIC_ANGLE_DEG == g_servo_deg);
}

static bool status_seen (uint8_t status)
{
    return (0u != (g_status_seen & (1u << status)));
}

/* Heading error from the line, degrees. */
static float32_t sim_heading_off_deg (void)
{
    return fabsf(g_heading_rad) / SIM_RAD_PER_DEG;
}

/* A finished run is past the box, back over the line and pointing along
 * it, and line following has taken over. */
static void assert_rejoined (const sim_box_t * p_box)
{
    float32_t x = 0.0f;
    float32_t y = 0.0f;

    assert(g_b_finished && !g_b_aborted && g_b_following);
    assert(status_seen(ROBOT_AVOID_STATUS_SCANNING) &&
           status_seen(ROBOT_AVOID_STATUS_TURNING) &&
           status_seen(ROBOT_AVOID_STATUS_FORWARD) &&
           status_seen(ROBOT_AVOID_STATUS_PASSING) &&
           status_seen(ROBOT_AVOID_STATUS_REJOINING) &&
           status_seen(ROBOT_AVOID_STATUS_COMPLETE));
    /* The rear of the rover is past the obstacle. */
    assert((g_pos_x - SIM_BODY_REAR_MM) > p_box->x_max);
    /* Axle within 25 mm of the line, heading within 10 degrees of it, and
     * the front sensors straddle it for line following. */
    assert(fabsf(g_pos_y) < 25.0f);
    assert(sim_heading_off_deg() < 10.0f);
    sim_body_point(SIM_SENSOR_FWD_MM, 0.0f, &x, &y);
    assert(fabsf(y) < 40.0f);
}

/* ---------------- Scenarios ---------------- */

/* A 200 x 150 mm box on the line, reaching 40 mm further right than left:
 * back away 80 mm, sweep, go round on the left (the narrower way), pass it
 * watching it with the side sonar, and come back onto the line beyond it.
 * Black right beside the box (the line itself) does not end the run. */
static void test_left_route (void)
{
    static const sim_box_t box = {500.0f, 650.0f, -80.0f, 120.0f};
    app_avoid_ctx_t        ctx = sim_start(&box, 1u, SIM_INJECT_NONE);

    sim_run(&ctx);
    assert_rejoined(&box);
    assert(g_b_decision_left);
    assert((1u == g_reverse_count) &&
           (APP_AVOID_REVERSE_DISTANCE_MM == g_reverses_mm[0]));
    /* The width (200 mm) is reported with both edges, within 30 mm. */
    assert((g_obstacle_face_mm > 0u) && !g_b_obstacle_partial);
    assert((g_obstacle_width_mm > 170u) && (g_obstacle_width_mm < 230u));
}

/* The mirror image: round on the right, which leads with the barcode arm,
 * so the rover backs away 100 mm more first. */
static void test_right_route (void)
{
    static const sim_box_t box = {500.0f, 650.0f, -120.0f, 80.0f};
    app_avoid_ctx_t        ctx = sim_start(&box, 1u, SIM_INJECT_NONE);

    sim_run(&ctx);
    assert_rejoined(&box);
    assert(!g_b_decision_left);
    assert((2u == g_reverse_count) &&
           (APP_AVOID_REVERSE_DISTANCE_MM == g_reverses_mm[0]) &&
           (APP_AVOID_ARM_SIDE_EXTRA_REVERSE_MM == g_reverses_mm[1]));
}

/* Other sizes: a narrow deep box, a wide shallow one, a long one. */
static void test_box_sizes (void)
{
    static const sim_box_t boxes[3] = {{500.0f, 750.0f, -50.0f, 60.0f},
                                       {500.0f, 580.0f, -150.0f, 140.0f},
                                       {500.0f, 900.0f, -100.0f, 100.0f}};
    app_avoid_ctx_t        ctx      = {0};

    for (uint32_t index = 0u; index < 3u; index++)
    {
        ctx = sim_start(&boxes[index], 1u, SIM_INJECT_NONE);
        sim_run(&ctx);
        assert_rejoined(&boxes[index]);
    }
    /* The 290 mm wide box is measured within 40 mm. */
    ctx = sim_start(&boxes[1], 1u, SIM_INJECT_NONE);
    sim_run(&ctx);
    assert((g_obstacle_width_mm > 250u) && (g_obstacle_width_mm < 330u));
}

/* Walls close on both sides of a wide obstacle: no edge is found and the
 * fixed fallback route has under 20 cm each side, so the rover stays put. */
static void test_blocked (void)
{
    static const sim_box_t boxes[3] = {{500.0f, 600.0f, -2000.0f, 2000.0f},
                                       {0.0f, 2000.0f, -400.0f, -140.0f},
                                       {0.0f, 2000.0f, 140.0f, 400.0f}};
    app_avoid_ctx_t        ctx      = sim_start(boxes, 3u, SIM_INJECT_NONE);

    sim_run(&ctx);
    assert(g_b_aborted && !g_b_finished);
    assert(status_seen(ROBOT_AVOID_STATUS_BLOCKED));
    assert(!status_seen(ROBOT_AVOID_STATUS_TURNING));
    assert(fabsf(g_heading_rad) < SIM_TINY); /* It never turned. */
}

/* Compass drop-outs, a side sonar that never sees the box, and faults. */
static void test_faults (void)
{
    static const sim_box_t box = {500.0f, 650.0f, -80.0f, 120.0f};
    app_avoid_ctx_t        ctx = {0};

    /* A 300 ms compass drop-out while passing: rides through it. */
    ctx = sim_start(&box, 1u, SIM_INJECT_IMU_BLIP_ON_PASS);
    sim_run(&ctx);
    assert_rejoined(&box);

    /* The side sonar never sees the box: it assumes a 200 mm deep obstacle
     * (this one is 150 mm) and still gets round. */
    ctx = sim_start(&box, 1u, SIM_INJECT_SIDE_SONAR_DEAF);
    sim_run(&ctx);
    assert_rejoined(&box);

    /* A box much deeper than the 350 mm assumed, and the side sonar never
     * sees it: turning back, the front sonar finds it still there, so the
     * rover drives further along and tries again until it is past. */
    {
        static const sim_box_t deep = {500.0f, 1150.0f, -80.0f, 120.0f};

        ctx = sim_start(&deep, 1u, SIM_INJECT_SIDE_SONAR_DEAF);
        sim_run(&ctx);
        assert_rejoined(&deep);
        assert(ctx.pass_retries >= 1u);
    }

    /* The front IR misses the line on the way back: after at least
     * APP_RETURN_MIN_MM the tracked position (encoders + compass) is
     * trusted and it hands back past the box, pointing along the line
     * (beyond it by up to the minimum less the sidestep). */
    ctx = sim_start(&box, 1u, SIM_INJECT_IR_DEAF);
    sim_run(&ctx);
    /* ... but it never sees the line, so after searching both ways it
     * stops instead of driving off. */
    assert(g_b_aborted && !g_b_finished);
    assert(status_seen(ROBOT_AVOID_STATUS_TIMEOUT));
    assert((g_pos_x - SIM_BODY_REAR_MM) > box.x_max);

    /* Compass gone for good from the pass on: the encoders carry the
     * heading through the last pivots and it still rejoins the line. */
    ctx = sim_start(&box, 1u, SIM_INJECT_IMU_LOST_ON_PASS);
    sim_run(&ctx);
    assert_rejoined(&box);

    /* An encoder fault on the sidestep (after its first second). */
    ctx = sim_start(&box, 1u, SIM_INJECT_ENCODER_ON_SIDESTEP);
    sim_run(&ctx);
    assert(g_b_aborted && !g_b_finished);
    assert(status_seen(ROBOT_AVOID_STATUS_BLOCKED));

    /* Wheels blocked while turning: stops instead of pushing. */
    ctx = sim_start(&box, 1u, SIM_INJECT_STUCK_ON_PIVOT);
    sim_run(&ctx);
    assert(g_b_aborted && !g_b_finished);
    assert(status_seen(ROBOT_AVOID_STATUS_BLOCKED));
}

/* Heading helpers: shortest change through the 0/360 wrap, and which way
 * each scan angle looks. */
static void test_helpers (void)
{
    assert(20 == app_heading_change_x10(3590, 10));
    assert(-20 == app_heading_change_x10(10, 3590));
    assert(0 == app_heading_change_x10(3600, 0));
    assert(900 == app_heading_offset_x10(0, 900));
    assert(2700 == app_heading_offset_x10(0, -900));
    assert(!app_scan_angle_is_left(45));
    assert(app_scan_angle_is_left(135));
    assert(45 == app_sweep_angle(APP_SWEEP_45_INDEX));
    assert(90 == app_sweep_angle(APP_SWEEP_CENTRE_INDEX));
    assert(135 == app_sweep_angle(APP_SWEEP_135_INDEX));
    assert(150 == app_sweep_angle(APP_SWEEP_POINTS - 1u));
}

int main (void)
{
    test_helpers();
    test_left_route();
    test_right_route();
    test_box_sizes();
    test_blocked();
    test_faults();
    puts("PASS: avoidance round a box on the line in a simulated world: "
         "5-degree sweep measures the width, pivot out, sidestep, pass it "
         "with the side sonar, back onto the line beyond it (ignoring the "
         "line beside it) and aligned; left/right, three box sizes, "
         "blocked, compass drop-out, deaf side sonar (deep box: turn back finds it, drives further), IR missing the line "
         "(tracked position), compass lost (encoders), encoder "
         "fault and stuck stop");
    return 0;
}

/*** end of file ***/
