/**
 * @file    buddy5_avoidance.c
 * @brief   Buddy 5 ultrasonic obstacle-avoidance behaviour.
 *
 * Low-level ultrasonic timing and servo PWM remain in buddy5_ultrasonic.c.
 * This file owns only the 100 ms route state machine.
 *
 * The route is a fixed sequence of moves whose lengths are set by the
 * #defines below (the obstacle is a known box), not by watching the
 * obstacle while going round it:
 *
 *   1. obstacle 15 cm ahead while line following: stop and back away;
 *   2. sweep the sonar 30..150 degrees once: how far ahead the box is and
 *      which side has the nearer edge (that side is the way round);
 *   3. pivot 90 degrees away from the line, drive APP_SIDESTEP_MM sideways,
 *      pivot back to the line's heading;
 *   4. drive past the box: its distance + APP_BOX_DEPTH_MM + APP_PASS_TAIL_MM;
 *   5. pivot 90 degrees toward the line (if the sonar still sees the box
 *      there, turn back and drive APP_PASS_RETRY_MM further first) and drive
 *      until a front IR sensor crosses the line;
 *   6. drive the sensor-to-axle distance, pivot back to the line's heading
 *      and hand back to line following.
 *
 * Every turn is measured by the wheel encoders (the turn the two wheels'
 * difference makes on the track width), relative to the heading the rover
 * had on the line.  The compass is not used for control: on the track it
 * dropped out and the motor current upset it during pivots.  Its change over
 * the route is only printed for comparison.
 *
 * Tune on the track: APP_PIVOT_WHEEL_SLIP (90-degree pivots too far or too
 * short), APP_SIDESTEP_MM, APP_BOX_DEPTH_MM and APP_PASS_TAIL_MM.
 */

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "app.h"
#include "buddy2_motion.h"
#include "buddy3_ir.h"
#include "buddy4_imu.h"
#include "buddy5_avoidance.h"
#include "buddy5_ultrasonic.h"
#include "robot_controller.h"
#include "robot_telemetry.h"

#define BUDDY5_AVOIDANCE_PERIOD_MS (100u)

/* Every avoidance move runs at this fixed speed, NOT the dashboard's global
 * speed limit: at a low limit the PWM values below were scaled too low to
 * move the wheels.  All *_PWM values below are tuned for it. */
#define APP_AVOID_SPEED_PCT (60u)

/* ---- Sonar and servo ----------------------------------------------- */
#define APP_ULTRASONIC_ANGLE_DEG (90u)
#define APP_SERVO_SETTLE_MS      (500u)
/* The HC-SR04 needs this long between pings so an old echo is not read. */
#define APP_SONAR_REPING_MS  (60u)
#define APP_OBSTACLE_STOP_CM (15u)
/* Servo installation: angles below 90 look to the rover's right. */
#define APP_LOW_ANGLES_ARE_LEFT (0u)

/* ---- Backing away -------------------------------------------------- */
#define APP_BACKUP_MM (80.0f)
/* Pivoting right swings the left-front barcode IR arm toward the box, so
 * back away this much further before going round on the right. */
#define APP_ARM_EXTRA_BACKUP_MM (100.0f)
#define APP_REVERSE_PWM         (14000)
#define APP_REVERSE_TIMEOUT_MS  (6000u)

/* ---- Sweep: 30, 35, ... 150 degrees; 90 is straight ahead ------------- */
#define APP_SWEEP_FIRST_DEG      (30u)
#define APP_SWEEP_STEP_DEG       (5u)
#define APP_SWEEP_POINTS         (25u)
#define APP_SWEEP_CENTRE_INDEX   (12u)
#define APP_SWEEP_45_INDEX       (3u)
#define APP_SWEEP_135_INDEX      (21u)
#define APP_SWEEP_STEP_SETTLE_MS (100u)
/* Each point keeps the nearest of up to two valid echoes in three pings. */
#define APP_SWEEP_VALID_SAMPLES (2u)
#define APP_SWEEP_MAX_ATTEMPTS  (3u)
/* A reading is on the box's front face while its forward part is within
 * this of the face (nearer: something beside the box; farther: past it). */
#define APP_FACE_TOLERANCE_MM (50.0f)
/* The wide beam still echoes off the corner one step past it. */
#define APP_SWEEP_BEAM_STEPS (1u)

/* ---- The route (the hard-coded part: tune these) -------------------- */
/* Sideways leg.  A 200 mm box centred on the line: 100 mm edge + 75 mm
 * reach of the rover's arm side + a 120 mm gap.  It only grows when the
 * sweep measures an edge further out than that allows for. */
#define APP_SIDESTEP_MM     (400.0f)
#define APP_SIDESTEP_MAX_MM (560.0f)
/* How far each side of the rover reaches from its centreline (the barcode
 * arm sticks out on the LEFT), and the gap kept from the box. */
#define APP_ROVER_LEFT_REACH_MM  (75.0f)
#define APP_ROVER_RIGHT_REACH_MM (55.0f)
#define APP_SIDE_GAP_MM          (120.0f)
/* Box length along the line, assumed (the sonar cannot see it). */
#define APP_BOX_DEPTH_MM (250.0f)
/* Sonar ahead of the axle. */
#define APP_SONAR_AHEAD_MM (100.0f)
/* The axle goes this far past the box's far side before the turn back, so
 * the rover's tail and its cable clear the box while it pivots. */
#define APP_PASS_TAIL_MM (400.0f)
/* No echo straight ahead in the sweep: assume the box is this far away. */
#define APP_FALLBACK_FACE_MM (250.0f)
/* ... and neither edge seen: go to the side with more room, if it has at
 * least this much at the sweep's end. */
#define APP_MIN_CLEARANCE_CM (20u)
/* Turning (or driving) back toward the line, the box this near in front
 * means the rover is not past it yet: turn back along the line, drive
 * APP_PASS_RETRY_MM further, try again, at most APP_PASS_MAX_RETRIES. */
#define APP_TURN_BACK_CLEAR_CM (25u)
#define APP_RETURN_CLEAR_CM    (20u)
#define APP_PASS_RETRY_MM      (200.0f)
#define APP_PASS_MAX_RETRIES   (3u)

/* ---- Back onto the line -------------------------------------------- */
/* Black only counts once the return leg has come this close to the
 * sidestep distance (black earlier is another line on the mat). */
#define APP_RETURN_EARLY_MM (150.0f)
/* The return leg gives up this far beyond the sidestep. */
#define APP_RETURN_EXTRA_MM (250.0f)
/* Crossing the line, black ends within this much travel; longer black is a
 * line running the rover's way (a junction branch): ignored. */
#define APP_RETURN_CROSS_MAX_MM (50.0f)
/* Front IR sensors to axle: drive this far after they find the line so the
 * last pivot turns on it (as ROBOT_JUNCTION_AXLE_MM). */
#define APP_ALIGN_FORWARD_MM (150.0f)
/* No sensor touched the line in the last pivot: sweep this far one way and
 * then the other, stopping as soon as one finds it. */
#define APP_SEARCH_SWEEP_DEG (35.0f)
/* The search turns about 10 degrees per 100 ms step, so at the normal task
 * period it stopped up to that far past the line.  While searching the task
 * runs every 10 ms instead and stops within about a degree. */
#define APP_SEARCH_PERIOD_MS (10u)

/* ---- Pivots (encoder heading) -------------------------------------- */
#define APP_PIVOT_FAST_PWM      (14000)
#define APP_PIVOT_SLOW_PWM      (11000)
#define APP_PIVOT_SLOW_ZONE_DEG (20.0f)
#define APP_PIVOT_STOP_DEG      (4.0f)
#define APP_PIVOT_TIMEOUT_MS    (6000u)
/* On the spot the wheels scrub, so the rover turns only this fraction of
 * what the wheel travel says.  Pivots come out short: raise it; long: lower
 * it. */
#define APP_PIVOT_WHEEL_SLIP (0.85f)
#define APP_DEG_PER_RAD      (57.2957795f)

/* ---- Straight legs (heading held by the encoders) ------------------ */
/* Both wheels get at least the base: the heading correction only ever
 * speeds up the wheel that lags.  Slowing the other one instead took the
 * (weaker) right motor down to about 4500 after the speed scaling, where it
 * stalled and Buddy 2 reported an encoder fault (track, 25 Nov). */
#define APP_FORWARD_BASE_PWM       (9000)
#define APP_FORWARD_DEADBAND_DEG   (0.5f)
#define APP_FORWARD_KP_PER_DEG     (300.0f)
#define APP_FORWARD_MAX_CORRECTION (2500.0f)
#define APP_LEG_TIMEOUT_MS         (15000u)
/* Encoder faults are ignored for this long at the start of a leg. */
#define APP_STALE_FAULT_MS (1000u)

/* ---- Stuck check: wheels moved under this in one window ------------ */
#define APP_STALL_WINDOW_MS (1000u)
#define APP_STALL_MIN_MM    (15.0f)

#define APP_RESULT_VISIBLE_MS (1100u)
#define APP_MM_PER_CM         (10.0f)

/* cos and sin of 0, 5, ... 60 degrees off-axis, by sweep steps from the
 * centre.  The kernel's syslib.h clashes with <math.h>. */
static const float32_t g_sweep_cos[APP_SWEEP_CENTRE_INDEX + 1u] = {
    1.0f,        0.99619470f, 0.98480775f, 0.96592583f, 0.93969262f,
    0.90630779f, 0.86602540f, 0.81915204f, 0.76604444f, 0.70710678f,
    0.64278761f, 0.57357644f, 0.5f};
static const float32_t g_sweep_sin[APP_SWEEP_CENTRE_INDEX + 1u] = {
    0.0f,        0.08715574f, 0.17364818f, 0.25881905f, 0.34202014f,
    0.42261826f, 0.5f,        0.57357644f, 0.64278761f, 0.70710678f,
    0.76604444f, 0.81915204f, 0.86602540f};

/* ------------------------------------------------------------------ *
 *  Types
 * ------------------------------------------------------------------ */

typedef enum
{
    APP_AVOID_WATCH_FORWARD = 0, /* line following: watch ahead          */
    APP_AVOID_BACKUP,            /* backing away from it                 */
    APP_AVOID_SWEEP,             /* sonar sweep, one angle per step      */
    APP_AVOID_CHOOSE,            /* servo back ahead, then pick the side */
    APP_AVOID_ARM_BACKUP,        /* extra reverse before going right     */
    APP_AVOID_PIVOT,             /* turn on the spot                     */
    APP_AVOID_LEG                /* straight leg                         */
} app_avoid_state_t;

/** What comes after a pivot. */
typedef enum
{
    APP_NEXT_SIDESTEP = 0,
    APP_NEXT_PASS,
    APP_NEXT_PASS_MORE,
    APP_NEXT_RETURN,
    APP_NEXT_FINISH,
    APP_NEXT_SEARCH_OTHER,
    APP_NEXT_SEARCH_FAIL
} app_next_t;

typedef enum
{
    APP_LEG_SIDESTEP = 0, /* away from the line                    */
    APP_LEG_PASS,         /* along the line, past the box          */
    APP_LEG_PASS_MORE,    /* further along: not past it yet        */
    APP_LEG_RETURN,       /* toward the line until the IR finds it */
    APP_LEG_ALIGN         /* sensor-to-axle distance onto the line */
} app_leg_t;

/** One side of the box as the sweep saw it. */
typedef struct
{
    float32_t edge_mm;  /**< Centreline to the box edge, sideways. */
    bool      b_found;  /**< The sweep saw the box end.            */
    bool      b_nearer; /**< Something nearer than the box there.  */
} app_side_t;

typedef struct
{
    app_avoid_state_t state;
    uint32_t          state_started_ms;
    uint32_t          finished_ms;
    bool              b_result_pending_clear;
    /* Sweep. */
    uint32_t sweep_cm[APP_SWEEP_POINTS];
    uint8_t  sweep_index;
    uint32_t sweep_settle_ms;
    uint8_t  ping_attempts;
    uint8_t  ping_valid;
    uint32_t ping_nearest_cm;
    /* Route. */
    bool      b_go_right;
    float32_t sidestep_mm;
    float32_t pass_mm;
    uint8_t   pass_retries;
    /* Encoder heading, degrees from the line's heading, right positive. */
    float32_t heading_deg;
    float32_t head_left_mm;
    float32_t head_right_mm;
    int16_t   compass_start_x10; /* -1: no compass at the start */
    /* Pivot. */
    float32_t  pivot_target_deg;
    float32_t  pivot_last_error_deg;
    float32_t  search_sign;
    app_next_t next;
    /* Leg. */
    app_leg_t leg;
    float32_t leg_heading_deg;
    float32_t leg_target_mm;
    float32_t leg_left_mm;
    float32_t leg_right_mm;
    bool      b_black;      /* return: front IR on black    */
    bool      b_black_long; /* ... for too long: a branch   */
    float32_t black_mm;     /* where the black started      */
    /* Stuck check. */
    app_avoid_state_t stall_state;
    uint32_t          stall_started_ms;
    float32_t         stall_left_mm;
    float32_t         stall_right_mm;
} app_avoid_ctx_t;

/* ------------------------------------------------------------------ *
 *  Prototypes
 * ------------------------------------------------------------------ */

static void app_step (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_watch_forward (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_backup (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_sweep (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_choose (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static bool app_face_mm (const uint32_t * p_cm, float32_t * p_face_mm);
static bool app_pick_side (app_avoid_ctx_t * p_ctx, bool b_face,
                           float32_t face_mm, uint32_t left_cm,
                           uint32_t right_cm);
static void app_start_chosen_route (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                                    float32_t face_mm, uint32_t left_cm,
                                    uint32_t right_cm);
static void app_measure_side (const uint32_t * p_cm, float32_t face_mm,
                              bool b_low, app_side_t * p_side);
static void app_begin_route (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_start_pivot (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                             float32_t target_deg, app_next_t next);
static void app_pivot (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_pivot_done (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_start_leg (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                           app_leg_t leg, float32_t heading_deg,
                           float32_t target_mm);
static void app_leg (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static bool app_leg_check (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                           float32_t progress_mm);
static bool app_return_line (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                             float32_t progress_mm);
static bool app_still_beside (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                              uint32_t clear_cm);
static bool app_front_on_line (void);
static bool app_front_on_black_now (void);
static bool app_sonar_is_close (uint32_t distance_cm);
static void app_heading_update (app_avoid_ctx_t * p_ctx);
static bool app_is_stuck (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_stall_start (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static bool app_start_reverse (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                               float32_t distance_mm, app_avoid_state_t state);
static void app_fail (app_avoid_ctx_t * p_ctx, uint32_t now_ms, uint8_t status,
                      const char * p_reason);
static void app_complete (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_end (app_avoid_ctx_t * p_ctx, uint32_t now_ms, bool b_abort);
static uint8_t   app_sweep_angle (uint8_t index);
static bool      app_low_is_left (void);
static float32_t app_abs (float32_t value);
static float32_t app_away_deg (const app_avoid_ctx_t * p_ctx);
static int16_t   app_deg_x10 (float32_t degrees);
static uint32_t  app_period_ms (const app_avoid_ctx_t * p_ctx);

/**
 * @brief Run the Buddy 5 obstacle-avoidance state machine every 100 ms.
 */
void buddy5_avoidance_task (INT stacd, void * p_exinf)
{
    app_avoid_ctx_t ctx = {0};

    (void) stacd;
    (void) p_exinf;

    ctx.state = APP_AVOID_WATCH_FORWARD;
    buddy5_set_servo_angle(APP_ULTRASONIC_ANGLE_DEG);
    (void) tk_dly_tsk(APP_SERVO_SETTLE_MS);

    for (;;)
    {
        app_step(&ctx, app_now_ms());
        (void) tk_dly_tsk(app_period_ms(&ctx));
    }
}

/* ------------------------------------------------------------------ *
 *  State machine
 * ------------------------------------------------------------------ */

static void app_step (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    if (p_ctx->b_result_pending_clear &&
        ((now_ms - p_ctx->finished_ms) >= APP_RESULT_VISIBLE_MS))
    {
        robot_telemetry_clear_avoidance_scan();
        p_ctx->b_result_pending_clear = false;
    }

    /* A dashboard command (including stop) cancels a run. */
    if ((APP_AVOID_WATCH_FORWARD != p_ctx->state) &&
        !robot_controller_obstacle_avoidance_active())
    {
        buddy2_stop();
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_CANCELLED, 0,
                                             0u);
        p_ctx->finished_ms            = now_ms;
        p_ctx->b_result_pending_clear = true;
        buddy5_set_servo_angle(APP_ULTRASONIC_ANGLE_DEG);
        p_ctx->state = APP_AVOID_WATCH_FORWARD;
    }

    if ((APP_AVOID_PIVOT == p_ctx->state) || (APP_AVOID_LEG == p_ctx->state))
    {
        app_heading_update(p_ctx);
    }

    if (p_ctx->state != p_ctx->stall_state)
    {
        p_ctx->stall_state = p_ctx->state;
        app_stall_start(p_ctx, now_ms);
    }

    switch (p_ctx->state)
    {
        case APP_AVOID_WATCH_FORWARD:
            app_watch_forward(p_ctx, now_ms);
        break;

        case APP_AVOID_BACKUP:
        case APP_AVOID_ARM_BACKUP:
            app_backup(p_ctx, now_ms);
        break;

        case APP_AVOID_SWEEP:
            app_sweep(p_ctx, now_ms);
        break;

        case APP_AVOID_CHOOSE:
            app_choose(p_ctx, now_ms);
        break;

        case APP_AVOID_PIVOT:
            app_pivot(p_ctx, now_ms);
        break;

        case APP_AVOID_LEG:
            app_leg(p_ctx, now_ms);
        break;

        default:
            app_end(p_ctx, now_ms, true);
        break;
    }
}

/* Watch the forward distance and start avoidance at 15 cm. */
static void app_watch_forward (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    uint32_t front_cm = buddy5_measure_cm(now_ms);

    if (app_sonar_is_close(front_cm) &&
        robot_controller_begin_obstacle_avoidance())
    {
        tm_printf((UB *) "[avoid] obstacle %u cm: backing %u mm\n",
                  (UINT) front_cm, (UINT) APP_BACKUP_MM);
        robot_telemetry_clear_avoidance_scan();
        p_ctx->b_result_pending_clear = false;
        if (!app_start_reverse(p_ctx, now_ms, APP_BACKUP_MM, APP_AVOID_BACKUP))
        {
            app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                     "reverse move rejected");
        }
    }
}

/* Wait for a reverse move: then sweep (first backup) or begin the route
 * (extra backup for the barcode arm). */
static void app_backup (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    buddy2_motion_state_t motion = {0};
    uint8_t               index  = 0u;

    if (buddy2_motion_active())
    {
        if ((now_ms - p_ctx->state_started_ms) >= APP_REVERSE_TIMEOUT_MS)
        {
            app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_TIMEOUT,
                     "reverse timed out");
        }
    }
    else if (APP_AVOID_ARM_BACKUP == p_ctx->state)
    {
        buddy2_stop();
        robot_telemetry_set_avoidance_turn_pwm(0, 0);
        app_begin_route(p_ctx, now_ms);
    }
    else
    {
        buddy2_get_motion_state(&motion);
        buddy2_stop();
        robot_telemetry_set_avoidance_turn_pwm(0, 0);
        if (motion.left_encoder_fault || motion.right_encoder_fault)
        {
            app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                     "encoder fault while backing away");
        }
        else
        {
            for (index = 0u; index < APP_SWEEP_POINTS; index++)
            {
                p_ctx->sweep_cm[index] = BUDDY5_NO_ECHO;
            }
            p_ctx->sweep_index     = 0u;
            p_ctx->ping_attempts   = 0u;
            p_ctx->ping_valid      = 0u;
            p_ctx->ping_nearest_cm = BUDDY5_NO_ECHO;
            /* The first move swings the servo 60 degrees. */
            p_ctx->sweep_settle_ms  = APP_SERVO_SETTLE_MS;
            p_ctx->state            = APP_AVOID_SWEEP;
            p_ctx->state_started_ms = now_ms;
            buddy5_set_servo_angle(app_sweep_angle(0u));
            robot_telemetry_set_obstacle_size(0u, 0u, false);
            robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_SCANNING, 0,
                                                 0u);
        }
    }
}

/* One sweep angle per step: ping until two echoes or three tries, keep the
 * nearest, then step the servo on (or back to the centre at the end). */
static void app_sweep (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    buddy5_ping_result_t result      = BUDDY5_PING_TIMEOUT_RISE;
    uint32_t             distance_cm = 0u;

    if ((now_ms - p_ctx->state_started_ms) >= p_ctx->sweep_settle_ms)
    {
        while ((p_ctx->ping_valid < APP_SWEEP_VALID_SAMPLES) &&
               (p_ctx->ping_attempts < APP_SWEEP_MAX_ATTEMPTS))
        {
            if (0u != p_ctx->ping_attempts)
            {
                (void) tk_dly_tsk(APP_SONAR_REPING_MS);
            }
            distance_cm = buddy5_sample_with_result_cm(&result);
            p_ctx->ping_attempts++;
            if (BUDDY5_PING_VALID == result)
            {
                if ((0u == p_ctx->ping_valid) ||
                    (distance_cm < p_ctx->ping_nearest_cm))
                {
                    p_ctx->ping_nearest_cm = distance_cm;
                }
                p_ctx->ping_valid++;
            }
        }
        p_ctx->sweep_cm[p_ctx->sweep_index] = p_ctx->ping_nearest_cm;
        tm_printf((UB *) "[avoid] sweep %u deg: %u cm\n",
                  (UINT) app_sweep_angle(p_ctx->sweep_index),
                  (UINT) p_ctx->ping_nearest_cm);

        p_ctx->sweep_index++;
        p_ctx->ping_attempts    = 0u;
        p_ctx->ping_valid       = 0u;
        p_ctx->ping_nearest_cm  = BUDDY5_NO_ECHO;
        p_ctx->sweep_settle_ms  = APP_SWEEP_STEP_SETTLE_MS;
        p_ctx->state_started_ms = now_ms;
        if (p_ctx->sweep_index < APP_SWEEP_POINTS)
        {
            buddy5_set_servo_angle(app_sweep_angle(p_ctx->sweep_index));
        }
        else
        {
            buddy5_set_servo_angle(APP_ULTRASONIC_ANGLE_DEG);
            p_ctx->state = APP_AVOID_CHOOSE;
        }
    }
}

/* Once the servo faces ahead again, pick the side and start the route. */
static void app_choose (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    float32_t face_mm    = 0.0f;
    uint32_t  left_cm    = 0u;
    uint32_t  right_cm   = 0u;
    bool      b_face     = false;
    bool      b_low_left = app_low_is_left();

    if ((now_ms - p_ctx->state_started_ms) >= APP_SERVO_SETTLE_MS)
    {
        left_cm  = p_ctx->sweep_cm[b_low_left ? APP_SWEEP_45_INDEX
                                              : APP_SWEEP_135_INDEX];
        right_cm = p_ctx->sweep_cm[b_low_left ? APP_SWEEP_135_INDEX
                                              : APP_SWEEP_45_INDEX];
        robot_telemetry_set_avoidance_scan_values(left_cm, right_cm);

        b_face = app_face_mm(p_ctx->sweep_cm, &face_mm);
        if (!app_pick_side(p_ctx, b_face, face_mm, left_cm, right_cm))
        {
            app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                     "both sides under 20 cm");
        }
        else
        {
            app_start_chosen_route(p_ctx, now_ms,
                                   b_face ? face_mm : APP_FALLBACK_FACE_MM,
                                   left_cm, right_cm);
        }
    }
}

/**
 * @brief The box's front face: the nearest forward part of the readings
 *        within 10 degrees of straight ahead.
 * @return false if none of those readings had an echo.
 */
static bool app_face_mm (const uint32_t * p_cm, float32_t * p_face_mm)
{
    float32_t forward = 0.0f;
    uint8_t   index   = 0u;
    uint8_t   steps   = 0u;
    bool      b_face  = false;

    for (index = APP_SWEEP_CENTRE_INDEX - 2u;
         index <= (APP_SWEEP_CENTRE_INDEX + 2u); index++)
    {
        if (BUDDY5_NO_ECHO != p_cm[index])
        {
            /* index is within 2 of the centre, so steps is 0..2. */
            steps = (index < APP_SWEEP_CENTRE_INDEX)
                        ? (uint8_t) (APP_SWEEP_CENTRE_INDEX - index)
                        : (uint8_t) (index - APP_SWEEP_CENTRE_INDEX);
            forward =
                (float32_t) p_cm[index] * APP_MM_PER_CM * g_sweep_cos[steps];
            if (!b_face || (forward < *p_face_mm))
            {
                *p_face_mm = forward;
            }
            b_face = true;
        }
    }

    return b_face;
}

/**
 * @brief Choose the way round and the sideways distance.
 *
 * Both edges seen and open: the nearer one (a tie goes left, keeping the
 * barcode arm away from the box).  Neither usable: the side with more room
 * at 45/135 degrees, unless both are under APP_MIN_CLEARANCE_CM.
 *
 * @return false if both sides are blocked.
 */
static bool app_pick_side (app_avoid_ctx_t * p_ctx, bool b_face,
                           float32_t face_mm, uint32_t left_cm,
                           uint32_t right_cm)
{
    app_side_t   low        = {0};
    app_side_t   high       = {0};
    app_side_t * p_left     = app_low_is_left() ? &low : &high;
    app_side_t * p_right    = app_low_is_left() ? &high : &low;
    float32_t    left_need  = 0.0f;
    float32_t    right_need = 0.0f;
    bool         b_left_ok  = false;
    bool         b_right_ok = false;
    bool         b_route    = true;

    if (b_face)
    {
        app_measure_side(p_ctx->sweep_cm, face_mm, true, &low);
        app_measure_side(p_ctx->sweep_cm, face_mm, false, &high);
        /* Sonar distances are a few metres at most, so they fit UINT. */
        tm_printf((UB *) "[avoid] box %u mm ahead, left edge %u mm%s, right "
                         "edge %u mm%s\n",
                  (UINT) face_mm, (UINT) p_left->edge_mm,
                  p_left->b_found ? "" : " (not seen)", (UINT) p_right->edge_mm,
                  p_right->b_found ? "" : " (not seen)");
        /* Both are positive and a few hundred millimetres. */
        robot_telemetry_set_obstacle_size(
            (uint32_t) face_mm, (uint32_t) (p_left->edge_mm + p_right->edge_mm),
            !(p_left->b_found && p_right->b_found));
    }

    /* Round the left, the rover's right side passes the box; and the
     * reverse. */
    left_need  = p_left->edge_mm + APP_ROVER_RIGHT_REACH_MM + APP_SIDE_GAP_MM;
    right_need = p_right->edge_mm + APP_ROVER_LEFT_REACH_MM + APP_SIDE_GAP_MM;
    b_left_ok  = b_face && p_left->b_found && !p_left->b_nearer &&
                 (left_need <= APP_SIDESTEP_MAX_MM);
    b_right_ok = b_face && p_right->b_found && !p_right->b_nearer &&
                 (right_need <= APP_SIDESTEP_MAX_MM);

    if (b_left_ok || b_right_ok)
    {
        p_ctx->b_go_right =
            !b_left_ok || (b_right_ok && (p_right->edge_mm < p_left->edge_mm));
        p_ctx->sidestep_mm = p_ctx->b_go_right ? right_need : left_need;
    }
    else if ((BUDDY5_NO_ECHO != left_cm) && (BUDDY5_NO_ECHO != right_cm) &&
             (left_cm < APP_MIN_CLEARANCE_CM) &&
             (right_cm < APP_MIN_CLEARANCE_CM))
    {
        b_route = false;
    }
    else
    {
        tm_printf((UB *) "[avoid] no usable edge; 45/135 deg: right %u cm, "
                         "left %u cm\n",
                  (UINT) right_cm, (UINT) left_cm);
        p_ctx->b_go_right =
            (BUDDY5_NO_ECHO != left_cm) &&
            ((BUDDY5_NO_ECHO == right_cm) || (right_cm > left_cm));
        p_ctx->sidestep_mm = 0.0f;
    }

    /* The sidestep is fixed unless the edge needs more. */
    if (p_ctx->sidestep_mm < APP_SIDESTEP_MM)
    {
        p_ctx->sidestep_mm = APP_SIDESTEP_MM;
    }

    return b_route;
}

/* Size the pass, publish the decision and start: going right, after the
 * extra backup for the barcode arm. */
static void app_start_chosen_route (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                                    float32_t face_mm, uint32_t left_cm,
                                    uint32_t right_cm)
{
    /* Axle to the box's far side, plus the tail.  The extra arm backup
     * (going right) moves the box that much further away. */
    p_ctx->pass_mm = APP_SONAR_AHEAD_MM + face_mm + APP_BOX_DEPTH_MM +
                     APP_PASS_TAIL_MM +
                     (p_ctx->b_go_right ? APP_ARM_EXTRA_BACKUP_MM : 0.0f);
    tm_printf((UB *) "[avoid] go round %s: sidestep %u mm, pass %u mm\n",
              p_ctx->b_go_right ? "right" : "left", (UINT) p_ctx->sidestep_mm,
              (UINT) p_ctx->pass_mm);
    robot_telemetry_set_avoidance_scan(left_cm, right_cm, !p_ctx->b_go_right);

    if (!p_ctx->b_go_right)
    {
        app_begin_route(p_ctx, now_ms);
    }
    else if (!app_start_reverse(p_ctx, now_ms, APP_ARM_EXTRA_BACKUP_MM,
                                APP_AVOID_ARM_BACKUP))
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                 "extra reverse rejected");
    }
    else
    {
        /* Backing up; app_backup() begins the route. */
    }
}

/**
 * @brief Walk outward from the sweep's centre to one edge of the box.
 *
 * A reading stays on the box while it has an echo and its forward part is
 * within APP_FACE_TOLERANCE_MM of the face.  The first one that is open,
 * farther or nearer is past the edge; nearer means something else is beside
 * the box on that side.  The edge is APP_SWEEP_BEAM_STEPS inside the last
 * reading on the face (the beam is wide).
 *
 * @param b_low True for the servo angles below 90 degrees.
 */
static void app_measure_side (const uint32_t * p_cm, float32_t face_mm,
                              bool b_low, app_side_t * p_side)
{
    float32_t forward   = 0.0f;
    float32_t hit_mm    = 0.0f;
    uint8_t   step      = 0u;
    uint8_t   index     = 0u;
    uint8_t   last_step = 0u;
    bool      b_on_face = true;

    p_side->b_nearer = false;
    for (step = 1u; (step <= APP_SWEEP_CENTRE_INDEX) && b_on_face; step++)
    {
        /* step is 1..12, so the index stays within 0..24. */
        index     = b_low ? (uint8_t) (APP_SWEEP_CENTRE_INDEX - step)
                          : (uint8_t) (APP_SWEEP_CENTRE_INDEX + step);
        b_on_face = (BUDDY5_NO_ECHO != p_cm[index]);
        if (b_on_face)
        {
            forward =
                (float32_t) p_cm[index] * APP_MM_PER_CM * g_sweep_cos[step];
            p_side->b_nearer = (forward < (face_mm - APP_FACE_TOLERANCE_MM));
            b_on_face        = !p_side->b_nearer &&
                               (forward <= (face_mm + APP_FACE_TOLERANCE_MM));
        }
        if (b_on_face)
        {
            last_step = step;
            hit_mm    = (float32_t) p_cm[index] * APP_MM_PER_CM;
        }
    }
    p_side->b_found = !b_on_face;
    p_side->edge_mm =
        (last_step > APP_SWEEP_BEAM_STEPS)
            ? (hit_mm * g_sweep_sin[last_step - APP_SWEEP_BEAM_STEPS])
            : (face_mm * g_sweep_sin[1]);
}

/* The heading now is the line's heading (the rover only reversed
 * straight): zero it and pivot away from the line. */
static void app_begin_route (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    buddy2_motion_state_t motion = {0};
    buddy4_imu_state_t    imu    = {0};

    buddy2_get_motion_state(&motion);
    buddy4_get_state(&imu);
    p_ctx->heading_deg   = 0.0f;
    p_ctx->head_left_mm  = motion.left_distance_mm;
    p_ctx->head_right_mm = motion.right_distance_mm;
    /* -1 marks "no compass": real headings are 0..3599. */
    p_ctx->compass_start_x10 = imu.mag_ok ? imu.heading_deg_x10 : (int16_t) -1;
    p_ctx->pass_retries      = 0u;
    app_start_pivot(p_ctx, now_ms, app_away_deg(p_ctx), APP_NEXT_SIDESTEP);
}

/* ------------------------------------------------------------------ *
 *  Pivots
 * ------------------------------------------------------------------ */

static void app_start_pivot (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                             float32_t target_deg, app_next_t next)
{
    buddy2_stop();
    p_ctx->pivot_target_deg     = target_deg;
    p_ctx->pivot_last_error_deg = target_deg - p_ctx->heading_deg;
    p_ctx->next                 = next;
    p_ctx->state_started_ms     = now_ms;
    p_ctx->state                = APP_AVOID_PIVOT;
    buddy3_clear_line_seen_event();
    tm_printf((UB *) "[avoid] pivot from %d to %d deg\n",
              (INT) p_ctx->heading_deg, (INT) target_deg);
}

static void app_pivot (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    float32_t error_deg = p_ctx->pivot_target_deg - p_ctx->heading_deg;
    int16_t   pwm       = APP_PIVOT_FAST_PWM;
    bool      b_search  = (APP_NEXT_SEARCH_OTHER == p_ctx->next) ||
                          (APP_NEXT_SEARCH_FAIL == p_ctx->next);

    if ((APP_NEXT_RETURN == p_ctx->next) &&
        app_still_beside(p_ctx, now_ms, APP_TURN_BACK_CLEAR_CM))
    {
        /* Turning back toward the line, the box is in front. */
    }
    else if (b_search && app_front_on_line())
    {
        tm_printf((UB *) "[avoid] search found the line\n");
        app_complete(p_ctx, now_ms);
    }
    else if (app_is_stuck(p_ctx, now_ms))
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                 "stuck while turning");
    }
    else if ((app_abs(error_deg) <= APP_PIVOT_STOP_DEG) ||
             ((error_deg > 0.0f) != (p_ctx->pivot_last_error_deg > 0.0f)))
    {
        /* There, or it went past in the last step: stop, do not swing. */
        buddy2_stop();
        robot_telemetry_set_avoidance_turn_pwm(0, 0);
        app_pivot_done(p_ctx, now_ms);
    }
    else if ((now_ms - p_ctx->state_started_ms) >= APP_PIVOT_TIMEOUT_MS)
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_TIMEOUT, "turn timed out");
    }
    else
    {
        p_ctx->pivot_last_error_deg = error_deg;
        if (app_abs(error_deg) <= APP_PIVOT_SLOW_ZONE_DEG)
        {
            pwm = APP_PIVOT_SLOW_PWM;
        }
        if (error_deg < 0.0f)
        {
            pwm = (int16_t) -pwm; /* turn left; |pwm| <= 14000 */
        }
        /* Left wheel forward turns right.  |pwm| <= 14000, so -pwm fits. */
        (void) buddy2_set_motor_pwm_at_speed(pwm, (int16_t) -pwm,
                                             APP_AVOID_SPEED_PCT);
        robot_telemetry_set_avoidance_turn_pwm(pwm, (int16_t) -pwm);
        /* The target's magnitude is at most 90 degrees: 900 tenths. */
        robot_telemetry_set_avoidance_status(
            ROBOT_AVOID_STATUS_TURNING, app_deg_x10(p_ctx->heading_deg),
            (uint16_t) app_deg_x10(app_abs(p_ctx->pivot_target_deg)));
    }
}

/* A pivot reached its heading: start the next part of the route. */
static void app_pivot_done (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    float32_t toward_line = -app_away_deg(p_ctx);

    switch (p_ctx->next)
    {
        case APP_NEXT_SIDESTEP:
            app_start_leg(p_ctx, now_ms, APP_LEG_SIDESTEP,
                          p_ctx->pivot_target_deg, p_ctx->sidestep_mm);
        break;

        case APP_NEXT_PASS:
            app_start_leg(p_ctx, now_ms, APP_LEG_PASS, 0.0f, p_ctx->pass_mm);
        break;

        case APP_NEXT_PASS_MORE:
            app_start_leg(p_ctx, now_ms, APP_LEG_PASS_MORE, 0.0f,
                          APP_PASS_RETRY_MM);
        break;

        case APP_NEXT_RETURN:
            app_start_leg(p_ctx, now_ms, APP_LEG_RETURN, toward_line,
                          p_ctx->sidestep_mm + APP_RETURN_EXTRA_MM);
        break;

        case APP_NEXT_FINISH:
            /* Only a front sensor on black NOW counts: touching the line
             * somewhere during the turn does not mean the rover ended up
             * on it. */
            if (app_front_on_black_now())
            {
                app_complete(p_ctx, now_ms);
            }
            else
            {
                /* Pointing along the line but no sensor on it: search
                 * straight away, toward where it should be first. */
                tm_printf((UB *) "[avoid] line not under the sensors; "
                                 "searching\n");
                p_ctx->search_sign = (toward_line > 0.0f) ? 1.0f : -1.0f;
                app_start_pivot(p_ctx, now_ms,
                                p_ctx->search_sign * APP_SEARCH_SWEEP_DEG,
                                APP_NEXT_SEARCH_OTHER);
            }
        break;

        case APP_NEXT_SEARCH_OTHER:
            app_start_pivot(p_ctx, now_ms,
                            -p_ctx->search_sign * APP_SEARCH_SWEEP_DEG,
                            APP_NEXT_SEARCH_FAIL);
        break;

        default:
            app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_TIMEOUT,
                     "line not found after searching both ways");
        break;
    }
}

/* ------------------------------------------------------------------ *
 *  Straight legs
 * ------------------------------------------------------------------ */

static void app_start_leg (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                           app_leg_t leg, float32_t heading_deg,
                           float32_t target_mm)
{
    buddy2_motion_state_t motion = {0};

    buddy2_get_motion_state(&motion);
    p_ctx->leg              = leg;
    p_ctx->leg_heading_deg  = heading_deg;
    p_ctx->leg_target_mm    = target_mm;
    p_ctx->leg_left_mm      = motion.left_distance_mm;
    p_ctx->leg_right_mm     = motion.right_distance_mm;
    p_ctx->b_black          = false;
    p_ctx->b_black_long     = false;
    p_ctx->black_mm         = 0.0f;
    p_ctx->state_started_ms = now_ms;
    p_ctx->state            = APP_AVOID_LEG;
    if (APP_LEG_RETURN == leg)
    {
        buddy3_clear_line_seen_event();
    }
    /* Leg targets are positive and at most about 1.5 m. */
    robot_telemetry_set_avoidance_forward_progress(0u, (uint16_t) target_mm);
    tm_printf((UB *) "[avoid] leg %u: %u mm\n", (UINT) leg, (UINT) target_mm);
}

/* Drive the current leg holding its heading; end it on its condition. */
static void app_leg (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    buddy2_motion_state_t motion     = {0};
    float32_t             progress   = 0.0f;
    float32_t             error_deg  = 0.0f;
    float32_t             correction = 0.0f;
    int16_t               left_pwm   = 0;
    int16_t               right_pwm  = 0;
    uint8_t               status     = ROBOT_AVOID_STATUS_FORWARD;

    buddy2_get_motion_state(&motion);
    progress = 0.5f * (app_abs(motion.left_distance_mm - p_ctx->leg_left_mm) +
                       app_abs(motion.right_distance_mm - p_ctx->leg_right_mm));
    /* A leg stops at its target or after 15 s, so both stay under a few
     * metres. */
    robot_telemetry_set_avoidance_forward_progress(
        (uint16_t) progress, (uint16_t) p_ctx->leg_target_mm);

    /* A fault in a leg's first second is left over from an earlier stall. */
    if ((motion.left_encoder_fault || motion.right_encoder_fault) &&
        ((now_ms - p_ctx->state_started_ms) >= APP_STALE_FAULT_MS))
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                 "encoder fault while driving");
    }
    else if (app_is_stuck(p_ctx, now_ms))
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                 "stuck while driving");
    }
    else if ((now_ms - p_ctx->state_started_ms) >= APP_LEG_TIMEOUT_MS)
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_TIMEOUT, "leg timed out");
    }
    else if (!app_leg_check(p_ctx, now_ms, progress))
    {
        if ((APP_LEG_PASS == p_ctx->leg) || (APP_LEG_PASS_MORE == p_ctx->leg))
        {
            status = ROBOT_AVOID_STATUS_PASSING;
        }
        else if ((APP_LEG_RETURN == p_ctx->leg) ||
                 (APP_LEG_ALIGN == p_ctx->leg))
        {
            status = ROBOT_AVOID_STATUS_REJOINING;
        }
        else
        {
            status = ROBOT_AVOID_STATUS_FORWARD;
        }
        /* Right of the held heading: speed up the right wheel to turn back
         * (left of it: the left wheel). */
        error_deg = p_ctx->heading_deg - p_ctx->leg_heading_deg;
        if (app_abs(error_deg) > APP_FORWARD_DEADBAND_DEG)
        {
            correction = error_deg * APP_FORWARD_KP_PER_DEG;
        }
        if (correction > APP_FORWARD_MAX_CORRECTION)
        {
            correction = APP_FORWARD_MAX_CORRECTION;
        }
        else if (correction < -APP_FORWARD_MAX_CORRECTION)
        {
            correction = -APP_FORWARD_MAX_CORRECTION;
        }
        else
        {
            /* Within the limit. */
        }
        /* The whole wheel difference (twice the correction, as when one
         * wheel went up and the other down) goes on the lagging wheel. */
        /* At most 9000 + 2 * 2500 = 14000, so both fit int16_t. */
        left_pwm =
            (int16_t) ((float32_t) APP_FORWARD_BASE_PWM +
                       ((correction < 0.0f) ? (-2.0f * correction) : 0.0f));
        right_pwm =
            (int16_t) ((float32_t) APP_FORWARD_BASE_PWM +
                       ((correction > 0.0f) ? (2.0f * correction) : 0.0f));
        (void) buddy2_set_motor_pwm_at_speed(left_pwm, right_pwm,
                                             APP_AVOID_SPEED_PCT);
        robot_telemetry_set_avoidance_turn_pwm(left_pwm, right_pwm);
        robot_telemetry_set_avoidance_status(status, app_deg_x10(error_deg),
                                             0u);
    }
    else
    {
        /* The leg ended (or the run stopped) this step. */
    }
}

/** @return true if the leg ended (or the run stopped) this step. */
static bool app_leg_check (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                           float32_t progress_mm)
{
    bool b_ended = true;

    switch (p_ctx->leg)
    {
        case APP_LEG_SIDESTEP:
            if (app_sonar_is_close(buddy5_measure_cm(now_ms)))
            {
                app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                         "something in the way of the sidestep");
            }
            else if (progress_mm >= p_ctx->leg_target_mm)
            {
                app_start_pivot(p_ctx, now_ms, 0.0f, APP_NEXT_PASS);
            }
            else
            {
                b_ended = false;
            }
        break;

        case APP_LEG_PASS:
        case APP_LEG_PASS_MORE:
            if (app_sonar_is_close(buddy5_measure_cm(now_ms)))
            {
                app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                         "something ahead while passing");
            }
            else if (progress_mm >= p_ctx->leg_target_mm)
            {
                app_start_pivot(p_ctx, now_ms, -app_away_deg(p_ctx),
                                APP_NEXT_RETURN);
            }
            else
            {
                b_ended = false;
            }
        break;

        case APP_LEG_RETURN:
            if (app_still_beside(p_ctx, now_ms, APP_RETURN_CLEAR_CM) ||
                app_return_line(p_ctx, now_ms, progress_mm))
            {
                /* Re-routed past the box, or found the line. */
            }
            else if (progress_mm >= p_ctx->leg_target_mm)
            {
                app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_TIMEOUT,
                         "line not found on the way back");
            }
            else
            {
                b_ended = false;
            }
        break;

        default: /* APP_LEG_ALIGN */
            if (progress_mm >= p_ctx->leg_target_mm)
            {
                app_start_pivot(p_ctx, now_ms, 0.0f, APP_NEXT_FINISH);
            }
            else
            {
                b_ended = false;
            }
        break;
    }

    return b_ended;
}

/**
 * @brief Watch the front IR sensors on the way back to the line.
 *
 * Crossing the line, black ends within APP_RETURN_CROSS_MAX_MM: then the
 * align leg drives the rest of the sensor-to-axle distance.  Longer black is
 * a line running the rover's way and is ignored.  Black before the return
 * has come within APP_RETURN_EARLY_MM of the sidestep is another line.
 *
 * @return true if the line was found (the align leg has started).
 */
static bool app_return_line (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                             float32_t progress_mm)
{
    bool      b_left    = false;
    bool      b_right   = false;
    bool      b_side    = false;
    bool      b_seen    = buddy3_take_line_seen_event();
    bool      b_found   = false;
    float32_t remaining = 0.0f;

    buddy3_get_colors(&b_left, &b_right, &b_side);
    if (progress_mm < (p_ctx->sidestep_mm - APP_RETURN_EARLY_MM))
    {
        p_ctx->b_black = false; /* too early: not the track's line */
    }
    else if (!p_ctx->b_black)
    {
        /* The latched event catches a line crossed between two steps. */
        if (b_seen || b_left || b_right)
        {
            p_ctx->b_black      = true;
            p_ctx->b_black_long = false;
            p_ctx->black_mm     = progress_mm;
        }
    }
    else if (b_left || b_right)
    {
        if (!p_ctx->b_black_long &&
            ((progress_mm - p_ctx->black_mm) > APP_RETURN_CROSS_MAX_MM))
        {
            p_ctx->b_black_long = true;
            tm_printf((UB *) "[avoid] long black: a branch, ignored\n");
        }
    }
    else if (p_ctx->b_black_long)
    {
        p_ctx->b_black      = false; /* white again after a branch */
        p_ctx->b_black_long = false;
    }
    else
    {
        remaining = APP_ALIGN_FORWARD_MM - (progress_mm - p_ctx->black_mm);
        if (remaining < 0.0f)
        {
            remaining = 0.0f;
        }
        tm_printf((UB *) "[avoid] crossed the line after %u mm; aligning\n",
                  (UINT) progress_mm);
        app_start_leg(p_ctx, now_ms, APP_LEG_ALIGN, p_ctx->leg_heading_deg,
                      remaining);
        b_found = true;
    }

    return b_found;
}

/**
 * @brief Turning or driving back toward the line, the box in front means
 *        the rover is not past it: turn back along the line and drive
 *        further (or stop after APP_PASS_MAX_RETRIES).
 * @return true if the box was there (the route has been changed).
 */
static bool app_still_beside (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                              uint32_t clear_cm)
{
    uint32_t front_cm  = buddy5_measure_cm(now_ms);
    bool     b_blocked = (BUDDY5_NO_ECHO != front_cm) && (front_cm <= clear_cm);

    if (b_blocked)
    {
        buddy2_stop();
        robot_telemetry_set_avoidance_turn_pwm(0, 0);
        if (p_ctx->pass_retries >= APP_PASS_MAX_RETRIES)
        {
            app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                     "box still in the way after driving further");
        }
        else
        {
            p_ctx->pass_retries++;
            tm_printf((UB *) "[avoid] box %u cm ahead turning back: %u mm "
                             "further (try %u)\n",
                      (UINT) front_cm, (UINT) APP_PASS_RETRY_MM,
                      (UINT) p_ctx->pass_retries);
            app_start_pivot(p_ctx, now_ms, 0.0f, APP_NEXT_PASS_MORE);
        }
    }

    return b_blocked;
}

/* A front IR sensor is on black now, or was since the last check. */
static bool app_front_on_line (void)
{
    bool b_left  = false;
    bool b_right = false;
    bool b_side  = false;
    bool b_seen  = buddy3_take_line_seen_event();

    buddy3_get_colors(&b_left, &b_right, &b_side);

    return b_seen || b_left || b_right;
}

/* A front IR sensor is on black at this moment (no latched event). */
static bool app_front_on_black_now (void)
{
    bool b_left  = false;
    bool b_right = false;
    bool b_side  = false;

    buddy3_get_colors(&b_left, &b_right, &b_side);

    return b_left || b_right;
}

static bool app_sonar_is_close (uint32_t distance_cm)
{
    return (BUDDY5_NO_ECHO != distance_cm) &&
           (distance_cm <= APP_OBSTACLE_STOP_CM);
}

/* ------------------------------------------------------------------ *
 *  Encoder heading, stuck check and endings
 * ------------------------------------------------------------------ */

/* Add this step's turn: the wheels' difference over the track width
 * (left wheel ahead turns right), scaled for scrub when pivoting. */
static void app_heading_update (app_avoid_ctx_t * p_ctx)
{
    buddy2_motion_state_t motion   = {0};
    buddy2_calibration_t  geometry = {0};
    float32_t             left_mm  = 0.0f;
    float32_t             right_mm = 0.0f;
    float32_t             turn_deg = 0.0f;

    buddy2_get_motion_state(&motion);
    buddy2_get_calibration(&geometry);
    left_mm              = motion.left_distance_mm - p_ctx->head_left_mm;
    right_mm             = motion.right_distance_mm - p_ctx->head_right_mm;
    p_ctx->head_left_mm  = motion.left_distance_mm;
    p_ctx->head_right_mm = motion.right_distance_mm;
    if (geometry.track_width_mm > 0.0f)
    {
        turn_deg =
            ((left_mm - right_mm) / geometry.track_width_mm) * APP_DEG_PER_RAD;
        if ((left_mm * right_mm) < 0.0f)
        {
            turn_deg *= APP_PIVOT_WHEEL_SLIP;
        }
    }
    p_ctx->heading_deg += turn_deg;
}

static void app_stall_start (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    buddy2_motion_state_t motion = {0};

    buddy2_get_motion_state(&motion);
    p_ctx->stall_started_ms = now_ms;
    p_ctx->stall_left_mm    = motion.left_distance_mm;
    p_ctx->stall_right_mm   = motion.right_distance_mm;
}

/* The wheels moved under APP_STALL_MIN_MM in the last window. */
static bool app_is_stuck (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    buddy2_motion_state_t motion  = {0};
    bool                  b_stuck = false;

    if ((now_ms - p_ctx->stall_started_ms) >= APP_STALL_WINDOW_MS)
    {
        buddy2_get_motion_state(&motion);
        b_stuck =
            (0.5f * (app_abs(motion.left_distance_mm - p_ctx->stall_left_mm) +
                     app_abs(motion.right_distance_mm -
                             p_ctx->stall_right_mm))) < APP_STALL_MIN_MM;
        app_stall_start(p_ctx, now_ms);
    }

    return b_stuck;
}

/* Stop and reverse by distance_mm; false if Buddy 2 rejected the move. */
static bool app_start_reverse (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                               float32_t distance_mm, app_avoid_state_t state)
{
    bool b_started = false;

    buddy2_stop();
    if (buddy2_move_backward_at_speed(distance_mm, APP_REVERSE_PWM,
                                      APP_AVOID_SPEED_PCT))
    {
        robot_telemetry_set_avoidance_turn_pwm(-APP_REVERSE_PWM,
                                               -APP_REVERSE_PWM);
        /* Reverse distances are 80 or 100 mm. */
        robot_telemetry_set_avoidance_forward_progress(0u,
                                                       (uint16_t) distance_mm);
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_REVERSING, 0,
                                             0u);
        p_ctx->state_started_ms = now_ms;
        p_ctx->state            = state;
        b_started               = true;
    }

    return b_started;
}

static void app_fail (app_avoid_ctx_t * p_ctx, uint32_t now_ms, uint8_t status,
                      const char * p_reason)
{
    buddy2_stop();
    robot_telemetry_set_avoidance_turn_pwm(0, 0);
    tm_printf((UB *) "[avoid] %s; stopped\n", p_reason);
    robot_telemetry_set_avoidance_status(status, 0, 0u);
    app_end(p_ctx, now_ms, true);
}

/* Back on the line: print the compass for comparison and hand back. */
static void app_complete (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    buddy4_imu_state_t imu        = {0};
    int32_t            change_x10 = 0;

    buddy2_stop();
    robot_telemetry_set_avoidance_turn_pwm(0, 0);
    buddy4_get_state(&imu);
    if (imu.mag_ok && (p_ctx->compass_start_x10 >= 0))
    {
        change_x10 = (int32_t) imu.heading_deg_x10 - p_ctx->compass_start_x10;
        change_x10 = (change_x10 > 1800) ? (change_x10 - 3600) : change_x10;
        change_x10 = (change_x10 < -1800) ? (change_x10 + 3600) : change_x10;
        tm_printf((UB *) "[avoid] back on the line: encoder heading %d, "
                         "compass change %d (tenths of a degree)\n",
                  (INT) app_deg_x10(p_ctx->heading_deg), (INT) change_x10);
    }
    else
    {
        tm_printf((UB *) "[avoid] back on the line\n");
    }
    robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_COMPLETE, 0, 0u);
    app_end(p_ctx, now_ms, false);
}

/* Leave avoidance: a finished run hands back to line following, an aborted
 * one stays stopped. */
static void app_end (app_avoid_ctx_t * p_ctx, uint32_t now_ms, bool b_abort)
{
    p_ctx->finished_ms            = now_ms;
    p_ctx->b_result_pending_clear = true;
    buddy2_stop();
    buddy5_set_servo_angle(APP_ULTRASONIC_ANGLE_DEG);
    if (b_abort)
    {
        robot_controller_abort_obstacle_avoidance();
    }
    else
    {
        robot_controller_finish_obstacle_avoidance();
    }
    p_ctx->state = APP_AVOID_WATCH_FORWARD;
}

/* ------------------------------------------------------------------ *
 *  Helpers
 * ------------------------------------------------------------------ */

/* Servo angle of one sweep point: 30..150 degrees. */
static uint8_t app_sweep_angle (uint8_t index)
{
    /* index is 0..24: at most 30 + 24 * 5 = 150. */
    return (uint8_t) (APP_SWEEP_FIRST_DEG +
                      ((uint32_t) index * APP_SWEEP_STEP_DEG));
}

static bool app_low_is_left (void)
{
    return (0u != APP_LOW_ANGLES_ARE_LEFT);
}

static float32_t app_abs (float32_t value)
{
    return (value < 0.0f) ? -value : value;
}

/* Heading pointing away from the line: right is positive. */
static float32_t app_away_deg (const app_avoid_ctx_t * p_ctx)
{
    return p_ctx->b_go_right ? 90.0f : -90.0f;
}

/* Task period: short while the line search turns, so it stops on the line
 * instead of up to a whole step past it. */
static uint32_t app_period_ms (const app_avoid_ctx_t * p_ctx)
{
    bool b_search = (APP_AVOID_PIVOT == p_ctx->state) &&
                    ((APP_NEXT_SEARCH_OTHER == p_ctx->next) ||
                     (APP_NEXT_SEARCH_FAIL == p_ctx->next));

    return b_search ? APP_SEARCH_PERIOD_MS : BUDDY5_AVOIDANCE_PERIOD_MS;
}

/* Degrees to tenths for telemetry, clamped to int16_t. */
static int16_t app_deg_x10 (float32_t degrees)
{
    float32_t x10 = degrees * 10.0f;

    if (x10 > 32000.0f)
    {
        x10 = 32000.0f;
    }
    else if (x10 < -32000.0f)
    {
        x10 = -32000.0f;
    }
    else
    {
        /* In range. */
    }

    /* Clamped to +/-32000 above. */
    return (int16_t) x10;
}

/*** end of file ***/

