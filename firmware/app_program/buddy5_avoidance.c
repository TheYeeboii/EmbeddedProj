/**
 * @file    buddy5_avoidance.c
 * @brief   Buddy 5 ultrasonic obstacle-avoidance behaviour.
 *
 * Low-level ultrasonic timing and servo PWM remain in buddy5_ultrasonic.c.
 * This file owns only the 100 ms route state machine: scan, decide, turn,
 * forward movement, heading recovery, and final line handoff.
 */

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "app_main.h"
#include "buddy2_motion.h"
#include "buddy3_ir.h"
#include "buddy4_imu.h"
#include "buddy5_avoidance.h"
#include "buddy5_ultrasonic.h"
#include "robot_controller.h"
#include "robot_telemetry.h"

#define BUDDY5_AVOIDANCE_PERIOD_MS (100u)

/* ---- Buddy 5 route configuration ----------------------------------- */
#define APP_ULTRASONIC_ANGLE_DEG  (90u)
#define APP_SERVO_SETTLE_MS       (500u)
#define APP_SONAR_PING_SPACING_MS (100u)

/* Servo installation: 45 degrees looks to rover right; 135 to rover left.
 * Confirmed from the physical left-obstacle scan: it was reported at 135 deg.
 */
#define APP_OBSTACLE_STOP_CM   (15u)
#define APP_SCAN_ANGLE_45_DEG  (45u)
#define APP_SCAN_ANGLE_135_DEG (135u)
#define APP_SCAN_45_IS_LEFT    (0u)

#define APP_AVOID_TURN_DEG                 (30.0f)
#define APP_AVOID_TURN_TIMEOUT_MS          (5000u)
#define APP_AVOID_ARC_MONITOR_CLEARANCE_CM (25u)
#define APP_AVOID_ARC_EXTRA_TURN_DEG       (10.0f)
/* While moving through the outer arc and first diagonal, keep looking toward
 * the obstacle-side for this long. This catches a rear corner that remains
 * too close without stopping the rover to scan. */
#define APP_AVOID_OBSTACLE_SIDE_MONITOR_MS (2000u)
#define APP_AVOID_REVERSE_DISTANCE_MM      (80.0f)
#define APP_AVOID_REVERSE_PWM              (9000)
#define APP_AVOID_REVERSE_TIMEOUT_MS       (6000u)
#define APP_AVOID_FORWARD_DISTANCE_MM      (280.0f)
/* One bounded extra section preserves a predictable avoidance route. */
#define APP_AVOID_FORWARD_EXTRA_DISTANCE_MM (100.0f)
#define APP_AVOID_FORWARD_EXTRA_TURN_DEG    (10.0f)
#define APP_AVOID_FINAL_FORWARD_DISTANCE_MM (250.0f)

/* Automatic IMU heading feedback used during all three forward legs. */
#define APP_AVOID_FORWARD_BASE_PWM             (9000)
#define APP_AVOID_FORWARD_HEADING_DEADBAND_X10 (5)
#define APP_AVOID_FORWARD_HEADING_KP           (30)
#define APP_AVOID_FORWARD_MAX_CORRECTION_PWM   (2500)
#define APP_AVOID_FORWARD_TIMEOUT_MS           (12000u)

#define APP_AVOID_RECOVER_TOLERANCE_X10   (20)
#define APP_AVOID_RECOVER_TIMEOUT_MS      (5000u)
#define APP_AVOID_SIDE_SCAN_VALID_SAMPLES (5u)
#define APP_AVOID_SIDE_SCAN_MAX_ATTEMPTS  (12u)
#define APP_AVOID_SIDE_SCAN_MIN_VALID     (3u)
#define APP_AVOID_MIN_CLEARANCE_CM        (20u)
#define APP_AVOID_RESULT_VISIBLE_MS       (1100u)

/* Forward arcs stop at the IMU's 30-degree target.  Match the outer-wheel
 * drive on both sides and retain enough inner-wheel drive for reliable motion.
 */
#define APP_AVOID_LEFT_TURN_OUTER_PWM  (14000)
#define APP_AVOID_RIGHT_TURN_OUTER_PWM (14000)
#define APP_AVOID_ARC_INNER_PWM        (4000)

/* Angles are handled in tenths of a degree. */
#define APP_DEG_X10         (10.0f)
#define APP_HALF_TURN_X10   (1800)
#define APP_FULL_TURN_X10   (3600)
#define APP_TURN_TARGET_X10 ((uint16_t) (APP_AVOID_TURN_DEG * APP_DEG_X10))

/* ------------------------------------------------------------------ *
 *  Sensors: IMU, ultrasonic monitoring, and obstacle avoidance
 * ------------------------------------------------------------------ */

typedef enum
{
    /* Normal mode: sample only the forward-facing ultrasonic sensor. */
    APP_AVOID_WATCH_FORWARD = 0,
    /* Back away from a close front obstacle before scanning around it. */
    APP_AVOID_WAIT_REVERSE_CLEARANCE,
    /* Settle at 45 degrees, then collect several temporary side samples. */
    APP_AVOID_WAIT_45,
    /* Settle at 135 degrees, then collect several temporary side samples. */
    APP_AVOID_WAIT_135,
    /* Re-centre the servo before choosing and starting the forward arc. */
    APP_AVOID_WAIT_CENTER,
    /* Settle the obstacle-side scan angle before turning out. */
    APP_AVOID_WAIT_OUTWARD_MONITOR_SETTLE,
    /* Drive an outward or inward arc until the IMU heading change reaches
     * the target. */
    APP_AVOID_WAIT_TURN_ARC,
    /* Drive straight for either avoidance forward distance. */
    APP_AVOID_WAIT_FORWARD_DISTANCE,
    /* Curve back to the heading saved before the first avoidance arc. */
    APP_AVOID_WAIT_RECOVER_HEADING
} app_avoid_state_t;

typedef enum
{
    APP_AVOID_FORWARD_OUTWARD_DIAGONAL = 0,
    APP_AVOID_FORWARD_STRAIGHT,
    APP_AVOID_FORWARD_INWARD_DIAGONAL
} app_avoid_forward_leg_t;

/** @brief Accumulate one side scan without changing forward sonar telemetry. */
typedef struct
{
    uint8_t  attempts;
    uint8_t  valid;
    uint8_t  timeout_rise;
    uint8_t  timeout_fall;
    uint8_t  high_before_trigger;
    uint32_t total_cm;
} app_side_scan_t;

/** Everything the avoidance task remembers between its 100 ms steps. */
typedef struct
{
    app_avoid_state_t       state;
    uint32_t                scan_45_cm;
    uint32_t                scan_135_cm;
    uint32_t                state_started_ms;
    uint32_t                side_scan_last_ping_ms;
    uint32_t                finished_ms;
    uint32_t                outward_monitor_started_ms;
    uint32_t                outward_monitor_last_ping_ms;
    uint32_t                outward_forward_monitor_started_ms;
    uint32_t                outward_forward_monitor_last_ping_ms;
    uint32_t                forward_sonar_center_started_ms;
    int16_t                 arc_start_heading_x10;
    int16_t                 forward_start_heading_x10;
    uint16_t                outward_arc_target_x10;
    uint16_t                return_arc_target_x10;
    float32_t               forward_target_mm;
    float32_t               paired_diagonal_distance_mm;
    bool                    b_outward_turn_left;
    bool                    b_outward_arc_extended;
    bool                    b_outward_forward_extended;
    bool                    b_outward_forward_monitor_active;
    bool                    b_outward_forward_center_settling;
    bool                    b_turning_inward;
    bool                    b_result_pending_clear;
    app_avoid_forward_leg_t forward_leg;
    app_side_scan_t         scan_45;
    app_side_scan_t         scan_135;
    buddy2_motion_state_t   forward_start_motion;
} app_avoid_ctx_t;

static bool         app_sonar_is_close (uint32_t distance_cm);
static bool         app_scan_angle_is_left (uint8_t angle_deg);
static void         app_side_scan_reset (app_side_scan_t * p_scan);
static const char * app_ping_failure_name (buddy5_ping_result_t result);
static bool     app_side_scan_ping (uint8_t angle_deg, app_side_scan_t * p_scan,
                                    uint32_t * p_average_cm);
static void     app_publish_avoidance_scan_values (uint32_t scan_45_cm,
                                                   uint32_t scan_135_cm);
static void     app_publish_avoidance_decision (uint32_t scan_45_cm,
                                                uint32_t scan_135_cm,
                                                bool     b_turn_left);
static int16_t  app_heading_change_x10 (int16_t start_heading_x10,
                                        int16_t now_heading_x10);
static uint16_t app_abs_heading_change_x10 (int16_t change_x10);
static int16_t app_heading_offset_x10 (int16_t heading_x10, int16_t offset_x10);
static bool    app_start_heading_arc (bool      b_turn_left,
                                      int16_t * p_start_heading_x10);
static uint8_t app_outward_monitor_angle (bool b_turn_left);
static void    app_check_outward_clearance (app_avoid_ctx_t * p_ctx);
static void    app_check_outward_forward_clearance (app_avoid_ctx_t * p_ctx);
static bool    app_start_clearance_reverse (app_avoid_ctx_t * p_ctx,
                                            uint32_t          now_ms);
static void    app_begin_side_scan (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void    app_end_avoidance (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                                  bool b_abort);
static void app_start_forward_leg (app_avoid_ctx_t * p_ctx,
                                   int16_t now_heading_x10, float32_t target_mm,
                                   app_avoid_forward_leg_t leg,
                                   uint32_t                now_ms);
static void app_restart_for_obstacle (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_step (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_watch_forward (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_reverse_clearance (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_side_scan_step (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_choose_route (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_outward_settle (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_turn_arc (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_arc_complete (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                              int16_t heading_x10);
static void app_forward_distance (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static bool app_forward_sonar_ready (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_forward_progress (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                                  int16_t heading_x10);
static void app_forward_leg_done (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                                  int16_t heading_error_x10);
static void app_forward_drive (int16_t heading_error_x10);
static void app_recover_heading (app_avoid_ctx_t * p_ctx, uint32_t now_ms);

/**
 * @brief Run the Buddy 5 obstacle-avoidance state machine every 100 ms.
 *
 * In normal operation it publishes only 90-degree forward ultrasonic
 * readings.  The 45- and 135-degree samples use non-telemetry side-scan
 * pings, so they stay in temporary variables and never replace dashboard
 * telemetry.
 */
void buddy5_avoidance_task (INT stacd, void * p_exinf)
{
    app_avoid_ctx_t ctx = {0};

    (void) stacd;
    (void) p_exinf;

    ctx.state                       = APP_AVOID_WATCH_FORWARD;
    ctx.scan_45_cm                  = BUDDY5_NO_ECHO;
    ctx.scan_135_cm                 = BUDDY5_NO_ECHO;
    ctx.outward_arc_target_x10      = APP_TURN_TARGET_X10;
    ctx.return_arc_target_x10       = APP_TURN_TARGET_X10;
    ctx.forward_target_mm           = APP_AVOID_FORWARD_DISTANCE_MM;
    ctx.paired_diagonal_distance_mm = APP_AVOID_FORWARD_DISTANCE_MM;
    ctx.forward_leg                 = APP_AVOID_FORWARD_OUTWARD_DIAGONAL;
    app_side_scan_reset(&ctx.scan_45);
    app_side_scan_reset(&ctx.scan_135);

    /* Normal monitoring starts with the ultrasonic sensor facing forward. */
    buddy5_set_servo_angle(APP_ULTRASONIC_ANGLE_DEG);
    (void) tk_dly_tsk(APP_SERVO_SETTLE_MS);

    for (;;)
    {
        app_step(&ctx, app_now_ms());
        (void) tk_dly_tsk(BUDDY5_AVOIDANCE_PERIOD_MS);
    }
}

/* ------------------------------------------------------------------ *
 *  State machine
 * ------------------------------------------------------------------ */

/* One 100 ms step: housekeeping, cancellation, then the current state. */
static void app_step (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    /* IMU is sampled by its independent owner task. */
    if (p_ctx->b_result_pending_clear &&
        ((now_ms - p_ctx->finished_ms) >= APP_AVOID_RESULT_VISIBLE_MS))
    {
        robot_telemetry_clear_avoidance_scan();
        p_ctx->b_result_pending_clear = false;
    }

    /* A dashboard command (including stop) cancels a scan safely. */
    if ((APP_AVOID_WATCH_FORWARD != p_ctx->state) &&
        !robot_controller_obstacle_avoidance_active())
    {
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_CANCELLED, 0,
                                             APP_TURN_TARGET_X10);
        p_ctx->finished_ms            = now_ms;
        p_ctx->b_result_pending_clear = true;
        buddy5_set_servo_angle(APP_ULTRASONIC_ANGLE_DEG);
        p_ctx->state = APP_AVOID_WATCH_FORWARD;
    }

    switch (p_ctx->state)
    {
        case APP_AVOID_WATCH_FORWARD:
            app_watch_forward(p_ctx, now_ms);
        break;

        case APP_AVOID_WAIT_REVERSE_CLEARANCE:
            app_reverse_clearance(p_ctx, now_ms);
        break;

        case APP_AVOID_WAIT_45:
        case APP_AVOID_WAIT_135:
            app_side_scan_step(p_ctx, now_ms);
        break;

        case APP_AVOID_WAIT_CENTER:
            app_choose_route(p_ctx, now_ms);
        break;

        case APP_AVOID_WAIT_OUTWARD_MONITOR_SETTLE:
            app_outward_settle(p_ctx, now_ms);
        break;

        case APP_AVOID_WAIT_TURN_ARC:
            app_turn_arc(p_ctx, now_ms);
        break;

        case APP_AVOID_WAIT_FORWARD_DISTANCE:
            app_forward_distance(p_ctx, now_ms);
        break;

        case APP_AVOID_WAIT_RECOVER_HEADING:
            app_recover_heading(p_ctx, now_ms);
        break;

        default:
            app_end_avoidance(p_ctx, now_ms, false);
        break;
    }
}

/* Step 1: watch the forward distance and start avoidance at 15 cm. */
static void app_watch_forward (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    uint32_t front_cm = buddy5_measure_cm(now_ms);

    /* robot_controller_begin_obstacle_avoidance() prepares the robot for
     * avoidance and stops line following. */
    if (app_sonar_is_close(front_cm) &&
        robot_controller_begin_obstacle_avoidance())
    {
        tm_printf((UB *) "[avoid] obstacle %u cm: backing %u mm before "
                         "scanning sides\n",
                  (UINT) front_cm, (UINT) APP_AVOID_REVERSE_DISTANCE_MM);
        app_restart_for_obstacle(p_ctx, now_ms);
    }
}

/* Back away from the obstacle, then scan both sides. */
static void app_reverse_clearance (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    buddy2_motion_state_t reverse_motion = {0};

    if (!buddy2_motion_active())
    {
        buddy2_get_motion_state(&reverse_motion);
        buddy2_stop();
        robot_telemetry_set_avoidance_turn_pwm(0, 0);
        if (reverse_motion.left_encoder_fault ||
            reverse_motion.right_encoder_fault)
        {
            tm_printf((UB *) "[avoid] encoder fault while backing away; "
                             "remaining stopped\n");
            robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_BLOCKED, 0,
                                                 0u);
            robot_controller_abort_obstacle_avoidance();
            buddy5_set_servo_angle(APP_ULTRASONIC_ANGLE_DEG);
            p_ctx->state                  = APP_AVOID_WATCH_FORWARD;
            p_ctx->finished_ms            = now_ms;
            p_ctx->b_result_pending_clear = true;
        }
        else
        {
            tm_printf((UB *) "[avoid] backed away %u mm; scanning sides\n",
                      (UINT) APP_AVOID_REVERSE_DISTANCE_MM);
            app_begin_side_scan(p_ctx, now_ms);
        }
    }
    else if ((now_ms - p_ctx->state_started_ms) >= APP_AVOID_REVERSE_TIMEOUT_MS)
    {
        buddy2_stop();
        tm_printf((UB *) "[avoid] reverse clearance timed out; remaining "
                         "stopped\n");
        robot_telemetry_set_avoidance_turn_pwm(0, 0);
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_TIMEOUT, 0, 0u);
        app_end_avoidance(p_ctx, now_ms, true);
    }
    else
    {
        /* Still reversing. */
    }
}

/* Steps 2 and 3: ping one side until its scan completes, then move on (45
 * degrees, then 135 degrees, then back to the centre). */
static void app_side_scan_step (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    bool b_at_45 = (APP_AVOID_WAIT_45 == p_ctx->state);

    if (((now_ms - p_ctx->state_started_ms) >= APP_SERVO_SETTLE_MS) &&
        ((now_ms - p_ctx->side_scan_last_ping_ms) >= APP_SONAR_PING_SPACING_MS))
    {
        p_ctx->side_scan_last_ping_ms = now_ms;
        if (b_at_45)
        {
            if (app_side_scan_ping(APP_SCAN_ANGLE_45_DEG, &p_ctx->scan_45,
                                   &p_ctx->scan_45_cm))
            {
                /* Point at 135 degrees with an empty temporary scan. */
                buddy5_set_servo_angle(APP_SCAN_ANGLE_135_DEG);
                app_side_scan_reset(&p_ctx->scan_135);
                p_ctx->state_started_ms       = now_ms;
                p_ctx->side_scan_last_ping_ms = now_ms;
                p_ctx->state                  = APP_AVOID_WAIT_135;
            }
        }
        else if (app_side_scan_ping(APP_SCAN_ANGLE_135_DEG, &p_ctx->scan_135,
                                    &p_ctx->scan_135_cm))
        {
            buddy5_set_servo_angle(APP_ULTRASONIC_ANGLE_DEG);
            p_ctx->state_started_ms = now_ms;
            p_ctx->state            = APP_AVOID_WAIT_CENTER;
        }
        else
        {
            /* The 135-degree scan needs more pings. */
        }
    }
}

/* Step 4: decide to turn left or right, or stay stopped. */
static void app_choose_route (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    uint8_t chosen_angle     = APP_SCAN_ANGLE_135_DEG;
    bool    b_turn_left      = false;
    bool    b_scan_45_valid  = (BUDDY5_NO_ECHO != p_ctx->scan_45_cm);
    bool    b_scan_135_valid = (BUDDY5_NO_ECHO != p_ctx->scan_135_cm);
    bool    b_scan_45_usable =
        b_scan_45_valid && (p_ctx->scan_45_cm >= APP_AVOID_MIN_CLEARANCE_CM);
    bool b_scan_135_usable =
        b_scan_135_valid && (p_ctx->scan_135_cm >= APP_AVOID_MIN_CLEARANCE_CM);

    if ((now_ms - p_ctx->state_started_ms) < APP_SERVO_SETTLE_MS)
    {
        /* Wait for the servo to face forward again. */
    }
    else
    {
        /* Publish both physical-side readings before deciding. This keeps a
         * blocked route visible in the dashboard monitor. */
        app_publish_avoidance_scan_values(p_ctx->scan_45_cm,
                                          p_ctx->scan_135_cm);

        /* A single reading is not enough to prove a safe route.  In
         * particular, never turn into the only side that returned an echo
         * while the other physical side failed its scan.  The dashboard
         * keeps both values visible and the rover remains stopped for an
         * operator re-scan. */
        if (!b_scan_45_valid || !b_scan_135_valid)
        {
            tm_printf(
                (UB *) "[avoid] incomplete side scan; no turn: 45 deg "
                       "(%s) valid=%u/%u, 135 deg (%s) valid=%u/%u\n",
                app_scan_angle_is_left(APP_SCAN_ANGLE_45_DEG) ? "left"
                                                              : "right",
                (UINT) p_ctx->scan_45.valid, (UINT) p_ctx->scan_45.attempts,
                app_scan_angle_is_left(APP_SCAN_ANGLE_135_DEG) ? "left"
                                                               : "right",
                (UINT) p_ctx->scan_135.valid, (UINT) p_ctx->scan_135.attempts);
            robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_BLOCKED, 0,
                                                 0u);
            app_end_avoidance(p_ctx, now_ms, true);
        }
        else if (!b_scan_45_usable && !b_scan_135_usable)
        {
            tm_printf((UB *) "[avoid] no usable route: 45=%u cm 135=%u cm "
                             "(need at least %u cm); remaining stopped\n",
                      (UINT) p_ctx->scan_45_cm, (UINT) p_ctx->scan_135_cm,
                      (UINT) APP_AVOID_MIN_CLEARANCE_CM);
            robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_BLOCKED, 0,
                                                 0u);
            app_end_avoidance(p_ctx, now_ms, true);
        }
        else
        {
            /* Both sides were validated above; tight scans cannot win. */
            if (b_scan_45_usable && (!b_scan_135_usable ||
                                     (p_ctx->scan_45_cm >= p_ctx->scan_135_cm)))
            {
                chosen_angle = APP_SCAN_ANGLE_45_DEG;
            }
            b_turn_left                = app_scan_angle_is_left(chosen_angle);
            p_ctx->b_outward_turn_left = b_turn_left;
            tm_printf((UB *) "[avoid] choose %u deg (%s): %u cm vs %u cm\n",
                      (UINT) chosen_angle, b_turn_left ? "left" : "right",
                      (UINT) p_ctx->scan_45_cm, (UINT) p_ctx->scan_135_cm);
            app_publish_avoidance_decision(p_ctx->scan_45_cm,
                                           p_ctx->scan_135_cm, b_turn_left);
            /* The selected direction is now in telemetry; scan values are no
             * longer needed by the turning state. */
            p_ctx->scan_45_cm  = BUDDY5_NO_ECHO;
            p_ctx->scan_135_cm = BUDDY5_NO_ECHO;
            app_side_scan_reset(&p_ctx->scan_45);
            app_side_scan_reset(&p_ctx->scan_135);
            p_ctx->outward_arc_target_x10     = APP_TURN_TARGET_X10;
            p_ctx->return_arc_target_x10      = p_ctx->outward_arc_target_x10;
            p_ctx->b_outward_arc_extended     = false;
            p_ctx->outward_monitor_started_ms = 0u;
            buddy5_set_servo_angle(app_outward_monitor_angle(b_turn_left));
            tm_printf((UB *) "[avoid] turn-out %s: monitor %s side for first "
                             "%u ms of arc\n",
                      b_turn_left ? "left" : "right",
                      b_turn_left ? "right" : "left",
                      (UINT) APP_AVOID_OBSTACLE_SIDE_MONITOR_MS);
            p_ctx->state_started_ms = now_ms;
            p_ctx->state            = APP_AVOID_WAIT_OUTWARD_MONITOR_SETTLE;
        }
    }
}

/* Let the servo settle on the obstacle side, then start the outward arc. */
static void app_outward_settle (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    if ((now_ms - p_ctx->state_started_ms) < APP_SERVO_SETTLE_MS)
    {
        /* Servo still moving. */
    }
    else if (!app_start_heading_arc(p_ctx->b_outward_turn_left,
                                    &p_ctx->arc_start_heading_x10))
    {
        tm_printf((UB *) "[avoid] IMU heading unavailable; remaining "
                         "stopped\n");
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_IMU_UNAVAILABLE,
                                             0, p_ctx->outward_arc_target_x10);
        app_end_avoidance(p_ctx, now_ms, true);
    }
    else
    {
        robot_telemetry_set_avoidance_turn_pwm(
            p_ctx->b_outward_turn_left ? APP_AVOID_ARC_INNER_PWM
                                       : APP_AVOID_RIGHT_TURN_OUTER_PWM,
            p_ctx->b_outward_turn_left ? APP_AVOID_LEFT_TURN_OUTER_PWM
                                       : APP_AVOID_ARC_INNER_PWM);
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_TURNING, 0,
                                             p_ctx->outward_arc_target_x10);
        /* The start heading is a valid compass value, 0..3599. */
        tm_printf((UB *) "[avoid] forward arc from %u.%u deg; target change "
                         "%u.%u deg\n",
                  (UINT) (p_ctx->arc_start_heading_x10 / 10),
                  (UINT) (p_ctx->arc_start_heading_x10 % 10),
                  (UINT) (p_ctx->outward_arc_target_x10 / 10u),
                  (UINT) (p_ctx->outward_arc_target_x10 % 10u));
        /* Start the finite opposite-side check with the motors now moving.
         * A timeout/no echo never stalls this turn. */
        p_ctx->outward_monitor_started_ms   = now_ms;
        p_ctx->outward_monitor_last_ping_ms = now_ms;
        p_ctx->state_started_ms             = now_ms;
        p_ctx->state                        = APP_AVOID_WAIT_TURN_ARC;
    }
}

/* Step 5: turn out (or back in) until the IMU heading changed by the
 * target, about 30 degrees. */
static void app_turn_arc (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    buddy4_imu_state_t imu                = {0};
    int16_t            heading_change_x10 = 0;
    uint16_t           turn_target_x10    = 0u;
    bool               b_arc_turn_left    = false;
    int32_t            directed_change    = 0;

    /* The final turn can reach the line before the diagonal starts.  Buddy 3
     * latches its 10 ms readings, so a brief black-line observation is not
     * missed by this 100 ms task.  The line has priority over all remaining
     * avoidance movement. */
    if (p_ctx->b_turning_inward && buddy3_take_line_seen_event())
    {
        buddy2_stop();
        tm_printf((UB *) "[avoid] line captured during turn-in; handing back "
                         "to line following\n");
        robot_telemetry_set_avoidance_turn_pwm(0, 0);
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_COMPLETE, 0,
                                             0u);
        app_end_avoidance(p_ctx, now_ms, false);
    }
    else
    {
        buddy4_get_state(&imu);
        if (!imu.mag_ok)
        {
            buddy2_stop();
            tm_printf((UB *) "[avoid] IMU heading lost during arc; remaining "
                             "stopped\n");
            robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_IMU_LOST, 0,
                                                 APP_TURN_TARGET_X10);
            app_end_avoidance(p_ctx, now_ms, false);
        }
        else
        {
            /* While turning out, watch only the obstacle-side for the first
             * two seconds. Left turns watch right (45 degrees); right turns
             * watch left (135 degrees). */
            if (!p_ctx->b_turning_inward &&
                ((now_ms - p_ctx->outward_monitor_started_ms) <=
                 APP_AVOID_OBSTACLE_SIDE_MONITOR_MS) &&
                ((now_ms - p_ctx->outward_monitor_last_ping_ms) >=
                 APP_SONAR_PING_SPACING_MS))
            {
                p_ctx->outward_monitor_last_ping_ms = now_ms;
                app_check_outward_clearance(p_ctx);
            }
            heading_change_x10 = app_heading_change_x10(
                p_ctx->arc_start_heading_x10, imu.heading_deg_x10);
            turn_target_x10 = p_ctx->b_turning_inward
                                  ? p_ctx->return_arc_target_x10
                                  : p_ctx->outward_arc_target_x10;
            robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_TURNING,
                                                 heading_change_x10,
                                                 turn_target_x10);
            /* Compass heading increases on right turns. Only progress in the
             * commanded direction can complete this arc. */
            b_arc_turn_left = p_ctx->b_turning_inward
                                  ? !p_ctx->b_outward_turn_left
                                  : p_ctx->b_outward_turn_left;
            directed_change = b_arc_turn_left ? -(int32_t) heading_change_x10
                                              : heading_change_x10;
            if (directed_change >= (int32_t) turn_target_x10)
            {
                tm_printf((UB *) "[avoid] %s arc changed heading by %d.%d "
                                 "deg; stopped\n",
                          p_ctx->b_turning_inward ? "turn-in" : "turn-out",
                          (INT) (heading_change_x10 / 10),
                          (INT) app_abs_heading_change_x10(
                              (int16_t) (heading_change_x10 % 10)));
                app_arc_complete(p_ctx, now_ms, imu.heading_deg_x10);
            }
            else if ((now_ms - p_ctx->state_started_ms) >=
                     APP_AVOID_TURN_TIMEOUT_MS)
            {
                buddy2_stop();
                tm_printf((UB *) "[avoid] IMU arc timeout; remaining "
                                 "stopped\n");
                robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_TIMEOUT,
                                                     heading_change_x10,
                                                     APP_TURN_TARGET_X10);
                app_end_avoidance(p_ctx, now_ms, false);
            }
            else
            {
                /* Still turning. */
            }
        }
    }
}

/* An arc reached its heading target: start the matching diagonal leg. */
static void app_arc_complete (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                              int16_t heading_x10)
{
    if (!p_ctx->b_turning_inward)
    {
        buddy2_stop();
        robot_telemetry_set_avoidance_turn_pwm(0, 0);
        /* The sensor has been looking at the obstacle-side throughout the
         * arc. Keep it there while moving the first diagonal leg, rather
         * than stopping for another two-second clearance check. */
        p_ctx->paired_diagonal_distance_mm = APP_AVOID_FORWARD_DISTANCE_MM;
        /* The later turn-in must mirror the actual outer arc, including any
         * extension made while turning. */
        p_ctx->return_arc_target_x10            = p_ctx->outward_arc_target_x10;
        p_ctx->b_outward_forward_extended       = false;
        p_ctx->b_outward_forward_monitor_active = true;
        p_ctx->b_outward_forward_center_settling    = false;
        p_ctx->outward_forward_monitor_started_ms   = now_ms;
        p_ctx->outward_forward_monitor_last_ping_ms = now_ms;
        /* Distances are a few hundred millimetres, so they fit UINT. */
        tm_printf((UB *) "[avoid] outward diagonal forward %u mm; monitor %s "
                         "side for first %u ms while moving\n",
                  (UINT) p_ctx->paired_diagonal_distance_mm,
                  p_ctx->b_outward_turn_left ? "right" : "left",
                  (UINT) APP_AVOID_OBSTACLE_SIDE_MONITOR_MS);
        app_start_forward_leg(p_ctx, heading_x10,
                              p_ctx->paired_diagonal_distance_mm,
                              APP_AVOID_FORWARD_OUTWARD_DIAGONAL, now_ms);
    }
    else
    {
        /* Mirror any extra outward travel so the matching return diagonal
         * does not leave the rover offset. */
        buddy5_set_servo_angle(APP_ULTRASONIC_ANGLE_DEG);
        tm_printf((UB *) "[avoid] inward diagonal forward %u mm (matching "
                         "outward)\n",
                  (UINT) p_ctx->paired_diagonal_distance_mm);
        app_start_forward_leg(p_ctx, heading_x10,
                              p_ctx->paired_diagonal_distance_mm,
                              APP_AVOID_FORWARD_INWARD_DIAGONAL, now_ms);
    }
}

/* Step 6: drive a heading-held forward leg. */
static void app_forward_distance (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    buddy4_imu_state_t imu      = {0};
    uint32_t           front_cm = BUDDY5_NO_ECHO;
    bool               b_busy   = false;

    buddy4_get_state(&imu);
    if (!imu.mag_ok)
    {
        buddy2_stop();
        tm_printf((UB *) "[avoid] IMU heading lost during forward move; "
                         "remaining stopped\n");
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_IMU_LOST, 0,
                                             0u);
        app_end_avoidance(p_ctx, now_ms, false);
        b_busy = true;
    }
    /* The final inward diagonal has reached the line.  This event is
     * latched by Buddy 3's 10 ms sampling task. */
    else if ((APP_AVOID_FORWARD_INWARD_DIAGONAL == p_ctx->forward_leg) &&
             buddy3_take_line_seen_event())
    {
        buddy2_stop();
        tm_printf((UB *) "[avoid] line captured during inward diagonal; "
                         "handing back to line following\n");
        robot_telemetry_set_avoidance_turn_pwm(0, 0);
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_COMPLETE, 0,
                                             0u);
        app_end_avoidance(p_ctx, now_ms, false);
        b_busy = true;
    }
    else if (app_forward_sonar_ready(p_ctx, now_ms))
    {
        /* Once the sensor is forward-facing, a new front obstacle interrupts
         * either forward leg and restarts avoidance. */
        front_cm = buddy5_measure_cm(now_ms);
        if (app_sonar_is_close(front_cm))
        {
            tm_printf((UB *) "[avoid] new front obstacle %u cm during "
                             "forward; backing away before rescan\n",
                      (UINT) front_cm);
            app_restart_for_obstacle(p_ctx, now_ms);
            b_busy = true;
        }
    }
    else
    {
        /* Sonar is looking sideways or settling: no forward check now. */
    }

    if (!b_busy)
    {
        app_forward_progress(p_ctx, now_ms, imu.heading_deg_x10);
    }
}

/* Run the obstacle-side monitor of the first diagonal and the sonar
 * re-centring.  Returns true when the sonar faces forward and is settled. */
static bool app_forward_sonar_ready (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    bool b_ready = true;

    /* The first diagonal continues to look at the obstacle-side for two
     * seconds while moving. A close side reading grows both matching
     * diagonal distances together. */
    if ((APP_AVOID_FORWARD_OUTWARD_DIAGONAL == p_ctx->forward_leg) &&
        p_ctx->b_outward_forward_monitor_active)
    {
        b_ready = false;
        if ((now_ms - p_ctx->outward_forward_monitor_started_ms) <=
            APP_AVOID_OBSTACLE_SIDE_MONITOR_MS)
        {
            if ((now_ms - p_ctx->outward_forward_monitor_last_ping_ms) >=
                APP_SONAR_PING_SPACING_MS)
            {
                p_ctx->outward_forward_monitor_last_ping_ms = now_ms;
                app_check_outward_forward_clearance(p_ctx);
                p_ctx->forward_target_mm = p_ctx->paired_diagonal_distance_mm;
            }
        }
        else
        {
            /* Keep driving while the sensor goes back to forward view; only
             * forward pings pause during servo settle. */
            p_ctx->b_outward_forward_monitor_active  = false;
            p_ctx->b_outward_forward_center_settling = true;
            p_ctx->forward_sonar_center_started_ms   = now_ms;
            buddy5_set_servo_angle(APP_ULTRASONIC_ANGLE_DEG);
            tm_printf((UB *) "[avoid] outward-forward side monitor complete; "
                             "centring sonar while moving\n");
        }
    }

    if (p_ctx->b_outward_forward_center_settling)
    {
        if ((now_ms - p_ctx->forward_sonar_center_started_ms) <
            APP_SERVO_SETTLE_MS)
        {
            b_ready = false;
        }
        else
        {
            p_ctx->b_outward_forward_center_settling = false;
        }
    }

    return b_ready;
}

/* A forward leg reached its distance: finish, recover heading, or turn in. */
static void app_forward_leg_done (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                                  int16_t heading_error_x10)
{
    bool b_inward_turn_left = !p_ctx->b_outward_turn_left;

    buddy2_stop();
    robot_telemetry_set_avoidance_turn_pwm(0, 0);
    if (APP_AVOID_FORWARD_INWARD_DIAGONAL == p_ctx->forward_leg)
    {
        tm_printf((UB *) "[avoid] inward diagonal forward %u mm complete\n",
                  (UINT) p_ctx->forward_target_mm);
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_COMPLETE,
                                             heading_error_x10, 0u);
        app_end_avoidance(p_ctx, now_ms, false);
    }
    else if (APP_AVOID_FORWARD_OUTWARD_DIAGONAL == p_ctx->forward_leg)
    {
        /* A short outward leg can finish before the two-second monitor
         * expires. Centre the sonar before the next route segment then. */
        if (p_ctx->b_outward_forward_monitor_active)
        {
            p_ctx->b_outward_forward_monitor_active  = false;
            p_ctx->b_outward_forward_center_settling = true;
            p_ctx->forward_sonar_center_started_ms   = now_ms;
            buddy5_set_servo_angle(APP_ULTRASONIC_ANGLE_DEG);
        }
        tm_printf((UB *) "[avoid] heading-held forward %u mm complete; "
                         "recovering start heading\n",
                  (UINT) p_ctx->forward_target_mm);
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_RECOVERING,
                                             heading_error_x10,
                                             APP_AVOID_RECOVER_TOLERANCE_X10);
        p_ctx->state_started_ms = now_ms;
        p_ctx->state            = APP_AVOID_WAIT_RECOVER_HEADING;
    }
    /* The 25 cm straight section is complete.  Turn in the opposite
     * direction of the original outward arc, then take the final diagonal
     * leg back toward the route. */
    else if (!app_start_heading_arc(b_inward_turn_left,
                                    &p_ctx->arc_start_heading_x10))
    {
        tm_printf((UB *) "[avoid] IMU unavailable for turn-in arc; remaining "
                         "stopped\n");
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_IMU_UNAVAILABLE,
                                             0, p_ctx->return_arc_target_x10);
        app_end_avoidance(p_ctx, now_ms, false);
    }
    else
    {
        p_ctx->b_turning_inward = true;
        /* Only black seen during the final return-to-line phase may finish
         * avoidance. */
        buddy3_clear_line_seen_event();
        robot_telemetry_set_avoidance_turn_pwm(
            b_inward_turn_left ? APP_AVOID_ARC_INNER_PWM
                               : APP_AVOID_RIGHT_TURN_OUTER_PWM,
            b_inward_turn_left ? APP_AVOID_LEFT_TURN_OUTER_PWM
                               : APP_AVOID_ARC_INNER_PWM);
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_TURNING, 0,
                                             p_ctx->return_arc_target_x10);
        tm_printf((UB *) "[avoid] straight forward complete; turn-in %s arc, "
                         "target change %u.%u deg\n",
                  b_inward_turn_left ? "left" : "right",
                  (UINT) (p_ctx->return_arc_target_x10 / 10u),
                  (UINT) (p_ctx->return_arc_target_x10 % 10u));
        p_ctx->state_started_ms = now_ms;
        p_ctx->state            = APP_AVOID_WAIT_TURN_ARC;
    }
}

/* Proportional heading hold for a forward leg. */
static void app_forward_drive (int16_t heading_error_x10)
{
    int32_t correction_pwm = 0;
    int16_t left_pwm       = 0;
    int16_t right_pwm      = 0;

    if ((heading_error_x10 > APP_AVOID_FORWARD_HEADING_DEADBAND_X10) ||
        (heading_error_x10 < -APP_AVOID_FORWARD_HEADING_DEADBAND_X10))
    {
        correction_pwm =
            (int32_t) heading_error_x10 * APP_AVOID_FORWARD_HEADING_KP;
    }
    if (correction_pwm > APP_AVOID_FORWARD_MAX_CORRECTION_PWM)
    {
        correction_pwm = APP_AVOID_FORWARD_MAX_CORRECTION_PWM;
    }
    else if (correction_pwm < -APP_AVOID_FORWARD_MAX_CORRECTION_PWM)
    {
        correction_pwm = -APP_AVOID_FORWARD_MAX_CORRECTION_PWM;
    }
    else
    {
        /* Within the correction limit. */
    }

    /* Positive compass error is right of the target: correct left. Negative
     * error needs the opposite correction.  Base +/- 2500 fits int16_t. */
    left_pwm  = (int16_t) (APP_AVOID_FORWARD_BASE_PWM - correction_pwm);
    right_pwm = (int16_t) (APP_AVOID_FORWARD_BASE_PWM + correction_pwm);
    buddy2_set_motor_pwm(left_pwm, right_pwm);
    robot_telemetry_set_avoidance_turn_pwm(left_pwm, right_pwm);
    robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_FORWARD,
                                         heading_error_x10, 0u);
}

/* Curve back to the heading saved before the outward arc. */
static void app_recover_heading (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    buddy4_imu_state_t imu               = {0};
    int16_t            heading_error_x10 = 0;
    int16_t            left_pwm          = 0;
    int16_t            right_pwm         = 0;

    buddy4_get_state(&imu);
    if (!imu.mag_ok)
    {
        buddy2_stop();
        tm_printf((UB *) "[avoid] IMU heading lost during recovery; remaining "
                         "stopped\n");
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_IMU_LOST, 0,
                                             APP_AVOID_RECOVER_TOLERANCE_X10);
        app_end_avoidance(p_ctx, now_ms, false);
    }
    else
    {
        heading_error_x10 = app_heading_change_x10(p_ctx->arc_start_heading_x10,
                                                   imu.heading_deg_x10);
        if (app_abs_heading_change_x10(heading_error_x10) <=
            APP_AVOID_RECOVER_TOLERANCE_X10)
        {
            buddy2_stop();
            tm_printf((UB *) "[avoid] recovered start heading (error %d.%d "
                             "deg); starting final %u mm forward\n",
                      (INT) (heading_error_x10 / 10),
                      (INT) app_abs_heading_change_x10(
                          (int16_t) (heading_error_x10 % 10)),
                      (UINT) APP_AVOID_FINAL_FORWARD_DISTANCE_MM);
            app_start_forward_leg(p_ctx, imu.heading_deg_x10,
                                  APP_AVOID_FINAL_FORWARD_DISTANCE_MM,
                                  APP_AVOID_FORWARD_STRAIGHT, now_ms);
        }
        else if ((now_ms - p_ctx->state_started_ms) >=
                 APP_AVOID_RECOVER_TIMEOUT_MS)
        {
            buddy2_stop();
            tm_printf((UB *) "[avoid] heading recovery timed out at %d.%d deg; "
                             "remaining stopped\n",
                      (INT) (heading_error_x10 / 10),
                      (INT) app_abs_heading_change_x10(
                          (int16_t) (heading_error_x10 % 10)));
            robot_telemetry_set_avoidance_turn_pwm(0, 0);
            robot_telemetry_set_avoidance_status(
                ROBOT_AVOID_STATUS_TIMEOUT, heading_error_x10,
                APP_AVOID_RECOVER_TOLERANCE_X10);
            app_end_avoidance(p_ctx, now_ms, false);
        }
        else
        {
            /* Negative compass error is left of the original heading, so
             * curve right; positive error curves left. */
            if (heading_error_x10 < 0)
            {
                left_pwm  = APP_AVOID_RIGHT_TURN_OUTER_PWM;
                right_pwm = APP_AVOID_ARC_INNER_PWM;
            }
            else
            {
                left_pwm  = APP_AVOID_ARC_INNER_PWM;
                right_pwm = APP_AVOID_LEFT_TURN_OUTER_PWM;
            }
            buddy2_set_motor_pwm(left_pwm, right_pwm);
            robot_telemetry_set_avoidance_turn_pwm(left_pwm, right_pwm);
            robot_telemetry_set_avoidance_status(
                ROBOT_AVOID_STATUS_RECOVERING, heading_error_x10,
                APP_AVOID_RECOVER_TOLERANCE_X10);
        }
    }
}

/* ------------------------------------------------------------------ *
 *  Helpers
 * ------------------------------------------------------------------ */

/** @brief Return true for a valid ultrasonic reading at or inside the stop
 * distance. */
static bool app_sonar_is_close (uint32_t distance_cm)
{
    return ((BUDDY5_NO_ECHO != distance_cm) &&
            (distance_cm <= APP_OBSTACLE_STOP_CM));
}

/**
 * @brief Convert a servo scan angle to the rover's physical left/right side.
 *
 * APP_SCAN_45_IS_LEFT documents the servo installation and can be flipped if
 * the servo is mounted as a mirror image.
 */
static bool app_scan_angle_is_left (uint8_t angle_deg)
{
    bool b_left = (0u == APP_SCAN_45_IS_LEFT);

    if (APP_SCAN_ANGLE_45_DEG == angle_deg)
    {
        b_left = (0u != APP_SCAN_45_IS_LEFT);
    }

    return b_left;
}

/** @brief Clear the temporary readings and error counters for one angle. */
static void app_side_scan_reset (app_side_scan_t * p_scan)
{
    p_scan->attempts            = 0u;
    p_scan->valid               = 0u;
    p_scan->timeout_rise        = 0u;
    p_scan->timeout_fall        = 0u;
    p_scan->high_before_trigger = 0u;
    p_scan->total_cm            = 0u;
}

/** @brief Return readable text for an ultrasonic failure reported by
 * Buddy 5. */
static const char * app_ping_failure_name (buddy5_ping_result_t result)
{
    const char * p_name = "echo HIGH before trigger";

    if (BUDDY5_PING_TIMEOUT_RISE == result)
    {
        p_name = "timeout waiting for rise";
    }
    else if (BUDDY5_PING_TIMEOUT_FALL == result)
    {
        p_name = "timeout waiting for fall";
    }
    else
    {
        p_name = "echo HIGH before trigger";
    }

    return p_name;
}

/**
 * @brief Take one temporary side-scan ping and report whether the scan is
 * complete.
 *
 * A scan completes after five valid readings or twelve total attempts.
 * Invalid readings are retried, but fewer than three valid readings reject
 * that side.
 */
static bool app_side_scan_ping (uint8_t angle_deg, app_side_scan_t * p_scan,
                                uint32_t * p_average_cm)
{
    buddy5_ping_result_t result      = BUDDY5_PING_TIMEOUT_RISE;
    uint32_t             distance_cm = buddy5_sample_with_result_cm(&result);
    bool                 b_complete  = false;

    p_scan->attempts++;
    if (BUDDY5_PING_VALID == result)
    {
        p_scan->valid++;
        p_scan->total_cm += distance_cm;
    }
    else
    {
        if (BUDDY5_PING_TIMEOUT_RISE == result)
        {
            p_scan->timeout_rise++;
        }
        else if (BUDDY5_PING_TIMEOUT_FALL == result)
        {
            p_scan->timeout_fall++;
        }
        else
        {
            p_scan->high_before_trigger++;
        }
        tm_printf((UB *) "[avoid] scan %u deg (%s) attempt %u/%u failed: %s\n",
                  (UINT) angle_deg,
                  app_scan_angle_is_left(angle_deg) ? "left" : "right",
                  (UINT) p_scan->attempts,
                  (UINT) APP_AVOID_SIDE_SCAN_MAX_ATTEMPTS,
                  app_ping_failure_name(result));
    }

    b_complete = (p_scan->valid >= APP_AVOID_SIDE_SCAN_VALID_SAMPLES) ||
                 (p_scan->attempts >= APP_AVOID_SIDE_SCAN_MAX_ATTEMPTS);
    if (b_complete && (p_scan->valid >= APP_AVOID_SIDE_SCAN_MIN_VALID))
    {
        *p_average_cm = p_scan->total_cm / p_scan->valid;
        tm_printf((UB *) "[avoid] scan %u deg (%s): valid=%u/%u average=%u "
                         "cm rise_timeout=%u fall_timeout=%u "
                         "high_before_trigger=%u\n",
                  (UINT) angle_deg,
                  app_scan_angle_is_left(angle_deg) ? "left" : "right",
                  (UINT) p_scan->valid, (UINT) p_scan->attempts,
                  (UINT) *p_average_cm, (UINT) p_scan->timeout_rise,
                  (UINT) p_scan->timeout_fall,
                  (UINT) p_scan->high_before_trigger);
    }
    else if (b_complete)
    {
        *p_average_cm = BUDDY5_NO_ECHO;
        tm_printf((UB *) "[avoid] scan %u deg (%s) rejected: valid=%u/%u "
                         "rise_timeout=%u fall_timeout=%u "
                         "high_before_trigger=%u\n",
                  (UINT) angle_deg,
                  app_scan_angle_is_left(angle_deg) ? "left" : "right",
                  (UINT) p_scan->valid, (UINT) p_scan->attempts,
                  (UINT) p_scan->timeout_rise, (UINT) p_scan->timeout_fall,
                  (UINT) p_scan->high_before_trigger);
    }
    else
    {
        /* More pings needed. */
    }

    return b_complete;
}

/**
 * @brief Copy side-scan values to rover/debug before choosing a route.
 *
 * This makes blocked readings visible without changing Buddy 5's saved
 * forward-facing distance, which remains the dashboard ultrasonic telemetry.
 */
static void app_publish_avoidance_scan_values (uint32_t scan_45_cm,
                                               uint32_t scan_135_cm)
{
    if (app_scan_angle_is_left(APP_SCAN_ANGLE_45_DEG))
    {
        robot_telemetry_set_avoidance_scan_values(scan_45_cm, scan_135_cm);
    }
    else
    {
        robot_telemetry_set_avoidance_scan_values(scan_135_cm, scan_45_cm);
    }
}

/** @brief Publish the selected left/right decision without formatting
 * telemetry here. */
static void app_publish_avoidance_decision (uint32_t scan_45_cm,
                                            uint32_t scan_135_cm,
                                            bool     b_turn_left)
{
    if (app_scan_angle_is_left(APP_SCAN_ANGLE_45_DEG))
    {
        robot_telemetry_set_avoidance_scan(scan_45_cm, scan_135_cm,
                                           b_turn_left);
    }
    else
    {
        robot_telemetry_set_avoidance_scan(scan_135_cm, scan_45_cm,
                                           b_turn_left);
    }
}

/**
 * @brief Return the signed shortest heading change from start to current.
 *
 * Headings are stored in tenths of a degree (0..3599).  The wrap handling
 * means a transition from 359 degrees to 0 degrees is treated as +1 degree,
 * not a 359-degree turn.
 */
static int16_t app_heading_change_x10 (int16_t start_heading_x10,
                                       int16_t now_heading_x10)
{
    int32_t change_x10 =
        (int32_t) now_heading_x10 - (int32_t) start_heading_x10;

    if (change_x10 > APP_HALF_TURN_X10)
    {
        change_x10 -= APP_FULL_TURN_X10;
    }
    else if (change_x10 < -APP_HALF_TURN_X10)
    {
        change_x10 += APP_FULL_TURN_X10;
    }
    else
    {
        /* Already the shortest way round. */
    }

    /* Within +/-1800 here. */
    return (int16_t) change_x10;
}

/** @brief Return the non-negative magnitude of a signed heading change. */
static uint16_t app_abs_heading_change_x10 (int16_t change_x10)
{
    /* |int16_t| is at most 32768, which fits uint16_t. */
    return (change_x10 < 0) ? (uint16_t) (-(int32_t) change_x10)
                            : (uint16_t) change_x10;
}

/** @brief Add a signed tenth-degree change and retain a compass value in
 * 0..3599. */
static int16_t app_heading_offset_x10 (int16_t heading_x10, int16_t offset_x10)
{
    int32_t adjusted_x10 = (int32_t) heading_x10 + (int32_t) offset_x10;

    while (adjusted_x10 < 0)
    {
        adjusted_x10 += APP_FULL_TURN_X10;
    }
    while (adjusted_x10 >= APP_FULL_TURN_X10)
    {
        adjusted_x10 -= APP_FULL_TURN_X10;
    }

    /* 0..3599 here. */
    return (int16_t) adjusted_x10;
}

/**
 * @brief Record the current heading and begin a forward arc to the chosen
 * side.
 *
 * A left arc uses a slower left wheel and faster right wheel; a right arc
 * swaps those commands.  Both values remain positive, so the rover moves
 * forward while turning rather than rotating on the spot.
 *
 * @return true if a valid compass heading was available and the arc started.
 */
static bool app_start_heading_arc (bool      b_turn_left,
                                   int16_t * p_start_heading_x10)
{
    buddy4_imu_state_t imu       = {0};
    bool               b_started = false;

    if (NULL != p_start_heading_x10)
    {
        buddy4_get_state(&imu);
        if (imu.mag_ok)
        {
            *p_start_heading_x10 = imu.heading_deg_x10;
            if (b_turn_left)
            {
                buddy2_set_motor_pwm(APP_AVOID_ARC_INNER_PWM,
                                     APP_AVOID_LEFT_TURN_OUTER_PWM);
            }
            else
            {
                buddy2_set_motor_pwm(APP_AVOID_RIGHT_TURN_OUTER_PWM,
                                     APP_AVOID_ARC_INNER_PWM);
            }
            b_started = true;
        }
    }

    return b_started;
}

/** @brief Return the scan angle looking toward the obstacle-side of an
 * outward arc. */
static uint8_t app_outward_monitor_angle (bool b_turn_left)
{
    uint8_t angle = APP_SCAN_ANGLE_135_DEG;

    if (app_scan_angle_is_left(APP_SCAN_ANGLE_45_DEG) != b_turn_left)
    {
        angle = APP_SCAN_ANGLE_45_DEG;
    }

    return angle;
}

/** @brief Extend the outward turn once when the obstacle-side remains too
 * close. */
static void app_check_outward_clearance (app_avoid_ctx_t * p_ctx)
{
    bool                 b_turn_left   = p_ctx->b_outward_turn_left;
    buddy5_ping_result_t result        = BUDDY5_PING_TIMEOUT_RISE;
    uint32_t             distance_cm   = buddy5_sample_with_result_cm(&result);
    uint8_t              monitor_angle = app_outward_monitor_angle(b_turn_left);

    if (BUDDY5_PING_VALID != result)
    {
        tm_printf(
            (UB *) "[avoid] outward monitor %s (%u deg) unavailable: %s\n",
            b_turn_left ? "right" : "left", (UINT) monitor_angle,
            app_ping_failure_name(result));
    }
    else
    {
        tm_printf((UB *) "[avoid] outward monitor %s (%u deg): %u cm\n",
                  b_turn_left ? "right" : "left", (UINT) monitor_angle,
                  (UINT) distance_cm);
        if (!p_ctx->b_outward_arc_extended &&
            (distance_cm <= APP_AVOID_ARC_MONITOR_CLEARANCE_CM))
        {
            p_ctx->b_outward_arc_extended = true;
            /* 40.0 degrees in tenths fits uint16_t. */
            p_ctx->outward_arc_target_x10 =
                (uint16_t) ((APP_AVOID_TURN_DEG +
                             APP_AVOID_ARC_EXTRA_TURN_DEG) *
                            APP_DEG_X10);
            tm_printf((UB *) "[avoid] obstacle-side clearance %u cm; "
                             "extending turn-out to %u.%u deg\n",
                      (UINT) distance_cm,
                      (UINT) (p_ctx->outward_arc_target_x10 / 10u),
                      (UINT) (p_ctx->outward_arc_target_x10 % 10u));
        }
    }
}

/**
 * @brief Extend both matching diagonal legs and their outward angle once
 *        when the obstacle-side is still close during the first diagonal.
 *
 * The context keeps the distances and return angle, then reuses them for
 * the return diagonal so the extra outward path does not leave the rover
 * offset.
 */
static void app_check_outward_forward_clearance (app_avoid_ctx_t * p_ctx)
{
    bool                 b_turn_left   = p_ctx->b_outward_turn_left;
    buddy5_ping_result_t result        = BUDDY5_PING_TIMEOUT_RISE;
    uint32_t             distance_cm   = buddy5_sample_with_result_cm(&result);
    uint8_t              monitor_angle = app_outward_monitor_angle(b_turn_left);
    /* 10.0 degrees in tenths fits int16_t. */
    int16_t heading_offset_x10 =
        (int16_t) (APP_AVOID_FORWARD_EXTRA_TURN_DEG * APP_DEG_X10);

    if (BUDDY5_PING_VALID != result)
    {
        tm_printf((UB *) "[avoid] outward-forward monitor %s (%u deg) "
                         "unavailable: %s\n",
                  b_turn_left ? "right" : "left", (UINT) monitor_angle,
                  app_ping_failure_name(result));
    }
    else
    {
        tm_printf((UB *) "[avoid] outward-forward monitor %s (%u deg): %u "
                         "cm\n",
                  b_turn_left ? "right" : "left", (UINT) monitor_angle,
                  (UINT) distance_cm);
        if (!p_ctx->b_outward_forward_extended &&
            (distance_cm <= APP_AVOID_ARC_MONITOR_CLEARANCE_CM))
        {
            p_ctx->b_outward_forward_extended = true;
            p_ctx->paired_diagonal_distance_mm +=
                APP_AVOID_FORWARD_EXTRA_DISTANCE_MM;
            p_ctx->forward_start_heading_x10 = app_heading_offset_x10(
                p_ctx->forward_start_heading_x10,
                b_turn_left ? (int16_t) -heading_offset_x10
                            : heading_offset_x10);
            p_ctx->return_arc_target_x10 += (uint16_t) heading_offset_x10;
            tm_printf((UB *) "[avoid] obstacle-side clearance %u cm; "
                             "extending both diagonal forward legs to %u mm "
                             "and heading by %u.%u deg\n",
                      (UINT) distance_cm,
                      (UINT) p_ctx->paired_diagonal_distance_mm,
                      (UINT) (heading_offset_x10 / 10),
                      (UINT) (heading_offset_x10 % 10));
        }
    }
}

/** @brief Stop and reverse by a measured distance, creating room to scan
 * and turn.  Returns false if Buddy 2 rejected the move. */
static bool app_start_clearance_reverse (app_avoid_ctx_t * p_ctx,
                                         uint32_t          now_ms)
{
    bool b_started = false;

    buddy2_stop();
    if (buddy2_move_backward(APP_AVOID_REVERSE_DISTANCE_MM,
                             APP_AVOID_REVERSE_PWM))
    {
        robot_telemetry_set_avoidance_turn_pwm(-APP_AVOID_REVERSE_PWM,
                                               -APP_AVOID_REVERSE_PWM);
        robot_telemetry_set_avoidance_forward_progress(
            0u, (uint16_t) APP_AVOID_REVERSE_DISTANCE_MM);
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_REVERSING, 0,
                                             0u);
        p_ctx->state_started_ms = now_ms;
        p_ctx->state            = APP_AVOID_WAIT_REVERSE_CLEARANCE;
        b_started               = true;
    }

    return b_started;
}

/** @brief Reset temporary readings and start a new two-sided scan at 45
 * degrees. */
static void app_begin_side_scan (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    p_ctx->scan_45_cm  = BUDDY5_NO_ECHO;
    p_ctx->scan_135_cm = BUDDY5_NO_ECHO;
    app_side_scan_reset(&p_ctx->scan_45);
    app_side_scan_reset(&p_ctx->scan_135);
    buddy5_set_servo_angle(APP_SCAN_ANGLE_45_DEG);
    p_ctx->state                  = APP_AVOID_WAIT_45;
    p_ctx->state_started_ms       = now_ms;
    p_ctx->side_scan_last_ping_ms = now_ms;
    robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_SCANNING, 0, 0u);
}

/**
 * @brief Leave avoidance mode and keep the result visible for a moment.
 *
 * Faces the ultrasonic sensor forward again and resumes normal
 * forward-distance monitoring.  A finished run tells the robot controller
 * that avoidance is complete (it may hand back to line following); an
 * aborted run stops the motors and does not hand back.
 */
static void app_end_avoidance (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                               bool b_abort)
{
    p_ctx->finished_ms            = now_ms;
    p_ctx->b_result_pending_clear = true;
    if (b_abort)
    {
        buddy2_stop();
    }
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

/**
 * @brief Start one heading-held forward leg of the avoidance route.
 *
 * Both diagonal legs and the final straight leg need the same preparation:
 * save the encoder baseline and current IMU heading, command equal base PWM,
 * reset forward-progress telemetry, and enter the forward state.  The caller
 * supplies only what differs between legs: its target distance and type.
 */
static void app_start_forward_leg (app_avoid_ctx_t * p_ctx,
                                   int16_t now_heading_x10, float32_t target_mm,
                                   app_avoid_forward_leg_t leg, uint32_t now_ms)
{
    buddy2_get_motion_state(&p_ctx->forward_start_motion);
    p_ctx->forward_start_heading_x10 = now_heading_x10;
    p_ctx->forward_target_mm         = target_mm;
    p_ctx->forward_leg               = leg;

    buddy2_set_motor_pwm(APP_AVOID_FORWARD_BASE_PWM,
                         APP_AVOID_FORWARD_BASE_PWM);
    robot_telemetry_set_avoidance_turn_pwm(APP_AVOID_FORWARD_BASE_PWM,
                                           APP_AVOID_FORWARD_BASE_PWM);
    /* Leg distances are a few hundred millimetres. */
    robot_telemetry_set_avoidance_forward_progress(0u, (uint16_t) target_mm);
    robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_FORWARD, 0, 0u);

    p_ctx->state_started_ms = now_ms;
    p_ctx->state            = APP_AVOID_WAIT_FORWARD_DISTANCE;
}

/* A (new) front obstacle: forget the old route and back away to rescan. */
static void app_restart_for_obstacle (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    robot_telemetry_clear_avoidance_scan();
    p_ctx->b_result_pending_clear = false;
    p_ctx->b_outward_turn_left    = false;
    p_ctx->b_turning_inward       = false;
    p_ctx->forward_leg            = APP_AVOID_FORWARD_OUTWARD_DIAGONAL;
    if (!app_start_clearance_reverse(p_ctx, now_ms))
    {
        tm_printf((UB *) "[avoid] reverse move rejected; remaining "
                         "stopped\n");
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_BLOCKED, 0, 0u);
        app_end_avoidance(p_ctx, now_ms, true);
    }
}

/* Check the encoders, report progress and finish, time out or steer the
 * current forward leg. */
static void app_forward_progress (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                                  int16_t heading_x10)
{
    buddy2_motion_state_t forward_motion    = {0};
    float32_t             progress_mm       = 0.0f;
    float32_t             left_moved_mm     = 0.0f;
    float32_t             right_moved_mm    = 0.0f;
    int16_t               heading_error_x10 = 0;

    buddy2_get_motion_state(&forward_motion);
    /* Both repaired wheels contribute to centre travel. A failed encoder
     * must not silently turn this into a one-wheel move. */
    if (forward_motion.left_encoder_fault || forward_motion.right_encoder_fault)
    {
        buddy2_stop();
        robot_telemetry_set_avoidance_turn_pwm(0, 0);
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_BLOCKED, 0, 0u);
        tm_printf((UB *) "[avoid] encoder fault during forward; "
                         "stopped\n");
        robot_controller_abort_obstacle_avoidance();
        buddy5_set_servo_angle(APP_ULTRASONIC_ANGLE_DEG);
        p_ctx->state                  = APP_AVOID_WATCH_FORWARD;
        p_ctx->finished_ms            = now_ms;
        p_ctx->b_result_pending_clear = true;
    }
    else
    {
        left_moved_mm  = forward_motion.left_distance_mm -
                         p_ctx->forward_start_motion.left_distance_mm;
        right_moved_mm = forward_motion.right_distance_mm -
                         p_ctx->forward_start_motion.right_distance_mm;
        left_moved_mm = (left_moved_mm < 0.0f) ? -left_moved_mm : left_moved_mm;
        right_moved_mm =
            (right_moved_mm < 0.0f) ? -right_moved_mm : right_moved_mm;
        progress_mm = (left_moved_mm + right_moved_mm) * 0.5f;
        /* Progress and target are a few hundred millimetres. */
        robot_telemetry_set_avoidance_forward_progress(
            (uint16_t) progress_mm, (uint16_t) p_ctx->forward_target_mm);
        heading_error_x10 = app_heading_change_x10(
            p_ctx->forward_start_heading_x10, heading_x10);

        if (progress_mm >= p_ctx->forward_target_mm)
        {
            app_forward_leg_done(p_ctx, now_ms, heading_error_x10);
        }
        else if ((now_ms - p_ctx->state_started_ms) >=
                 APP_AVOID_FORWARD_TIMEOUT_MS)
        {
            buddy2_stop();
            tm_printf((UB *) "[avoid] heading-held forward timed out at "
                             "%u/%u mm; remaining stopped\n",
                      (UINT) progress_mm, (UINT) p_ctx->forward_target_mm);
            robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_TIMEOUT,
                                                 heading_error_x10, 0u);
            app_end_avoidance(p_ctx, now_ms, false);
        }
        else
        {
            app_forward_drive(heading_error_x10);
        }
    }
}

/*** end of file ***/

