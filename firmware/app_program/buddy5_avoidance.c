/**
 * @file    buddy5_avoidance.c
 * @brief   Buddy 5 ultrasonic obstacle-avoidance behaviour.
 *
 * Low-level ultrasonic timing and servo PWM remain in buddy5_ultrasonic.c.
 * This file owns only the 100 ms route state machine: width sweep, decide,
 * turn, forward movement, heading recovery, and final line handoff.
 *
 * The width sweep points the sonar at 30..150 degrees in 15-degree steps.
 * Every reading is split into a forward and a sideways part; readings that
 * lie on the obstacle's front face give the sideways position of its left
 * and right edges.  The rover goes round the edge that is closer to its
 * centreline, and the diagonal legs are made just long enough to clear it.
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

/* Every avoidance move runs at this fixed speed, NOT the dashboard's global
 * speed limit: at a low limit (e.g. 35 %) the PWM values below were scaled
 * too low to move the wheels and the reverse step timed out.  60 % is the
 * limit the route was verified with, so all *_PWM values below are tuned
 * for it. */
#define APP_AVOID_SPEED_PCT (60u)

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
/* Going round on the right leads with the left-front barcode arm, so turn
 * further away from the obstacle before the diagonal starts.  The turn-in
 * mirrors whatever the turn-out actually reached, and the diagonals are
 * sized with the matching sine below. */
#define APP_AVOID_ARM_SIDE_TURN_DEG        (60.0f)
#define APP_AVOID_TURN_TIMEOUT_MS          (5000u)
#define APP_AVOID_ARC_MONITOR_CLEARANCE_CM (25u)
#define APP_AVOID_ARC_EXTRA_TURN_DEG       (10.0f)
/* While moving through the outer arc and first diagonal, keep looking toward
 * the obstacle-side for this long. This catches a rear corner that remains
 * too close without stopping the rover to scan. */
#define APP_AVOID_OBSTACLE_SIDE_MONITOR_MS (2000u)
#define APP_AVOID_REVERSE_DISTANCE_MM      (80.0f)
/* Going round on the right swings the left-front barcode IR arm toward the
 * obstacle during the turn-out arc, so back away this much further first.
 * Reversing straight does not change the measured edge positions. */
#define APP_AVOID_ARM_SIDE_EXTRA_REVERSE_MM (100.0f)
/* Reverse moves use the Buddy 2 speed PID, whose drive is mostly a
 * feed-forward of 0.65 * (this * APP_AVOID_SPEED_PCT / 100).  At 9000 that
 * was about 3500 (11 % duty): enough to keep a rolling rover moving, but
 * not to start a wheel from a standstill, so the second reverse after the
 * sweep stalled and tripped the 800 ms encoder fault.  14000 gives about
 * 5460, the same drive as the forward legs. */
#define APP_AVOID_REVERSE_PWM              (14000)
#define APP_AVOID_REVERSE_TIMEOUT_MS       (6000u)
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
/* Encoder faults are ignored for this long at the start of a forward leg. */
#define APP_AVOID_STALE_FAULT_MS (1000u)

#define APP_AVOID_RECOVER_TOLERANCE_X10 (20)
#define APP_AVOID_RECOVER_TIMEOUT_MS    (5000u)
#define APP_AVOID_RESULT_VISIBLE_MS     (1100u)

/* Stall check for every driven part of the route (turn arcs, forward legs,
 * heading recovery).  If the two wheels moved less than this on average in
 * one window, the rover is pushing against something (often the barcode arm
 * caught on the obstacle): back away and measure again instead of pushing
 * until a timeout.  The first window includes the start-up, so the limit is
 * small.  After APP_AVOID_MAX_STALL_RETRIES rescans the rover stops. */
#define APP_AVOID_STALL_WINDOW_MS   (1000u)
#define APP_AVOID_STALL_MIN_MM      (15.0f)
#define APP_AVOID_MAX_STALL_RETRIES (3u)

/* ---- Obstacle width sweep ------------------------------------------ */
/* 30, 45, ... 150 degrees; 90 is straight ahead.  The HC-SR04 beam is about
 * 15 degrees wide, so a finer step would not place an edge any better.  The
 * sweep includes 45 and 135 degrees, which the dashboard shows as the
 * right/left scan values. */
#define APP_SWEEP_FIRST_DEG      (30u)
#define APP_SWEEP_STEP_DEG       (15u)
#define APP_SWEEP_POINTS         (9u)
#define APP_SWEEP_CENTRE_INDEX   (4u)
#define APP_SWEEP_45_INDEX       (1u)
#define APP_SWEEP_135_INDEX      (7u)
#define APP_SWEEP_STEP_SETTLE_MS (200u)
/* Each point keeps the nearest of up to two valid echoes in four pings. */
#define APP_SWEEP_VALID_SAMPLES (2u)
#define APP_SWEEP_MAX_ATTEMPTS  (4u)
/* A reading is still on the obstacle's front face while its forward part is
 * no more than this beyond the nearest centre reading. */
#define APP_SWEEP_FACE_TOLERANCE_MM (100.0f)

/* ---- Route sizing from the measured edge --------------------------- */
/* How far the rover reaches out on each side of its centreline (the sonar
 * axis).  The barcode IR sensor arm sticks out on the LEFT, so the rover is
 * about 130 mm wide in total but not symmetric.  Going round on the right
 * puts the left side (with the arm) next to the obstacle, and the reverse.
 * Measure from the sonar centre to the outermost part on each side. */
#define APP_ROVER_LEFT_EXTENT_MM  (75.0f)
#define APP_ROVER_RIGHT_EXTENT_MM (55.0f)
/* Gap kept between the rover's outermost part and the obstacle edge. */
#define APP_AVOID_SIDE_MARGIN_MM (40.0f)
/* Sideways shift made by the turn-out arc and the heading-recovery arc
 * together, not counting the diagonal.  With a 150 mm track and 14000/4000
 * wheel PWM the arc radius is about 135 mm, so each 30-degree arc shifts
 * about 18 mm.  Measure on the floor and tune. */
#define APP_AVOID_ARCS_LATERAL_MM (40.0f)
#define APP_AVOID_DIAGONAL_MIN_MM (150.0f)
/* Longer than this means the obstacle is too wide on that side. */
#define APP_AVOID_DIAGONAL_MAX_MM (600.0f)
/* When the sweep cannot measure the obstacle, use the fixed route the rover
 * used before the width sweep: this diagonal, toward the side with more
 * room, if that side has at least the minimum clearance. */
#define APP_AVOID_FALLBACK_DIAGONAL_MM (280.0f)
#define APP_AVOID_MIN_CLEARANCE_CM     (20u)

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
#define APP_ARM_SIDE_TURN_TARGET_X10 \
    ((uint16_t) (APP_AVOID_ARM_SIDE_TURN_DEG * APP_DEG_X10))
#define APP_ARC_EXTRA_TURN_X10 \
    ((uint16_t) (APP_AVOID_ARC_EXTRA_TURN_DEG * APP_DEG_X10))

#define APP_MM_PER_CM (10.0f)

/* Trigonometry is tabulated: the kernel's syslib.h declares its own size_t,
 * which clashes with <math.h>, and the RP2040 has no FPU anyway.
 * sin(APP_AVOID_TURN_DEG = 30 degrees), sin(APP_AVOID_ARM_SIDE_TURN_DEG =
 * 60 degrees) and tan(half a 15-degree step).  Change these with the
 * angles. */
#define APP_AVOID_TURN_SIN          (0.5f)
#define APP_AVOID_ARM_SIDE_TURN_SIN (0.86602540f)
#define APP_SWEEP_HALF_STEP_TAN (0.13165250f)

/* cos and sin of 0, 15, 30, 45 and 60 degrees off-axis, indexed by how many
 * sweep steps a point is from the centre. */
static const float32_t g_sweep_cos[APP_SWEEP_CENTRE_INDEX + 1u] = {
    1.0f, 0.96592583f, 0.86602540f, 0.70710678f, 0.5f};
static const float32_t g_sweep_sin[APP_SWEEP_CENTRE_INDEX + 1u] = {
    0.0f, 0.25881905f, 0.5f, 0.70710678f, 0.86602540f};

/* ------------------------------------------------------------------ *
 *  Sensors: IMU, ultrasonic monitoring, and obstacle avoidance
 * ------------------------------------------------------------------ */

typedef enum
{
    /* Normal mode: sample only the forward-facing ultrasonic sensor. */
    APP_AVOID_WATCH_FORWARD = 0,
    /* Back away from a close front obstacle before scanning around it. */
    APP_AVOID_WAIT_REVERSE_CLEARANCE,
    /* Step the servo across 30..150 degrees, pinging at every angle. */
    APP_AVOID_WAIT_SWEEP,
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

/** @brief Pings at one sweep angle, kept out of forward sonar telemetry. */
typedef struct
{
    uint8_t  attempts;
    uint8_t  valid;
    uint32_t nearest_cm;
} app_sweep_point_t;

/** @brief What the sweep says about going round one side of the obstacle. */
typedef struct
{
    float32_t edge_mm;     /**< Centreline to obstacle edge, sideways.  */
    float32_t diagonal_mm; /**< Diagonal leg length that clears it.     */
    bool      b_edge_found; /**< The sweep saw the obstacle end.        */
    bool      b_usable;     /**< Edge found and diagonal within limits. */
} app_side_plan_t;

/** Everything the avoidance task remembers between its 100 ms steps. */
typedef struct
{
    app_avoid_state_t       state;
    uint32_t                sweep_cm[APP_SWEEP_POINTS];
    uint32_t                sweep_settle_ms;
    uint32_t                state_started_ms;
    uint32_t                sweep_last_ping_ms;
    uint32_t                finished_ms;
    uint32_t                outward_monitor_started_ms;
    uint32_t                outward_monitor_last_ping_ms;
    uint32_t                outward_forward_monitor_started_ms;
    uint32_t                outward_forward_monitor_last_ping_ms;
    uint32_t                forward_sonar_center_started_ms;
    uint32_t                stall_window_started_ms;
    float32_t               stall_left_start_mm;
    float32_t               stall_right_start_mm;
    app_avoid_state_t       stall_state;
    uint8_t                 stall_retries;
    int16_t                 arc_start_heading_x10;
    int16_t                 forward_start_heading_x10;
    uint16_t                outward_arc_target_x10;
    uint16_t                return_arc_target_x10;
    float32_t               forward_target_mm;
    float32_t               planned_diagonal_mm;
    float32_t               paired_diagonal_distance_mm;
    bool                    b_outward_turn_left;
    bool                    b_outward_arc_extended;
    bool                    b_outward_forward_extended;
    bool                    b_outward_forward_monitor_active;
    bool                    b_outward_forward_center_settling;
    bool                    b_turning_inward;
    bool                    b_result_pending_clear;
    bool                    b_arm_side_reverse;
    app_avoid_forward_leg_t forward_leg;
    uint8_t                 sweep_index;
    app_sweep_point_t       sweep_point;
    buddy2_motion_state_t   forward_start_motion;
} app_avoid_ctx_t;

static bool         app_sonar_is_close (uint32_t distance_cm);
static bool         app_scan_angle_is_left (uint8_t angle_deg);
static const char * app_ping_failure_name (buddy5_ping_result_t result);
static uint8_t      app_sweep_angle (uint8_t index);
static void         app_sweep_point_reset (app_sweep_point_t * p_point);
static bool         app_sweep_ping (uint8_t             angle_deg,
                                    app_sweep_point_t * p_point,
                                    uint32_t *          p_result_cm);
static void         app_sweep_point_mm (uint8_t index, uint32_t distance_cm,
                                        float32_t * p_forward_mm,
                                        float32_t * p_lateral_mm);
static bool app_sweep_face_mm (const uint32_t * p_sweep_cm,
                               float32_t *      p_face_mm);
static void app_plan_side (const uint32_t * p_sweep_cm, float32_t face_mm,
                           bool b_low_side, app_side_plan_t * p_plan);
static void app_log_obstacle_size (float32_t               face_mm,
                                   const app_side_plan_t * p_low,
                                   const app_side_plan_t * p_high);
static void app_start_route (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                             bool b_low_side, float32_t diagonal_mm);
static void app_fallback_route (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                                const char * p_reason);
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
                                            uint32_t          now_ms,
                                            float32_t         distance_mm);
static void    app_begin_width_sweep (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
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
static void app_sweep_step (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
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
static void app_stall_window_start (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static bool app_handle_stall (app_avoid_ctx_t * p_ctx, uint32_t now_ms);

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

    /* sweep_cm[] is all BUDDY5_NO_ECHO (0) from the initialiser above. */
    ctx.state                       = APP_AVOID_WATCH_FORWARD;
    ctx.outward_arc_target_x10      = APP_TURN_TARGET_X10;
    ctx.return_arc_target_x10       = APP_TURN_TARGET_X10;
    ctx.forward_target_mm           = APP_AVOID_DIAGONAL_MIN_MM;
    ctx.planned_diagonal_mm         = APP_AVOID_DIAGONAL_MIN_MM;
    ctx.paired_diagonal_distance_mm = APP_AVOID_DIAGONAL_MIN_MM;
    ctx.forward_leg                 = APP_AVOID_FORWARD_OUTWARD_DIAGONAL;
    app_sweep_point_reset(&ctx.sweep_point);

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

    /* Every new route state starts a fresh stall window. */
    if (p_ctx->state != p_ctx->stall_state)
    {
        p_ctx->stall_state = p_ctx->state;
        app_stall_window_start(p_ctx, now_ms);
    }

    switch (p_ctx->state)
    {
        case APP_AVOID_WATCH_FORWARD:
            app_watch_forward(p_ctx, now_ms);
        break;

        case APP_AVOID_WAIT_REVERSE_CLEARANCE:
            app_reverse_clearance(p_ctx, now_ms);
        break;

        case APP_AVOID_WAIT_SWEEP:
            app_sweep_step(p_ctx, now_ms);
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
        p_ctx->stall_retries = 0u;
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
        if (p_ctx->b_arm_side_reverse)
        {
            /* The extra reverse only adds room for the barcode arm; the
             * route is already measured.  A stalled wheel here must not
             * abandon it: the arc drives harder and its ticks clear the
             * latched encoder fault before the first forward leg checks. */
            p_ctx->b_arm_side_reverse = false;
            if (reverse_motion.left_encoder_fault ||
                reverse_motion.right_encoder_fault)
            {
                tm_printf((UB *) "[avoid] extra reverse stalled (encoder "
                                 "fault); starting turn-out anyway\n");
            }
            else
            {
                tm_printf((UB *) "[avoid] backed away %u mm more for the "
                                 "barcode arm; starting turn-out\n",
                          (UINT) APP_AVOID_ARM_SIDE_EXTRA_REVERSE_MM);
            }
            /* The servo already turned to the obstacle-side while backing,
             * but give it the settle time before the arc starts. */
            p_ctx->state_started_ms = now_ms;
            p_ctx->state            = APP_AVOID_WAIT_OUTWARD_MONITOR_SETTLE;
        }
        else if (reverse_motion.left_encoder_fault ||
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
            tm_printf((UB *) "[avoid] backed away %u mm; sweeping to measure "
                             "obstacle width\n",
                      (UINT) APP_AVOID_REVERSE_DISTANCE_MM);
            app_begin_width_sweep(p_ctx, now_ms);
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

/* Steps 2 and 3: ping at one sweep angle until that point is complete, then
 * step the servo on (30 to 150 degrees), then back to the centre. */
static void app_sweep_step (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    uint8_t angle_deg = app_sweep_angle(p_ctx->sweep_index);

    if (((now_ms - p_ctx->state_started_ms) >= p_ctx->sweep_settle_ms) &&
        ((now_ms - p_ctx->sweep_last_ping_ms) >= APP_SONAR_PING_SPACING_MS))
    {
        p_ctx->sweep_last_ping_ms = now_ms;
        if (app_sweep_ping(angle_deg, &p_ctx->sweep_point,
                           &p_ctx->sweep_cm[p_ctx->sweep_index]))
        {
            p_ctx->sweep_index++;
            p_ctx->state_started_ms = now_ms;
            if (p_ctx->sweep_index < APP_SWEEP_POINTS)
            {
                app_sweep_point_reset(&p_ctx->sweep_point);
                p_ctx->sweep_settle_ms = APP_SWEEP_STEP_SETTLE_MS;
                buddy5_set_servo_angle(app_sweep_angle(p_ctx->sweep_index));
            }
            else
            {
                buddy5_set_servo_angle(APP_ULTRASONIC_ANGLE_DEG);
                p_ctx->state = APP_AVOID_WAIT_CENTER;
            }
        }
    }
}

/* Step 4: measure the obstacle from the sweep, go round the side whose edge
 * is closer to the centreline, or fall back to the fixed route. */
static void app_choose_route (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    app_side_plan_t low_plan  = {0};
    app_side_plan_t high_plan = {0};
    float32_t       face_mm   = 0.0f;
    bool            b_go_low  = false;

    if ((now_ms - p_ctx->state_started_ms) < APP_SERVO_SETTLE_MS)
    {
        /* Wait for the servo to face forward again. */
    }
    else
    {
        /* Publish the 45/135-degree readings before deciding. This keeps a
         * blocked route visible in the dashboard monitor. */
        app_publish_avoidance_scan_values(
            p_ctx->sweep_cm[APP_SWEEP_45_INDEX],
            p_ctx->sweep_cm[APP_SWEEP_135_INDEX]);

        if (!app_sweep_face_mm(p_ctx->sweep_cm, &face_mm))
        {
            /* Without the front face there is nothing to measure the edges
             * against. */
            app_fallback_route(p_ctx, now_ms, "no echo ahead during sweep");
        }
        else
        {
            app_plan_side(p_ctx->sweep_cm, face_mm, true, &low_plan);
            app_plan_side(p_ctx->sweep_cm, face_mm, false, &high_plan);
            app_log_obstacle_size(face_mm, &low_plan, &high_plan);

            if (!low_plan.b_usable && !high_plan.b_usable)
            {
                /* Usually something else near the obstacle hid both edges;
                 * the fixed route still works then. */
                app_fallback_route(p_ctx, now_ms,
                                   "no obstacle edge found on either side");
            }
            else
            {
                /* The shorter diagonal is the narrower way round. */
                b_go_low = low_plan.b_usable &&
                           (!high_plan.b_usable ||
                            (low_plan.diagonal_mm <= high_plan.diagonal_mm));
                app_start_route(p_ctx, now_ms, b_go_low,
                                b_go_low ? low_plan.diagonal_mm
                                         : high_plan.diagonal_mm);
            }
        }
    }
}

/**
 * @brief Fall back to the fixed route when the sweep cannot measure the
 *        obstacle.
 *
 * Uses the same rule as before the width sweep: both the 45 and 135-degree
 * readings must have an echo, the side with more room wins, it needs at
 * least APP_AVOID_MIN_CLEARANCE_CM, and the diagonals are the fixed
 * APP_AVOID_FALLBACK_DIAGONAL_MM.  Otherwise the rover stays stopped.
 */
static void app_fallback_route (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                                const char * p_reason)
{
    uint32_t low_cm  = p_ctx->sweep_cm[APP_SWEEP_45_INDEX];
    uint32_t high_cm = p_ctx->sweep_cm[APP_SWEEP_135_INDEX];

    tm_printf((UB *) "[avoid] %s; fixed route: 45 deg %u cm, 135 deg %u cm\n",
              p_reason, (UINT) low_cm, (UINT) high_cm);
    if ((BUDDY5_NO_ECHO == low_cm) || (BUDDY5_NO_ECHO == high_cm))
    {
        tm_printf((UB *) "[avoid] incomplete side readings; remaining "
                         "stopped\n");
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_BLOCKED, 0,
                                             0u);
        app_end_avoidance(p_ctx, now_ms, true);
    }
    else if ((low_cm < APP_AVOID_MIN_CLEARANCE_CM) &&
             (high_cm < APP_AVOID_MIN_CLEARANCE_CM))
    {
        tm_printf((UB *) "[avoid] both sides under %u cm; remaining "
                         "stopped\n",
                  (UINT) APP_AVOID_MIN_CLEARANCE_CM);
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_BLOCKED, 0,
                                             0u);
        app_end_avoidance(p_ctx, now_ms, true);
    }
    else
    {
        app_start_route(p_ctx, now_ms, (low_cm >= high_cm),
                        APP_AVOID_FALLBACK_DIAGONAL_MM);
    }
}

/* Commit to one side: publish it, size both diagonals, and point the sonar
 * at the obstacle for the turn-out. */
static void app_start_route (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                             bool b_low_side, float32_t diagonal_mm)
{
    bool b_turn_left = app_scan_angle_is_left(
        b_low_side ? APP_SCAN_ANGLE_45_DEG : APP_SCAN_ANGLE_135_DEG);

    p_ctx->b_outward_turn_left = b_turn_left;
    p_ctx->planned_diagonal_mm = diagonal_mm;
    /* Diagonals are at most APP_AVOID_DIAGONAL_MAX_MM, so they fit UINT. */
    tm_printf((UB *) "[avoid] go round %s; diagonal legs %u mm\n",
              b_turn_left ? "left" : "right", (UINT) diagonal_mm);
    app_publish_avoidance_decision(p_ctx->sweep_cm[APP_SWEEP_45_INDEX],
                                   p_ctx->sweep_cm[APP_SWEEP_135_INDEX],
                                   b_turn_left);
    p_ctx->outward_arc_target_x10     = b_turn_left
                                            ? APP_TURN_TARGET_X10
                                            : APP_ARM_SIDE_TURN_TARGET_X10;
    p_ctx->return_arc_target_x10      = p_ctx->outward_arc_target_x10;
    p_ctx->b_outward_arc_extended     = false;
    p_ctx->outward_monitor_started_ms = 0u;
    buddy5_set_servo_angle(app_outward_monitor_angle(b_turn_left));
    tm_printf((UB *) "[avoid] turn-out %s: monitor %s side for first %u ms "
                     "of arc\n",
              b_turn_left ? "left" : "right", b_turn_left ? "right" : "left",
              (UINT) APP_AVOID_OBSTACLE_SIDE_MONITOR_MS);

    if (b_turn_left)
    {
        p_ctx->state_started_ms = now_ms;
        p_ctx->state            = APP_AVOID_WAIT_OUTWARD_MONITOR_SETTLE;
    }
    else
    {
        /* A right turn-out leads with the left-front barcode arm: make room
         * for it first.  app_reverse_clearance() then starts the arc. */
        p_ctx->b_arm_side_reverse = true;
        if (!app_start_clearance_reverse(p_ctx, now_ms,
                                         APP_AVOID_ARM_SIDE_EXTRA_REVERSE_MM))
        {
            p_ctx->b_arm_side_reverse = false;
            tm_printf((UB *) "[avoid] extra reverse rejected; remaining "
                             "stopped\n");
            robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_BLOCKED, 0,
                                                 0u);
            app_end_avoidance(p_ctx, now_ms, true);
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
    else if (app_handle_stall(p_ctx, now_ms))
    {
        /* Wheels blocked: backing away to measure again. */
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
        p_ctx->paired_diagonal_distance_mm = p_ctx->planned_diagonal_mm;
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
    else if (app_handle_stall(p_ctx, now_ms))
    {
        /* Wheels blocked: backing away to measure again. */
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
    /* APP_AVOID_SPEED_PCT is a valid constant, so this cannot fail. */
    (void) buddy2_set_motor_pwm_at_speed(left_pwm, right_pwm,
                                         APP_AVOID_SPEED_PCT);
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
    else if (app_handle_stall(p_ctx, now_ms))
    {
        /* Wheels blocked: backing away to measure again. */
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
            (void) buddy2_set_motor_pwm_at_speed(left_pwm, right_pwm,
                                                 APP_AVOID_SPEED_PCT);
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

/** @brief Return the servo angle of one sweep point (30..150 degrees). */
static uint8_t app_sweep_angle (uint8_t index)
{
    /* At most 30 + 8 * 15 = 150, which fits uint8_t. */
    return (uint8_t) (APP_SWEEP_FIRST_DEG + ((uint32_t) index *
                                             APP_SWEEP_STEP_DEG));
}

/** @brief Clear the ping counters for the next sweep angle. */
static void app_sweep_point_reset (app_sweep_point_t * p_point)
{
    p_point->attempts   = 0u;
    p_point->valid      = 0u;
    p_point->nearest_cm = BUDDY5_NO_ECHO;
}

/**
 * @brief Take one non-telemetry ping at the current sweep angle and report
 *        whether that angle is finished.
 *
 * An angle finishes after two valid echoes or four attempts.  It keeps the
 * nearest valid echo, because a near reading means something is there; one
 * valid echo is enough.  An angle with no valid echo reads BUDDY5_NO_ECHO,
 * which the route planner treats as open space.
 */
static bool app_sweep_ping (uint8_t angle_deg, app_sweep_point_t * p_point,
                            uint32_t * p_result_cm)
{
    buddy5_ping_result_t result      = BUDDY5_PING_TIMEOUT_RISE;
    uint32_t             distance_cm = buddy5_sample_with_result_cm(&result);
    bool                 b_complete  = false;
    const char *         p_side      = "ahead";

    if (angle_deg < APP_ULTRASONIC_ANGLE_DEG)
    {
        p_side = app_scan_angle_is_left(APP_SCAN_ANGLE_45_DEG) ? "left"
                                                               : "right";
    }
    else if (angle_deg > APP_ULTRASONIC_ANGLE_DEG)
    {
        p_side = app_scan_angle_is_left(APP_SCAN_ANGLE_135_DEG) ? "left"
                                                                : "right";
    }
    else
    {
        /* Straight ahead. */
    }

    p_point->attempts++;
    if (BUDDY5_PING_VALID == result)
    {
        if ((0u == p_point->valid) || (distance_cm < p_point->nearest_cm))
        {
            p_point->nearest_cm = distance_cm;
        }
        p_point->valid++;
    }

    b_complete = (p_point->valid >= APP_SWEEP_VALID_SAMPLES) ||
                 (p_point->attempts >= APP_SWEEP_MAX_ATTEMPTS);
    if (b_complete)
    {
        *p_result_cm = p_point->nearest_cm;
        tm_printf((UB *) "[avoid] sweep %u deg (%s): %u cm valid=%u/%u\n",
                  (UINT) angle_deg, p_side, (UINT) p_point->nearest_cm,
                  (UINT) p_point->valid, (UINT) p_point->attempts);
    }

    return b_complete;
}

/**
 * @brief Split one sweep reading into its forward and sideways parts.
 *
 * Both are in millimetres relative to the sensor, which sits on the rover
 * centreline: forward is along the rover's heading, sideways is the
 * distance from the centreline (always positive; the index gives the side).
 */
static void app_sweep_point_mm (uint8_t index, uint32_t distance_cm,
                                float32_t * p_forward_mm,
                                float32_t * p_lateral_mm)
{
    /* index is 0..8, so the step from the centre is 0..4. */
    uint8_t   steps_off   = (index < APP_SWEEP_CENTRE_INDEX)
                                ? (uint8_t) (APP_SWEEP_CENTRE_INDEX - index)
                                : (uint8_t) (index - APP_SWEEP_CENTRE_INDEX);
    float32_t distance_mm = (float32_t) distance_cm * APP_MM_PER_CM;

    *p_forward_mm = distance_mm * g_sweep_cos[steps_off];
    *p_lateral_mm = distance_mm * g_sweep_sin[steps_off];
}

/**
 * @brief Find how far ahead the obstacle's front face is.
 *
 * Uses the nearest forward part of the 75, 90 and 105-degree readings, so a
 * single missed echo straight ahead does not lose the obstacle.
 *
 * @return false if none of those three angles returned an echo.
 */
static bool app_sweep_face_mm (const uint32_t * p_sweep_cm,
                               float32_t *      p_face_mm)
{
    float32_t forward_mm = 0.0f;
    float32_t lateral_mm = 0.0f;
    uint8_t   index      = 0u;
    bool      b_found    = false;

    for (index = APP_SWEEP_CENTRE_INDEX - 1u;
         index <= (APP_SWEEP_CENTRE_INDEX + 1u); index++)
    {
        if (BUDDY5_NO_ECHO != p_sweep_cm[index])
        {
            app_sweep_point_mm(index, p_sweep_cm[index], &forward_mm,
                               &lateral_mm);
            if (!b_found || (forward_mm < *p_face_mm))
            {
                *p_face_mm = forward_mm;
            }
            b_found = true;
        }
    }

    return b_found;
}

/**
 * @brief Find one edge of the obstacle and size the route round it.
 *
 * Walks outward from the centre of the sweep.  A reading stays on the
 * obstacle while it has an echo and its forward part is within
 * APP_SWEEP_FACE_TOLERANCE_MM of the front face; the first reading that is
 * open or much farther marks the edge.  The sideways part of the last
 * reading on the face is the edge position.  The beam is as wide as one
 * sweep step, so the last hit sees about half a step past the true edge and
 * the gap to the first miss is about half a step the other way; the two
 * roughly cancel.
 *
 * The diagonal leg runs at the turn-out angle to the original heading
 * (APP_AVOID_TURN_DEG on the left, APP_AVOID_ARM_SIDE_TURN_DEG on the
 * right), so it moves the rover sideways by diagonal * sin(turn).  It must
 * cover the edge plus the rover's reach on the side that passes the
 * obstacle plus a margin, less what the two arcs already shift.
 *
 * @param b_low_side True for the servo angles below 90 degrees.
 */
static void app_plan_side (const uint32_t * p_sweep_cm, float32_t face_mm,
                           bool b_low_side, app_side_plan_t * p_plan)
{
    float32_t forward_mm = 0.0f;
    float32_t lateral_mm = 0.0f;
    float32_t shift_mm   = 0.0f;
    float32_t reach_mm   = 0.0f;
    uint8_t   step       = 0u;
    uint8_t   index      = 0u;
    bool      b_on_face  = true;
    bool      b_go_left  = false;

    /* Even a thin post fills the centre beam, so the edge is at least half
     * a sweep step out. */
    p_plan->edge_mm      = face_mm * APP_SWEEP_HALF_STEP_TAN;
    p_plan->b_edge_found = false;

    for (step = 1u; (step <= APP_SWEEP_CENTRE_INDEX) && b_on_face; step++)
    {
        /* step <= 4 keeps the index in 0..8. */
        index = b_low_side ? (uint8_t) (APP_SWEEP_CENTRE_INDEX - step)
                           : (uint8_t) (APP_SWEEP_CENTRE_INDEX + step);
        b_on_face = (BUDDY5_NO_ECHO != p_sweep_cm[index]);
        if (b_on_face)
        {
            app_sweep_point_mm(index, p_sweep_cm[index], &forward_mm,
                               &lateral_mm);
            b_on_face =
                (forward_mm <= (face_mm + APP_SWEEP_FACE_TOLERANCE_MM));
        }

        if (b_on_face)
        {
            if (lateral_mm > p_plan->edge_mm)
            {
                p_plan->edge_mm = lateral_mm;
            }
        }
        else
        {
            p_plan->b_edge_found = true;
        }
    }

    /* Going round on the left leaves the obstacle on the rover's right, so
     * the right side's reach matters, and the reverse.  Going round on the
     * right also turns out further (the barcode arm side). */
    b_go_left = app_scan_angle_is_left(b_low_side ? APP_SCAN_ANGLE_45_DEG
                                                  : APP_SCAN_ANGLE_135_DEG);
    reach_mm  = b_go_left ? APP_ROVER_RIGHT_EXTENT_MM
                          : APP_ROVER_LEFT_EXTENT_MM;
    shift_mm  = (p_plan->edge_mm + reach_mm + APP_AVOID_SIDE_MARGIN_MM) -
                APP_AVOID_ARCS_LATERAL_MM;
    p_plan->diagonal_mm =
        shift_mm / (b_go_left ? APP_AVOID_TURN_SIN
                              : APP_AVOID_ARM_SIDE_TURN_SIN);
    if (p_plan->diagonal_mm < APP_AVOID_DIAGONAL_MIN_MM)
    {
        p_plan->diagonal_mm = APP_AVOID_DIAGONAL_MIN_MM;
    }
    p_plan->b_usable = p_plan->b_edge_found &&
                       (p_plan->diagonal_mm <= APP_AVOID_DIAGONAL_MAX_MM);
}

/** @brief Print the measured face distance, both edges and the width. */
static void app_log_obstacle_size (float32_t               face_mm,
                                   const app_side_plan_t * p_low,
                                   const app_side_plan_t * p_high)
{
    bool                    b_low_is_left =
        app_scan_angle_is_left(APP_SCAN_ANGLE_45_DEG);
    const app_side_plan_t * p_left  = b_low_is_left ? p_low : p_high;
    const app_side_plan_t * p_right = b_low_is_left ? p_high : p_low;

    /* The HC-SR04 reads at most about 4 m, so every value is a few
     * thousand millimetres at most and is cast to the %u type. */
    tm_printf((UB *) "[avoid] obstacle %u mm ahead, width %s%u mm "
                     "(left edge %u mm%s, right edge %u mm%s)\n",
              (UINT) face_mm,
              (p_left->b_edge_found && p_right->b_edge_found) ? "" : ">= ",
              (UINT) (p_left->edge_mm + p_right->edge_mm),
              (UINT) p_left->edge_mm,
              p_left->b_edge_found ? "" : " not found",
              (UINT) p_right->edge_mm,
              p_right->b_edge_found ? "" : " not found");
    tm_printf((UB *) "[avoid] diagonal left %u mm%s, right %u mm%s\n",
              (UINT) p_left->diagonal_mm,
              p_left->b_usable ? "" : " (unusable)",
              (UINT) p_right->diagonal_mm,
              p_right->b_usable ? "" : " (unusable)");
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
                (void) buddy2_set_motor_pwm_at_speed(
                    APP_AVOID_ARC_INNER_PWM, APP_AVOID_LEFT_TURN_OUTER_PWM,
                    APP_AVOID_SPEED_PCT);
            }
            else
            {
                (void) buddy2_set_motor_pwm_at_speed(
                    APP_AVOID_RIGHT_TURN_OUTER_PWM, APP_AVOID_ARC_INNER_PWM,
                    APP_AVOID_SPEED_PCT);
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
            /* Added once to a 30 or 60-degree target: at most 700 tenths,
             * which fits uint16_t. */
            p_ctx->outward_arc_target_x10 = (uint16_t) (
                p_ctx->outward_arc_target_x10 + APP_ARC_EXTRA_TURN_X10);
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

/** @brief Stop and reverse by distance_mm, creating room to scan and turn.
 *  Returns false if Buddy 2 rejected the move. */
static bool app_start_clearance_reverse (app_avoid_ctx_t * p_ctx,
                                         uint32_t          now_ms,
                                         float32_t         distance_mm)
{
    bool b_started = false;

    buddy2_stop();
    if (buddy2_move_backward_at_speed(distance_mm, APP_AVOID_REVERSE_PWM,
                                      APP_AVOID_SPEED_PCT))
    {
        robot_telemetry_set_avoidance_turn_pwm(-APP_AVOID_REVERSE_PWM,
                                               -APP_AVOID_REVERSE_PWM);
        /* Reverse distances are about 100 mm, so they fit uint16_t. */
        robot_telemetry_set_avoidance_forward_progress(
            0u, (uint16_t) distance_mm);
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_REVERSING, 0,
                                             0u);
        p_ctx->state_started_ms = now_ms;
        p_ctx->state            = APP_AVOID_WAIT_REVERSE_CLEARANCE;
        b_started               = true;
    }

    return b_started;
}

/** @brief Clear the sweep readings and start a new sweep at its first
 * angle. */
static void app_begin_width_sweep (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    uint8_t index = 0u;

    for (index = 0u; index < APP_SWEEP_POINTS; index++)
    {
        p_ctx->sweep_cm[index] = BUDDY5_NO_ECHO;
    }
    p_ctx->sweep_index = 0u;
    app_sweep_point_reset(&p_ctx->sweep_point);
    /* The first move swings the servo 60 degrees: give it the full settle
     * time.  Later 15-degree steps use the shorter step settle. */
    p_ctx->sweep_settle_ms = APP_SERVO_SETTLE_MS;
    buddy5_set_servo_angle(app_sweep_angle(0u));
    p_ctx->state                  = APP_AVOID_WAIT_SWEEP;
    p_ctx->state_started_ms       = now_ms;
    p_ctx->sweep_last_ping_ms = now_ms;
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

    (void) buddy2_set_motor_pwm_at_speed(APP_AVOID_FORWARD_BASE_PWM,
                                         APP_AVOID_FORWARD_BASE_PWM,
                                         APP_AVOID_SPEED_PCT);
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
    p_ctx->b_arm_side_reverse     = false;
    if (!app_start_clearance_reverse(p_ctx, now_ms,
                                     APP_AVOID_REVERSE_DISTANCE_MM))
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
     * must not silently turn this into a one-wheel move.  A fault seen in
     * the leg's first APP_AVOID_STALE_FAULT_MS is left over from an earlier
     * stall (Buddy 2 needs 800 ms without ticks to raise a new one), and the
     * first tick clears it; it only counts if it is still there after. */
    if ((forward_motion.left_encoder_fault ||
         forward_motion.right_encoder_fault) &&
        ((now_ms - p_ctx->state_started_ms) >= APP_AVOID_STALE_FAULT_MS))
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

/** @brief Start a stall window at the wheels' current distances. */
static void app_stall_window_start (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    buddy2_motion_state_t motion = {0};

    buddy2_get_motion_state(&motion);
    p_ctx->stall_window_started_ms = now_ms;
    p_ctx->stall_left_start_mm     = motion.left_distance_mm;
    p_ctx->stall_right_start_mm    = motion.right_distance_mm;
}

/**
 * @brief Check whether the rover is stuck while it is driving, and if so
 *        back away and measure the obstacle again.
 *
 * Call it every step of a driven state.  At the end of each
 * APP_AVOID_STALL_WINDOW_MS window it compares how far the wheels moved on
 * average with APP_AVOID_STALL_MIN_MM.  A stuck rover backs away and runs
 * the width sweep again; after APP_AVOID_MAX_STALL_RETRIES of those it
 * stops instead of pushing for ever.
 *
 * @return true if a stall was handled (the caller must not drive this step).
 */
static bool app_handle_stall (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    buddy2_motion_state_t motion   = {0};
    float32_t             left_mm  = 0.0f;
    float32_t             right_mm = 0.0f;
    float32_t             moved_mm = 0.0f;
    bool                  b_stuck  = false;

    if ((now_ms - p_ctx->stall_window_started_ms) >= APP_AVOID_STALL_WINDOW_MS)
    {
        buddy2_get_motion_state(&motion);
        left_mm  = motion.left_distance_mm - p_ctx->stall_left_start_mm;
        right_mm = motion.right_distance_mm - p_ctx->stall_right_start_mm;
        left_mm  = (left_mm < 0.0f) ? -left_mm : left_mm;
        right_mm = (right_mm < 0.0f) ? -right_mm : right_mm;
        moved_mm = (left_mm + right_mm) * 0.5f;
        b_stuck  = (moved_mm < APP_AVOID_STALL_MIN_MM);

        if (!b_stuck)
        {
            app_stall_window_start(p_ctx, now_ms);
        }
        else if (p_ctx->stall_retries >= APP_AVOID_MAX_STALL_RETRIES)
        {
            buddy2_stop();
            robot_telemetry_set_avoidance_turn_pwm(0, 0);
            /* moved_mm is under APP_AVOID_STALL_MIN_MM, so it fits UINT. */
            tm_printf((UB *) "[avoid] still stuck (%u mm in %u ms) after %u "
                             "rescans; remaining stopped\n",
                      (UINT) moved_mm, (UINT) APP_AVOID_STALL_WINDOW_MS,
                      (UINT) APP_AVOID_MAX_STALL_RETRIES);
            robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_BLOCKED, 0,
                                                 0u);
            app_end_avoidance(p_ctx, now_ms, true);
        }
        else
        {
            buddy2_stop();
            robot_telemetry_set_avoidance_turn_pwm(0, 0);
            p_ctx->stall_retries++;
            tm_printf((UB *) "[avoid] stuck: moved %u mm in %u ms; backing "
                             "away and rescanning (try %u of %u)\n",
                      (UINT) moved_mm, (UINT) APP_AVOID_STALL_WINDOW_MS,
                      (UINT) p_ctx->stall_retries,
                      (UINT) APP_AVOID_MAX_STALL_RETRIES);
            app_restart_for_obstacle(p_ctx, now_ms);
        }
    }

    return b_stuck;
}

/*** end of file ***/

