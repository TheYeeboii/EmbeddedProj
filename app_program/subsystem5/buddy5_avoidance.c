/**
 * @file    buddy5_avoidance.c
 * @brief   Buddy 5 ultrasonic obstacle-avoidance behaviour.
 *
 * Low-level ultrasonic timing and servo PWM remain in buddy5_ultrasonic.c.
 * This file owns only the 100 ms route state machine.
 *
 * When line following meets an obstacle (15 cm ahead), the rover backs
 * away, sweeps the sonar from 30 to 150 degrees in 5-degree steps to find
 * the obstacle's front face and both edges, and goes round the side with
 * the narrower way round:
 *
 *   1. pivot 90 degrees away from the obstacle (compass),
 *   2. sidestep far enough to clear the measured edge,
 *   3. pivot back to the line's heading,
 *   4. drive past the obstacle, watching it with the side-facing sonar
 *      until it has ended and the whole rover is past it,
 *   5. pivot 90 degrees toward the line and drive until a front IR sensor
 *      finds it (black is ignored before this step: the line continues
 *      right next to the obstacle),
 *   6. drive the sensor-to-axle distance, pivot back to the line's heading
 *      and hand back to line following.
 *
 * The line is the x axis of this description; every turn is measured with
 * the Buddy 4 compass against the heading the rover had on the line.
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
 * speed limit: at a low limit (e.g. 35 %) the PWM values below were scaled
 * too low to move the wheels.  All *_PWM values below are tuned for it. */
#define APP_AVOID_SPEED_PCT (60u)

/* ---- Sonar and servo ----------------------------------------------- */
#define APP_ULTRASONIC_ANGLE_DEG  (90u)
#define APP_SERVO_SETTLE_MS       (500u)
/* The HC-SR04 needs this long between pings so an old echo is not read. */
#define APP_SONAR_REPING_MS (60u)

/* Servo installation: 45 degrees looks to rover right; 135 to rover left.
 * Confirmed from the physical left-obstacle scan: it was reported at 135 deg.
 */
#define APP_OBSTACLE_STOP_CM   (15u)
#define APP_SCAN_ANGLE_45_DEG  (45u)
#define APP_SCAN_ANGLE_135_DEG (135u)
#define APP_SCAN_45_IS_LEFT    (0u)

/* ---- Backing away -------------------------------------------------- */
#define APP_AVOID_REVERSE_DISTANCE_MM (80.0f)
/* Pivoting right swings the left-front barcode IR arm toward the obstacle,
 * so back away this much further before going round on the right. */
#define APP_AVOID_ARM_SIDE_EXTRA_REVERSE_MM (100.0f)
/* Reverse moves use the Buddy 2 speed PID, whose drive is mostly a
 * feed-forward of 0.65 * (this * APP_AVOID_SPEED_PCT / 100): about 5460,
 * enough to start a wheel from a standstill. */
#define APP_AVOID_REVERSE_PWM        (14000)
#define APP_AVOID_REVERSE_TIMEOUT_MS (6000u)

/* ---- Obstacle width sweep ------------------------------------------ */
/* 30, 35, ... 150 degrees; 90 is straight ahead.  Fine steps place each
 * edge to a few degrees; the beam correction below removes the bias of the
 * wide HC-SR04 beam.  The sweep includes 45 and 135 degrees, which the
 * dashboard shows as the right/left scan values. */
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
/* A reading is on the obstacle's front face while its forward part is
 * within this of the face, nearer or farther.  On the track, things beside
 * the rover (nearer than the face) and behind the box (farther) made the
 * box look up to twice as wide with the old one-sided 100 mm. */
#define APP_SWEEP_FACE_TOLERANCE_MM (50.0f)
/* The beam still returns an echo from the obstacle's corner while it points
 * up to this many sweep steps past it, so the last echo on the face is that
 * far outside the true edge.  Calibrate with a box of known width: the
 * dashboard width should match it. */
#define APP_SWEEP_BEAM_STEPS (1u)

/* ---- Route sizing from the measured edge --------------------------- */
/* How far the rover reaches out on each side of its centreline (the sonar
 * axis).  The barcode IR sensor arm sticks out on the LEFT.  Going round on
 * the right puts the left side (with the arm) next to the obstacle, and the
 * reverse.  Measure from the sonar centre to the outermost part. */
#define APP_ROVER_LEFT_EXTENT_MM  (75.0f)
#define APP_ROVER_RIGHT_EXTENT_MM (55.0f)
/* Gap kept between the rover's outermost part and the obstacle edge.  60 mm
 * clipped the box on the track (25 Nov): 120 mm. */
#define APP_AVOID_SIDE_MARGIN_MM (120.0f)
#define APP_SIDESTEP_MIN_MM      (120.0f)
/* Wider than this on a side means that side is too far round. */
#define APP_SIDESTEP_MAX_MM (560.0f)
/* When the sweep cannot measure the obstacle: sidestep this far toward the
 * side with more room, if that side has at least the minimum clearance. */
#define APP_AVOID_FALLBACK_SIDESTEP_MM (300.0f)
#define APP_AVOID_FALLBACK_FACE_MM     (250.0f)
#define APP_AVOID_MIN_CLEARANCE_CM     (20u)

/* ---- Pivots -------------------------------------------------------- */
/* Turn on the spot (wheels opposite) until the compass reaches the target.
 * The last APP_PIVOT_SLOW_ZONE_X10 runs slower so the stop is accurate;
 * within APP_PIVOT_STOP_X10 counts as there (the next leg holds the exact
 * heading anyway). */
#define APP_PIVOT_FAST_PWM       (14000)
#define APP_PIVOT_SLOW_PWM       (11000)
#define APP_PIVOT_SLOW_ZONE_X10  (200)
#define APP_PIVOT_STOP_X10       (50)
#define APP_PIVOT_TIMEOUT_MS     (6000u)
#define APP_QUARTER_TURN_X10     (900)

/* ---- Heading-held straight legs ------------------------------------ */
#define APP_AVOID_FORWARD_BASE_PWM             (9000)
#define APP_AVOID_FORWARD_HEADING_DEADBAND_X10 (5)
#define APP_AVOID_FORWARD_HEADING_KP           (30)
#define APP_AVOID_FORWARD_MAX_CORRECTION_PWM   (2500)
#define APP_AVOID_LEG_TIMEOUT_MS               (15000u)
/* Encoder faults are ignored for this long at the start of a leg. */
#define APP_AVOID_STALE_FAULT_MS (1000u)

/* ---- Passing the obstacle ------------------------------------------ */
/* The sonar looks 70 degrees to the obstacle side (sin 70 = 0.940); the
 * sweep only goes to 30/150, so check the servo reaches 20/160 freely. */
#define APP_PASS_LOOK_RIGHT_DEG (20u)
#define APP_PASS_LOOK_LEFT_DEG  (160u)
/* 70 degrees off the rover's heading, in tenths. */
#define APP_PASS_LOOK_OFF_AXIS_X10 (700)
/* A side reading nearer than the original line plus this is the obstacle. */
#define APP_PASS_NEAR_BEYOND_LINE_MM (100.0f)
/* Keep the planned gap while passing: steer away when the side reading is
 * nearer than planned and back in when farther, this many tenths of a
 * degree per millimetre of error, at most APP_PASS_STEER_MAX_X10. */
#define APP_PASS_STEER_X10_PER_MM (3.0f)
#define APP_PASS_STEER_MAX_X10    (250)
/* Consecutive readings (one per 100 ms) that confirm "alongside" and
 * "passed". */
#define APP_PASS_SEEN_COUNT  (2u)
#define APP_PASS_CLEAR_COUNT (5u)
/* "Ended" is only believed this far past the obstacle's front face: on the
 * track a few missed side echoes ended a pass 42 cm in, and the turn back
 * pushed into the box. */
#define APP_PASS_MIN_DEPTH_MM (100.0f)
/* After the sonar passes the obstacle's far end, drive on this far so the
 * whole rover (and the cable trailing behind it) is past it before the
 * pivot toward the line.  250 mm still caught the box on the track
 * (25 Nov): 400 mm. */
#define APP_PASS_TAIL_MM (400.0f)
/* Never seen alongside this far beyond its front face: the side sonar is
 * missing it.  Assume it is this deep, then pass it as above. */
#define APP_PASS_UNSEEN_MM        (300.0f)
#define APP_PASS_ASSUMED_DEPTH_MM (350.0f)
/* Still alongside after this much travel: it is a wall, not an obstacle. */
#define APP_PASS_MAX_MM (1200.0f)
/* Turning back toward the line (and driving back to it) the front sonar
 * keeps watching: the obstacle this near means the rover is not past it
 * yet.  It then turns back along the line, drives APP_PASS_RETRY_MM
 * further and tries again, at most APP_PASS_MAX_RETRIES times. */
#define APP_TURN_BACK_CLEAR_CM (25u)
#define APP_RETURN_CLEAR_CM    (20u)
#define APP_PASS_RETRY_MM      (200.0f)
#define APP_PASS_MAX_RETRIES   (3u)

/* ---- Back onto the line -------------------------------------------- */
/* The leg toward the line may run this far beyond the sidestep before it
 * gives up (the front sensors reach the line before the centre does). */
#define APP_RETURN_EXTRA_MM (250.0f)
/* Front sensors to axle: drive this far after they find the line, so the
 * pivot back to the line's heading turns on the line.  Same as the junction
 * turns (robot_controller.c, ROBOT_JUNCTION_AXLE_MM). */
#define APP_ALIGN_FORWARD_MM (100.0f)
/* The tracked position says the front sensors should be over the line, but
 * they saw nothing: drive on this far for them to confirm it, then trust
 * the position. */
#define APP_RETURN_CONFIRM_MM (60.0f)
/* The tracked position is only trusted after the rover has driven at least
 * this far back toward the line (on the track it put the line too close
 * and the rover turned along the track before reaching it), and the leg
 * may run to APP_RETURN_MAX_MM. */
#define APP_RETURN_MIN_MM (300.0f)
#define APP_RETURN_MAX_MM (600.0f)
/* After the last pivot along the line, the rover must see the line: if no
 * front sensor touched it during that pivot, it sweeps this far one way
 * (toward where the line should be) and then the other, stopping as soon
 * as a sensor finds it.  Not found: it stops rather than drive off. */
#define APP_SEARCH_SWEEP_X10 (350)
/* On the way back the rover drives across the original line, so its black
 * ends within this much travel.  Black that lasts longer is a line running
 * the same way as the rover (on the track, 25 Nov: a junction's branch
 * beside the box, which the rover took for the line and followed off the
 * mat): it is ignored. */
#define APP_RETURN_CROSS_MAX_MM (50.0f)
/* ... and a crossed line only counts once the tracked position puts the
 * front sensors within this of the original line: the mat has other lines,
 * and on the track (25 Nov) one was crossed 5 cm into the way back. */
#define APP_RETURN_LINE_WINDOW_MM (150.0f)

/* ---- Heading -------------------------------------------------------- */
/* The compass is the heading, but on the track it drops out, and the
 * motors' current upsets it during pivots.  Between readings the heading is
 * carried on from the wheel encoders (the turn the two wheels' difference
 * makes on the track width; on the spot the wheels scrub, so the rover turns
 * only APP_PIVOT_WHEEL_SLIP of that).  A compass reading more than
 * APP_COMPASS_GATE_X10 away from that estimate is a glitch and is ignored,
 * unless APP_COMPASS_AGREE_COUNT readings in a row say so. */
#define APP_PIVOT_WHEEL_SLIP    (0.85f)
#define APP_COMPASS_GATE_X10    (250)
#define APP_COMPASS_AGREE_COUNT (5u)
#define APP_X10_PER_RAD         (572.957795f)

/* ---- Stuck check --------------------------------------------------- */
/* If the two wheels moved less than this on average in one window while
 * driving, the rover is pushing against something: stop. */
#define APP_AVOID_STALL_WINDOW_MS (1000u)
#define APP_AVOID_STALL_MIN_MM    (15.0f)

#define APP_AVOID_RESULT_VISIBLE_MS (1100u)

/* Angles are handled in tenths of a degree. */
#define APP_HALF_TURN_X10 (1800)
#define APP_FULL_TURN_X10 (3600)
#define APP_MM_PER_CM     (10.0f)

/* Trigonometry is tabulated: the kernel's syslib.h declares its own size_t,
 * which clashes with <math.h>, and the RP2040 has no FPU anyway.  cos and
 * sin of 0, 5, ... 60 degrees off-axis, indexed by how many sweep steps a
 * point is from the centre. */
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
    /* Normal mode: sample only the forward-facing ultrasonic sensor. */
    APP_AVOID_WATCH_FORWARD = 0,
    /* Back away from a close front obstacle (or further, for the arm). */
    APP_AVOID_WAIT_REVERSE_CLEARANCE,
    /* Step the servo across 30..150 degrees, pinging at every angle. */
    APP_AVOID_WAIT_SWEEP,
    /* Re-centre the servo, then measure the obstacle and pick a side. */
    APP_AVOID_WAIT_CENTER,
    /* Turn on the spot to a compass heading. */
    APP_AVOID_WAIT_PIVOT,
    /* Drive a heading-held straight leg. */
    APP_AVOID_WAIT_LEG
} app_avoid_state_t;

/** What comes after a pivot. */
typedef enum
{
    APP_AFTER_PIVOT_SIDESTEP = 0,
    APP_AFTER_PIVOT_PASS,
    APP_AFTER_PIVOT_RETURN,
    APP_AFTER_PIVOT_FINISH,
    APP_AFTER_PIVOT_PASS_MORE,
    APP_AFTER_PIVOT_SEARCH_OTHER, /* first search sweep done: other way */
    APP_AFTER_PIVOT_SEARCH_FAIL   /* second sweep done: not found       */
} app_after_pivot_t;

/** The straight legs of the route. */
typedef enum
{
    APP_LEG_SIDESTEP = 0, /* away from the line, beside the obstacle      */
    APP_LEG_PASS,         /* along the line's heading, past the obstacle  */
    APP_LEG_RETURN,       /* back toward the line until a sensor finds it */
    APP_LEG_ALIGN,        /* sensor-to-axle distance onto the line        */
    APP_LEG_PASS_MORE     /* further along the line: not past it yet      */
} app_leg_t;

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
    float32_t edge_mm;      /**< Centreline to obstacle edge, sideways.  */
    float32_t sidestep_mm;  /**< Sideways move that clears it.           */
    bool      b_edge_found; /**< The sweep saw the obstacle end.        */
    bool      b_usable;     /**< Edge found and sidestep within limits. */
} app_side_plan_t;

/** Everything the avoidance task remembers between its 100 ms steps. */
typedef struct
{
    app_avoid_state_t     state;
    uint32_t              sweep_cm[APP_SWEEP_POINTS];
    uint32_t              sweep_settle_ms;
    uint32_t              state_started_ms;
    uint32_t              finished_ms;
    uint32_t              stall_window_started_ms;
    float32_t             stall_left_start_mm;
    float32_t             stall_right_start_mm;
    app_avoid_state_t     stall_state;
    bool                  b_result_pending_clear;
    bool                  b_arm_side_reverse;
    uint8_t               sweep_index;
    app_sweep_point_t     sweep_point;
    /* Heading: compass, carried on by the encoders between readings. */
    bool                  b_head_valid;
    float32_t             head_x10;
    float32_t             head_left_mm;
    float32_t             head_right_mm;
    uint8_t               head_reject;
    /* The route. */
    bool                  b_go_right;       /**< Round the right side.   */
    int16_t               line_heading_x10; /**< Heading along the line. */
    float32_t             sidestep_mm;
    float32_t             face_mm;          /**< Sonar to obstacle face. */
    float32_t             pass_gap_mm;      /**< Centreline to its side. */
    /* Position of the axle from where the route began, tracked from the
     * encoders (distance) and the compass (direction): x along the line,
     * y sideways, right positive. */
    bool                  b_tracking;
    float32_t             pos_x_mm;
    float32_t             pos_y_mm;
    float32_t             track_left_mm;
    float32_t             track_right_mm;
    /* Pivot in progress. */
    int16_t               pivot_target_x10;
    int16_t               pivot_start_x10;
    int16_t               pivot_last_error_x10;
    bool                  b_pivot_saw_line; /**< Front IR touched black. */
    int16_t               search_sign;      /**< First sweep: +1 right.  */
    app_after_pivot_t     after_pivot;
    /* Leg in progress. */
    app_leg_t             leg;
    int16_t               leg_heading_x10;
    float32_t             leg_target_mm;
    buddy2_motion_state_t leg_start_motion;
    /* Passing the obstacle. */
    uint8_t               pass_near_count;
    uint8_t               pass_clear_count;
    bool                  b_pass_alongside;
    bool                  b_pass_cleared;
    float32_t             pass_cleared_mm;
    int16_t               pass_steer_x10; /**< Gap-keeping heading offset. */
    uint8_t               pass_retries;   /**< Turn-backs that found it.   */
    bool                  b_return_trusting; /**< Sensors should be on it. */
    float32_t             return_trust_mm;
    bool                  b_return_black;  /**< Front sensors on black.    */
    bool                  b_return_along;  /**< ... for too long: a branch. */
    float32_t             return_black_mm; /**< Where the black started.   */
} app_avoid_ctx_t;

/* ------------------------------------------------------------------ *
 *  Prototypes
 * ------------------------------------------------------------------ */

static void app_step (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_watch_forward (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_reverse_clearance (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_sweep_step (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_choose_route (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_fallback_route (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                                const char * p_reason);
static void app_start_route (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                             bool b_go_right, float32_t sidestep_mm,
                             float32_t face_mm, float32_t edge_mm);
static void app_track_position (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static float32_t app_offset_mm (const app_avoid_ctx_t * p_ctx);
static float32_t app_sine (int16_t angle_x10);
static float32_t app_cosine (int16_t angle_x10);
static void app_begin_route (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_start_pivot (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                             int16_t target_x10, app_after_pivot_t after);
static void app_pivot (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_pivot_done (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_start_leg (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                           app_leg_t leg, int16_t heading_x10,
                           float32_t target_mm);
static void app_leg (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static bool app_leg_check (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                           float32_t progress_mm);
static bool app_pass_update (app_avoid_ctx_t * p_ctx, float32_t progress_mm);
static bool app_front_blocked (uint32_t now_ms);
static bool app_front_on_line (void);
static bool app_return_line (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                             float32_t progress_mm);
static bool app_still_beside (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                              uint32_t clear_cm);
static void app_forward_drive (int16_t heading_error_x10, uint8_t status);
static float32_t app_leg_progress_mm (const app_avoid_ctx_t * p_ctx,
                                      const buddy2_motion_state_t * p_motion);
static bool app_heading (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                         int16_t * p_heading_x10);
static void app_heading_start (app_avoid_ctx_t * p_ctx);
static void app_heading_update (app_avoid_ctx_t * p_ctx);
static void app_stall_window_start (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static bool app_is_stuck (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_fail (app_avoid_ctx_t * p_ctx, uint32_t now_ms, uint8_t status,
                      const char * p_reason);
static bool app_start_clearance_reverse (app_avoid_ctx_t * p_ctx,
                                         uint32_t now_ms, float32_t distance_mm);
static void app_begin_width_sweep (app_avoid_ctx_t * p_ctx, uint32_t now_ms);
static void app_end_avoidance (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                               bool b_abort);
static bool app_sonar_is_close (uint32_t distance_cm);
static bool app_scan_angle_is_left (uint8_t angle_deg);
static const char * app_ping_failure_name (buddy5_ping_result_t result);
static uint8_t app_sweep_angle (uint8_t index);
static void    app_sweep_point_reset (app_sweep_point_t * p_point);
static bool    app_sweep_ping (uint8_t angle_deg, app_sweep_point_t * p_point,
                               uint32_t * p_result_cm);
static void    app_sweep_point_mm (uint8_t index, uint32_t distance_cm,
                                   float32_t * p_forward_mm,
                                   float32_t * p_lateral_mm);
static bool    app_sweep_face_mm (const uint32_t * p_sweep_cm,
                                  float32_t *      p_face_mm);
static void    app_plan_side (const uint32_t * p_sweep_cm, float32_t face_mm,
                              bool b_low_side, app_side_plan_t * p_plan);
static void    app_log_obstacle_size (float32_t               face_mm,
                                      const app_side_plan_t * p_low,
                                      const app_side_plan_t * p_high);
static void    app_publish_avoidance_scan_values (uint32_t scan_45_cm,
                                                  uint32_t scan_135_cm);
static void    app_publish_avoidance_decision (uint32_t scan_45_cm,
                                               uint32_t scan_135_cm,
                                               bool     b_turn_left);
static int16_t  app_heading_change_x10 (int16_t start_heading_x10,
                                        int16_t now_heading_x10);
static uint16_t app_abs_heading_change_x10 (int16_t change_x10);
static int16_t app_heading_offset_x10 (int16_t heading_x10, int16_t offset_x10);

/**
 * @brief Run the Buddy 5 obstacle-avoidance state machine every 100 ms.
 *
 * In normal operation it publishes only 90-degree forward ultrasonic
 * readings.  Sweep and side pings use non-telemetry pings, so they never
 * replace the dashboard's ultrasonic distance.
 */
void buddy5_avoidance_task (INT stacd, void * p_exinf)
{
    app_avoid_ctx_t ctx = {0};

    (void) stacd;
    (void) p_exinf;

    /* sweep_cm[] is all BUDDY5_NO_ECHO (0) from the initialiser above. */
    ctx.state = APP_AVOID_WATCH_FORWARD;
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
    if (p_ctx->b_result_pending_clear &&
        ((now_ms - p_ctx->finished_ms) >= APP_AVOID_RESULT_VISIBLE_MS))
    {
        robot_telemetry_clear_avoidance_scan();
        p_ctx->b_result_pending_clear = false;
    }

    /* A dashboard command (including stop) cancels a run safely. */
    if ((APP_AVOID_WATCH_FORWARD != p_ctx->state) &&
        !robot_controller_obstacle_avoidance_active())
    {
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_CANCELLED, 0,
                                             0u);
        p_ctx->finished_ms            = now_ms;
        p_ctx->b_result_pending_clear = true;
        buddy5_set_servo_angle(APP_ULTRASONIC_ANGLE_DEG);
        p_ctx->state = APP_AVOID_WATCH_FORWARD;
    }

    /* Update the heading, then where the axle is, while the route drives. */
    if ((APP_AVOID_WAIT_PIVOT == p_ctx->state) ||
        (APP_AVOID_WAIT_LEG == p_ctx->state))
    {
        app_heading_update(p_ctx);
        app_track_position(p_ctx, now_ms);
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

        case APP_AVOID_WAIT_PIVOT:
            app_pivot(p_ctx, now_ms);
        break;

        case APP_AVOID_WAIT_LEG:
            app_leg(p_ctx, now_ms);
        break;

        default:
            app_end_avoidance(p_ctx, now_ms, true);
        break;
    }
}

/* Watch the forward distance and start avoidance at 15 cm. */
static void app_watch_forward (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    uint32_t front_cm = buddy5_measure_cm(now_ms);

    /* robot_controller_begin_obstacle_avoidance() prepares the robot for
     * avoidance and stops line following. */
    if (app_sonar_is_close(front_cm) &&
        robot_controller_begin_obstacle_avoidance())
    {
        tm_printf((UB *) "[avoid] obstacle %u cm: backing %u mm before "
                         "scanning\n",
                  (UINT) front_cm, (UINT) APP_AVOID_REVERSE_DISTANCE_MM);
        robot_telemetry_clear_avoidance_scan();
        p_ctx->b_result_pending_clear = false;
        p_ctx->b_arm_side_reverse     = false;
        p_ctx->b_head_valid           = false;
        if (!app_start_clearance_reverse(p_ctx, now_ms,
                                         APP_AVOID_REVERSE_DISTANCE_MM))
        {
            app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                     "reverse move rejected");
        }
    }
}

/* Back away from the obstacle, then scan (or, after the extra reverse for
 * the barcode arm, start the route). */
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
             * abandon it. */
            p_ctx->b_arm_side_reverse = false;
            p_ctx->face_mm += APP_AVOID_ARM_SIDE_EXTRA_REVERSE_MM;
            app_begin_route(p_ctx, now_ms);
        }
        else if (reverse_motion.left_encoder_fault ||
                 reverse_motion.right_encoder_fault)
        {
            app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                     "encoder fault while backing away");
        }
        else
        {
            tm_printf((UB *) "[avoid] backed away; sweeping to measure the "
                             "obstacle\n");
            app_begin_width_sweep(p_ctx, now_ms);
        }
    }
    else if ((now_ms - p_ctx->state_started_ms) >= APP_AVOID_REVERSE_TIMEOUT_MS)
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_TIMEOUT,
                 "reverse timed out");
    }
    else
    {
        /* Still reversing. */
    }
}

/* Ping at one sweep angle until that point is complete, then step the servo
 * on (30 to 150 degrees), then back to the centre. */
static void app_sweep_step (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    uint8_t angle_deg  = app_sweep_angle(p_ctx->sweep_index);
    bool    b_complete = false;

    if ((now_ms - p_ctx->state_started_ms) >= p_ctx->sweep_settle_ms)
    {
        /* Ping until the point is complete, with the HC-SR04's gap between
         * pings; at most three pings, so this step stays short. */
        b_complete = app_sweep_ping(angle_deg, &p_ctx->sweep_point,
                                    &p_ctx->sweep_cm[p_ctx->sweep_index]);
        while (!b_complete)
        {
            (void) tk_dly_tsk(APP_SONAR_REPING_MS);
            b_complete = app_sweep_ping(angle_deg, &p_ctx->sweep_point,
                                        &p_ctx->sweep_cm[p_ctx->sweep_index]);
        }

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

/* Measure the obstacle from the sweep and go round the side that needs the
 * smaller sidestep, or fall back to the fixed route. */
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
        /* Publish the 45/135-degree readings before deciding, so a blocked
         * route is visible on the dashboard too. */
        app_publish_avoidance_scan_values(p_ctx->sweep_cm[APP_SWEEP_45_INDEX],
                                          p_ctx->sweep_cm[APP_SWEEP_135_INDEX]);

        if (!app_sweep_face_mm(p_ctx->sweep_cm, &face_mm))
        {
            app_fallback_route(p_ctx, now_ms, "no echo ahead during sweep");
        }
        else
        {
            app_plan_side(p_ctx->sweep_cm, face_mm, true, &low_plan);
            app_plan_side(p_ctx->sweep_cm, face_mm, false, &high_plan);
            app_log_obstacle_size(face_mm, &low_plan, &high_plan);

            if (!low_plan.b_usable && !high_plan.b_usable)
            {
                app_fallback_route(p_ctx, now_ms,
                                   "no obstacle edge found on either side");
            }
            else
            {
                /* The smaller sidestep is the narrower way round. */
                b_go_low = low_plan.b_usable &&
                           (!high_plan.b_usable ||
                            (low_plan.sidestep_mm <= high_plan.sidestep_mm));
                app_start_route(
                    p_ctx, now_ms,
                    !app_scan_angle_is_left(b_go_low ? APP_SCAN_ANGLE_45_DEG
                                                     : APP_SCAN_ANGLE_135_DEG),
                    b_go_low ? low_plan.sidestep_mm : high_plan.sidestep_mm,
                    face_mm, b_go_low ? low_plan.edge_mm : high_plan.edge_mm);
            }
        }
    }
}

/**
 * @brief Fall back to a fixed sidestep when the sweep cannot measure the
 *        obstacle.
 *
 * Both the 45 and 135-degree readings must have an echo, the side with more
 * room wins, and it needs at least APP_AVOID_MIN_CLEARANCE_CM.  Otherwise the
 * rover stays stopped.
 */
static void app_fallback_route (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                                const char * p_reason)
{
    uint32_t low_cm  = p_ctx->sweep_cm[APP_SWEEP_45_INDEX];
    uint32_t high_cm = p_ctx->sweep_cm[APP_SWEEP_135_INDEX];
    bool     b_low   = (low_cm >= high_cm);

    tm_printf((UB *) "[avoid] %s; fixed route: 45 deg %u cm, 135 deg %u cm\n",
              p_reason, (UINT) low_cm, (UINT) high_cm);
    if ((BUDDY5_NO_ECHO == low_cm) || (BUDDY5_NO_ECHO == high_cm))
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                 "incomplete side readings");
    }
    else if ((low_cm < APP_AVOID_MIN_CLEARANCE_CM) &&
             (high_cm < APP_AVOID_MIN_CLEARANCE_CM))
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                 "both sides under 20 cm");
    }
    else
    {
        app_start_route(
            p_ctx, now_ms,
            !app_scan_angle_is_left(b_low ? APP_SCAN_ANGLE_45_DEG
                                          : APP_SCAN_ANGLE_135_DEG),
            APP_AVOID_FALLBACK_SIDESTEP_MM, APP_AVOID_FALLBACK_FACE_MM, 0.0f);
    }
}

/* Commit to one side and publish it.  Going round on the right backs away
 * further first (barcode arm); app_reverse_clearance() then begins. */
static void app_start_route (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                             bool b_go_right, float32_t sidestep_mm,
                             float32_t face_mm, float32_t edge_mm)
{
    p_ctx->b_go_right  = b_go_right;
    p_ctx->sidestep_mm = sidestep_mm;
    p_ctx->face_mm     = face_mm;
    /* Gap to keep from the obstacle's side while passing: the reach of the
     * passing side plus the margin, or more if the sidestep was rounded up.
     * Round the right, the left (arm) side passes it. */
    p_ctx->pass_gap_mm =
        (b_go_right ? APP_ROVER_LEFT_EXTENT_MM : APP_ROVER_RIGHT_EXTENT_MM) +
        APP_AVOID_SIDE_MARGIN_MM;
    if ((edge_mm > 0.0f) && ((sidestep_mm - edge_mm) > p_ctx->pass_gap_mm))
    {
        p_ctx->pass_gap_mm = sidestep_mm - edge_mm;
    }
    /* Distances are at most a few hundred millimetres, so they fit UINT. */
    tm_printf((UB *) "[avoid] go round %s: sidestep %u mm, obstacle %u mm "
                     "ahead\n",
              b_go_right ? "right" : "left", (UINT) sidestep_mm,
              (UINT) face_mm);
    app_publish_avoidance_decision(p_ctx->sweep_cm[APP_SWEEP_45_INDEX],
                                   p_ctx->sweep_cm[APP_SWEEP_135_INDEX],
                                   !b_go_right);

    if (!b_go_right)
    {
        app_begin_route(p_ctx, now_ms);
    }
    else
    {
        p_ctx->b_arm_side_reverse = true;
        if (!app_start_clearance_reverse(p_ctx, now_ms,
                                         APP_AVOID_ARM_SIDE_EXTRA_REVERSE_MM))
        {
            p_ctx->b_arm_side_reverse = false;
            app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                     "extra reverse rejected");
        }
    }
}

/* Record the line's heading (the rover has only reversed straight so far)
 * and pivot away from the obstacle. */
static void app_begin_route (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    int16_t heading_x10 = 0;

    app_heading_start(p_ctx);
    if (!app_heading(p_ctx, now_ms, &heading_x10))
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_IMU_UNAVAILABLE,
                 "no heading");
    }
    else
    {
        buddy2_motion_state_t motion = {0};

        /* Position tracking starts here, on the line. */
        buddy2_get_motion_state(&motion);
        p_ctx->line_heading_x10 = heading_x10;
        p_ctx->b_tracking       = true;
        p_ctx->pass_retries     = 0u;
        p_ctx->pos_x_mm         = 0.0f;
        p_ctx->pos_y_mm         = 0.0f;
        p_ctx->track_left_mm    = motion.left_distance_mm;
        p_ctx->track_right_mm   = motion.right_distance_mm;
        app_start_pivot(p_ctx, now_ms,
                        app_heading_offset_x10(
                            heading_x10,
                            p_ctx->b_go_right ? APP_QUARTER_TURN_X10
                                              : -APP_QUARTER_TURN_X10),
                        APP_AFTER_PIVOT_SIDESTEP);
    }
}

/* ------------------------------------------------------------------ *
 *  Pivots
 * ------------------------------------------------------------------ */

static void app_start_pivot (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                             int16_t target_x10, app_after_pivot_t after)
{
    int16_t heading_x10 = target_x10;

    (void) app_heading(p_ctx, now_ms, &heading_x10);
    buddy2_stop();
    p_ctx->pivot_target_x10     = target_x10;
    p_ctx->pivot_start_x10      = heading_x10;
    p_ctx->pivot_last_error_x10 = app_heading_change_x10(heading_x10, target_x10);
    p_ctx->after_pivot          = after;
    p_ctx->b_pivot_saw_line     = false;
    p_ctx->state_started_ms = now_ms;
    p_ctx->state            = APP_AVOID_WAIT_PIVOT;
    /* The last pivots watch for the line: forget black seen before. */
    buddy3_clear_line_seen_event();
    /* Headings are 0..3599, so they fit UINT. */
    tm_printf((UB *) "[avoid] pivot from %u to %u (tenths of a degree)\n",
              (UINT) heading_x10, (UINT) target_x10);
}

/* Turn on the spot toward the target heading; slower near the end. */
static void app_pivot (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    int16_t heading_x10 = 0;
    int16_t error_x10   = 0;
    int16_t pwm         = APP_PIVOT_FAST_PWM;

    if (!app_heading(p_ctx, now_ms, &heading_x10))
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_IMU_LOST,
                 "IMU heading lost while turning");
    }
    else if ((APP_AFTER_PIVOT_RETURN == p_ctx->after_pivot) &&
             app_still_beside(p_ctx, now_ms, APP_TURN_BACK_CLEAR_CM))
    {
        /* Turning back toward the line, the obstacle is in front. */
    }
    else if (((APP_AFTER_PIVOT_SEARCH_OTHER == p_ctx->after_pivot) ||
              (APP_AFTER_PIVOT_SEARCH_FAIL == p_ctx->after_pivot)) &&
             app_front_on_line())
    {
        /* Searching, and a front sensor found the line: hand back now. */
        buddy2_stop();
        robot_telemetry_set_avoidance_turn_pwm(0, 0);
        tm_printf((UB *) "[avoid] search found the line; handing back to "
                         "line following\n");
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_COMPLETE, 0,
                                             0u);
        app_end_avoidance(p_ctx, now_ms, false);
    }
    else if (app_is_stuck(p_ctx, now_ms))
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                 "stuck while turning");
    }
    else
    {
        /* The last pivot along the line notes whether it touched it. */
        if ((APP_AFTER_PIVOT_FINISH == p_ctx->after_pivot) &&
            app_front_on_line())
        {
            p_ctx->b_pivot_saw_line = true;
        }
        /* Positive error: the target is clockwise (right) of the rover. */
        error_x10 = app_heading_change_x10(heading_x10, p_ctx->pivot_target_x10);
        robot_telemetry_set_avoidance_status(
            ROBOT_AVOID_STATUS_TURNING,
            app_heading_change_x10(p_ctx->pivot_start_x10, heading_x10),
            app_abs_heading_change_x10(app_heading_change_x10(
                p_ctx->pivot_start_x10, p_ctx->pivot_target_x10)));
        /* Close enough, or it went past the target in the last step (the
         * sign flipped): stop rather than swing back and forth. */
        if ((app_abs_heading_change_x10(error_x10) <=
             (uint16_t) APP_PIVOT_STOP_X10) ||
            ((error_x10 > 0) != (p_ctx->pivot_last_error_x10 > 0)))
        {
            buddy2_stop();
            robot_telemetry_set_avoidance_turn_pwm(0, 0);
            app_pivot_done(p_ctx, now_ms);
        }
        else if ((now_ms - p_ctx->state_started_ms) >= APP_PIVOT_TIMEOUT_MS)
        {
            app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_TIMEOUT,
                     "turn timed out");
        }
        else
        {
            p_ctx->pivot_last_error_x10 = error_x10;
            if (app_abs_heading_change_x10(error_x10) <=
                (uint16_t) APP_PIVOT_SLOW_ZONE_X10)
            {
                pwm = APP_PIVOT_SLOW_PWM;
            }
            if (error_x10 > 0)
            {
                (void) buddy2_set_motor_pwm_at_speed(pwm, (int16_t) -pwm,
                                                     APP_AVOID_SPEED_PCT);
                robot_telemetry_set_avoidance_turn_pwm(pwm, (int16_t) -pwm);
            }
            else
            {
                (void) buddy2_set_motor_pwm_at_speed((int16_t) -pwm, pwm,
                                                     APP_AVOID_SPEED_PCT);
                robot_telemetry_set_avoidance_turn_pwm((int16_t) -pwm, pwm);
            }
        }
    }
}

/* A pivot reached its heading: start the next part of the route. */
static void app_pivot_done (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    int16_t toward_line_x10 = app_heading_offset_x10(
        p_ctx->line_heading_x10, p_ctx->b_go_right ? -APP_QUARTER_TURN_X10
                                                   : APP_QUARTER_TURN_X10);

    if (APP_AFTER_PIVOT_SIDESTEP == p_ctx->after_pivot)
    {
        app_start_leg(p_ctx, now_ms, APP_LEG_SIDESTEP, p_ctx->pivot_target_x10,
                      p_ctx->sidestep_mm);
    }
    else if (APP_AFTER_PIVOT_PASS == p_ctx->after_pivot)
    {
        /* The obstacle's face is as far ahead of the sonar as when it was
         * measured: the sidestep and the two pivots only moved it aside. */
        app_start_leg(p_ctx, now_ms, APP_LEG_PASS, p_ctx->line_heading_x10,
                      APP_PASS_MAX_MM);
    }
    else if (APP_AFTER_PIVOT_PASS_MORE == p_ctx->after_pivot)
    {
        app_start_leg(p_ctx, now_ms, APP_LEG_PASS_MORE, p_ctx->line_heading_x10,
                      APP_PASS_RETRY_MM);
    }
    else if (APP_AFTER_PIVOT_RETURN == p_ctx->after_pivot)
    {
        /* Only black seen from here on is the line beyond the obstacle. */
        buddy3_clear_line_seen_event();
        app_start_leg(p_ctx, now_ms, APP_LEG_RETURN, toward_line_x10,
                      ((p_ctx->sidestep_mm + APP_RETURN_EXTRA_MM) >
                       APP_RETURN_MAX_MM)
                          ? (p_ctx->sidestep_mm + APP_RETURN_EXTRA_MM)
                          : APP_RETURN_MAX_MM);
    }
    else if ((APP_AFTER_PIVOT_FINISH == p_ctx->after_pivot) &&
             !p_ctx->b_pivot_saw_line && !app_front_on_line())
    {
        /* Pointing along the line, but no sensor has touched it: it may be
         * beside the rover.  Sweep toward where it should be first (the
         * way the rover was driving to reach it). */
        p_ctx->search_sign = p_ctx->b_go_right ? (int16_t) -1 : (int16_t) 1;
        tm_printf((UB *) "[avoid] line not under the sensors; searching\n");
        app_start_pivot(p_ctx, now_ms,
                        app_heading_offset_x10(
                            p_ctx->line_heading_x10,
                            (int16_t) (p_ctx->search_sign *
                                       APP_SEARCH_SWEEP_X10)),
                        APP_AFTER_PIVOT_SEARCH_OTHER);
    }
    else if (APP_AFTER_PIVOT_SEARCH_OTHER == p_ctx->after_pivot)
    {
        app_start_pivot(p_ctx, now_ms,
                        app_heading_offset_x10(
                            p_ctx->line_heading_x10,
                            (int16_t) (-p_ctx->search_sign *
                                       APP_SEARCH_SWEEP_X10)),
                        APP_AFTER_PIVOT_SEARCH_FAIL);
    }
    else if (APP_AFTER_PIVOT_SEARCH_FAIL == p_ctx->after_pivot)
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_TIMEOUT,
                 "line not found after searching both ways");
    }
    else
    {
        tm_printf((UB *) "[avoid] back on the line; handing back to line "
                         "following\n");
        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_COMPLETE, 0,
                                             0u);
        app_end_avoidance(p_ctx, now_ms, false);
    }
}

/* True when a front IR sensor is on black now, or was since the last
 * check (Buddy 3 latches black seen between this task's steps). */
static bool app_front_on_line (void)
{
    bool b_left  = false;
    bool b_right = false;
    bool b_side  = false;
    bool b_seen  = buddy3_take_line_seen_event();

    buddy3_get_colors(&b_left, &b_right, &b_side);

    return b_seen || b_left || b_right;
}

/* ------------------------------------------------------------------ *
 *  Straight legs
 * ------------------------------------------------------------------ */

static void app_start_leg (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                           app_leg_t leg, int16_t heading_x10,
                           float32_t target_mm)
{
    buddy2_get_motion_state(&p_ctx->leg_start_motion);
    p_ctx->leg              = leg;
    p_ctx->leg_heading_x10  = heading_x10;
    p_ctx->leg_target_mm    = target_mm;
    p_ctx->pass_near_count  = 0u;
    p_ctx->pass_clear_count = 0u;
    p_ctx->b_pass_alongside = false;
    p_ctx->b_pass_cleared   = false;
    p_ctx->pass_cleared_mm  = 0.0f;
    p_ctx->pass_steer_x10   = 0;
    p_ctx->b_return_trusting = false;
    p_ctx->return_trust_mm   = 0.0f;
    p_ctx->b_return_black    = false;
    p_ctx->b_return_along    = false;
    p_ctx->return_black_mm   = 0.0f;
    p_ctx->state_started_ms = now_ms;
    p_ctx->state            = APP_AVOID_WAIT_LEG;
    /* Leg distances are at most a few hundred millimetres. */
    robot_telemetry_set_avoidance_forward_progress(0u, (uint16_t) target_mm);
    tm_printf((UB *) "[avoid] %s leg, up to %u mm\n",
              (APP_LEG_SIDESTEP == leg)
                  ? "sidestep"
                  : ((APP_LEG_PASS == leg)
                         ? "pass"
                         : ((APP_LEG_RETURN == leg)
                                ? "return"
                                : ((APP_LEG_ALIGN == leg) ? "align"
                                                          : "pass further"))),
              (UINT) target_mm);
}

/* Drive the current leg with heading hold and finish it on its condition. */
static void app_leg (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    buddy2_motion_state_t motion      = {0};
    int16_t               heading_x10 = 0;
    float32_t             progress_mm = 0.0f;
    uint8_t               status      = ROBOT_AVOID_STATUS_FORWARD;

    buddy2_get_motion_state(&motion);
    progress_mm = app_leg_progress_mm(p_ctx, &motion);
    /* Progress is at most a little over APP_PASS_MAX_MM. */
    robot_telemetry_set_avoidance_forward_progress(
        (uint16_t) progress_mm, (uint16_t) p_ctx->leg_target_mm);

    if (!app_heading(p_ctx, now_ms, &heading_x10))
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_IMU_LOST,
                 "IMU heading lost while driving");
    }
    /* A fault seen in a leg's first second is left over from an earlier
     * stall; the first tick clears it. */
    else if ((motion.left_encoder_fault || motion.right_encoder_fault) &&
             ((now_ms - p_ctx->state_started_ms) >= APP_AVOID_STALE_FAULT_MS))
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                 "encoder fault while driving");
    }
    else if (app_is_stuck(p_ctx, now_ms))
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                 "stuck while driving");
    }
    else if ((now_ms - p_ctx->state_started_ms) >= APP_AVOID_LEG_TIMEOUT_MS)
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_TIMEOUT, "leg timed out");
    }
    else if (!app_leg_check(p_ctx, now_ms, progress_mm))
    {
        /* Leg still running: hold its heading. */
        if ((APP_LEG_PASS == p_ctx->leg) || (APP_LEG_PASS_MORE == p_ctx->leg))
        {
            status = ROBOT_AVOID_STATUS_PASSING;
        }
        else if ((APP_LEG_RETURN == p_ctx->leg) || (APP_LEG_ALIGN == p_ctx->leg))
        {
            status = ROBOT_AVOID_STATUS_REJOINING;
        }
        else
        {
            status = ROBOT_AVOID_STATUS_FORWARD;
        }
        /* While passing, the gap-keeping offset bends the held heading. */
        app_forward_drive(
            app_heading_change_x10(
                app_heading_offset_x10(p_ctx->leg_heading_x10,
                                       (APP_LEG_PASS == p_ctx->leg)
                                           ? p_ctx->pass_steer_x10
                                           : 0),
                heading_x10),
            status);
    }
    else
    {
        /* The leg finished or failed this step. */
    }
}

/**
 * @brief Check the current leg's end condition and start what follows.
 * @return true if the leg ended (or the run stopped) this step.
 */
static bool app_leg_check (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                           float32_t progress_mm)
{
    bool b_ended = true;
    int16_t look = (int16_t) (p_ctx->b_go_right ? APP_PASS_LOOK_LEFT_DEG
                                                : APP_PASS_LOOK_RIGHT_DEG);

    if ((APP_LEG_SIDESTEP == p_ctx->leg) && app_front_blocked(now_ms))
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                 "something in the way of the sidestep");
    }
    /* The sidestep ends when the tracked position is far enough from the
     * line (not just the distance driven: the heading may be off). */
    else if ((APP_LEG_SIDESTEP == p_ctx->leg) &&
             (app_offset_mm(p_ctx) >= p_ctx->sidestep_mm))
    {
        /* Look at the obstacle's side (servo settles during the pivot). */
        buddy5_set_servo_angle((uint8_t) look);
        app_start_pivot(p_ctx, now_ms, p_ctx->line_heading_x10,
                        APP_AFTER_PIVOT_PASS);
    }
    else if ((APP_LEG_PASS == p_ctx->leg) && app_pass_update(p_ctx, progress_mm))
    {
        buddy5_set_servo_angle(APP_ULTRASONIC_ANGLE_DEG);
        tm_printf((UB *) "[avoid] passed the obstacle after %u mm; turning "
                         "back toward the line\n",
                  (UINT) progress_mm);
        app_start_pivot(p_ctx, now_ms,
                        app_heading_offset_x10(
                            p_ctx->line_heading_x10,
                            p_ctx->b_go_right ? -APP_QUARTER_TURN_X10
                                              : APP_QUARTER_TURN_X10),
                        APP_AFTER_PIVOT_RETURN);
    }
    else if ((APP_LEG_PASS == p_ctx->leg) &&
             (progress_mm >= p_ctx->leg_target_mm))
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                 "obstacle still alongside: too long to pass");
    }
    else if ((APP_LEG_SIDESTEP == p_ctx->leg) &&
             (progress_mm >= (2.0f * p_ctx->leg_target_mm)))
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_BLOCKED,
                 "sidestep is not moving away from the line");
    }
    else if ((APP_LEG_RETURN == p_ctx->leg) &&
             app_still_beside(p_ctx, now_ms, APP_RETURN_CLEAR_CM))
    {
        /* On the way back the obstacle is in front: not past it yet. */
    }
    else if ((APP_LEG_PASS_MORE == p_ctx->leg) &&
             (progress_mm >= p_ctx->leg_target_mm))
    {
        app_start_pivot(p_ctx, now_ms,
                        app_heading_offset_x10(
                            p_ctx->line_heading_x10,
                            p_ctx->b_go_right ? -APP_QUARTER_TURN_X10
                                              : APP_QUARTER_TURN_X10),
                        APP_AFTER_PIVOT_RETURN);
    }
    else if ((APP_LEG_RETURN == p_ctx->leg) &&
             app_return_line(p_ctx, now_ms, progress_mm))
    {
        /* Crossed the original line: aligning on it. */
    }
    else if ((APP_LEG_RETURN == p_ctx->leg) && !p_ctx->b_return_trusting &&
             (app_offset_mm(p_ctx) <= APP_ALIGN_FORWARD_MM))
    {
        /* By the tracked position the sensors are over the line now; give
         * them a little further to confirm it. */
        p_ctx->b_return_trusting = true;
        p_ctx->return_trust_mm   = progress_mm;
        b_ended                  = false;
    }
    else if ((APP_LEG_RETURN == p_ctx->leg) && p_ctx->b_return_trusting &&
             (progress_mm >= (p_ctx->return_trust_mm + APP_RETURN_CONFIRM_MM)) &&
             (progress_mm >= APP_RETURN_MIN_MM))
    {
        /* Offset at most APP_ALIGN_FORWARD_MM here. */
        tm_printf((UB *) "[avoid] IR did not see the line; trusting the "
                         "tracked position (%d mm off)\n",
                  (INT) app_offset_mm(p_ctx));
        app_start_leg(p_ctx, now_ms, APP_LEG_ALIGN, p_ctx->leg_heading_x10,
                      (app_offset_mm(p_ctx) > 0.0f) ? app_offset_mm(p_ctx)
                                                    : 0.0f);
    }
    else if ((APP_LEG_RETURN == p_ctx->leg) &&
             (progress_mm >= p_ctx->leg_target_mm))
    {
        app_fail(p_ctx, now_ms, ROBOT_AVOID_STATUS_TIMEOUT,
                 "line not found on the way back");
    }
    else if ((APP_LEG_ALIGN == p_ctx->leg) &&
             (progress_mm >= p_ctx->leg_target_mm))
    {
        app_start_pivot(p_ctx, now_ms, p_ctx->line_heading_x10,
                        APP_AFTER_PIVOT_FINISH);
    }
    else
    {
        b_ended = false;
    }

    return b_ended;
}

/**
 * @brief Follow the obstacle with the side-facing sonar while passing it.
 *
 * First it waits until the obstacle is alongside (two near readings), then
 * until it has ended (three clear readings), then drives APP_PASS_TAIL_MM
 * more so the rover's rear is past it too.  If the sonar never sees it
 * alongside, it assumes an APP_PASS_ASSUMED_DEPTH_MM deep obstacle.
 *
 * @return true once the rover is past the obstacle.
 */
static bool app_pass_update (app_avoid_ctx_t * p_ctx, float32_t progress_mm)
{
    buddy5_ping_result_t result      = BUDDY5_PING_TIMEOUT_RISE;
    uint32_t             distance_cm = buddy5_sample_with_result_cm(&result);
    int16_t              heading_x10 = p_ctx->line_heading_x10;
    int16_t              away_x10    = 0;
    float32_t            side_mm     = 0.0f;
    bool                 b_near      = false;

    /* The sideways distance to the obstacle's side, allowing for the rover
     * being turned away from it (or toward it): the sonar then looks more
     * (or less) square-on.  Assuming the rover is parallel made it read
     * nearer as it turned away, so it turned away more (on the track it
     * ran off 25 degrees).  Round the right, turning right is away. */
    (void) app_heading(p_ctx, 0u, &heading_x10);
    away_x10 = app_heading_change_x10(p_ctx->line_heading_x10, heading_x10);
    if (!p_ctx->b_go_right)
    {
        away_x10 = (int16_t) -away_x10;
    }
    side_mm = (float32_t) distance_cm * APP_MM_PER_CM *
              app_sine((int16_t) (APP_PASS_LOOK_OFF_AXIS_X10 + away_x10));
    b_near  = (BUDDY5_PING_VALID == result) &&
              (side_mm < (app_offset_mm(p_ctx) + APP_PASS_NEAR_BEYOND_LINE_MM));
    float32_t steer = 0.0f;

    /* Show the side reading on the dashboard (scan left/right) while
     * passing: round the right, the obstacle is on the left. */
    if (p_ctx->b_go_right)
    {
        robot_telemetry_set_avoidance_scan_values(distance_cm, BUDDY5_NO_ECHO);
    }
    else
    {
        robot_telemetry_set_avoidance_scan_values(BUDDY5_NO_ECHO, distance_cm);
    }

    /* Keep the gap while it is alongside: nearer than planned steers away
     * from it, farther steers back in.  Away is right (heading up) when
     * going round the right. */
    p_ctx->pass_steer_x10 = 0;
    if (p_ctx->b_pass_alongside && !p_ctx->b_pass_cleared && b_near)
    {
        steer = (p_ctx->pass_gap_mm - side_mm) * APP_PASS_STEER_X10_PER_MM;
        if (steer > (float32_t) APP_PASS_STEER_MAX_X10)
        {
            steer = (float32_t) APP_PASS_STEER_MAX_X10;
        }
        else if (steer < -(float32_t) APP_PASS_STEER_MAX_X10)
        {
            steer = -(float32_t) APP_PASS_STEER_MAX_X10;
        }
        else
        {
            /* Within the steering limit. */
        }
        /* Clamped to +/-100 above, so it fits int16_t. */
        p_ctx->pass_steer_x10 =
            (int16_t) (p_ctx->b_go_right ? steer : -steer);
    }

    if (!p_ctx->b_pass_alongside)
    {
        p_ctx->pass_near_count = b_near ? (uint8_t) (p_ctx->pass_near_count + 1u)
                                        : 0u;
        if (p_ctx->pass_near_count >= APP_PASS_SEEN_COUNT)
        {
            p_ctx->b_pass_alongside = true;
            tm_printf((UB *) "[avoid] obstacle alongside at %u mm (side %u "
                             "mm)\n",
                      (UINT) progress_mm, (UINT) side_mm);
        }
        else if (!p_ctx->b_pass_cleared &&
                 (progress_mm >= (p_ctx->face_mm + APP_PASS_UNSEEN_MM)))
        {
            /* The side sonar is missing it: assume a typical depth. */
            p_ctx->b_pass_cleared  = true;
            p_ctx->pass_cleared_mm = p_ctx->face_mm + APP_PASS_ASSUMED_DEPTH_MM;
            tm_printf((UB *) "[avoid] obstacle not seen alongside; assuming "
                             "%u mm deep\n",
                      (UINT) APP_PASS_ASSUMED_DEPTH_MM);
        }
        else
        {
            /* Still approaching it. */
        }
    }
    else if (!p_ctx->b_pass_cleared)
    {
        p_ctx->pass_clear_count = b_near
                                      ? 0u
                                      : (uint8_t) (p_ctx->pass_clear_count + 1u);
        if ((p_ctx->pass_clear_count >= APP_PASS_CLEAR_COUNT) &&
            (progress_mm >= (p_ctx->face_mm + APP_PASS_MIN_DEPTH_MM)))
        {
            p_ctx->b_pass_cleared  = true;
            p_ctx->pass_cleared_mm = progress_mm;
            tm_printf((UB *) "[avoid] obstacle ended at %u mm\n",
                      (UINT) progress_mm);
        }
    }
    else
    {
        /* Past its end: only the tail distance is left. */
    }

    return p_ctx->b_pass_cleared &&
           (progress_mm >= (p_ctx->pass_cleared_mm + APP_PASS_TAIL_MM));
}

/**
 * @brief While turning or driving back toward the line, check that the
 *        obstacle is not in front.  If it is, the rover is not past it yet:
 *        turn back along the line and drive further (or stop after
 *        APP_PASS_MAX_RETRIES).
 * @return true if the obstacle was there (the route has been changed).
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
                     "obstacle still in the way after driving further");
        }
        else
        {
            p_ctx->pass_retries++;
            tm_printf((UB *) "[avoid] obstacle %u cm ahead while turning "
                             "back: driving %u mm further (try %u of %u)\n",
                      (UINT) front_cm, (UINT) APP_PASS_RETRY_MM,
                      (UINT) p_ctx->pass_retries, (UINT) APP_PASS_MAX_RETRIES);
            app_start_pivot(p_ctx, now_ms, p_ctx->line_heading_x10,
                            APP_AFTER_PIVOT_PASS_MORE);
        }
    }

    return b_blocked;
}

/**
 * @brief Watch the front IR sensors on the way back to the line.
 *
 * The rover drives across the original line, so its black ends within
 * APP_RETURN_CROSS_MAX_MM: that is the line, and the align leg starts (the
 * sensors are already past it by the black's length).  Black that lasts
 * longer is a line running the rover's way, such as a junction's branch:
 * it is ignored until the sensors are on white again.
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
    float32_t crossed   = 0.0f;
    float32_t remaining = 0.0f;

    buddy3_get_colors(&b_left, &b_right, &b_side);
    if (!p_ctx->b_return_black)
    {
        /* The latched event catches a line crossed between two steps. */
        if (b_seen || b_left || b_right)
        {
            p_ctx->b_return_black  = true;
            p_ctx->b_return_along  = false;
            p_ctx->return_black_mm = progress_mm;
        }
    }
    else if (b_left || b_right)
    {
        if (!p_ctx->b_return_along &&
            ((progress_mm - p_ctx->return_black_mm) > APP_RETURN_CROSS_MAX_MM))
        {
            p_ctx->b_return_along = true;
            tm_printf((UB *) "[avoid] black for over %u mm: a line running "
                             "our way (a branch), ignored\n",
                      (UINT) APP_RETURN_CROSS_MAX_MM);
        }
    }
    else
    {
        /* White again. */
        if (!p_ctx->b_return_along &&
            (app_offset_mm(p_ctx) >
             (APP_ALIGN_FORWARD_MM + APP_RETURN_LINE_WINDOW_MM)))
        {
            /* A crossing, but the tracked position says the original line
             * is still well ahead: another line on the mat. */
            tm_printf((UB *) "[avoid] crossed a line %d mm out: not the "
                             "track, keep going\n",
                      (INT) app_offset_mm(p_ctx));
            p_ctx->b_return_black = false;
        }
        else if (!p_ctx->b_return_along)
        {
            crossed   = progress_mm - p_ctx->return_black_mm;
            remaining = APP_ALIGN_FORWARD_MM - crossed;
            if (remaining < 0.0f)
            {
                remaining = 0.0f;
            }
            /* The axle is now `remaining` from the line: that corrects the
             * tracked position. */
            p_ctx->pos_y_mm = p_ctx->b_go_right ? remaining : -remaining;
            tm_printf((UB *) "[avoid] crossed the line (%u mm of black) "
                             "after %u mm; aligning\n",
                      (UINT) crossed, (UINT) progress_mm);
            app_start_leg(p_ctx, now_ms, APP_LEG_ALIGN, p_ctx->leg_heading_x10,
                          remaining);
            b_found = true;
        }
        else
        {
            p_ctx->b_return_black = false;
            p_ctx->b_return_along = false;
        }
    }

    return b_found;
}

/* True when the forward-facing sonar sees something within the stop
 * distance. */
static bool app_front_blocked (uint32_t now_ms)
{
    return app_sonar_is_close(buddy5_measure_cm(now_ms));
}

/* Proportional heading hold for a straight leg. */
static void app_forward_drive (int16_t heading_error_x10, uint8_t status)
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

    /* Positive compass error is right of the target: correct left.  Base
     * +/- 2500 fits int16_t. */
    left_pwm  = (int16_t) (APP_AVOID_FORWARD_BASE_PWM - correction_pwm);
    right_pwm = (int16_t) (APP_AVOID_FORWARD_BASE_PWM + correction_pwm);
    /* APP_AVOID_SPEED_PCT is a valid constant, so this cannot fail. */
    (void) buddy2_set_motor_pwm_at_speed(left_pwm, right_pwm,
                                         APP_AVOID_SPEED_PCT);
    robot_telemetry_set_avoidance_turn_pwm(left_pwm, right_pwm);
    robot_telemetry_set_avoidance_status(status, heading_error_x10, 0u);
}

/* Average distance both wheels moved since the leg started. */
static float32_t app_leg_progress_mm (const app_avoid_ctx_t *       p_ctx,
                                      const buddy2_motion_state_t * p_motion)
{
    float32_t left_mm =
        p_motion->left_distance_mm - p_ctx->leg_start_motion.left_distance_mm;
    float32_t right_mm =
        p_motion->right_distance_mm - p_ctx->leg_start_motion.right_distance_mm;

    left_mm  = (left_mm < 0.0f) ? -left_mm : left_mm;
    right_mm = (right_mm < 0.0f) ? -right_mm : right_mm;

    return (left_mm + right_mm) * 0.5f;
}

/* ------------------------------------------------------------------ *
 *  Position tracking (encoders for distance, compass for direction)
 * ------------------------------------------------------------------ */

/* Add this step's travel to the axle position: centre travel from the
 * encoders, split along and across the line by the compass heading. */
static void app_track_position (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    buddy2_motion_state_t motion      = {0};
    float32_t             step_mm     = 0.0f;
    int16_t               heading_x10 = 0;
    int16_t               angle_x10   = 0;

    if (p_ctx->b_tracking)
    {
        buddy2_get_motion_state(&motion);
        step_mm = 0.5f * ((motion.left_distance_mm - p_ctx->track_left_mm) +
                          (motion.right_distance_mm - p_ctx->track_right_mm));
        p_ctx->track_left_mm  = motion.left_distance_mm;
        p_ctx->track_right_mm = motion.right_distance_mm;
        if (app_heading(p_ctx, now_ms, &heading_x10))
        {
            angle_x10 =
                app_heading_change_x10(p_ctx->line_heading_x10, heading_x10);
            p_ctx->pos_x_mm += step_mm * app_cosine(angle_x10);
            p_ctx->pos_y_mm += step_mm * app_sine(angle_x10);
        }
    }
}

/* How far the axle is from the line, on the side the route goes round
 * (positive while it is out there). */
static float32_t app_offset_mm (const app_avoid_ctx_t * p_ctx)
{
    return p_ctx->b_go_right ? p_ctx->pos_y_mm : -p_ctx->pos_y_mm;
}

/* Sine of an angle in tenths of a degree, -180.0 to +180.0 degrees
 * (Bhaskara's formula, within 0.2 % of full scale).  math.h cannot be
 * included here: its size_t clashes with the T-Kernel headers. */
static float32_t app_sine (int16_t angle_x10)
{
    float32_t degrees = (float32_t) angle_x10 / 10.0f;
    float32_t product = 0.0f;
    float32_t sine    = 0.0f;
    bool      b_minus = (degrees < 0.0f);

    if (b_minus)
    {
        degrees = -degrees;
    }
    if (degrees > 180.0f)
    {
        degrees = 180.0f;
    }
    product = degrees * (180.0f - degrees);
    sine    = (4.0f * product) / (40500.0f - product);

    return b_minus ? -sine : sine;
}

/* Cosine of an angle in tenths of a degree, -180.0 to +180.0 degrees. */
static float32_t app_cosine (int16_t angle_x10)
{
    int16_t magnitude = (angle_x10 < 0) ? (int16_t) -angle_x10 : angle_x10;

    /* 900 - (0..1800) is -900..900, inside app_sine's range. */
    return app_sine((int16_t) (900 - magnitude));
}

/* ------------------------------------------------------------------ *
 *  Compass, stall check and endings
 * ------------------------------------------------------------------ */

/**
 * @brief Start the heading at the compass (or, with no compass, at 0: every
 *        turn is relative to the line's heading, so any start will do).
 */
static void app_heading_start (app_avoid_ctx_t * p_ctx)
{
    buddy4_imu_state_t    imu    = {0};
    buddy2_motion_state_t motion = {0};

    buddy4_get_state(&imu);
    buddy2_get_motion_state(&motion);
    p_ctx->head_x10      = imu.mag_ok ? (float32_t) imu.heading_deg_x10 : 0.0f;
    p_ctx->head_left_mm  = motion.left_distance_mm;
    p_ctx->head_right_mm = motion.right_distance_mm;
    p_ctx->head_reject   = 0u;
    p_ctx->b_head_valid  = true;
    if (!imu.mag_ok)
    {
        tm_printf((UB *) "[avoid] compass unavailable: turning by the wheel "
                         "encoders\n");
    }
}

/**
 * @brief Carry the heading on by this step's wheel travel, then take the
 *        compass reading unless it is a glitch (see APP_COMPASS_GATE_X10).
 */
static void app_heading_update (app_avoid_ctx_t * p_ctx)
{
    buddy4_imu_state_t    imu      = {0};
    buddy2_motion_state_t motion   = {0};
    buddy2_calibration_t  geometry = {0};
    float32_t             left_mm  = 0.0f;
    float32_t             right_mm = 0.0f;
    float32_t             turn_x10 = 0.0f;
    int16_t               diff_x10 = 0;

    if (p_ctx->b_head_valid)
    {
        buddy2_get_motion_state(&motion);
        buddy2_get_calibration(&geometry);
        left_mm  = motion.left_distance_mm - p_ctx->head_left_mm;
        right_mm = motion.right_distance_mm - p_ctx->head_right_mm;
        p_ctx->head_left_mm  = motion.left_distance_mm;
        p_ctx->head_right_mm = motion.right_distance_mm;
        if (geometry.track_width_mm > 0.0f)
        {
            /* Left wheel ahead turns right: the compass heading rises. */
            turn_x10 = ((left_mm - right_mm) / geometry.track_width_mm) *
                       APP_X10_PER_RAD;
            if ((left_mm * right_mm) < 0.0f)
            {
                turn_x10 *= APP_PIVOT_WHEEL_SLIP;
            }
        }
        p_ctx->head_x10 += turn_x10;
        while (p_ctx->head_x10 < 0.0f)
        {
            p_ctx->head_x10 += (float32_t) APP_FULL_TURN_X10;
        }
        while (p_ctx->head_x10 >= (float32_t) APP_FULL_TURN_X10)
        {
            p_ctx->head_x10 -= (float32_t) APP_FULL_TURN_X10;
        }

        buddy4_get_state(&imu);
        if (imu.mag_ok)
        {
            /* head_x10 is 0..3599 here, so it fits int16_t. */
            diff_x10 = app_heading_change_x10((int16_t) p_ctx->head_x10,
                                              imu.heading_deg_x10);
            if ((app_abs_heading_change_x10(diff_x10) <=
                 (uint16_t) APP_COMPASS_GATE_X10) ||
                (p_ctx->head_reject >= APP_COMPASS_AGREE_COUNT))
            {
                p_ctx->head_x10    = (float32_t) imu.heading_deg_x10;
                p_ctx->head_reject = 0u;
            }
            else
            {
                p_ctx->head_reject++;
            }
        }
    }
}

/**
 * @brief The current heading (tenths of a degree, 0..3599).
 * @return false before app_heading_start().
 */
static bool app_heading (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                         int16_t * p_heading_x10)
{
    (void) now_ms;
    if (p_ctx->b_head_valid)
    {
        /* 0..3599 after app_heading_update(), so it fits int16_t. */
        *p_heading_x10 = (int16_t) p_ctx->head_x10;
    }

    return p_ctx->b_head_valid;
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
 * @brief True when the wheels moved under APP_AVOID_STALL_MIN_MM on average
 *        in the last APP_AVOID_STALL_WINDOW_MS while driving.
 */
static bool app_is_stuck (app_avoid_ctx_t * p_ctx, uint32_t now_ms)
{
    buddy2_motion_state_t motion   = {0};
    float32_t             left_mm  = 0.0f;
    float32_t             right_mm = 0.0f;
    bool                  b_stuck  = false;

    if ((now_ms - p_ctx->stall_window_started_ms) >= APP_AVOID_STALL_WINDOW_MS)
    {
        buddy2_get_motion_state(&motion);
        left_mm  = motion.left_distance_mm - p_ctx->stall_left_start_mm;
        right_mm = motion.right_distance_mm - p_ctx->stall_right_start_mm;
        left_mm  = (left_mm < 0.0f) ? -left_mm : left_mm;
        right_mm = (right_mm < 0.0f) ? -right_mm : right_mm;
        b_stuck  = (((left_mm + right_mm) * 0.5f) < APP_AVOID_STALL_MIN_MM);
        app_stall_window_start(p_ctx, now_ms);
    }

    return b_stuck;
}

/* Stop the run: report why on the console and the dashboard. */
static void app_fail (app_avoid_ctx_t * p_ctx, uint32_t now_ms, uint8_t status,
                      const char * p_reason)
{
    buddy2_stop();
    robot_telemetry_set_avoidance_turn_pwm(0, 0);
    tm_printf((UB *) "[avoid] %s; stopped\n", p_reason);
    robot_telemetry_set_avoidance_status(status, 0, 0u);
    app_end_avoidance(p_ctx, now_ms, true);
}

/** @brief Stop and reverse by distance_mm, creating room to scan and turn.
 *  Returns false if Buddy 2 rejected the move. */
static bool app_start_clearance_reverse (app_avoid_ctx_t * p_ctx,
                                         uint32_t now_ms, float32_t distance_mm)
{
    bool b_started = false;

    buddy2_stop();
    if (buddy2_move_backward_at_speed(distance_mm, APP_AVOID_REVERSE_PWM,
                                      APP_AVOID_SPEED_PCT))
    {
        robot_telemetry_set_avoidance_turn_pwm(-APP_AVOID_REVERSE_PWM,
                                               -APP_AVOID_REVERSE_PWM);
        /* Reverse distances are about 100 mm, so they fit uint16_t. */
        robot_telemetry_set_avoidance_forward_progress(0u,
                                                       (uint16_t) distance_mm);
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

    robot_telemetry_set_obstacle_size(0u, 0u, false); /* new obstacle */

    for (index = 0u; index < APP_SWEEP_POINTS; index++)
    {
        p_ctx->sweep_cm[index] = BUDDY5_NO_ECHO;
    }
    p_ctx->sweep_index = 0u;
    app_sweep_point_reset(&p_ctx->sweep_point);
    /* The first move swings the servo 60 degrees: give it the full settle
     * time.  Later 5-degree steps use the shorter step settle. */
    p_ctx->sweep_settle_ms = APP_SERVO_SETTLE_MS;
    buddy5_set_servo_angle(app_sweep_angle(0u));
    p_ctx->state            = APP_AVOID_WAIT_SWEEP;
    p_ctx->state_started_ms = now_ms;
    robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_SCANNING, 0, 0u);
}

/**
 * @brief Leave avoidance mode and keep the result visible for a moment.
 *
 * Faces the ultrasonic sensor forward again and resumes normal
 * forward-distance monitoring.  A finished run tells the robot controller
 * that avoidance is complete (it hands back to line following); an aborted
 * run stops the motors and does not hand back.
 */
static void app_end_avoidance (app_avoid_ctx_t * p_ctx, uint32_t now_ms,
                               bool b_abort)
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

    if (BUDDY5_PING_VALID == result)
    {
        p_name = "valid";
    }
    else if (BUDDY5_PING_TIMEOUT_RISE == result)
    {
        p_name = "no echo";
    }
    else if (BUDDY5_PING_TIMEOUT_FALL == result)
    {
        p_name = "echo too long";
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
    /* At most 30 + 24 * 5 = 150, which fits uint8_t. */
    return (uint8_t) (APP_SWEEP_FIRST_DEG +
                      ((uint32_t) index * APP_SWEEP_STEP_DEG));
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
 * An angle finishes after two valid echoes or three attempts.  It keeps the
 * nearest valid echo.  An angle with no valid echo reads BUDDY5_NO_ECHO,
 * which the route planner treats as open space.
 */
static bool app_sweep_ping (uint8_t angle_deg, app_sweep_point_t * p_point,
                            uint32_t * p_result_cm)
{
    buddy5_ping_result_t result      = BUDDY5_PING_TIMEOUT_RISE;
    uint32_t             distance_cm = buddy5_sample_with_result_cm(&result);
    bool                 b_complete  = false;

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
        tm_printf((UB *) "[avoid] sweep %u deg: %u cm (%u/%u, %s)\n",
                  (UINT) angle_deg, (UINT) p_point->nearest_cm,
                  (UINT) p_point->valid, (UINT) p_point->attempts,
                  app_ping_failure_name(result));
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
    /* index is 0..24, so the step from the centre is 0..12. */
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
 * Uses the nearest forward part of the readings within 10 degrees of
 * straight ahead, so a single missed echo does not lose the obstacle.
 *
 * @return false if none of those angles returned an echo.
 */
static bool app_sweep_face_mm (const uint32_t * p_sweep_cm,
                               float32_t *      p_face_mm)
{
    float32_t forward_mm = 0.0f;
    float32_t lateral_mm = 0.0f;
    uint8_t   index      = 0u;
    bool      b_found    = false;

    for (index = APP_SWEEP_CENTRE_INDEX - 2u;
         index <= (APP_SWEEP_CENTRE_INDEX + 2u); index++)
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
 * @brief Find one edge of the obstacle and size the sidestep round it.
 *
 * Walks outward from the centre of the sweep.  A reading stays on the
 * obstacle while it has an echo and its forward part is within
 * APP_SWEEP_FACE_TOLERANCE_MM of the front face; the first reading that is
 * open, much farther or much nearer marks the edge.  A nearer reading is
 * something else beside the obstacle, so that side is not used.  The wide beam still echoes off the
 * corner for APP_SWEEP_BEAM_STEPS past it, so the edge is taken that many
 * steps inside the last echo on the face.
 *
 * The sidestep covers the edge plus the rover's reach on the side that
 * passes the obstacle plus a margin.
 *
 * @param b_low_side True for the servo angles below 90 degrees.
 */
static void app_plan_side (const uint32_t * p_sweep_cm, float32_t face_mm,
                           bool b_low_side, app_side_plan_t * p_plan)
{
    float32_t forward_mm = 0.0f;
    float32_t lateral_mm = 0.0f;
    float32_t reach_mm   = 0.0f;
    float32_t hit_mm     = 0.0f;
    uint8_t   step       = 0u;
    uint8_t   index      = 0u;
    uint8_t   last_step  = 0u;
    bool      b_on_face  = true;
    bool      b_go_left  = false;
    bool      b_nearer   = false;

    for (step = 1u; (step <= APP_SWEEP_CENTRE_INDEX) && b_on_face; step++)
    {
        /* step <= 12 keeps the index in 0..24. */
        index     = b_low_side ? (uint8_t) (APP_SWEEP_CENTRE_INDEX - step)
                               : (uint8_t) (APP_SWEEP_CENTRE_INDEX + step);
        b_on_face = (BUDDY5_NO_ECHO != p_sweep_cm[index]);
        if (b_on_face)
        {
            app_sweep_point_mm(index, p_sweep_cm[index], &forward_mm,
                               &lateral_mm);
            b_nearer  = (forward_mm < (face_mm - APP_SWEEP_FACE_TOLERANCE_MM));
            b_on_face = !b_nearer &&
                        (forward_mm <= (face_mm + APP_SWEEP_FACE_TOLERANCE_MM));
        }
        if (b_on_face)
        {
            last_step = step;
            hit_mm    = (float32_t) p_sweep_cm[index] * APP_MM_PER_CM;
        }
    }
    p_plan->b_edge_found = !b_on_face;

    /* The true edge is APP_SWEEP_BEAM_STEPS inside the last echo; even a
     * thin post fills the centre beam, so at least one step out. */
    if (last_step > APP_SWEEP_BEAM_STEPS)
    {
        p_plan->edge_mm = hit_mm * g_sweep_sin[last_step - APP_SWEEP_BEAM_STEPS];
    }
    else
    {
        p_plan->edge_mm = face_mm * g_sweep_sin[1];
    }

    /* Going round on the left leaves the obstacle on the rover's right, so
     * the right side's reach matters, and the reverse. */
    b_go_left = app_scan_angle_is_left(b_low_side ? APP_SCAN_ANGLE_45_DEG
                                                  : APP_SCAN_ANGLE_135_DEG);
    reach_mm = b_go_left ? APP_ROVER_RIGHT_EXTENT_MM : APP_ROVER_LEFT_EXTENT_MM;
    p_plan->sidestep_mm = p_plan->edge_mm + reach_mm + APP_AVOID_SIDE_MARGIN_MM;
    if (p_plan->sidestep_mm < APP_SIDESTEP_MIN_MM)
    {
        p_plan->sidestep_mm = APP_SIDESTEP_MIN_MM;
    }
    /* Something nearer than the face beside it ends the obstacle (for the
     * width), but it is in the way on that side: do not go round there. */
    p_plan->b_usable = p_plan->b_edge_found && !b_nearer &&
                       (p_plan->sidestep_mm <= APP_SIDESTEP_MAX_MM);
}

/** @brief Print the measured face distance, both edges and the width. */
static void app_log_obstacle_size (float32_t               face_mm,
                                   const app_side_plan_t * p_low,
                                   const app_side_plan_t * p_high)
{
    bool b_low_is_left = app_scan_angle_is_left(APP_SCAN_ANGLE_45_DEG);
    const app_side_plan_t * p_left  = b_low_is_left ? p_low : p_high;
    const app_side_plan_t * p_right = b_low_is_left ? p_high : p_low;

    /* The HC-SR04 reads at most about 4 m, so every value is a few
     * thousand millimetres at most and is cast to the %u type. */
    tm_printf((UB *) "[avoid] obstacle %u mm ahead, width %s%u mm "
                     "(left edge %u mm%s, right edge %u mm%s)\n",
              (UINT) face_mm,
              (p_left->b_edge_found && p_right->b_edge_found) ? "" : ">= ",
              (UINT) (p_left->edge_mm + p_right->edge_mm),
              (UINT) p_left->edge_mm, p_left->b_edge_found ? "" : " not found",
              (UINT) p_right->edge_mm,
              p_right->b_edge_found ? "" : " not found");
    /* Report it on the dashboard as well (rover/debug). */
    robot_telemetry_set_obstacle_size(
        (uint32_t) face_mm, (uint32_t) (p_left->edge_mm + p_right->edge_mm),
        !(p_left->b_edge_found && p_right->b_edge_found));
    tm_printf((UB *) "[avoid] sidestep left %u mm%s, right %u mm%s\n",
              (UINT) p_left->sidestep_mm, p_left->b_usable ? "" : " (unusable)",
              (UINT) p_right->sidestep_mm,
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

/*** end of file ***/
