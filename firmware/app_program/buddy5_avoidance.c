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
#define APP_ULTRASONIC_ANGLE_DEG       (90u)
#define APP_SERVO_SETTLE_MS             (500u)
#define APP_SONAR_PING_SPACING_MS       (100u)

/* Servo installation: 45 degrees looks to rover left; 135 to rover right. */
#define APP_OBSTACLE_STOP_CM            (15u)
#define APP_SCAN_ANGLE_45_DEG           (45u)
#define APP_SCAN_ANGLE_135_DEG          (135u)
#define APP_SCAN_45_IS_LEFT             (1u)

#define APP_AVOID_TURN_DEG              (30.0f)
#define APP_AVOID_TURN_TIMEOUT_MS       (5000u)
#define APP_AVOID_FORWARD_DISTANCE_MM   (280.0f)
#define APP_AVOID_FINAL_FORWARD_DISTANCE_MM (250.0f)

/* Automatic IMU heading feedback used during all three forward legs. */
#define APP_AVOID_FORWARD_BASE_PWM              (9000)
#define APP_AVOID_FORWARD_HEADING_DEADBAND_X10 (5)
#define APP_AVOID_FORWARD_HEADING_KP            (30)
#define APP_AVOID_FORWARD_MAX_CORRECTION_PWM    (2500)
#define APP_AVOID_FORWARD_TIMEOUT_MS            (12000u)


#define APP_AVOID_RECOVER_TOLERANCE_X10    (20)
#define APP_AVOID_RECOVER_TIMEOUT_MS        (5000u)
#define APP_AVOID_SIDE_SCAN_VALID_SAMPLES   (5u)
#define APP_AVOID_SIDE_SCAN_MAX_ATTEMPTS    (8u)
#define APP_AVOID_SIDE_SCAN_MIN_VALID       (3u)
#define APP_AVOID_MIN_CLEARANCE_CM           (20u)
#define APP_AVOID_RESULT_VISIBLE_MS          (1100u)

/* Forward arcs: a larger left outer PWM is required for a right arc. */
#define APP_AVOID_LEFT_TURN_OUTER_PWM  (9000)
#define APP_AVOID_RIGHT_TURN_OUTER_PWM (14000)
#define APP_AVOID_ARC_INNER_PWM        (4500)

/* ------------------------------------------------------------------ *
 *  Sensors: IMU, ultrasonic monitoring, and obstacle avoidance
 * ------------------------------------------------------------------ */

typedef enum
{
    /* Normal mode: sample only the forward-facing ultrasonic sensor. */
    APP_AVOID_WATCH_FORWARD = 0,
    /* Settle at 45 degrees, then collect several temporary left-side samples. */
    APP_AVOID_WAIT_45,
    /* Settle at 135 degrees, then collect several temporary right-side samples. */
    APP_AVOID_WAIT_135,
    /* Re-centre the servo before choosing and starting the forward arc. */
    APP_AVOID_WAIT_CENTER,
    /* Drive an outward or inward arc until the IMU heading change reaches the target. */
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

/** @brief Return true for a valid ultrasonic reading at or inside the stop distance. */
static bool app_sonar_is_close(uint32_t distance_cm)
{
    return (BUDDY5_NO_ECHO != distance_cm) && (distance_cm <= APP_OBSTACLE_STOP_CM);
}

/**
 * @brief Convert a servo scan angle to the rover's physical left/right side.
 *
 * APP_SCAN_45_IS_LEFT documents the servo installation and can be flipped if
 * the servo is mounted as a mirror image.
 */
static bool app_scan_angle_is_left(uint8_t angle_deg)
{
    if (APP_SCAN_ANGLE_45_DEG == angle_deg)
    {
        return (0u != APP_SCAN_45_IS_LEFT);
    }
    return (0u == APP_SCAN_45_IS_LEFT);
}

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

/** @brief Clear the temporary readings and error counters for one scan angle. */
static void app_side_scan_reset(app_side_scan_t *p_scan)
{
    p_scan->attempts            = 0u;
    p_scan->valid               = 0u;
    p_scan->timeout_rise        = 0u;
    p_scan->timeout_fall        = 0u;
    p_scan->high_before_trigger = 0u;
    p_scan->total_cm            = 0u;
}

/** @brief Return readable text for an ultrasonic failure reported by Buddy 5. */
static const char *app_ping_failure_name(buddy5_ping_result_t result)
{
    if (BUDDY5_PING_TIMEOUT_RISE == result)
    {
        return "timeout waiting for rise";
    }
    if (BUDDY5_PING_TIMEOUT_FALL == result)
    {
        return "timeout waiting for fall";
    }
    return "echo HIGH before trigger";
}

/**
 * @brief Take one temporary side-scan ping and report whether the scan is complete.
 *
 * A scan completes after five valid readings or eight total attempts. Invalid
 * readings are retried, but fewer than three valid readings reject that side.
 */
static bool app_side_scan_ping(uint8_t angle_deg, app_side_scan_t *p_scan,
                               uint32_t *p_average_cm)
{
    buddy5_ping_result_t result;
    uint32_t             distance_cm;

    distance_cm = buddy5_sample_with_result_cm(&result);
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
        tm_printf((UB *)"[avoid] scan %u deg (%s) attempt %u/%u failed: %s\n",
                  (UINT)angle_deg,
                  app_scan_angle_is_left(angle_deg) ? "left" : "right",
                  (UINT)p_scan->attempts, (UINT)APP_AVOID_SIDE_SCAN_MAX_ATTEMPTS,
                  app_ping_failure_name(result));
    }

    if ((p_scan->valid < APP_AVOID_SIDE_SCAN_VALID_SAMPLES) &&
        (p_scan->attempts < APP_AVOID_SIDE_SCAN_MAX_ATTEMPTS))
    {
        return false;
    }

    if (p_scan->valid >= APP_AVOID_SIDE_SCAN_MIN_VALID)
    {
        *p_average_cm = p_scan->total_cm / p_scan->valid;
        tm_printf((UB *)"[avoid] scan %u deg (%s): valid=%u/%u average=%u cm "
                  "rise_timeout=%u fall_timeout=%u high_before_trigger=%u\n",
                  (UINT)angle_deg,
                  app_scan_angle_is_left(angle_deg) ? "left" : "right",
                  (UINT)p_scan->valid, (UINT)p_scan->attempts, (UINT)*p_average_cm,
                  (UINT)p_scan->timeout_rise, (UINT)p_scan->timeout_fall,
                  (UINT)p_scan->high_before_trigger);
    }
    else
    {
        *p_average_cm = BUDDY5_NO_ECHO;
        tm_printf((UB *)"[avoid] scan %u deg (%s) rejected: valid=%u/%u "
                  "rise_timeout=%u fall_timeout=%u high_before_trigger=%u\n",
                  (UINT)angle_deg,
                  app_scan_angle_is_left(angle_deg) ? "left" : "right",
                  (UINT)p_scan->valid, (UINT)p_scan->attempts,
                  (UINT)p_scan->timeout_rise, (UINT)p_scan->timeout_fall,
                  (UINT)p_scan->high_before_trigger);
    }
    return true;
}

/**
 * @brief Copy side-scan values to rover/debug before choosing a route.
 *
 * This makes blocked readings visible without changing Buddy 5's saved
 * forward-facing distance, which remains the dashboard ultrasonic telemetry.
 */
static void app_publish_avoidance_scan_values(uint32_t scan_45_cm, uint32_t scan_135_cm)
{
    uint32_t ultra_left_cm;
    uint32_t ultra_right_cm;

    if (app_scan_angle_is_left(APP_SCAN_ANGLE_45_DEG))
    {
        ultra_left_cm  = scan_45_cm;
        ultra_right_cm = scan_135_cm;
    }
    else
    {
        ultra_left_cm  = scan_135_cm;
        ultra_right_cm = scan_45_cm;
    }

    robot_telemetry_set_avoidance_scan_values(ultra_left_cm, ultra_right_cm);
}

/** @brief Publish the selected left/right decision without formatting telemetry here. */
static void app_publish_avoidance_decision(uint32_t scan_45_cm, uint32_t scan_135_cm,
                                           bool turn_left)
{
    uint32_t ultra_left_cm;
    uint32_t ultra_right_cm;

    if (app_scan_angle_is_left(APP_SCAN_ANGLE_45_DEG))
    {
        ultra_left_cm  = scan_45_cm;
        ultra_right_cm = scan_135_cm;
    }
    else
    {
        ultra_left_cm  = scan_135_cm;
        ultra_right_cm = scan_45_cm;
    }

    robot_telemetry_set_avoidance_scan(ultra_left_cm, ultra_right_cm, turn_left);
}

/**
 * @brief Return the signed shortest heading change from start to current.
 *
 * Headings are stored in tenths of a degree (0..3599).  The wrap handling
 * means a transition from 359 degrees to 0 degrees is treated as +1 degree,
 * not a 359-degree turn.
 */
static int16_t app_heading_change_x10(int16_t start_heading_x10, int16_t current_heading_x10)
{
    int32_t change_x10 = (int32_t)current_heading_x10 - (int32_t)start_heading_x10;

    if (change_x10 > 1800)
    {
        change_x10 -= 3600;
    }
    else if (change_x10 < -1800)
    {
        change_x10 += 3600;
    }
    return (int16_t)change_x10;
}

/** @brief Return the non-negative magnitude of a signed heading change. */
static uint16_t app_abs_heading_change_x10(int16_t change_x10)
{
    return (change_x10 < 0) ? (uint16_t)(-(int32_t)change_x10) : (uint16_t)change_x10;
}

/**
 * @brief Record the current heading and begin a forward arc to the chosen side.
 *
 * A left arc uses a slower left wheel and faster right wheel; a right arc
 * swaps those commands.  Both values remain positive, so the rover moves
 * forward while turning rather than rotating on the spot.
 *
 * @return true if a valid compass heading was available and the arc started.
 */
static bool app_start_heading_arc(bool turn_left, int16_t *p_start_heading_x10)
{
    buddy4_imu_state_t imu;

    if (NULL == p_start_heading_x10)
    {
        return false;
    }
    buddy4_get_state(&imu);
    if (!imu.mag_ok)
    {
        return false;
    }

    *p_start_heading_x10 = imu.heading_deg_x10;
    if (turn_left)
    {
        buddy2_set_motor_pwm(APP_AVOID_ARC_INNER_PWM, APP_AVOID_LEFT_TURN_OUTER_PWM);
    }
    else
    {
        buddy2_set_motor_pwm(APP_AVOID_RIGHT_TURN_OUTER_PWM, APP_AVOID_ARC_INNER_PWM);
    }
    return true;
}

/** @brief Aim the servo at 45 degrees and enter the first side-scan wait state. */
static void app_start_side_scan(app_avoid_state_t *p_state)
{
    buddy5_set_servo_angle(APP_SCAN_ANGLE_45_DEG);
    *p_state = APP_AVOID_WAIT_45;
}

/**
 * @brief Safely leave avoidance mode.
 *
 * Faces the ultrasonic sensor forward again, tells the robot controller that
 * avoidance is complete, and resumes normal forward-distance monitoring.
 */
static void app_finish_avoidance(app_avoid_state_t *p_state)
{
    buddy5_set_servo_angle(APP_ULTRASONIC_ANGLE_DEG);
    robot_controller_finish_obstacle_avoidance();
    *p_state = APP_AVOID_WATCH_FORWARD;
}

/**
 * @brief Start one heading-held forward leg of the avoidance route.
 *
 * Both diagonal legs and the final straight leg need the same preparation:
 * save the encoder baseline and current IMU heading, command equal base PWM,
 * reset forward-progress telemetry, and enter the forward state.  The caller
 * supplies only what differs between legs: its target distance and type.
 */
static void app_start_forward_leg(buddy2_motion_state_t *p_start_motion,
                                  int16_t *p_start_heading_x10,
                                  float *p_target_mm,
                                  app_avoid_forward_leg_t *p_leg,
                                  uint32_t *p_state_started_ms,
                                  app_avoid_state_t *p_state,
                                  int16_t current_heading_x10,
                                  float target_mm,
                                  app_avoid_forward_leg_t leg,
                                  uint32_t now_ms)
{
    buddy2_get_motion_state(p_start_motion);
    *p_start_heading_x10 = current_heading_x10;
    *p_target_mm = target_mm;
    *p_leg = leg;

    buddy2_set_motor_pwm(APP_AVOID_FORWARD_BASE_PWM,
                          APP_AVOID_FORWARD_BASE_PWM);
    robot_telemetry_set_avoidance_turn_pwm(APP_AVOID_FORWARD_BASE_PWM,
                                            APP_AVOID_FORWARD_BASE_PWM);
    robot_telemetry_set_avoidance_forward_progress(0u, (uint16_t)target_mm);
    robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_FORWARD, 0, 0u);

    *p_state_started_ms = now_ms;
    *p_state = APP_AVOID_WAIT_FORWARD_DISTANCE;
}

/**
 * @brief Run Buddy 4/5 sensor updates and the obstacle-avoidance state machine.
 *
 * In normal operation it publishes only 90-degree forward ultrasonic readings.
 * The 45- and 135-degree samples use non-telemetry side-scan pings, so
 * they stay in temporary variables and never replace dashboard telemetry.
 */
void buddy5_avoidance_task(INT stacd, void *p_exinf)
{
    app_avoid_state_t state = APP_AVOID_WATCH_FORWARD;
    uint32_t          scan_45_cm = BUDDY5_NO_ECHO;
    uint32_t          scan_135_cm = BUDDY5_NO_ECHO;
    uint32_t          state_started_ms = 0u;
    uint32_t          side_scan_last_ping_ms = 0u;
    uint32_t          avoidance_finished_ms = 0u;
    int16_t           arc_start_heading_x10 = 0;
    int16_t           forward_start_heading_x10 = 0;
    float             forward_target_mm = APP_AVOID_FORWARD_DISTANCE_MM;
    bool              outward_turn_left = false;
    bool              turning_inward = false;
    app_avoid_forward_leg_t forward_leg = APP_AVOID_FORWARD_OUTWARD_DIAGONAL;
    bool              avoidance_result_pending_clear = false;
    app_side_scan_t   scan_45;
    app_side_scan_t   scan_135;
    buddy2_motion_state_t forward_start_motion;

    (void)stacd;
    (void)p_exinf;

    /* Normal monitoring starts with the ultrasonic sensor facing forward. */
    buddy5_set_servo_angle(APP_ULTRASONIC_ANGLE_DEG);
    (void)tk_dly_tsk(APP_SERVO_SETTLE_MS);

    while (1)
    {
        uint32_t now_ms = app_now_ms();

        /* IMU is sampled by its independent owner task. */

        if (avoidance_result_pending_clear &&
            ((now_ms - avoidance_finished_ms) >= APP_AVOID_RESULT_VISIBLE_MS))
        {
            robot_telemetry_clear_avoidance_scan();
            avoidance_result_pending_clear = false;
        }

        /* A dashboard command (including stop) cancels a scan safely. */
        if ((APP_AVOID_WATCH_FORWARD != state) &&
            !robot_controller_obstacle_avoidance_active())
        {
            robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_CANCELLED, 0,
                                                  (uint16_t)(APP_AVOID_TURN_DEG * 10.0f));
            avoidance_finished_ms = now_ms;
            avoidance_result_pending_clear = true;
            buddy5_set_servo_angle(APP_ULTRASONIC_ANGLE_DEG);
            state = APP_AVOID_WATCH_FORWARD;
        }

        switch (state)
        {
            case APP_AVOID_WATCH_FORWARD:
            {
                uint32_t front_cm = buddy5_measure_cm(now_ms);
                /*
                Step 1:
                app_sonar_is_close: check if the distance from the obstacle = 15cm
                robot_controller_begin_obstacle_avoidance: prep robot for avoidance, stop follow line
                */
                if (app_sonar_is_close(front_cm) &&
                    robot_controller_begin_obstacle_avoidance())
                {
                    tm_printf((UB *)"[avoid] obstacle %u cm: stopped; scanning sides\n",
                              (UINT)front_cm);
                    robot_telemetry_clear_avoidance_scan();
                    robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_SCANNING, 0, 0u);
                    avoidance_result_pending_clear = false;
                    outward_turn_left = false;
                    turning_inward = false;
                    forward_leg = APP_AVOID_FORWARD_OUTWARD_DIAGONAL;
                    scan_45_cm = BUDDY5_NO_ECHO;
                    scan_135_cm = BUDDY5_NO_ECHO;
                    app_side_scan_reset(&scan_45);
                    app_side_scan_reset(&scan_135);
                    app_start_side_scan(&state);
                    state_started_ms = now_ms;
                    side_scan_last_ping_ms = now_ms;
                }
                break;
            }

            case APP_AVOID_WAIT_45:
                if (((now_ms - state_started_ms) >= APP_SERVO_SETTLE_MS) &&
                    ((now_ms - side_scan_last_ping_ms) >= APP_SONAR_PING_SPACING_MS))
                {
                    side_scan_last_ping_ms = now_ms;
                    /*
                        Step 2: scan 45 degrees ultrasonic distance
                    */
                    if (app_side_scan_ping(APP_SCAN_ANGLE_45_DEG, &scan_45, &scan_45_cm))
                    {
                        /*
                        Step 3: set servo to 135 degree and make sure temporary variable is empty
                        */
                        buddy5_set_servo_angle(APP_SCAN_ANGLE_135_DEG);
                        app_side_scan_reset(&scan_135);
                        state_started_ms = now_ms;
                        side_scan_last_ping_ms = now_ms;
                        state = APP_AVOID_WAIT_135;
                    }
                }
                break;
            /* Step 3: Scan 135 degree ultrasonic distance*/
            case APP_AVOID_WAIT_135:
                if (((now_ms - state_started_ms) >= APP_SERVO_SETTLE_MS) &&
                    ((now_ms - side_scan_last_ping_ms) >= APP_SONAR_PING_SPACING_MS))
                {
                    side_scan_last_ping_ms = now_ms;
                    if (app_side_scan_ping(APP_SCAN_ANGLE_135_DEG, &scan_135, &scan_135_cm))
                    {
                        buddy5_set_servo_angle(APP_ULTRASONIC_ANGLE_DEG);
                        state_started_ms = now_ms;
                        state = APP_AVOID_WAIT_CENTER;
                    }
                }
                break;
            /* Step 4: Decided to turn left or right */
            case APP_AVOID_WAIT_CENTER:
                if ((now_ms - state_started_ms) >= APP_SERVO_SETTLE_MS)
                {
                    uint8_t chosen_angle;
                    bool    started;
                    bool    turn_left;
                    bool    scan_45_usable;
                    bool    scan_135_usable;

                    scan_45_usable = (BUDDY5_NO_ECHO != scan_45_cm) &&
                                     (scan_45_cm >= APP_AVOID_MIN_CLEARANCE_CM);
                    scan_135_usable = (BUDDY5_NO_ECHO != scan_135_cm) &&
                                      (scan_135_cm >= APP_AVOID_MIN_CLEARANCE_CM);
                    /* Publish both physical-side readings before deciding.  This
                     * keeps a blocked route visible in the dashboard monitor. */
                    app_publish_avoidance_scan_values(scan_45_cm, scan_135_cm);
                    if (!scan_45_usable && !scan_135_usable)
                    {
                        tm_printf((UB *)"[avoid] no usable route: 45=%u cm 135=%u cm "
                                  "(need at least %u cm); remaining stopped\n",
                                  (UINT)scan_45_cm, (UINT)scan_135_cm,
                                  (UINT)APP_AVOID_MIN_CLEARANCE_CM);
                        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_BLOCKED, 0, 0u);
                        avoidance_finished_ms = now_ms;
                        avoidance_result_pending_clear = true;
                        app_finish_avoidance(&state);
                        break;
                    }
                    /* Invalid or tight scans are not open space and cannot win. */
                    if (scan_45_usable &&
                        (!scan_135_usable || (scan_45_cm >= scan_135_cm)))
                    {
                        chosen_angle = APP_SCAN_ANGLE_45_DEG;
                    }
                    else
                    {
                        chosen_angle = APP_SCAN_ANGLE_135_DEG;
                    }
                    turn_left = app_scan_angle_is_left(chosen_angle);
                    outward_turn_left = turn_left;
                    tm_printf((UB *)"[avoid] choose %u deg (%s): %u cm vs %u cm\n",
                              (UINT)chosen_angle, turn_left ? "left" : "right",
                              (UINT)scan_45_cm, (UINT)scan_135_cm);
                    app_publish_avoidance_decision(scan_45_cm, scan_135_cm, turn_left);
                    /* The selected direction is now in telemetry; scan values
                     * are no longer needed by the turning state. */
                    scan_45_cm = BUDDY5_NO_ECHO;
                    scan_135_cm = BUDDY5_NO_ECHO;
                    app_side_scan_reset(&scan_45);
                    app_side_scan_reset(&scan_135);
                    started = app_start_heading_arc(turn_left, &arc_start_heading_x10);
                    if (!started)
                    {
                        tm_printf((UB *)"[avoid] IMU heading unavailable; remaining stopped\n");
                        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_IMU_UNAVAILABLE,
                                                              0,
                                                              (uint16_t)(APP_AVOID_TURN_DEG * 10.0f));
                        avoidance_finished_ms = now_ms;
                        avoidance_result_pending_clear = true;
                        app_finish_avoidance(&state);
                    }
                    else
                    {
                        robot_telemetry_set_avoidance_turn_pwm(
                            turn_left ? APP_AVOID_ARC_INNER_PWM : APP_AVOID_RIGHT_TURN_OUTER_PWM,
                            turn_left ? APP_AVOID_LEFT_TURN_OUTER_PWM : APP_AVOID_ARC_INNER_PWM);
                        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_TURNING, 0,
                                                              (uint16_t)(APP_AVOID_TURN_DEG * 10.0f));
                        tm_printf((UB *)"[avoid] forward arc from %u.%u deg; target change %u.%u deg\n",
                                  (UINT)(arc_start_heading_x10 / 10),
                                  (UINT)(arc_start_heading_x10 % 10),
                                  (UINT)((uint32_t)(APP_AVOID_TURN_DEG * 10.0f) / 10u),
                                  (UINT)((uint32_t)(APP_AVOID_TURN_DEG * 10.0f) % 10u));
                        state_started_ms = now_ms;
                        state = APP_AVOID_WAIT_TURN_ARC;
                    }
                }
                break;
            /* Step 5: Turn out, based on the imu direction +/- 30 degrees */
            case APP_AVOID_WAIT_TURN_ARC:
            {
                buddy4_imu_state_t imu;
                int16_t            heading_change_x10;

                /* The final turn can reach the line before the diagonal starts.
                 * Buddy 3 latches its 10 ms readings, so a brief black-line
                 * observation is not missed by this 100 ms task.  The line has
                 * priority over all remaining avoidance movement. */
                if (turning_inward && buddy3_take_line_seen_event())
                {
                    buddy2_stop();
                    tm_printf((UB *)"[avoid] line captured during turn-in; "
                                       "handing back to line following\n");
                    robot_telemetry_set_avoidance_turn_pwm(0, 0);
                    robot_telemetry_set_avoidance_status(
                        ROBOT_AVOID_STATUS_COMPLETE, 0, 0u);
                    avoidance_finished_ms = now_ms;
                    avoidance_result_pending_clear = true;
                    app_finish_avoidance(&state);
                    break;
                }

                buddy4_get_state(&imu);
                if (!imu.mag_ok)
                {
                    buddy2_stop();
                    tm_printf((UB *)"[avoid] IMU heading lost during arc; remaining stopped\n");
                    robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_IMU_LOST, 0,
                                                          (uint16_t)(APP_AVOID_TURN_DEG * 10.0f));
                    avoidance_finished_ms = now_ms;
                    avoidance_result_pending_clear = true;
                    app_finish_avoidance(&state);
                }
                else
                {
                    heading_change_x10 = app_heading_change_x10(arc_start_heading_x10,
                                                                 imu.heading_deg_x10);
                    robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_TURNING,
                                                          heading_change_x10,
                                                          (uint16_t)(APP_AVOID_TURN_DEG * 10.0f));
                    if ((float)app_abs_heading_change_x10(heading_change_x10) >=
                        (APP_AVOID_TURN_DEG * 10.0f))
                    {
                        tm_printf((UB *)"[avoid] %s arc changed heading by %d.%d deg; stopped\n",
                                  turning_inward ? "turn-in" : "turn-out",
                                  (INT)(heading_change_x10 / 10),
                                  (INT)app_abs_heading_change_x10((int16_t)(heading_change_x10 % 10)));
                        /* Wheel encoders stop the leg after the configured distance.
                         * Raw PWM plus heading feedback still keeps the rover straight. */
                        tm_printf((UB *)"[avoid] %s diagonal forward %u mm\n",
                                  turning_inward ? "inward" : "outward",
                                  (UINT)APP_AVOID_FORWARD_DISTANCE_MM);
                        app_start_forward_leg(
                            &forward_start_motion, &forward_start_heading_x10,
                            &forward_target_mm, &forward_leg, &state_started_ms, &state,
                            imu.heading_deg_x10, APP_AVOID_FORWARD_DISTANCE_MM,
                            turning_inward ? APP_AVOID_FORWARD_INWARD_DIAGONAL :
                                            APP_AVOID_FORWARD_OUTWARD_DIAGONAL,
                            now_ms);
                    }
                    else if ((now_ms - state_started_ms) >= APP_AVOID_TURN_TIMEOUT_MS)
                    {
                        buddy2_stop();
                        tm_printf((UB *)"[avoid] IMU arc timeout; remaining stopped\n");
                        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_TIMEOUT,
                                                              heading_change_x10,
                                                              (uint16_t)(APP_AVOID_TURN_DEG * 10.0f));
                        avoidance_finished_ms = now_ms;
                        avoidance_result_pending_clear = true;
                        app_finish_avoidance(&state);
                    }
                }
                break;
            }
            /* Step 6: Move forward */
            case APP_AVOID_WAIT_FORWARD_DISTANCE:
            {
                buddy4_imu_state_t    imu;
                buddy2_motion_state_t forward_motion;
                uint32_t              front_cm;
                float                 progress_mm;
                float                 left_moved_mm;
                float                 right_moved_mm;
                int16_t               heading_error_x10;
                int32_t               correction_pwm;
                int16_t               left_pwm;
                int16_t               right_pwm;
                bool                  inward_turn_left;

                buddy4_get_state(&imu);
                if (!imu.mag_ok)
                {
                    buddy2_stop();
                    tm_printf((UB *)"[avoid] IMU heading lost during forward move; remaining stopped\n");
                    robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_IMU_LOST, 0, 0u);
                    avoidance_finished_ms = now_ms;
                    avoidance_result_pending_clear = true;
                    app_finish_avoidance(&state);
                    break;
                }

                /* The final inward diagonal has reached the line.  This event
                 * is latched by Buddy 3's 10 ms sampling task. */
                if ((APP_AVOID_FORWARD_INWARD_DIAGONAL == forward_leg) &&
                    buddy3_take_line_seen_event())
                {
                    buddy2_stop();
                    tm_printf((UB *)"[avoid] line captured during inward diagonal; "
                                       "handing back to line following\n");
                    robot_telemetry_set_avoidance_turn_pwm(0, 0);
                    robot_telemetry_set_avoidance_status(
                        ROBOT_AVOID_STATUS_COMPLETE, 0, 0u);
                    avoidance_finished_ms = now_ms;
                    avoidance_result_pending_clear = true;
                    app_finish_avoidance(&state);
                    break;
                }

                /* The servo was re-centred before the outward arc.  Take a
                 * fresh forward-facing ping on every 100 ms sensor cycle so a
                 * new obstacle stops either forward leg immediately. */
                front_cm = buddy5_measure_cm(now_ms);
                if (app_sonar_is_close(front_cm))
                {
                    buddy2_stop();
                    tm_printf((UB *)"[avoid] new front obstacle %u cm during forward; "
                                       "stopped and will rescan\n",
                              (UINT)front_cm);
                    robot_telemetry_set_avoidance_turn_pwm(0, 0);
                    robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_BLOCKED, 0, 0u);
                    avoidance_finished_ms = now_ms;
                    avoidance_result_pending_clear = true;
                    app_finish_avoidance(&state);
                    break;
                }

                buddy2_get_motion_state(&forward_motion);
                /* Progress = average of the wheels whose encoder works
                 * (Buddy 2 distance, already calibrated per wheel). */
                left_moved_mm  = forward_motion.left_distance_mm  - forward_start_motion.left_distance_mm;
                right_moved_mm = forward_motion.right_distance_mm - forward_start_motion.right_distance_mm;
                left_moved_mm  = (left_moved_mm  < 0.0f) ? -left_moved_mm  : left_moved_mm;
                right_moved_mm = (right_moved_mm < 0.0f) ? -right_moved_mm : right_moved_mm;
                if (forward_motion.left_encoder_fault && !forward_motion.right_encoder_fault)
                {
                    progress_mm = right_moved_mm;
                }
                else if (forward_motion.right_encoder_fault && !forward_motion.left_encoder_fault)
                {
                    progress_mm = left_moved_mm;
                }
                else
                {
                    progress_mm = (left_moved_mm + right_moved_mm) * 0.5f;
                }
                robot_telemetry_set_avoidance_forward_progress(
                    (uint16_t)progress_mm, (uint16_t)forward_target_mm);
                heading_error_x10 = app_heading_change_x10(forward_start_heading_x10,
                                                            imu.heading_deg_x10);

                if (progress_mm >= forward_target_mm)
                {
                    buddy2_stop();
                    robot_telemetry_set_avoidance_turn_pwm(0, 0);
                    if (APP_AVOID_FORWARD_INWARD_DIAGONAL == forward_leg)
                    {
                        tm_printf((UB *)"[avoid] inward diagonal forward %u mm complete\n",
                                  (UINT)forward_target_mm);
                        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_COMPLETE,
                                                              heading_error_x10, 0u);
                        avoidance_finished_ms = now_ms;
                        avoidance_result_pending_clear = true;
                        app_finish_avoidance(&state);
                    }
                    else if (APP_AVOID_FORWARD_OUTWARD_DIAGONAL == forward_leg)
                    {
                        tm_printf((UB *)"[avoid] heading-held forward %u mm complete; "
                                           "recovering start heading\n",
                                  (UINT)forward_target_mm);
                        robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_RECOVERING,
                                                              heading_error_x10,
                                                              APP_AVOID_RECOVER_TOLERANCE_X10);
                        state_started_ms = now_ms;
                        state = APP_AVOID_WAIT_RECOVER_HEADING;
                    }
                    else
                    {
                        /* The 25 cm straight section is complete.  Turn in the
                         * opposite direction of the original outward arc, then
                         * take the final diagonal leg back toward the route. */
                        inward_turn_left = !outward_turn_left;
                        if (!app_start_heading_arc(inward_turn_left, &arc_start_heading_x10))
                        {
                            tm_printf((UB *)"[avoid] IMU unavailable for turn-in arc; remaining stopped\n");
                            robot_telemetry_set_avoidance_status(
                                ROBOT_AVOID_STATUS_IMU_UNAVAILABLE, 0,
                                (uint16_t)(APP_AVOID_TURN_DEG * 10.0f));
                            avoidance_finished_ms = now_ms;
                            avoidance_result_pending_clear = true;
                            app_finish_avoidance(&state);
                        }
                        else
                        {
                            turning_inward = true;
                            /* Only black seen during the final return-to-line
                             * phase may finish avoidance. */
                            buddy3_clear_line_seen_event();
                            robot_telemetry_set_avoidance_turn_pwm(
                                inward_turn_left ? APP_AVOID_ARC_INNER_PWM :
                                                    APP_AVOID_RIGHT_TURN_OUTER_PWM,
                                inward_turn_left ? APP_AVOID_LEFT_TURN_OUTER_PWM :
                                                    APP_AVOID_ARC_INNER_PWM);
                            robot_telemetry_set_avoidance_status(
                                ROBOT_AVOID_STATUS_TURNING, 0,
                                (uint16_t)(APP_AVOID_TURN_DEG * 10.0f));
                            tm_printf((UB *)"[avoid] straight forward complete; turn-in %s arc, "
                                               "target change %u.%u deg\n",
                                      inward_turn_left ? "left" : "right",
                                      (UINT)((uint32_t)(APP_AVOID_TURN_DEG * 10.0f) / 10u),
                                      (UINT)((uint32_t)(APP_AVOID_TURN_DEG * 10.0f) % 10u));
                            state_started_ms = now_ms;
                            state = APP_AVOID_WAIT_TURN_ARC;
                        }
                    }
                }
                else if ((now_ms - state_started_ms) >= APP_AVOID_FORWARD_TIMEOUT_MS)
                {
                    buddy2_stop();
                    tm_printf((UB *)"[avoid] heading-held forward timed out at %u/%u mm; remaining stopped\n",
                              (UINT)progress_mm, (UINT)forward_target_mm);
                    robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_TIMEOUT,
                                                          heading_error_x10, 0u);
                    avoidance_finished_ms = now_ms;
                    avoidance_result_pending_clear = true;
                    app_finish_avoidance(&state);
                }
                else
                {
                    correction_pwm = 0;
                    if (heading_error_x10 > APP_AVOID_FORWARD_HEADING_DEADBAND_X10)
                    {
                        correction_pwm = (int32_t)heading_error_x10 * APP_AVOID_FORWARD_HEADING_KP;
                    }
                    else if (heading_error_x10 < -APP_AVOID_FORWARD_HEADING_DEADBAND_X10)
                    {
                        correction_pwm = (int32_t)heading_error_x10 * APP_AVOID_FORWARD_HEADING_KP;
                    }
                    if (correction_pwm > APP_AVOID_FORWARD_MAX_CORRECTION_PWM)
                    {
                        correction_pwm = APP_AVOID_FORWARD_MAX_CORRECTION_PWM;
                    }
                    else if (correction_pwm < -APP_AVOID_FORWARD_MAX_CORRECTION_PWM)
                    {
                        correction_pwm = -APP_AVOID_FORWARD_MAX_CORRECTION_PWM;
                    }

                    /* Positive heading error means an unintended left arc:
                     * speed up the left wheel and slow the right to correct right. */
                    left_pwm  = (int16_t)(APP_AVOID_FORWARD_BASE_PWM + correction_pwm);
                    right_pwm = (int16_t)(APP_AVOID_FORWARD_BASE_PWM - correction_pwm);
                    buddy2_set_motor_pwm(left_pwm, right_pwm);
                    robot_telemetry_set_avoidance_turn_pwm(left_pwm, right_pwm);
                    robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_FORWARD,
                                                          heading_error_x10, 0u);
                }
                break;
            }

            case APP_AVOID_WAIT_RECOVER_HEADING:
            {
                buddy4_imu_state_t imu;
                int16_t            heading_error_x10;
                int16_t            left_pwm;
                int16_t            right_pwm;

                buddy4_get_state(&imu);
                if (!imu.mag_ok)
                {
                    buddy2_stop();
                    tm_printf((UB *)"[avoid] IMU heading lost during recovery; remaining stopped\n");
                    robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_IMU_LOST, 0,
                                                          APP_AVOID_RECOVER_TOLERANCE_X10);
                    avoidance_finished_ms = now_ms;
                    avoidance_result_pending_clear = true;
                    app_finish_avoidance(&state);
                    break;
                }

                heading_error_x10 = app_heading_change_x10(arc_start_heading_x10,
                                                            imu.heading_deg_x10);
                if (app_abs_heading_change_x10(heading_error_x10) <=
                    APP_AVOID_RECOVER_TOLERANCE_X10)
                {
                    buddy2_stop();
                    tm_printf((UB *)"[avoid] recovered start heading (error %d.%d deg); "
                                       "starting final %u mm forward\n",
                              (INT)(heading_error_x10 / 10),
                               (INT)app_abs_heading_change_x10((int16_t)(heading_error_x10 % 10)),
                               (UINT)APP_AVOID_FINAL_FORWARD_DISTANCE_MM);
                    app_start_forward_leg(
                        &forward_start_motion, &forward_start_heading_x10,
                        &forward_target_mm, &forward_leg, &state_started_ms, &state,
                        imu.heading_deg_x10, APP_AVOID_FINAL_FORWARD_DISTANCE_MM,
                        APP_AVOID_FORWARD_STRAIGHT, now_ms);
                }
                else if ((now_ms - state_started_ms) >= APP_AVOID_RECOVER_TIMEOUT_MS)
                {
                    buddy2_stop();
                    tm_printf((UB *)"[avoid] heading recovery timed out at %d.%d deg; remaining stopped\n",
                              (INT)(heading_error_x10 / 10),
                              (INT)app_abs_heading_change_x10((int16_t)(heading_error_x10 % 10)));
                    robot_telemetry_set_avoidance_turn_pwm(0, 0);
                    robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_TIMEOUT,
                                                          heading_error_x10,
                                                          APP_AVOID_RECOVER_TOLERANCE_X10);
                    avoidance_finished_ms = now_ms;
                    avoidance_result_pending_clear = true;
                    app_finish_avoidance(&state);
                }
                else
                {
                    /* A positive error is still left of the original heading,
                     * so curve right; a negative error curves left. */
                    if (heading_error_x10 > 0)
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
                    robot_telemetry_set_avoidance_status(ROBOT_AVOID_STATUS_RECOVERING,
                                                          heading_error_x10,
                                                          APP_AVOID_RECOVER_TOLERANCE_X10);
                }
                break;
            }

            default:
                app_finish_avoidance(&state);
                break;
        }
        (void)tk_dly_tsk(BUDDY5_AVOIDANCE_PERIOD_MS);
    }
}


