/**
 * @file    robot_telemetry.c
 * @brief   Buddy 1: builds the JSON messages the dashboard shows.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy2_motion.h"
#include "buddy3_ir.h"
#include "buddy4_imu.h"
#include "buddy5_ultrasonic.h"
#include "comms.h"
#include "mqtt_config.h"
#include "robot_controller.h"
#include "robot_debug.h"
#include "robot_telemetry.h"

/* Write a value in tenths as "-12.3" (tm_sprintf has no %f). */
static void robot_format_x10(UB *p_out, int32_t value_x10)
{
    const char *p_sign = "";

    if (value_x10 < 0)
    {
        p_sign    = "-";
        value_x10 = -value_x10;
    }
    (void)tm_sprintf(p_out, (UB *)"%s%d.%d", p_sign,
                     (INT)(value_x10 / 10), (INT)(value_x10 % 10));
}

static const char *robot_bool(bool value)
{
    return value ? "true" : "false";
}

typedef struct
{
    uint32_t ultra_left_cm;
    uint32_t ultra_right_cm;
    bool     decision_left;
    bool     valid;
    uint8_t  status;
    int16_t  heading_change_x10;
    uint16_t heading_target_x10;
    int16_t  turn_left_pwm;
    int16_t  turn_right_pwm;
    uint16_t forward_progress_mm;
    uint16_t forward_target_mm;
} robot_avoidance_debug_t;

/* Updated by the sensor task and copied by the telemetry task. */
static robot_avoidance_debug_t g_avoidance_debug;

void robot_telemetry_set_avoidance_scan(uint32_t ultra_left_cm, uint32_t ultra_right_cm,
                                        bool decision_left)
{
    UINT interrupt_state;

    DI(interrupt_state);
    g_avoidance_debug.ultra_left_cm  = ultra_left_cm;
    g_avoidance_debug.ultra_right_cm = ultra_right_cm;
    g_avoidance_debug.decision_left  = decision_left;
    g_avoidance_debug.valid          = true;
    EI(interrupt_state);

    /* The telemetry module owns the human-readable decision summary as well
     * as the rover/debug JSON consumed by the dashboard. */
    ROBOT_ROUTINE_LOG((UB *)"[avoid telemetry] ultra_left: %u cm | ultra_right: %u cm | decision: %s\n",
              (UINT)ultra_left_cm, (UINT)ultra_right_cm,
              decision_left ? "left" : "right");
}

void robot_telemetry_set_avoidance_scan_values(uint32_t ultra_left_cm,
                                               uint32_t ultra_right_cm)
{
    UINT interrupt_state;

    DI(interrupt_state);
    g_avoidance_debug.ultra_left_cm  = ultra_left_cm;
    g_avoidance_debug.ultra_right_cm = ultra_right_cm;
    /* A blocked route has readings but deliberately has no left/right decision. */
    g_avoidance_debug.decision_left  = false;
    g_avoidance_debug.valid          = false;
    EI(interrupt_state);
}

void robot_telemetry_set_avoidance_status(uint8_t status, int16_t heading_change_x10,
                                          uint16_t heading_target_x10)
{
    UINT interrupt_state;

    DI(interrupt_state);
    g_avoidance_debug.status             = status;
    g_avoidance_debug.heading_change_x10 = heading_change_x10;
    g_avoidance_debug.heading_target_x10 = heading_target_x10;
    EI(interrupt_state);
}

void robot_telemetry_set_avoidance_turn_pwm(int16_t left_pwm, int16_t right_pwm)
{
    UINT interrupt_state;

    DI(interrupt_state);
    g_avoidance_debug.turn_left_pwm  = left_pwm;
    g_avoidance_debug.turn_right_pwm = right_pwm;
    EI(interrupt_state);
}

void robot_telemetry_set_avoidance_forward_progress(uint16_t progress_mm,
                                                     uint16_t target_mm)
{
    UINT interrupt_state;

    DI(interrupt_state);
    g_avoidance_debug.forward_progress_mm = progress_mm;
    g_avoidance_debug.forward_target_mm   = target_mm;
    EI(interrupt_state);
}

void robot_telemetry_clear_avoidance_scan(void)
{
    UINT interrupt_state;

    DI(interrupt_state);
    g_avoidance_debug.ultra_left_cm  = BUDDY5_NO_ECHO;
    g_avoidance_debug.ultra_right_cm = BUDDY5_NO_ECHO;
    g_avoidance_debug.decision_left  = false;
    g_avoidance_debug.valid          = false;
    g_avoidance_debug.status         = ROBOT_AVOID_STATUS_IDLE;
    g_avoidance_debug.heading_change_x10 = 0;
    g_avoidance_debug.heading_target_x10 = 0u;
    g_avoidance_debug.turn_left_pwm  = 0;
    g_avoidance_debug.turn_right_pwm = 0;
    g_avoidance_debug.forward_progress_mm = 0u;
    g_avoidance_debug.forward_target_mm   = 0u;
    EI(interrupt_state);
}

void robot_telemetry_build(char *p_buffer, uint32_t now_ms)
{
    buddy2_motion_state_t motion;
    buddy3_ir_state_t     ir;
    buddy4_imu_state_t    imu;
    buddy5_reading_t      sonar;
    uint32_t              drive_left;
    uint32_t              drive_right;
    const char           *p_state;
    UB                    heading[12];
    UB                    pitch[12];
    UB                    max_pitch[12];
    uint8_t               trim_left;
    uint8_t               trim_right;
    buddy3_calibration_t  line;
    INT                   line_base_pct;
    INT                   line_turn_pct;
    uint8_t               speed_pct;

    buddy2_get_motion_state(&motion);
    buddy2_get_trim(&trim_left, &trim_right);
    speed_pct = buddy2_get_speed_limit();
    buddy3_get_calibration(&line);
    /* Same percent-of-full-power units as the "line_speed" command. */
    line_base_pct = (INT)((((int32_t)line.base_pwm * 100) + (BUDDY2_PWM_MAX / 2)) / BUDDY2_PWM_MAX);
    line_turn_pct = (INT)(((line.proportional_gain * 100000.0f) / (float)BUDDY2_PWM_MAX) + 0.5f);
    buddy3_get_state(&ir);
    buddy4_get_state(&imu);
    buddy5_get_last(&sonar);
    robot_controller_get_drive(&drive_left, &drive_right);

    p_state = !imu.mag_ok ? "compass_error" : (!imu.accel_ok ? "imu_error" : "online");
    robot_format_x10(heading, imu.heading_deg_x10);
    robot_format_x10(pitch, imu.pitch_deg_x10);
    robot_format_x10(max_pitch, imu.max_pitch_deg_x10);

    (void)tm_sprintf((UB *)p_buffer,
        (UB *)"{\"robot_id\":\"%s\",\"ts_ms\":%u,\"state\":\"%s\","
              "\"left_ir\":%u,\"center_ir\":%u,\"right_ir\":%u,"
              "\"distance_cm\":%u,\"heading_deg\":%s,"
              "\"left_ticks\":%d,\"right_ticks\":%d,"
              "\"motor_left\":%u,\"motor_right\":%u,\"status\":\"%s\","
              "\"mode\":\"%s\",\"pitch_deg\":%s,\"max_pitch_deg\":%s,"
              "\"trim_left\":%u,\"trim_right\":%u,"
               "\"line_base\":%d,\"line_turn\":%d,\"speed\":%u,"
               "\"servo_angle_deg\":%u}",
        ROBOT_MQTT_CLIENT_ID, (UINT)now_ms, p_state,
        (UINT)ir.raw[0], (UINT)ir.raw[1], (UINT)ir.raw[2],
        (UINT)sonar.distance_cm, heading,
        (INT)motion.left_ticks, (INT)motion.right_ticks,
        (UINT)drive_left, (UINT)drive_right, p_state,
        robot_controller_mode_name(), pitch, max_pitch,
        (UINT)trim_left, (UINT)trim_right, line_base_pct, line_turn_pct,
        (UINT)speed_pct, (UINT)sonar.servo_angle_deg);
}

void robot_telemetry_build_debug(char *p_buffer)
{
    buddy3_ir_state_t      ir;
    robot_avoidance_debug_t avoidance;
    UINT                    interrupt_state;
    const char             *p_decision;
    const char             *p_avoid_status;

    buddy3_get_state(&ir);
    DI(interrupt_state);
    avoidance = g_avoidance_debug;
    EI(interrupt_state);
    p_decision = avoidance.valid ? (avoidance.decision_left ? "left" : "right") : "none";
    p_avoid_status = (ROBOT_AVOID_STATUS_SCANNING == avoidance.status) ? "scanning" :
                     (ROBOT_AVOID_STATUS_TURNING == avoidance.status) ? "turning" :
                     (ROBOT_AVOID_STATUS_COMPLETE == avoidance.status) ? "complete" :
                     (ROBOT_AVOID_STATUS_BLOCKED == avoidance.status) ? "blocked" :
                     (ROBOT_AVOID_STATUS_IMU_UNAVAILABLE == avoidance.status) ? "imu_unavailable" :
                     (ROBOT_AVOID_STATUS_IMU_LOST == avoidance.status) ? "imu_lost" :
                     (ROBOT_AVOID_STATUS_TIMEOUT == avoidance.status) ? "timeout" :
                     (ROBOT_AVOID_STATUS_CANCELLED == avoidance.status) ? "cancelled" :
                     (ROBOT_AVOID_STATUS_FORWARD == avoidance.status) ? "forward" :
                     (ROBOT_AVOID_STATUS_RECOVERING == avoidance.status) ? "recovering" :
                     (ROBOT_AVOID_STATUS_REVERSING == avoidance.status) ? "reversing" : "idle";
    (void)tm_sprintf((UB *)p_buffer,
        (UB *)"{\"raw_left\":%u,\"raw_center\":%u,\"raw_right\":%u,"
              "\"norm_left\":%u,\"norm_center\":%u,\"norm_right\":%u,"
              "\"position\":%d,\"line\":%s,\"junction\":%s,"
               "\"barcode\":%s,\"command\":%u,"
               "\"ultra_left\":%u,\"ultra_right\":%u,\"decision\":\"%s\","
               "\"avoid_status\":\"%s\",\"avoid_heading_change_x10\":%d,"
               "\"avoid_heading_target_x10\":%u,\"avoid_motor_left_pwm\":%d,"
               "\"avoid_motor_right_pwm\":%d,\"avoid_forward_mm\":%u,"
               "\"avoid_forward_target_mm\":%u}",
        (UINT)ir.raw[0], (UINT)ir.raw[1], (UINT)ir.raw[2],
        (UINT)ir.normalized[0], (UINT)ir.normalized[1], (UINT)ir.normalized[2],
        (INT)ir.line_position,
        robot_bool(ir.line_seen), robot_bool(ir.junction_seen),
        robot_bool(ir.barcode_ready), (UINT)ir.barcode_command,
        (UINT)avoidance.ultra_left_cm, (UINT)avoidance.ultra_right_cm, p_decision,
        p_avoid_status, (INT)avoidance.heading_change_x10,
        (UINT)avoidance.heading_target_x10, (INT)avoidance.turn_left_pwm,
        (INT)avoidance.turn_right_pwm, (UINT)avoidance.forward_progress_mm,
        (UINT)avoidance.forward_target_mm);
}

void robot_telemetry_build_heartbeat(char *p_buffer)
{
    (void)tm_sprintf((UB *)p_buffer,
                     (UB *)"{\"robot_id\":\"%s\",\"state\":\"online\"}",
                     ROBOT_MQTT_CLIENT_ID);
}
