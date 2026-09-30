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

    buddy2_get_motion_state(&motion);
    buddy3_get_state(&ir);
    buddy4_get_state(&imu);
    buddy5_get_last(&sonar);
    robot_controller_get_drive(&drive_left, &drive_right);

    p_state = imu.mag_ok ? "online" : "compass_error";
    robot_format_x10(heading, imu.heading_deg_x10);
    robot_format_x10(pitch, imu.pitch_deg_x10);
    robot_format_x10(max_pitch, imu.max_pitch_deg_x10);

    (void)tm_sprintf((UB *)p_buffer,
        (UB *)"{\"robot_id\":\"%s\",\"ts_ms\":%u,\"state\":\"%s\","
              "\"left_ir\":%u,\"center_ir\":%u,\"right_ir\":%u,"
              "\"distance_cm\":%u,\"heading_deg\":%s,"
              "\"left_ticks\":%d,\"right_ticks\":%d,"
              "\"motor_left\":%u,\"motor_right\":%u,\"status\":\"%s\","
              "\"mode\":\"%s\",\"pitch_deg\":%s,\"max_pitch_deg\":%s}",
        ROBOT_MQTT_CLIENT_ID, (UINT)now_ms, p_state,
        (UINT)ir.raw[0], (UINT)ir.raw[1], (UINT)ir.raw[2],
        (UINT)sonar.distance_cm, heading,
        (INT)motion.left_ticks, (INT)motion.right_ticks,
        (UINT)drive_left, (UINT)drive_right, p_state,
        robot_controller_mode_name(), pitch, max_pitch);
}

void robot_telemetry_build_debug(char *p_buffer)
{
    buddy3_ir_state_t ir;

    buddy3_get_state(&ir);
    (void)tm_sprintf((UB *)p_buffer,
        (UB *)"{\"raw_left\":%u,\"raw_center\":%u,\"raw_right\":%u,"
              "\"norm_left\":%u,\"norm_center\":%u,\"norm_right\":%u,"
              "\"position\":%d,\"line\":%s,\"junction\":%s,"
              "\"barcode\":%s,\"command\":%u}",
        (UINT)ir.raw[0], (UINT)ir.raw[1], (UINT)ir.raw[2],
        (UINT)ir.normalized[0], (UINT)ir.normalized[1], (UINT)ir.normalized[2],
        (INT)ir.line_position,
        robot_bool(ir.line_seen), robot_bool(ir.junction_seen),
        robot_bool(ir.barcode_ready), (UINT)ir.barcode_command);
}

void robot_telemetry_build_heartbeat(char *p_buffer)
{
    (void)tm_sprintf((UB *)p_buffer,
                     (UB *)"{\"robot_id\":\"%s\",\"state\":\"online\"}",
                     ROBOT_MQTT_CLIENT_ID);
}
