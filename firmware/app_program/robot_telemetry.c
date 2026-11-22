/**
 * @file    robot_telemetry.c
 * @brief   Buddy 1: builds the packed binary messages the dashboard shows.
 *
 * Collects the latest values from each buddy's module, then hands them to
 * robot_wire.c, which packs them at their bit widths.  Values stay in their
 * native types until that point (pack at the boundary, not everywhere).
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy2_motion.h"
#include "buddy3_ir.h"
#include "buddy4_imu.h"
#include "buddy5_ultrasonic.h"
#include "pin_config.h"
#include "robot_controller.h"
#include "robot_debug.h"
#include "robot_fault.h"
#include "robot_telemetry.h"
#include "robot_wire.h"

/* line_speed percentages: x100 for percent, rounded half up. */
#define PERCENT            (100)
#define TELEMETRY_MS_PER_S (1000u)
#define TELEMETRY_IR_COUNT (3u)

typedef struct
{
    uint32_t ultra_left_cm;
    uint32_t ultra_right_cm;
    bool     b_decision_left;
    bool     b_valid;
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

static uint16_t telemetry_u16 (uint32_t value);
static uint8_t  telemetry_percent (int32_t value);
static uint8_t  telemetry_sensor_state (const buddy4_imu_state_t * p_imu);

void robot_telemetry_set_avoidance_scan (uint32_t ultra_left_cm,
                                         uint32_t ultra_right_cm,
                                         bool     decision_left)
{
    UINT interrupt_state = 0u;

    DI(interrupt_state);
    g_avoidance_debug.ultra_left_cm   = ultra_left_cm;
    g_avoidance_debug.ultra_right_cm  = ultra_right_cm;
    g_avoidance_debug.b_decision_left = decision_left;
    g_avoidance_debug.b_valid         = true;
    EI(interrupt_state);

    /* The telemetry module owns the human-readable decision summary as well
     * as the rover/debug message consumed by the dashboard. */
    ROBOT_ROUTINE_LOG((UB *) "[avoid telemetry] ultra_left: %u cm | "
                             "ultra_right: %u cm | decision: %s\n",
                      (UINT) ultra_left_cm, (UINT) ultra_right_cm,
                      decision_left ? "left" : "right");
}

void robot_telemetry_set_avoidance_scan_values (uint32_t ultra_left_cm,
                                                uint32_t ultra_right_cm)
{
    UINT interrupt_state = 0u;

    DI(interrupt_state);
    g_avoidance_debug.ultra_left_cm  = ultra_left_cm;
    g_avoidance_debug.ultra_right_cm = ultra_right_cm;
    /* A blocked route has readings but deliberately no left/right decision. */
    g_avoidance_debug.b_decision_left = false;
    g_avoidance_debug.b_valid         = false;
    EI(interrupt_state);
}

void robot_telemetry_set_avoidance_status (uint8_t  status,
                                           int16_t  heading_change_x10,
                                           uint16_t heading_target_x10)
{
    UINT interrupt_state = 0u;

    DI(interrupt_state);
    g_avoidance_debug.status             = status;
    g_avoidance_debug.heading_change_x10 = heading_change_x10;
    g_avoidance_debug.heading_target_x10 = heading_target_x10;
    EI(interrupt_state);
}

void robot_telemetry_set_avoidance_turn_pwm (int16_t left_pwm,
                                             int16_t right_pwm)
{
    UINT interrupt_state = 0u;

    DI(interrupt_state);
    g_avoidance_debug.turn_left_pwm  = left_pwm;
    g_avoidance_debug.turn_right_pwm = right_pwm;
    EI(interrupt_state);
}

void robot_telemetry_set_avoidance_forward_progress (uint16_t progress_mm,
                                                     uint16_t target_mm)
{
    UINT interrupt_state = 0u;

    DI(interrupt_state);
    g_avoidance_debug.forward_progress_mm = progress_mm;
    g_avoidance_debug.forward_target_mm   = target_mm;
    EI(interrupt_state);
}

void robot_telemetry_clear_avoidance_scan (void)
{
    UINT interrupt_state = 0u;

    DI(interrupt_state);
    g_avoidance_debug.ultra_left_cm       = BUDDY5_NO_ECHO;
    g_avoidance_debug.ultra_right_cm      = BUDDY5_NO_ECHO;
    g_avoidance_debug.b_decision_left     = false;
    g_avoidance_debug.b_valid             = false;
    g_avoidance_debug.status              = ROBOT_AVOID_STATUS_IDLE;
    g_avoidance_debug.heading_change_x10  = 0;
    g_avoidance_debug.heading_target_x10  = 0u;
    g_avoidance_debug.turn_left_pwm       = 0;
    g_avoidance_debug.turn_right_pwm      = 0;
    g_avoidance_debug.forward_progress_mm = 0u;
    g_avoidance_debug.forward_target_mm   = 0u;
    EI(interrupt_state);
}

uint32_t robot_telemetry_build (uint8_t * p_buffer, uint32_t capacity,
                                uint32_t now_ms)
{
    robot_wire_telemetry_t values      = {0};
    buddy2_motion_state_t  motion      = {0};
    buddy3_ir_state_t      ir_state    = {0};
    buddy4_imu_state_t     imu         = {0};
    buddy5_reading_t       sonar       = {0};
    buddy3_calibration_t   line        = {0};
    uint32_t               drive_left  = 0u;
    uint32_t               drive_right = 0u;
    uint32_t               index       = 0u;

    buddy2_get_motion_state(&motion);
    buddy2_get_trim(&values.trim_left_pct, &values.trim_right_pct);
    values.speed_pct = buddy2_get_speed_limit();
    buddy3_get_calibration(&line);
    buddy3_get_state(&ir_state);
    buddy4_get_state(&imu);
    buddy5_get_last(&sonar);
    robot_controller_get_drive(&drive_left, &drive_right);

    values.uptime_s     = now_ms / TELEMETRY_MS_PER_S;
    values.mode         = robot_controller_get_mode();
    values.sensor_state = telemetry_sensor_state(&imu);
    values.faults       = robot_fault_get();
    for (index = 0u; index < TELEMETRY_IR_COUNT; index++)
    {
        values.ir_raw[index] = ir_state.raw[index];
    }
    values.distance_cm     = telemetry_u16(sonar.distance_cm);
    values.servo_angle_deg = sonar.servo_angle_deg;
    /* The heading is 0..3599, never negative. */
    values.heading_x10 =
        (imu.heading_deg_x10 > 0) ? (uint16_t) imu.heading_deg_x10 : 0u;
    values.pitch_x10     = imu.pitch_deg_x10;
    values.max_pitch_x10 = imu.max_pitch_deg_x10;
    /* Two's-complement bit patterns: the low 16 bits are what is sent, and
     * the dashboard unwraps them into a running count. */
    values.left_ticks  = (uint32_t) motion.left_ticks;
    values.right_ticks = (uint32_t) motion.right_ticks;
    values.motor_left  = telemetry_u16(drive_left);
    values.motor_right = telemetry_u16(drive_right);
    /* Same percent-of-full-power units as the "line_speed" command. */
    values.line_base_pct = telemetry_percent(
        (((int32_t) line.base_pwm * PERCENT) + (BUDDY2_PWM_MAX / 2)) /
        BUDDY2_PWM_MAX);
    values.line_turn_pct =
        telemetry_percent((int32_t) (((line.proportional_gain * 100000.0f) /
                                      (float32_t) BUDDY2_PWM_MAX) +
                                     0.5f));

    return robot_wire_encode_telemetry(&values, p_buffer, capacity);
}

uint32_t robot_telemetry_build_debug (uint8_t * p_buffer, uint32_t capacity)
{
    robot_wire_debug_t      values          = {0};
    buddy3_ir_state_t       ir_state        = {0};
    robot_avoidance_debug_t avoidance       = {0};
    UINT                    interrupt_state = 0u;
    uint32_t                index           = 0u;

    buddy3_get_state(&ir_state);
    DI(interrupt_state);
    avoidance = g_avoidance_debug;
    EI(interrupt_state);

    for (index = 0u; index < TELEMETRY_IR_COUNT; index++)
    {
        values.ir_raw[index]  = ir_state.raw[index];
        values.ir_norm[index] = ir_state.normalized[index];
    }
    values.line_position   = ir_state.line_position;
    values.b_line          = ir_state.line_seen;
    values.b_junction      = ir_state.junction_seen;
    values.b_barcode       = ir_state.barcode_ready;
    values.barcode_command = ir_state.barcode_command;
    values.ultra_left_cm   = telemetry_u16(avoidance.ultra_left_cm);
    values.ultra_right_cm  = telemetry_u16(avoidance.ultra_right_cm);
    if (!avoidance.b_valid)
    {
        values.decision = ROBOT_WIRE_DECISION_NONE;
    }
    else if (avoidance.b_decision_left)
    {
        values.decision = ROBOT_WIRE_DECISION_LEFT;
    }
    else
    {
        values.decision = ROBOT_WIRE_DECISION_RIGHT;
    }
    values.avoid_status       = avoidance.status;
    values.heading_change_x10 = avoidance.heading_change_x10;
    values.heading_target_x10 = avoidance.heading_target_x10;
    values.turn_left_pwm      = avoidance.turn_left_pwm;
    values.turn_right_pwm     = avoidance.turn_right_pwm;
    values.forward_mm         = avoidance.forward_progress_mm;
    values.forward_target_mm  = avoidance.forward_target_mm;
    values.b_barcode_irq       = BARCODE_DIGITAL_INPUT;

    return robot_wire_encode_debug(&values, p_buffer, capacity);
}

uint32_t robot_telemetry_build_heartbeat (uint8_t * p_buffer, uint32_t capacity)
{
    uint32_t length = 0u;

    /* One byte: the header alone says "alive" and which layout version. */
    if ((NULL != p_buffer) && (capacity > 0u))
    {
        p_buffer[0] = robot_wire_header(ROBOT_WIRE_TYPE_HEARTBEAT);
        length      = 1u;
    }

    return length;
}

/* Saturate a counter-sized value to 16 bits; the packer then saturates it
 * again to the field's own width. */
static uint16_t telemetry_u16 (uint32_t value)
{
    return (value > UINT16_MAX) ? UINT16_MAX : (uint16_t) value;
}

/* Clamp a computed percentage to 0..100. */
static uint8_t telemetry_percent (int32_t value)
{
    uint8_t percent = 0u;

    if (value <= 0)
    {
        percent = 0u;
    }
    else if (value >= PERCENT)
    {
        percent = (uint8_t) PERCENT;
    }
    else
    {
        percent = (uint8_t) value; /* 1..99 here. */
    }

    return percent;
}

/* The compass is checked first; the accelerometer only when it is fine. */
static uint8_t telemetry_sensor_state (const buddy4_imu_state_t * p_imu)
{
    uint8_t state = ROBOT_WIRE_SENSORS_ONLINE;

    if (!p_imu->mag_ok)
    {
        state = ROBOT_WIRE_SENSORS_COMPASS_ERROR;
    }
    else if (!p_imu->accel_ok)
    {
        state = ROBOT_WIRE_SENSORS_IMU_ERROR;
    }
    else
    {
        state = ROBOT_WIRE_SENSORS_ONLINE;
    }

    return state;
}

/*** end of file ***/

