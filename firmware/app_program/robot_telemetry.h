/**
 * @file    robot_telemetry.h
 * @brief   Buddy 1: builds the packed binary messages the dashboard shows.
 *
 * Same topics as the old JSON messages; the layouts are in robot_wire.h and
 * docs/MQTT_BINARY_FORMAT.md, and dashboard/rover-wire.js decodes them.
 */

#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "robot_wire.h"

/** Buffer size the build functions need (the longest packed message). */
#define ROBOT_TELEMETRY_BUFFER_SIZE (ROBOT_WIRE_MESSAGE_MAX)

/**
 * @brief Pack rover/telemetry: sensors, encoders, motors, mode, faults.
 * @param p_buffer Output, ROBOT_TELEMETRY_BUFFER_SIZE bytes.
 * @param capacity Size of p_buffer.
 * @param now_ms   Time since boot, sent as whole seconds.
 * @return Message length in bytes (0 if it did not fit).
 */
uint32_t robot_telemetry_build (uint8_t * p_buffer, uint32_t capacity,
                                uint32_t now_ms);
/**
 * @brief Pack rover/debug: Buddy 3 IR / line / barcode and Buddy 5 avoidance.
 * @param p_buffer Output, ROBOT_TELEMETRY_BUFFER_SIZE bytes.
 * @param capacity Size of p_buffer.
 * @return Message length in bytes (0 if it did not fit).
 */
uint32_t robot_telemetry_build_debug (uint8_t * p_buffer, uint32_t capacity);
/** Save and serially report the latest temporary Buddy 5 decision for
 * rover/debug. */
void robot_telemetry_set_avoidance_scan (uint32_t ultra_left_cm,
                                         uint32_t ultra_right_cm,
                                         bool     decision_left);
/** Save side-scan values before a route decision (including blocked scans). */
void robot_telemetry_set_avoidance_scan_values (uint32_t ultra_left_cm,
                                                uint32_t ultra_right_cm);
/* Avoidance state sent to rover/debug (4 bits) for the dashboard. */
#define ROBOT_AVOID_STATUS_IDLE            (0u)
#define ROBOT_AVOID_STATUS_SCANNING        (1u)
#define ROBOT_AVOID_STATUS_TURNING         (2u)
#define ROBOT_AVOID_STATUS_COMPLETE        (3u)
#define ROBOT_AVOID_STATUS_BLOCKED         (4u)
#define ROBOT_AVOID_STATUS_IMU_UNAVAILABLE (5u)
#define ROBOT_AVOID_STATUS_IMU_LOST        (6u)
#define ROBOT_AVOID_STATUS_TIMEOUT         (7u)
#define ROBOT_AVOID_STATUS_CANCELLED       (8u)
#define ROBOT_AVOID_STATUS_FORWARD         (9u)
#define ROBOT_AVOID_STATUS_RECOVERING      (10u)
#define ROBOT_AVOID_STATUS_REVERSING       (11u)
/** Avoidance state, heading change and target sent to rover/debug. */
void robot_telemetry_set_avoidance_status (uint8_t  status,
                                           int16_t  heading_change_x10,
                                           uint16_t heading_target_x10);
/** Last left/right PWM commands used by the avoidance turn. */
void robot_telemetry_set_avoidance_turn_pwm (int16_t left_pwm,
                                             int16_t right_pwm);
/** Encoder progress of the heading-held forward segment, sent to rover/debug.
 */
void robot_telemetry_set_avoidance_forward_progress (uint16_t progress_mm,
                                                     uint16_t target_mm);
/** Clear the temporary avoidance decision after the turn has completed. */
void robot_telemetry_clear_avoidance_scan (void);
/**
 * @brief Pack rover/heartbeat (one header byte).
 * @param p_buffer Output, at least 1 byte.
 * @param capacity Size of p_buffer.
 * @return Message length in bytes (0 if it did not fit).
 */
uint32_t robot_telemetry_build_heartbeat (uint8_t * p_buffer,
                                          uint32_t  capacity);

#endif /* ROBOT_TELEMETRY_H */

/*** end of file ***/

