/**
 * @file    robot_telemetry.h
 * @brief   Buddy 1: builds the JSON messages the dashboard shows.
 *
 * Same topics and field names as the old main.c, so the existing dashboard
 * works unchanged.  A few new fields (mode, pitch) are added at the end.
 */

#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

/* Buffers passed in must be at least this big (tm_sprintf is unbounded). */
#define ROBOT_TELEMETRY_BUFFER_SIZE  (512u)

/** rover/telemetry: sensors, encoders, motors, mode. */
void robot_telemetry_build(char *p_buffer, uint32_t now_ms);
/** rover/debug: Buddy 3 IR / line / barcode detail. */
void robot_telemetry_build_debug(char *p_buffer);
/** Save and serially report the latest temporary Buddy 5 decision for rover/debug. */
void robot_telemetry_set_avoidance_scan(uint32_t ultra_left_cm, uint32_t ultra_right_cm,
                                        bool decision_left);
/** Save side-scan values before a route decision (including blocked scans). */
void robot_telemetry_set_avoidance_scan_values(uint32_t ultra_left_cm,
                                               uint32_t ultra_right_cm);
/* Avoidance state sent to rover/debug for the dashboard Serial monitor. */
#define ROBOT_AVOID_STATUS_IDLE             (0u)
#define ROBOT_AVOID_STATUS_SCANNING         (1u)
#define ROBOT_AVOID_STATUS_TURNING          (2u)
#define ROBOT_AVOID_STATUS_COMPLETE         (3u)
#define ROBOT_AVOID_STATUS_BLOCKED          (4u)
#define ROBOT_AVOID_STATUS_IMU_UNAVAILABLE  (5u)
#define ROBOT_AVOID_STATUS_IMU_LOST         (6u)
#define ROBOT_AVOID_STATUS_TIMEOUT          (7u)
#define ROBOT_AVOID_STATUS_CANCELLED        (8u)
#define ROBOT_AVOID_STATUS_FORWARD          (9u)
#define ROBOT_AVOID_STATUS_RECOVERING        (10u)
#define ROBOT_AVOID_STATUS_REVERSING         (11u)
void robot_telemetry_set_avoidance_status(uint8_t status, int16_t heading_change_x10,
                                          uint16_t heading_target_x10);
/** Last left/right PWM commands used by the avoidance turn. */
void robot_telemetry_set_avoidance_turn_pwm(int16_t left_pwm, int16_t right_pwm);
/** Encoder progress of the heading-held forward segment, sent to rover/debug. */
void robot_telemetry_set_avoidance_forward_progress(uint16_t progress_mm,
                                                     uint16_t target_mm);
/** Clear the temporary avoidance decision after the turn has completed. */
void robot_telemetry_clear_avoidance_scan(void);
/** rover/heartbeat */
void robot_telemetry_build_heartbeat(char *p_buffer);

#endif /* ROBOT_TELEMETRY_H */
