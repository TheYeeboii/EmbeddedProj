/**
 * @file    robot_telemetry.h
 * @brief   Buddy 1: builds the JSON messages the dashboard shows.
 *
 * Same topics and field names as the old main.c, so the existing dashboard
 * works unchanged.  A few new fields (mode, pitch) are added at the end.
 */

#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H

#include <stdint.h>

/* Buffers passed in must be at least this big (tm_sprintf is unbounded). */
#define ROBOT_TELEMETRY_BUFFER_SIZE  (512u)

/** rover/telemetry: sensors, encoders, motors, mode. */
void robot_telemetry_build(char *p_buffer, uint32_t now_ms);
/** rover/debug: Buddy 3 IR / line / barcode detail. */
void robot_telemetry_build_debug(char *p_buffer);
/** rover/heartbeat */
void robot_telemetry_build_heartbeat(char *p_buffer);

#endif /* ROBOT_TELEMETRY_H */
