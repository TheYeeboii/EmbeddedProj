/**
 * @file    robot_controller.h
 * @brief   Vehicle controller: decides what the robot does next.
 *
 * Takes the old main.c superloop logic and puts it in one place:
 *   - packed commands from the dashboard (MQTT "rover/commands"),
 *   - barcode diagnostics (letters and rejected reads),
 *   - the GP21 button (start / stop line following),
 *   - the safety stop for raw "drive" commands that are not refreshed.
 *
 * Called only from the controller task (app_main.c).
 */

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

/** Reset the controller to idle and set up the GP21 button. */
void robot_controller_init (void);

/**
 * @brief Execute one packed dashboard command (robot_wire.h layout).
 *
 * A command with the wrong length, an unknown opcode or out-of-range values
 * is ignored and reported on the USB console.  Distance/turn moves are
 * reserved for internal navigation.
 *
 * @param p_data Command bytes from comms_wait_command().
 * @param length Number of bytes.
 * @param now_ms Current time, for the drive safety timeout.
 */
void robot_controller_handle_command (const uint8_t * p_data, uint32_t length,
                                      uint32_t now_ms);
/** Periodic work: barcodes, button, safety timeout. */
void robot_controller_step (uint32_t now_ms);

/** Last raw drive command in dashboard units (0..65535), for telemetry. */
void robot_controller_get_drive (uint32_t * p_left, uint32_t * p_right);
/** "idle", "drive", "motion", "line" or "avoid", for the USB console. */
const char * robot_controller_mode_name (void);
/** Mode index for telemetry (3 bits): 0 idle, 1 drive, 2 motion, 3 line,
 *  4 avoid - the same order as robot_controller_mode_name(). */
uint8_t robot_controller_get_mode (void);

/** True while an existing forward/line/motion command may be protected by
 * the ultrasonic obstacle monitor. */
bool robot_controller_obstacle_watch_enabled (void);
/** Stop the current command and reserve the motors for obstacle avoidance. */
bool robot_controller_begin_obstacle_avoidance (void);
/** True while the obstacle-avoidance sequence owns the motors. */
bool robot_controller_obstacle_avoidance_active (void);
/** Release the motors after an avoidance sequence has completed or failed. */
void robot_controller_finish_obstacle_avoidance (void);
/** Stop a failed avoidance run without restarting line following. */
void robot_controller_abort_obstacle_avoidance (void);

#endif /* ROBOT_CONTROLLER_H */

/*** end of file ***/

