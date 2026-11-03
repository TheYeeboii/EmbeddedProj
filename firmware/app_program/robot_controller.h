/**
 * @file    robot_controller.h
 * @brief   Vehicle controller: decides what the robot does next.
 *
 * Takes the old main.c superloop logic and puts it in one place:
 *   - commands from the dashboard (MQTT "rover/commands"),
 *   - barcode actions (turn left / right / go straight, then resume the line),
 *   - the GP21 button (start / stop line following),
 *   - the safety stop for raw "drive" commands that are not refreshed.
 *
 * Called only from the controller task (app_main.c).
 */

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

void        robot_controller_init(void);
/** Execute one dashboard command: JSON like {"cmd":"drive","left":30000,
 *  "right":30000} or plain text like "drive 30000 30000".  Distance/turn
 *  moves are not dashboard commands; only barcode actions start them. */
void        robot_controller_handle_command(const char *p_command, uint32_t now_ms);
/** Periodic work: barcodes, button, safety timeout. */
void        robot_controller_step(uint32_t now_ms);

/** Last raw drive command in dashboard units (0..65535), for telemetry. */
void        robot_controller_get_drive(uint32_t *p_left, uint32_t *p_right);
/** "idle", "drive", "motion", "line" or "avoid", for telemetry. */
const char *robot_controller_mode_name(void);

/** True while an existing forward/line/motion command may be protected by
 * the ultrasonic obstacle monitor. */
bool        robot_controller_obstacle_watch_enabled(void);
/** Stop the current command and reserve the motors for obstacle avoidance. */
bool        robot_controller_begin_obstacle_avoidance(void);
/** True while the obstacle-avoidance sequence owns the motors. */
bool        robot_controller_obstacle_avoidance_active(void);
/** Release the motors after an avoidance sequence has completed or failed. */
void        robot_controller_finish_obstacle_avoidance(void);
/** Stop a failed avoidance run without restarting line following. */
void        robot_controller_abort_obstacle_avoidance(void);

#endif /* ROBOT_CONTROLLER_H */
