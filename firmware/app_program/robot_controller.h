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

#include <stdint.h>

void        robot_controller_init(void);
/** Execute one command: JSON like {"cmd":"move_forward","distance_mm":500}
 *  or plain text like "drive 30000 30000". */
void        robot_controller_handle_command(const char *p_command, uint32_t now_ms);
/** Periodic work: barcodes, button, safety timeout. */
void        robot_controller_step(uint32_t now_ms);

/** Last raw drive command in dashboard units (0..65535), for telemetry. */
void        robot_controller_get_drive(uint32_t *p_left, uint32_t *p_right);
/** "idle", "drive", "motion" or "line", for telemetry. */
const char *robot_controller_mode_name(void);

#endif /* ROBOT_CONTROLLER_H */
