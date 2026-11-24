/**
 * @file    app.h
 * @brief   Definitions shared by the whole team (all subsystems).
 *
 * Task priorities and the start-up helpers every subsystem uses to create
 * its tasks.  Owners, pins and interfaces are listed in TEAM.md.
 *
 *   Task         Pri  Period            Job                          Owner
 *   -----------  ---  ----------------  ---------------------------  -----
 *   (usb)          3  kernel            USB serial console           -
 *   (cyw43)        4  2 ms, kernel      Wi-Fi + lwIP + MQTT          S1
 *   motion         5  20 ms (cyclic)    buddy2_update(): PID, moves  S2
 *   line           6  1 ms (cyclic)     buddy3_update(): IR, barcode S3
 *   controller     7  20 ms             commands, barcodes, buttons  shared
 *   imu            9  50 ms target      IMU + hump processing        S4
 *   sensors       10  100 ms            ultrasonic + avoidance       S5
 *   telemetry     15  1 s               console + MQTT reports       S1
 *
 * Lower number = higher priority.  (usb) and (cyw43) are created by the
 * kernel itself.
 */

#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

/* Task priorities (see the table above). */
#define APP_PRI_MOTION     (5)
#define APP_PRI_LINE       (6)
#define APP_PRI_CONTROLLER (7)
#define APP_PRI_IMU        (9)
#define APP_PRI_SENSORS    (10)
#define APP_PRI_TELEMETRY  (15)

/** Milliseconds since boot (kernel operating time). */
uint32_t app_now_ms (void);

/**
 * @brief Report whether a kernel call succeeded; print the error if not.
 * @param p_what What was attempted, for the message.
 * @param result The kernel call's return value (E_OK or an object ID on
 *               success, a negative error code on failure).
 */
bool app_check (const char * p_what, INT result);

/** @brief Keep the rover stopped after a fatal start-up failure; never
 *         returns. */
void app_fail_forever (const char * p_what);

/**
 * @brief Create and start one task (stack APP_STACK_SIZE).  A failure is
 *        fatal: the rover cannot run without one of its tasks.
 * @param p_name   Name for start-up messages.
 * @param p_task   Task function.
 * @param priority One of the APP_PRI_* values.
 * @return The new task's ID.
 */
ID app_start_task (const char * p_name, void (*p_task)(INT, void *),
                   INT priority);

/**
 * @brief Create a cyclic handler that wakes a task every period_ms (the task
 *        sleeps with tk_slp_tsk()).  A failure is fatal.
 * @param p_name    Name for start-up messages.
 * @param p_task_id Static variable holding the task's ID (read by the
 *                  handler, so it must outlive the handler).
 * @param period_ms Wake-up period in milliseconds.
 */
void app_start_cyclic (const char * p_name, ID * p_task_id,
                       UINT period_ms);

#endif /* APP_H */

/*** end of file ***/
