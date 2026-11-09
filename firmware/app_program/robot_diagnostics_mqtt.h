/** @file robot_diagnostics_mqtt.h
 *
 * @brief Buddy 1: publishes the four diagnostic streams as JSON.
 *
 * Called every controller step (app_main.c).  Built only with
 * DIAGNOSTICS=1; robot_diagnostics_off.c provides empty versions otherwise.
 */
#ifndef ROBOT_DIAGNOSTICS_MQTT_H
#define ROBOT_DIAGNOSTICS_MQTT_H

#include <stdint.h>

/**
 * @brief Publish queued motor actions on rover/actions, or the current
 *        state once a second when the queue is empty.
 * @param now_ms Current time in milliseconds.
 */
void robot_action_publish (uint32_t now_ms);

/**
 * @brief Publish queued steering decisions on rover/steering, or the current
 *        state once a second when the queue is empty.
 * @param now_ms Current time in milliseconds.
 */
void robot_steering_publish (uint32_t now_ms);

/**
 * @brief Publish up to two queued barcode packets on rover/barcode.
 * @param now_ms Current time in milliseconds.
 */
void robot_barcode_publish (uint32_t now_ms);

/**
 * @brief Publish up to two queued IR batches on rover/ir.
 * @param now_ms Current time in milliseconds.
 */
void robot_ir_stream_publish (uint32_t now_ms);

#endif /* ROBOT_DIAGNOSTICS_MQTT_H */

/*** end of file ***/

