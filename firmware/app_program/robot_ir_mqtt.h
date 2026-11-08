/** @file robot_ir_mqtt.h
 *
 * @brief Publishes the continuous IR sample queue (robot_ir_stream) as JSON
 *        batches on the rover/ir_stream MQTT topic.
 */
#ifndef ROBOT_IR_MQTT_H
#define ROBOT_IR_MQTT_H

#include <stdint.h>

/**
 * @brief Publish up to two queued IR batches when the MQTT link is up.
 * @param now_ms Current time in milliseconds.
 */
void robot_ir_stream_publish (uint32_t now_ms);

#endif /* ROBOT_IR_MQTT_H */

/*** end of file ***/

