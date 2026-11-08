/** @file robot_steering_mqtt.h
 *
 * @brief Publishes queued steering decisions, and a once-a-second state
 *        snapshot when caught up, on the rover/steering MQTT topic.
 */
#ifndef ROBOT_STEERING_MQTT_H
#define ROBOT_STEERING_MQTT_H

#include <stdint.h>

/**
 * @brief Publish up to two steering packets when the MQTT link is up.
 * @param now_ms Current time in milliseconds.
 */
void robot_steering_publish (uint32_t now_ms);

#endif /* ROBOT_STEERING_MQTT_H */

/*** end of file ***/

