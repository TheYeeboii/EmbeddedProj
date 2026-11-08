/** @file robot_action_mqtt.h
 *
 * @brief Publishes queued steering actions and a once-a-second state
 *        snapshot on the rover/actions MQTT topic.
 */
#ifndef ROBOT_ACTION_MQTT_H
#define ROBOT_ACTION_MQTT_H

#include <stdint.h>

/**
 * @brief Publish queued actions.  Called by the controller task, never from
 *        the wheel-control critical path.  Drains at most two events per
 *        call; a current-state snapshot is sent each second when caught up.
 *        Network publication itself remains in the radio task.
 * @param now_ms Current time in milliseconds.
 */
void robot_action_publish (uint32_t now_ms);

#endif /* ROBOT_ACTION_MQTT_H */

/*** end of file ***/

