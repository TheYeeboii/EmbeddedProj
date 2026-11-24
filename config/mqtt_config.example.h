/**
 * @file mqtt_config.h
 * @brief Local MQTT settings. Copy this file to mqtt_config.h before building.
 *
 * Set the broker IP to the laptop's address on the rover's Wi-Fi network.
 * Keep the local mqtt_config.h file out of commits.
 */

#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

#define ROBOT_MQTT_BROKER_IP   "192.168.137.1"
#define ROBOT_MQTT_BROKER_PORT (1883u)
#define ROBOT_MQTT_CLIENT_ID   "buddy1"

#endif /* MQTT_CONFIG_H */
