/**
 * @file    mqtt_config.h
 * @brief   MQTT broker settings for the robot (Wi-Fi builds only).
 *
 * The broker is Mosquitto running on the laptop (see tools/ in the project
 * root).  Set the laptop's IP address on the robot's Wi-Fi network here.
 * The dashboard defaults to the same address.
 *
 * Wi-Fi name/password go in config/wifi_credentials.h (not committed).
 */

#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

#define ROBOT_MQTT_BROKER_IP     "192.168.137.1"   /* laptop running Mosquitto */
#define ROBOT_MQTT_BROKER_PORT   (1883u)
#define ROBOT_MQTT_CLIENT_ID     "buddy1"          /* shown on the dashboard   */

#endif /* MQTT_CONFIG_H */
