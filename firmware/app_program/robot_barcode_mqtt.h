/** @file robot_barcode_mqtt.h
 *
 * @brief Publishes barcode scan packets (robot_barcode_trace) as JSON on the
 *        rover/barcode MQTT topic.
 */
#ifndef ROBOT_BARCODE_MQTT_H
#define ROBOT_BARCODE_MQTT_H

#include <stdint.h>

/**
 * @brief Publish up to two barcode packets when the MQTT link is up.
 * @param now_ms Current time in milliseconds.
 */
void robot_barcode_publish (uint32_t now_ms);

#endif /* ROBOT_BARCODE_MQTT_H */

/*** end of file ***/

