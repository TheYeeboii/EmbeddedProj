/**
 * @file    mqtt_utk.h
 * @brief   Buddy 1: MQTT client, run from the kernel's radio service task.
 */

#ifndef ROBOT_MQTT_H
#define ROBOT_MQTT_H

struct netif;

/**
 * @brief One step of the MQTT client: connect / reconnect, subscribe,
 *        and publish whatever is waiting in the comms outbox.
 *
 * Called every radio poll (about every 2 ms) from cyw43_utk_task() in
 * lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c, once DHCP has finished.
 * lwIP is single-threaded here, so this is the ONLY place lwIP MQTT
 * functions may be called from.
 */
void robot_mqtt_poll (struct netif * p_netif);

#endif /* ROBOT_MQTT_H */

/*** end of file ***/

