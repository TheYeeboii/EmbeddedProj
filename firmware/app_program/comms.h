/**
 * @file    comms.h
 * @brief   Buddy 1: mailboxes between the robot tasks and the network.
 *
 * The network (Wi-Fi + lwIP + MQTT) runs inside the kernel's radio service
 * task, and lwIP may ONLY be called from there.  So robot tasks never touch
 * the network directly.  Instead:
 *
 *   robot task --comms_publish()-------> [outbox] --> robot_mqtt.c --> broker
 *   broker --> robot_mqtt.c --comms_put_incoming()--> [inbox] -->
 *                                     comms_wait_command() --> controller task
 *
 * The boxes are micro T-Kernel message buffers.  This header uses plain C
 * types only, because robot_mqtt.c (lwIP side) cannot include kernel headers.
 */

#ifndef COMMS_H
#define COMMS_H

#include <stdbool.h>
#include <stdint.h>

/* MQTT topics (same names the dashboard uses). */
#define COMMS_TOPIC_TELEMETRY   (0u)   /* rover/telemetry */
#define COMMS_TOPIC_DEBUG       (1u)   /* rover/debug     */
#define COMMS_TOPIC_HEARTBEAT   (2u)   /* rover/heartbeat */
#define COMMS_TOPIC_STATUS      (3u)   /* rover/status (retained) */
#define COMMS_TOPIC_ACTIONS     (4u)   /* rover/actions (non-retained) */
#define COMMS_TOPIC_COUNT       (5u)

#define COMMS_PAYLOAD_MAX       (512u) /* longest outgoing JSON, incl. NUL; keep >= ROBOT_TELEMETRY_BUFFER_SIZE */
#define COMMS_COMMAND_MAX       (128u) /* longest incoming command, incl. NUL */

/* Network state, for printing and telemetry. */
#define COMMS_LINK_OFFLINE      (0u)   /* no Wi-Fi build, or not joined   */
#define COMMS_LINK_WIFI_UP      (1u)   /* joined + IP address, no broker  */
#define COMMS_LINK_MQTT_UP      (2u)   /* connected to the MQTT broker    */

/** Create the message buffers.  Call once from usermain(). */
int32_t  comms_init(void);

/** Queue a JSON message for publishing.  Never blocks; drops the message
 *  (returns false) if the broker is not connected or the outbox is full. */
bool     comms_publish(uint32_t topic, const char *p_payload);

/** Wait up to timeout_ms for a command string from the network. */
bool     comms_wait_command(char *p_buffer, uint32_t buffer_size, int32_t timeout_ms);

uint32_t comms_get_link_state(void);

/* ---- Called by the network side (robot_mqtt.c) only ---- */
bool     comms_take_outgoing(uint32_t *p_topic, char *p_payload, uint32_t buffer_size);
void     comms_put_incoming(const char *p_data, uint32_t length);
void     comms_set_link_state(uint32_t state);

#endif /* COMMS_H */
