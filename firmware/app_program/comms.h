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
 * The boxes are micro T-Kernel message buffers.  They are binary-safe: every
 * message carries its length, so a 0x00 byte inside a packed payload does
 * not cut it short.  This header uses plain C types only, because
 * robot_mqtt.c (lwIP side) cannot include kernel headers.
 */

#ifndef COMMS_H
#define COMMS_H

#include <stdbool.h>
#include <stdint.h>

/* MQTT topics (same names the dashboard uses). */
#define COMMS_TOPIC_TELEMETRY (0u) /* rover/telemetry */
#define COMMS_TOPIC_DEBUG     (1u) /* rover/debug     */
#define COMMS_TOPIC_HEARTBEAT (2u) /* rover/heartbeat */
#define COMMS_TOPIC_STATUS    (3u) /* rover/status (retained) */
#define COMMS_TOPIC_ACTIONS   (4u) /* rover/actions (non-retained) */
#define COMMS_TOPIC_STEERING  (5u) /* rover/steering (non-retained) */
#define COMMS_TOPIC_BARCODE   (6u) /* rover/barcode (non-retained) */
#define COMMS_TOPIC_IR_STREAM (7u) /* rover/ir (non-retained) */
#define COMMS_TOPIC_COUNT     (8u)

/* Longest outgoing payload.  Every packed message fits in
 * ROBOT_WIRE_MESSAGE_MAX (32) bytes; the packed diagnostic streams (built
 * only with DIAGNOSTICS=1, see pico_rp2040.mk) need up to
 * ROBOT_WIRE_DIAG_MESSAGE_MAX (96).  Unset means the streams are built
 * (host tests).  comms.c checks both against robot_wire.h at compile time. */
#if defined(ROBOT_DIAGNOSTICS) && (0 == ROBOT_DIAGNOSTICS)
#define COMMS_PAYLOAD_MAX (32u)
#else
#define COMMS_PAYLOAD_MAX (96u)
#endif
/* Longest incoming command.  Packed commands are 1..5 bytes
 * (ROBOT_WIRE_COMMAND_MAX); anything longer is dropped, not truncated. */
#define COMMS_COMMAND_MAX (8u)

/* Network state (2 bits used), for printing and telemetry. */
#define COMMS_LINK_OFFLINE (0u) /* no Wi-Fi build, or not joined   */
#define COMMS_LINK_WIFI_UP (1u) /* joined + IP address, no broker  */
#define COMMS_LINK_MQTT_UP (2u) /* connected to the MQTT broker    */

/** Create the message buffers.  Call once from usermain(). */
int32_t comms_init (void);

/**
 * @brief Queue a binary message for publishing.  Never blocks.
 * @param topic    COMMS_TOPIC_* value.
 * @param p_data   Payload bytes (may contain 0x00).
 * @param length   Payload length, 1..COMMS_PAYLOAD_MAX.
 * @return False (message dropped) if the broker is not connected, the
 *         outbox is full or the arguments are invalid.
 */
bool comms_publish (uint32_t topic, const uint8_t * p_data, uint32_t length);

/**
 * @brief Wait up to timeout_ms for a command from the network.
 * @param p_buffer    Receives the command bytes.
 * @param buffer_size Size of p_buffer; at least COMMS_COMMAND_MAX.
 * @param p_length    Receives the number of bytes in the command.
 * @param timeout_ms  Longest wait in milliseconds.
 * @return True when a command was received.
 */
bool comms_wait_command (uint8_t * p_buffer, uint32_t buffer_size,
                         uint32_t * p_length, int32_t timeout_ms);

/** Current network state (COMMS_LINK_OFFLINE, _WIFI_UP or _MQTT_UP). */
uint8_t comms_get_link_state (void);

/* ---- Called by the network side (robot_mqtt.c) only ---- */
/**
 * @brief Take the next queued message for publishing, without blocking.
 * @param p_topic     Receives the COMMS_TOPIC_* value.
 * @param p_payload   Receives the payload bytes (not NUL-terminated).
 * @param buffer_size Size of p_payload; at least COMMS_PAYLOAD_MAX.
 * @param p_length    Receives the payload length.
 * @return True when a message was taken.
 */
bool comms_take_outgoing (uint32_t * p_topic, uint8_t * p_payload,
                          uint32_t buffer_size, uint32_t * p_length);

/** Queue a command received from the broker for the controller.  A
 *  command longer than COMMS_COMMAND_MAX is dropped. */
void comms_put_incoming (const uint8_t * p_data, uint32_t length);

/** Record the network state reported by the radio task, and keep the
 *  Wi-Fi-loss and MQTT-loss bits of robot_fault.h in step with it. */
void comms_set_link_state (uint8_t state);

#endif /* COMMS_H */

/*** end of file ***/

