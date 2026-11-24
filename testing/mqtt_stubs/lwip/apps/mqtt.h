#ifndef TEST_MQTT_H
#define TEST_MQTT_H
#include <stdint.h>
#include "lwip/ip_addr.h"
typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int err_t;
typedef int mqtt_connection_status_t;
typedef struct { int connected; } mqtt_client_t;
#define ERR_OK 0
#define ERR_MEM (-1)
#define ERR_CONN (-2)
#define MQTT_CONNECT_ACCEPTED 0
#define MQTT_DATA_FLAG_LAST 1
struct mqtt_connect_client_info_t {
 const char * client_id; const char * user; const char * pass; u16_t keep_alive;
 const char * will_topic; const char * will_msg; u8_t will_msg_len;
 u8_t will_qos; u8_t will_retain;
};
mqtt_client_t * mqtt_client_new(void);
int mqtt_client_is_connected(mqtt_client_t *);
void mqtt_set_inpub_callback(mqtt_client_t *,
 void (*)(void *, const char *, u32_t),
 void (*)(void *, const u8_t *, u16_t, u8_t), void *);
err_t mqtt_client_connect(mqtt_client_t *, const ip_addr_t *, u16_t,
 void (*)(mqtt_client_t *, void *, mqtt_connection_status_t), void *,
 const struct mqtt_connect_client_info_t *);
err_t mqtt_subscribe(mqtt_client_t *, const char *, u8_t, void *, void *);
err_t mqtt_publish(mqtt_client_t *, const char *, const void *, u16_t,
 u8_t, u8_t, void *, void *);
#endif
