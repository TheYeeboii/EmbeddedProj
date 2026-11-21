#ifndef TEST_MQTT_IP_H
#define TEST_MQTT_IP_H
typedef struct { unsigned addr; } ip_addr_t;
int ipaddr_aton(const char * text, ip_addr_t * address);
#endif
