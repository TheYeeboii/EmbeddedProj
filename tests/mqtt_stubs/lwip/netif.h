#ifndef TEST_MQTT_NETIF_H
#define TEST_MQTT_NETIF_H
struct netif { int up; int link; };
static inline int netif_is_up(struct netif * netif) { return netif->up; }
static inline int netif_is_link_up(struct netif * netif) { return netif->link; }
#endif
