/**
 * @file    robot_mqtt.c
 * @brief   Buddy 1: MQTT client, run from the kernel's radio service task.
 *
 * Network side of comms.h.  Ported from the MQTT part of the old main.c
 * (same topics, client id, keep-alive and "last will"), reorganised as a
 * small state machine that the radio service calls every poll.
 *
 * lwIP / SDK side: must NOT include <tk/tkernel.h> (see robot_hal.h).
 * Only compiled into Wi-Fi builds (make ... WIFI_MQTT=1).
 */

#if defined(TM_WIFI_MQTT) && (TM_WIFI_MQTT == 1)

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"

#include "comms.h"
#include "mqtt_config.h"
#include "robot_mqtt.h"

extern uint64_t time_us_64(void);

#define ROBOT_MQTT_RETRY_US          (5000000u)  /* wait 5 s between attempts */
#define ROBOT_MQTT_MAX_PUBLISH_PER_POLL (2u)

static const char *const g_topic_names[COMMS_TOPIC_COUNT] =
{
    "rover/telemetry",
    "rover/debug",
    "rover/heartbeat",
    "rover/status",
    "rover/actions",
    "rover/steering",
    "rover/barcode"
};

static mqtt_client_t *g_client            = NULL;
static bool           g_connecting        = false;
static bool           g_subscribe_pending = false;
static bool           g_online_pending    = false;
static uint64_t       g_next_attempt_us   = 0u;
static char           g_incoming[COMMS_COMMAND_MAX];
static uint32_t       g_incoming_length   = 0u;
static char           g_outgoing[COMMS_PAYLOAD_MAX];

/* ---------------- lwIP callbacks (radio task context) ---------------- */

static void robot_mqtt_incoming_publish(void *p_arg, const char *p_topic, u32_t total_length)
{
    (void)p_arg;
    (void)p_topic;          /* we only subscribe to rover/commands */
    (void)total_length;
    g_incoming_length = 0u;
}

static void robot_mqtt_incoming_data(void *p_arg, const u8_t *p_data, u16_t length, u8_t flags)
{
    uint32_t space;
    uint32_t copy;

    (void)p_arg;
    if (NULL != p_data)
    {
        space = (COMMS_COMMAND_MAX - 1u) - g_incoming_length;
        copy  = (length < space) ? length : space;
        memcpy(&g_incoming[g_incoming_length], p_data, copy);
        g_incoming_length += copy;
    }
    if (0u != (flags & MQTT_DATA_FLAG_LAST))
    {
        comms_put_incoming(g_incoming, g_incoming_length);
        g_incoming_length = 0u;
    }
}

static void robot_mqtt_connected(mqtt_client_t *p_client, void *p_arg,
                                 mqtt_connection_status_t status)
{
    (void)p_client;
    (void)p_arg;
    g_connecting = false;
    if (MQTT_CONNECT_ACCEPTED == status)
    {
        g_subscribe_pending = true;
        g_online_pending    = true;
    }
    else
    {
        g_next_attempt_us = time_us_64() + ROBOT_MQTT_RETRY_US;
    }
}

/* ---------------- state machine ---------------- */

static void robot_mqtt_start_connect(void)
{
    static const struct mqtt_connect_client_info_t client_info =
    {
        ROBOT_MQTT_CLIENT_ID,
        NULL,                               /* user     */
        NULL,                               /* password */
        60u,                                /* keep-alive, seconds */
        "rover/status",                     /* last will topic */
        "{\"robot_id\":\"" ROBOT_MQTT_CLIENT_ID "\",\"state\":\"offline\"}",
        0u,                                 /* will length: use strlen */
        0u,                                 /* will QoS */
        1u                                  /* will retained */
    };
    ip_addr_t broker;

    if (!ipaddr_aton(ROBOT_MQTT_BROKER_IP, &broker))
    {
        g_next_attempt_us = time_us_64() + ROBOT_MQTT_RETRY_US;
        return;
    }
    g_connecting = true;
    if (ERR_OK != mqtt_client_connect(g_client, &broker, ROBOT_MQTT_BROKER_PORT,
                                      robot_mqtt_connected, NULL, &client_info))
    {
        g_connecting      = false;
        g_next_attempt_us = time_us_64() + ROBOT_MQTT_RETRY_US;
    }
}

void robot_mqtt_poll(struct netif *p_netif)
{
    uint32_t topic;
    uint32_t sent;

    if ((NULL == p_netif) || !netif_is_up(p_netif) || !netif_is_link_up(p_netif))
    {
        comms_set_link_state(COMMS_LINK_OFFLINE);
        return;
    }

    if (NULL == g_client)
    {
        g_client = mqtt_client_new();
        if (NULL == g_client)
        {
            return;
        }
        mqtt_set_inpub_callback(g_client, robot_mqtt_incoming_publish,
                                robot_mqtt_incoming_data, NULL);
    }

    if (!mqtt_client_is_connected(g_client))
    {
        comms_set_link_state(COMMS_LINK_WIFI_UP);
        if (!g_connecting && (time_us_64() >= g_next_attempt_us))
        {
            robot_mqtt_start_connect();
        }
        return;
    }

    comms_set_link_state(COMMS_LINK_MQTT_UP);

    if (g_subscribe_pending)
    {
        if (ERR_OK == mqtt_subscribe(g_client, "rover/commands", 0u, NULL, NULL))
        {
            g_subscribe_pending = false;
        }
    }
    if (g_online_pending)
    {
        static const char online[] =
            "{\"robot_id\":\"" ROBOT_MQTT_CLIENT_ID "\",\"state\":\"online\"}";

        if (ERR_OK == mqtt_publish(g_client, g_topic_names[COMMS_TOPIC_STATUS],
                                   online, (u16_t)(sizeof(online) - 1u), 0u, 1u, NULL, NULL))
        {
            g_online_pending = false;
        }
    }

    /* Drain a couple of queued messages per poll. */
    for (sent = 0u; sent < ROBOT_MQTT_MAX_PUBLISH_PER_POLL; sent++)
    {
        if (!comms_take_outgoing(&topic, g_outgoing, sizeof(g_outgoing)))
        {
            break;
        }
        (void)mqtt_publish(g_client, g_topic_names[topic], g_outgoing,
                           (u16_t)strlen(g_outgoing), 0u, 0u, NULL, NULL);
    }
}

#endif /* TM_WIFI_MQTT */
