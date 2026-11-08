/** @file robot_action_mqtt.c
 *
 * @brief Publishes queued steering actions and a once-a-second state
 *        snapshot on the rover/actions MQTT topic.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "robot_action_log.h"
#include "robot_action_mqtt.h"
#include "comms.h"
#include "mqtt_config.h"

#define ACTION_PACKETS_PER_CALL 2u
#define ACTION_SNAPSHOT_MS      1000u
#define ACTION_PAYLOAD_SIZE     256u

static bool action_publish_events (char * p_payload);
static void action_publish_snapshot (char * p_payload, uint32_t now_ms);

/* Names indexed by the enum values in robot_action_log.h. */
static const char * const g_action_names[] = {
    "turn_left", "turn_right", "stop_turn_left", "stop_turn_right"};
static const char * const g_direction_names[] = {"none", "left", "right"};
static const char * const g_source_names[]    = {"pwm", "motion", "stop"};

void robot_action_publish (uint32_t now_ms)
{
    static char payload[ACTION_PAYLOAD_SIZE];

    if (COMMS_LINK_MQTT_UP == comms_get_link_state())
    {
        if (action_publish_events(payload))
        {
            action_publish_snapshot(payload, now_ms);
        }
    }
}

/* Publish up to ACTION_PACKETS_PER_CALL queued events.  Returns true when
 * the queue was drained, so a snapshot cannot overtake a transition. */
static bool action_publish_events (char * p_payload)
{
    robot_action_event_t event   = {0};
    uint32_t             sent    = 0u;
    bool                 b_more  = true;
    bool                 b_empty = false;

    for (sent = 0u; b_more && (sent < ACTION_PACKETS_PER_CALL); sent++)
    {
        if (!robot_action_peek(&event))
        {
            b_empty = true;
            b_more  = false;
        }
        else
        {
            /* tm_sprintf takes UB pointers; values are cast to the
             * UINT/INT their %u/%d conversions expect. */
            (void) tm_sprintf(
                (UB *) p_payload,
                (UB *) "{\"type\":\"action\",\"robot_id\":\"%s\",\"seq\":%u,"
                       "\"ts_ms\":%u,"
                       "\"action\":\"%s\",\"direction\":\"%s\","
                       "\"source\":\"%s\","
                       "\"left_pwm\":%d,\"right_pwm\":%d,\"dropped\":%u}",
                ROBOT_MQTT_CLIENT_ID, (UINT) event.seq, (UINT) event.ts_ms,
                g_action_names[event.action],
                g_direction_names[event.direction],
                g_source_names[event.source], (INT) event.left_pwm,
                (INT) event.right_pwm, (UINT) event.dropped);
            if (comms_publish(COMMS_TOPIC_ACTIONS, p_payload))
            {
                robot_action_ack(event.seq);
            }
            else
            {
                b_more = false;
            }
        }
    }

    /* Do not send a newer snapshot ahead of queued transitions. */
    if (b_more)
    {
        b_empty = !robot_action_peek(&event);
    }

    return b_empty;
}

/* Send the current state once a second while the queue is empty. */
static void action_publish_snapshot (char * p_payload, uint32_t now_ms)
{
    static uint32_t      last_snapshot_ms = 0u;
    static bool          b_have_snapshot  = false;
    robot_action_event_t event            = {0};

    if ((!b_have_snapshot ||
         ((now_ms - last_snapshot_ms) >= ACTION_SNAPSHOT_MS)) &&
        robot_action_snapshot_if_empty(&event))
    {
        (void) tm_sprintf(
            (UB *) p_payload,
            (UB *) "{\"type\":\"action_state\",\"robot_id\":\"%s\","
                   "\"seq\":%u,\"ts_ms\":%u,"
                   "\"direction\":\"%s\",\"source\":\"%s\","
                   "\"left_pwm\":%d,\"right_pwm\":%d,\"dropped\":%u}",
            ROBOT_MQTT_CLIENT_ID, (UINT) event.seq, (UINT) now_ms,
            g_direction_names[event.direction], g_source_names[event.source],
            (INT) event.left_pwm, (INT) event.right_pwm, (UINT) event.dropped);
        if (comms_publish(COMMS_TOPIC_ACTIONS, p_payload))
        {
            last_snapshot_ms = now_ms;
            b_have_snapshot  = true;
        }
    }
}

/*** end of file ***/

