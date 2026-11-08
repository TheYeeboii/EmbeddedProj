#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "robot_action_log.h"
#include "comms.h"
#include "mqtt_config.h"

void robot_action_publish (uint32_t now_ms)
{
    static uint32_t           last_snapshot_ms;
    static bool               have_snapshot;
    static char               payload[256];
    static const char * const actions[] = {"turn_left", "turn_right",
                                           "stop_turn_left", "stop_turn_right"};
    static const char * const directions[] = {"none", "left", "right"};
    static const char * const sources[]    = {"pwm", "motion", "stop"};
    robot_action_event_t      event;
    uint32_t                  sent;
    if (comms_get_link_state() != COMMS_LINK_MQTT_UP)
    {
        return;
    }

    for (sent = 0u; sent < 2u; sent++)
    {
        if (!robot_action_peek(&event))
        {
            break;
        }
        (void) tm_sprintf(
            (UB *) payload,
            (UB *) "{\"type\":\"action\",\"robot_id\":\"%s\",\"seq\":%u,\"ts_"
                   "ms\":%u,"
                   "\"action\":\"%s\",\"direction\":\"%s\",\"source\":\"%s\","
                   "\"left_pwm\":%d,\"right_pwm\":%d,\"dropped\":%u}",
            ROBOT_MQTT_CLIENT_ID, (UINT) event.seq, (UINT) event.ts_ms,
            actions[event.action], directions[event.direction],
            sources[event.source], (INT) event.left_pwm, (INT) event.right_pwm,
            (UINT) event.dropped);
        if (!comms_publish(COMMS_TOPIC_ACTIONS, payload))
        {
            return;
        }
        robot_action_ack(event.seq);
    }
    /* Do not send a newer snapshot ahead of queued transitions. */
    if (robot_action_peek(&event))
    {
        return;
    }
    if (have_snapshot && (now_ms - last_snapshot_ms) < 1000u)
    {
        return;
    }
    if (!robot_action_snapshot_if_empty(&event))
    {
        return;
    }
    (void) tm_sprintf((UB *) payload,
                      (UB *) "{\"type\":\"action_state\",\"robot_id\":\"%s\","
                             "\"seq\":%u,\"ts_ms\":%u,"
                             "\"direction\":\"%s\",\"source\":\"%s\","
                             "\"left_pwm\":%d,\"right_pwm\":%d,\"dropped\":%u}",
                      ROBOT_MQTT_CLIENT_ID, (UINT) event.seq, (UINT) now_ms,
                      directions[event.direction], sources[event.source],
                      (INT) event.left_pwm, (INT) event.right_pwm,
                      (UINT) event.dropped);
    if (comms_publish(COMMS_TOPIC_ACTIONS, payload))
    {
        last_snapshot_ms = now_ms;
        have_snapshot    = true;
    }
}

/*** end of file ***/

