/** @file robot_steering_mqtt.c
 *
 * @brief Publishes queued steering decisions, and a once-a-second state
 *        snapshot when caught up, on the rover/steering MQTT topic.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "comms.h"
#include "robot_steering_trace.h"
#include "robot_steering_mqtt.h"

#define STEERING_PACKETS_PER_CALL 2u
#define STEERING_SNAPSHOT_MS      1000u

static void steering_format (char *                         p_payload,
                             const robot_steering_event_t * p_event,
                             bool b_queued, uint32_t now);

void robot_steering_publish (uint32_t now_ms)
{
    static uint32_t        last_snapshot   = 0u;
    static bool            b_have_snapshot = false;
    static char            payload[COMMS_PAYLOAD_MAX];
    robot_steering_event_t event = {0};
    uint32_t               sent  = 0u;
    bool b_more = (COMMS_LINK_MQTT_UP == comms_get_link_state());

    for (sent = 0u; b_more && (sent < STEERING_PACKETS_PER_CALL); ++sent)
    {
        bool b_queued = robot_steering_peek(&event);

        /* When caught up, send the latest state at most once a second. */
        if (!b_queued && ((b_have_snapshot &&
                           ((now_ms - last_snapshot) < STEERING_SNAPSHOT_MS)) ||
                          !robot_steering_snapshot(&event)))
        {
            b_more = false;
        }
        else
        {
            steering_format(payload, &event, b_queued, now_ms);
            if (!comms_publish_text(COMMS_TOPIC_STEERING, payload))
            {
                b_more = false;
            }
            else if (b_queued)
            {
                robot_steering_ack(event.seq);
            }
            else
            {
                last_snapshot   = now_ms;
                b_have_snapshot = true;
                b_more          = false;
            }
        }
    }
}

/* All values are fixed-range integers. Worst-case JSON is <512 bytes; the
 * host serialization test exercises every numeric bound.  tm_sprintf takes
 * UB pointers and each value is cast to the UINT/INT its conversion
 * expects. */
static void steering_format (char *                         p_payload,
                             const robot_steering_event_t * p_event,
                             bool b_queued, uint32_t now)
{
    const robot_steering_input_t * p_in = &p_event->input;

    (void) tm_sprintf(
        (UB *) p_payload,
        (UB *) "{\"v\":1,\"type\":\"%s\",\"boot\":%u,\"seq\":%u,"
               "\"ts_ms\":%u,\"lag_ms\":%u,\"dropped\":%u,"
               "\"owner\":%u,\"phase\":%u,\"direction\":%u,"
               "\"requested\":[%d,%d],\"applied\":[%d,%d],"
               "\"encoder_ms\":%u,\"distance\":[%d,%d],\"speed\":[%d,%d],"
               "\"faults\":%u,"
               "\"sample_ms\":%u,\"ir\":[%u,%u,%u,%u,%u,%u,%u,%u,%u,%u],"
               "\"has_input\":%u}",
        b_queued ? "steering" : "steering_state",
        (UINT) robot_diagnostics_session(), (UINT) p_event->seq,
        (UINT) p_event->ts_ms, b_queued ? (UINT) (now - p_event->ts_ms) : 0u,
        (UINT) p_event->dropped, (UINT) p_event->owner, (UINT) p_in->phase,
        (UINT) p_event->direction, (INT) p_event->requested[0],
        (INT) p_event->requested[1], (INT) p_event->applied[0],
        (INT) p_event->applied[1], (UINT) p_event->encoder_ms,
        (INT) p_event->distance_x10[0], (INT) p_event->distance_x10[1],
        (INT) p_event->speed_x10[0], (INT) p_event->speed_x10[1],
        (UINT) p_event->faults, (UINT) p_in->ts_ms, (UINT) p_in->raw[0],
        (UINT) p_in->raw[1], (UINT) p_in->norm[0], (UINT) p_in->norm[1],
        (UINT) p_in->threshold, (UINT) p_in->calibration[0],
        (UINT) p_in->calibration[1], (UINT) p_in->calibration[2],
        (UINT) p_in->calibration[3], (UINT) p_in->dark,
        (UINT) p_event->has_input);
}

/*** end of file ***/

