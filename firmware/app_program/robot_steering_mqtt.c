#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "comms.h"
#include "robot_steering_trace.h"

void robot_steering_publish (uint32_t now)
{
    static uint32_t        last_snapshot;
    static bool            have_snapshot;
    static char            payload[COMMS_PAYLOAD_MAX];
    robot_steering_event_t e;
    unsigned               sent;
    if (comms_get_link_state() != COMMS_LINK_MQTT_UP)
    {
        return;
    }
    for (sent = 0; sent < 2; ++sent)
    {
        bool queued = robot_steering_peek(&e);
        if (!queued)
        {
            if (have_snapshot && now - last_snapshot < 1000u)
            {
                return;
            }
            if (!robot_steering_snapshot(&e))
            {
                return;
            }
        }
        /* All values are fixed-range integers. Worst-case JSON is <512 bytes;
         * the host serialization test exercises every numeric bound. */
        tm_sprintf(
            (UB *) payload,
            (UB *) "{\"v\":1,\"type\":\"%s\",\"boot\":%u,\"seq\":%u,\"ts_ms\":%"
                   "u,\"lag_ms\":%u,\"dropped\":%u,"
                   "\"owner\":%u,\"phase\":%u,\"direction\":%u,\"requested\":[%"
                   "d,%d],\"applied\":[%d,%d],"
                   "\"encoder_ms\":%u,\"distance\":[%d,%d],\"speed\":[%d,%d],"
                   "\"faults\":%u,"
                   "\"sample_ms\":%u,\"ir\":[%u,%u,%u,%u,%u,%u,%u,%u,%u,%u],"
                   "\"has_input\":%u}",
            queued ? "steering" : "steering_state",
            (UINT) robot_diagnostics_session(), (UINT) e.seq, (UINT) e.ts_ms,
            queued ? (UINT) (now - e.ts_ms) : 0u, (UINT) e.dropped,
            (UINT) e.owner, (UINT) e.input.phase, (UINT) e.direction,
            (INT) e.requested[0], (INT) e.requested[1], (INT) e.applied[0],
            (INT) e.applied[1], (UINT) e.encoder_ms, (INT) e.distance_x10[0],
            (INT) e.distance_x10[1], (INT) e.speed_x10[0], (INT) e.speed_x10[1],
            (UINT) e.faults, (UINT) e.input.ts_ms, (UINT) e.input.raw[0],
            (UINT) e.input.raw[1], (UINT) e.input.norm[0],
            (UINT) e.input.norm[1], (UINT) e.input.threshold,
            (UINT) e.input.calibration[0], (UINT) e.input.calibration[1],
            (UINT) e.input.calibration[2], (UINT) e.input.calibration[3],
            (UINT) e.input.dark, (UINT) e.has_input);
        if (!comms_publish(COMMS_TOPIC_STEERING, payload))
        {
            return;
        }
        if (queued)
        {
            robot_steering_ack(e.seq);
        }
        else
        {
            last_snapshot = now;
            have_snapshot = true;
            return;
        }
    }
}

/*** end of file ***/

