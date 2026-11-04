#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "comms.h"
#include "robot_steering_trace.h"
#include "robot_ir_stream.h"

void robot_ir_stream_publish(uint32_t now)
{
    static char payload[COMMS_PAYLOAD_MAX];
    robot_ir_stream_batch_t batch;
    unsigned sent, i;
    if (comms_get_link_state() != COMMS_LINK_MQTT_UP) return;
    for (sent = 0; sent < 2; ++sent) {
        INT length;
        if (!robot_ir_stream_peek(&batch)) return;
        /* Batch 100 Hz observations into roughly 25 Hz packets, with bounded
         * latency for a final short batch. No samples are downsampled. */
        if (batch.count < ROBOT_IR_STREAM_BATCH && now - batch.rows[0].ts_ms < 40u) return;
        length = tm_sprintf((UB *)payload, (UB *)
            "{\"v\":1,\"type\":\"ir_stream\",\"boot\":%u,\"ts_ms\":%u,\"lag_ms\":%u,\"dropped\":%u,"
            "\"period_ms\":10,\"white\":400,\"black\":600,\"rows\":[",
            (UINT)robot_diagnostics_session(), (UINT)batch.rows[batch.count-1].ts_ms,
            (UINT)(now-batch.rows[batch.count-1].ts_ms), (UINT)batch.dropped);
        for (i = 0; i < batch.count; ++i) {
            robot_ir_stream_sample_t *s = &batch.rows[i];
            length += tm_sprintf((UB *)payload+length, (UB *)"%s[%u,%u,%u,%u,%u,%d,%d,%u,%u,%u,%u]",
                i ? "," : "", (UINT)s->seq, (UINT)s->ts_ms, (UINT)s->raw,
                (UINT)s->normalized, (UINT)s->black, (INT)s->left_x10, (INT)s->right_x10,
                (UINT)s->blocked, (UINT)s->minimum, (UINT)s->maximum, (UINT)s->inverted);
        }
        (void)tm_sprintf((UB *)payload+length,(UB *)"]}");
        if (!comms_publish(COMMS_TOPIC_IR_STREAM,payload)) return;
        robot_ir_stream_ack(&batch);
    }
}
