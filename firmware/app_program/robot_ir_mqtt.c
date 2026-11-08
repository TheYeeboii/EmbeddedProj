/** @file robot_ir_mqtt.c
 *
 * @brief Publishes the continuous IR sample queue (robot_ir_stream) as JSON
 *        batches on the rover/ir_stream MQTT topic.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "comms.h"
#include "robot_steering_trace.h"
#include "robot_ir_stream.h"
#include "robot_ir_mqtt.h"

/* At most this many packets per call, so the network task is not starved. */
#define IR_MQTT_PACKETS_PER_CALL 2u
/* A short final batch is flushed once its oldest row is this old. */
#define IR_MQTT_FLUSH_MS 40u

static INT ir_mqtt_format (char *                          p_payload,
                           const robot_ir_stream_batch_t * p_batch,
                           uint32_t                        now);

void robot_ir_stream_publish (uint32_t now_ms)
{
    static char             payload[COMMS_PAYLOAD_MAX];
    robot_ir_stream_batch_t batch = {0};
    uint32_t                sent  = 0u;
    bool b_more = (COMMS_LINK_MQTT_UP == comms_get_link_state());

    for (sent = 0u; b_more && (sent < IR_MQTT_PACKETS_PER_CALL); ++sent)
    {
        if (!robot_ir_stream_peek(&batch))
        {
            b_more = false;
        }
        /* Batch 100 Hz observations into roughly 25 Hz packets, with bounded
         * latency for a final short batch. No samples are downsampled. */
        else if ((batch.count < ROBOT_IR_STREAM_BATCH) &&
                 ((now_ms - batch.rows[0].ts_ms) < IR_MQTT_FLUSH_MS))
        {
            b_more = false;
        }
        else
        {
            (void) ir_mqtt_format(payload, &batch, now_ms);
            if (comms_publish_text(COMMS_TOPIC_IR_STREAM, payload))
            {
                robot_ir_stream_ack(&batch);
            }
            else
            {
                b_more = false;
            }
        }
    }
}

/* Format one batch as JSON into p_payload (COMMS_PAYLOAD_MAX bytes; the
 * worst-case four-row packet is about 440 bytes). Returns the length. */
static INT ir_mqtt_format (char *                          p_payload,
                           const robot_ir_stream_batch_t * p_batch,
                           uint32_t                        now)
{
    const robot_ir_stream_sample_t * p_last =
        &p_batch->rows[p_batch->count - 1u];
    uint32_t row    = 0u;
    INT      length = 0;

    /* WARNING: tm_sprintf takes UB pointers; the char buffers are reused as
     * bytes and every value is cast to the UINT/INT its %u/%d expects. */
    length = tm_sprintf(
        (UB *) p_payload,
        (UB *) "{\"v\":1,\"type\":\"ir_stream\",\"boot\":%u,\"ts_ms\":%u,"
               "\"lag_ms\":%u,\"dropped\":%u,"
               "\"period_ms\":10,\"white\":400,\"black\":600,\"rows\":[",
        (UINT) robot_diagnostics_session(), (UINT) p_last->ts_ms,
        (UINT) (now - p_last->ts_ms), (UINT) p_batch->dropped);
    for (row = 0u; row < p_batch->count; ++row)
    {
        const robot_ir_stream_sample_t * p_row = &p_batch->rows[row];

        length += tm_sprintf((UB *) p_payload + length,
                             (UB *) "%s[%u,%u,%u,%u,%u,%d,%d,%u,%u,%u,%u]",
                             (0u != row) ? "," : "", (UINT) p_row->seq,
                             (UINT) p_row->ts_ms, (UINT) p_row->raw,
                             (UINT) p_row->normalized, (UINT) p_row->black,
                             (INT) p_row->left_x10, (INT) p_row->right_x10,
                             (UINT) p_row->blocked, (UINT) p_row->minimum,
                             (UINT) p_row->maximum, (UINT) p_row->inverted);
    }
    length += tm_sprintf((UB *) p_payload + length, (UB *) "]}");

    return length;
}

/*** end of file ***/

