/** @file robot_diagnostics_mqtt.c
 *
 * @brief Buddy 1: publishes the four diagnostic streams as packed binary.
 *
 *   rover/actions   type 8   Buddy 2 motor actions    (robot_action_log)
 *   rover/steering  type 9   steering decisions       (robot_steering_trace)
 *   rover/barcode   type 10  barcode scans            (robot_barcode_trace)
 *   rover/ir        type 11  continuous IR samples    (robot_ir_stream)
 *
 * Each publisher sends at most two queued records per call, from the
 * controller task, and stops early when the outbox is full or offline.
 * Built only with DIAGNOSTICS=1 (pico_rp2040.mk); the default build links
 * robot_diagnostics_off.c instead.
 *
 * Change Requirement 1: like the main messages, every field is packed at
 * the bits its range needs (robot_packed.h, most significant bit first);
 * the layouts are in docs/MQTT_BINARY_FORMAT.md and dashboard/rover-wire.js
 * decodes them back into the objects the tuning views use.  Counters and
 * timestamps that really use 32 bits (sequence numbers, boot id, ms
 * clocks) keep 32; drop counts and lags saturate at 16 bits.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "comms.h"
#include "robot_action_log.h"
#include "robot_barcode_trace.h"
#include "robot_diagnostics_mqtt.h"
#include "robot_ir_stream.h"
#include "robot_packed.h"
#include "robot_steering_trace.h"
#include "robot_wire.h"

/* Field widths in bits (docs/MQTT_BINARY_FORMAT.md, diagnostic streams). */
#define DIAG_BITS_HEADER  (8u)
#define DIAG_BITS_FLAG    (1u)
#define DIAG_BITS_U32     (32u)
#define DIAG_BITS_SAT16   (16u) /* drop counts and lags, saturating */
#define DIAG_BITS_PWM     (16u)
#define DIAG_BITS_SMALL2  (2u)
#define DIAG_BITS_OWNER   (3u)
#define DIAG_BITS_PHASE   (4u)
#define DIAG_BITS_DIST    (24u) /* x10 or x100 distances, +/-838 m */
#define DIAG_BITS_RAW     (12u)
#define DIAG_BITS_NORM    (10u)
#define DIAG_BITS_PTYPE   (3u)
#define DIAG_BITS_COUNT   (3u) /* rows in one packet, 1..4 */
#define DIAG_BITS_EDGE    (5u) /* edge index, 0..28 */
#define DIAG_BITS_SAMPLE  (9u) /* sample index, 0..511 */
#define DIAG_BITS_STATE   (3u)
#define DIAG_BITS_EDGES   (5u)  /* edges in a scan, 0..29 */
#define DIAG_BITS_SAMPLES (10u) /* samples in a scan, 0..512 */
#define DIAG_BITS_BLOCKED (6u)
#define DIAG_BITS_REASON  (3u)
#define DIAG_BITS_FORMAT  (3u)
#define DIAG_BITS_SYMBOL  (7u)
#define DIAG_BITS_ELEMENT (17u) /* min element x100, up to 1000 mm */
#define DIAG_BITS_OFFSET  (14u) /* left offset x10, +/-500 mm */
#define DIAG_BITS_GEOM    (16u) /* geometry x10 */
#define DIAG_BITS_DT      (16u) /* ms between IR rows, saturating */

/* ---- rover/actions ---- */
#define ACTION_PACKETS_PER_CALL 2u
#define ACTION_SNAPSHOT_MS      1000u

/* ---- rover/steering ---- */
#define STEERING_PACKETS_PER_CALL 2u
#define STEERING_SNAPSHOT_MS      1000u

/* ---- rover/barcode ---- */
#define BARCODE_PACKETS_PER_CALL 2u

/* ---- rover/ir ---- */
/* At most this many packets per call, so the network task is not starved. */
#define IR_MQTT_PACKETS_PER_CALL 2u
/* A short final batch is flushed once its oldest row is this old. */
#define IR_MQTT_FLUSH_MS 40u

static bool     action_publish_events (uint8_t * p_payload);
static void     action_publish_snapshot (uint8_t * p_payload, uint32_t now_ms);
static uint32_t action_pack (uint8_t *                    p_payload,
                             const robot_action_event_t * p_event, bool b_state,
                             uint32_t ts_ms);
static uint32_t steering_pack (uint8_t *                      p_payload,
                               const robot_steering_event_t * p_event,
                               bool b_queued, uint32_t now);
static uint32_t barcode_pack (uint8_t *                      p_payload,
                              const robot_barcode_packet_t * p_packet,
                              uint32_t                       now);
static void     barcode_put_config (robot_packed_writer_t *        p_writer,
                                    const robot_barcode_packet_t * p_packet);
static void     barcode_put_rows (robot_packed_writer_t *        p_writer,
                                  const robot_barcode_packet_t * p_packet);
static void     barcode_put_summary (robot_packed_writer_t *        p_writer,
                                     const robot_barcode_packet_t * p_packet);
static uint32_t ir_pack (uint8_t *                       p_payload,
                         const robot_ir_stream_batch_t * p_batch, uint32_t now);
static uint32_t diag_lag (uint32_t now, uint32_t ts_ms);
static void     diag_writer_start (robot_packed_writer_t * p_writer,
                                   uint8_t * p_payload, uint32_t type);

void robot_action_publish (uint32_t now_ms)
{
    static uint8_t payload[ROBOT_WIRE_DIAG_MESSAGE_MAX];

    if (COMMS_LINK_MQTT_UP == comms_get_link_state())
    {
        if (action_publish_events(payload))
        {
            action_publish_snapshot(payload, now_ms);
        }
    }
}

void robot_steering_publish (uint32_t now_ms)
{
    static uint32_t        last_snapshot   = 0u;
    static bool            b_have_snapshot = false;
    static uint8_t         payload[ROBOT_WIRE_DIAG_MESSAGE_MAX];
    robot_steering_event_t event  = {0};
    uint32_t               sent   = 0u;
    uint32_t               length = 0u;
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
            length = steering_pack(payload, &event, b_queued, now_ms);
            if (!comms_publish(COMMS_TOPIC_STEERING, payload, length))
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

void robot_barcode_publish (uint32_t now_ms)
{
    static uint8_t         payload[ROBOT_WIRE_DIAG_MESSAGE_MAX];
    robot_barcode_packet_t packet = {0};
    uint32_t               sent   = 0u;
    uint32_t               length = 0u;
    bool b_more = (COMMS_LINK_MQTT_UP == comms_get_link_state());

    for (sent = 0u; b_more && (sent < BARCODE_PACKETS_PER_CALL); ++sent)
    {
        if (!robot_barcode_packet_peek(&packet, now_ms))
        {
            b_more = false;
        }
        else
        {
            length = barcode_pack(payload, &packet, now_ms);
            if (comms_publish(COMMS_TOPIC_BARCODE, payload, length))
            {
                robot_barcode_packet_ack(&packet);
            }
            else
            {
                b_more = false;
            }
        }
    }
}

void robot_ir_stream_publish (uint32_t now_ms)
{
    static uint8_t          payload[ROBOT_WIRE_DIAG_MESSAGE_MAX];
    robot_ir_stream_batch_t batch  = {0};
    uint32_t                sent   = 0u;
    uint32_t                length = 0u;
    bool b_more = (COMMS_LINK_MQTT_UP == comms_get_link_state());

    for (sent = 0u; b_more && (sent < IR_MQTT_PACKETS_PER_CALL); ++sent)
    {
        if (!robot_ir_stream_peek(&batch))
        {
            b_more = false;
        }
        /* Batch observations at the configured cadence, with bounded
         * latency for a final short batch. No samples are downsampled. */
        else if ((batch.count < ROBOT_IR_STREAM_BATCH) &&
                 ((now_ms - batch.rows[0].ts_ms) < IR_MQTT_FLUSH_MS))
        {
            b_more = false;
        }
        else
        {
            length = ir_pack(payload, &batch, now_ms);
            if (comms_publish(COMMS_TOPIC_IR_STREAM, payload, length))
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

/* ---- rover/actions: type 8 ---- */
/* Publish up to ACTION_PACKETS_PER_CALL queued events.  Returns true when
 * the queue was drained, so a snapshot cannot overtake a transition. */
static bool action_publish_events (uint8_t * p_payload)
{
    robot_action_event_t event   = {0};
    uint32_t             sent    = 0u;
    uint32_t             length  = 0u;
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
            length = action_pack(p_payload, &event, false, event.ts_ms);
            if (comms_publish(COMMS_TOPIC_ACTIONS, p_payload, length))
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
static void action_publish_snapshot (uint8_t * p_payload, uint32_t now_ms)
{
    static uint32_t      last_snapshot_ms = 0u;
    static bool          b_have_snapshot  = false;
    robot_action_event_t event            = {0};
    uint32_t             length           = 0u;

    if ((!b_have_snapshot ||
         ((now_ms - last_snapshot_ms) >= ACTION_SNAPSHOT_MS)) &&
        robot_action_snapshot_if_empty(&event))
    {
        length = action_pack(p_payload, &event, true, now_ms);
        if (comms_publish(COMMS_TOPIC_ACTIONS, p_payload, length))
        {
            last_snapshot_ms = now_ms;
            b_have_snapshot  = true;
        }
    }
}

/* Action event (b_state false) or current state (true): 16 bytes. */
static uint32_t action_pack (uint8_t *                    p_payload,
                             const robot_action_event_t * p_event, bool b_state,
                             uint32_t ts_ms)
{
    robot_packed_writer_t writer = {NULL, 0u, 0u, false};

    diag_writer_start(&writer, p_payload, ROBOT_WIRE_TYPE_ACTION);
    robot_packed_put(&writer, b_state ? 1u : 0u, DIAG_BITS_FLAG);
    robot_packed_put(&writer, p_event->seq, DIAG_BITS_U32);
    robot_packed_put(&writer, ts_ms, DIAG_BITS_U32);
    robot_packed_put(&writer, p_event->dropped, DIAG_BITS_SAT16);
    robot_packed_put(&writer, b_state ? 0u : p_event->action, DIAG_BITS_SMALL2);
    robot_packed_put(&writer, p_event->direction, DIAG_BITS_SMALL2);
    robot_packed_put(&writer, p_event->source, DIAG_BITS_SMALL2);
    robot_packed_put_signed(&writer, p_event->left_pwm, DIAG_BITS_PWM);
    robot_packed_put_signed(&writer, p_event->right_pwm, DIAG_BITS_PWM);

    return robot_packed_length(&writer);
}

/* ---- rover/steering: type 9, 58 bytes ---- */
static uint32_t steering_pack (uint8_t *                      p_payload,
                               const robot_steering_event_t * p_event,
                               bool b_queued, uint32_t now)
{
    const robot_steering_input_t * p_in   = &p_event->input;
    robot_packed_writer_t          writer = {NULL, 0u, 0u, false};
    uint32_t                       index  = 0u;

    diag_writer_start(&writer, p_payload, ROBOT_WIRE_TYPE_STEERING);
    robot_packed_put(&writer, b_queued ? 0u : 1u, DIAG_BITS_FLAG); /* state */
    robot_packed_put(&writer, robot_diagnostics_session(), DIAG_BITS_U32);
    robot_packed_put(&writer, p_event->seq, DIAG_BITS_U32);
    robot_packed_put(&writer, p_event->ts_ms, DIAG_BITS_U32);
    robot_packed_put(&writer, b_queued ? diag_lag(now, p_event->ts_ms) : 0u,
                     DIAG_BITS_SAT16);
    robot_packed_put(&writer, p_event->dropped, DIAG_BITS_SAT16);
    robot_packed_put(&writer, p_event->owner, DIAG_BITS_OWNER);
    robot_packed_put(&writer, p_in->phase, DIAG_BITS_PHASE);
    robot_packed_put(&writer, p_event->direction, DIAG_BITS_SMALL2);
    for (index = 0u; index < 2u; index++)
    {
        robot_packed_put_signed(&writer, p_event->requested[index],
                                DIAG_BITS_PWM);
    }
    for (index = 0u; index < 2u; index++)
    {
        robot_packed_put_signed(&writer, p_event->applied[index],
                                DIAG_BITS_PWM);
    }
    robot_packed_put(&writer, p_event->encoder_ms, DIAG_BITS_U32);
    for (index = 0u; index < 2u; index++)
    {
        robot_packed_put_signed(&writer, p_event->distance_x10[index],
                                DIAG_BITS_DIST);
    }
    for (index = 0u; index < 2u; index++)
    {
        robot_packed_put_signed(&writer, p_event->speed_x10[index],
                                DIAG_BITS_PWM);
    }
    robot_packed_put(&writer, p_event->faults, DIAG_BITS_SMALL2);
    robot_packed_put(&writer, p_in->ts_ms, DIAG_BITS_U32);
    for (index = 0u; index < 2u; index++)
    {
        robot_packed_put(&writer, p_in->raw[index], DIAG_BITS_RAW);
    }
    for (index = 0u; index < 2u; index++)
    {
        robot_packed_put(&writer, p_in->norm[index], DIAG_BITS_NORM);
    }
    robot_packed_put(&writer, p_in->threshold, DIAG_BITS_NORM);
    for (index = 0u; index < 4u; index++)
    {
        robot_packed_put(&writer, p_in->calibration[index], DIAG_BITS_RAW);
    }
    robot_packed_put(&writer, p_in->dark, DIAG_BITS_SMALL2);
    robot_packed_put(&writer, p_event->has_input ? 1u : 0u, DIAG_BITS_FLAG);

    return robot_packed_length(&writer);
}

/* ---- rover/barcode: type 10 ---- */
/* Common header, then the part that depends on the packet type. */
static uint32_t barcode_pack (uint8_t *                      p_payload,
                              const robot_barcode_packet_t * p_packet,
                              uint32_t                       now)
{
    robot_packed_writer_t writer = {NULL, 0u, 0u, false};
    uint32_t              index  = 0u;

    diag_writer_start(&writer, p_payload, ROBOT_WIRE_TYPE_BARCODE);
    robot_packed_put(&writer, p_packet->type, DIAG_BITS_PTYPE);
    robot_packed_put(&writer, robot_diagnostics_session(), DIAG_BITS_U32);
    robot_packed_put(&writer, p_packet->seq, DIAG_BITS_U32);
    robot_packed_put(&writer, p_packet->scan, DIAG_BITS_U32);
    robot_packed_put(&writer, p_packet->ts_ms, DIAG_BITS_U32);
    robot_packed_put(&writer, p_packet->dropped, DIAG_BITS_SAT16);
    robot_packed_put(&writer, diag_lag(now, p_packet->ts_ms), DIAG_BITS_SAT16);

    if (BC_PACKET_START == p_packet->type)
    {
        robot_packed_put_signed(&writer, p_packet->leading, DIAG_BITS_DIST);
        barcode_put_config(&writer, p_packet);
    }
    else if ((BC_PACKET_EDGES == p_packet->type) ||
             (BC_PACKET_SAMPLES == p_packet->type))
    {
        barcode_put_rows(&writer, p_packet);
    }
    else
    {
        barcode_put_summary(&writer, p_packet);
    }

    if (BC_PACKET_STATE == p_packet->type)
    {
        barcode_put_config(&writer, p_packet);
        /* Side-sensor raw, normalized, black, then four signed distances. */
        robot_packed_put(&writer, (uint32_t) p_packet->readiness[0],
                         DIAG_BITS_RAW);
        robot_packed_put(&writer, (uint32_t) p_packet->readiness[1],
                         DIAG_BITS_NORM);
        robot_packed_put(&writer, (uint32_t) p_packet->readiness[2],
                         DIAG_BITS_FLAG);
        for (index = 3u; index < BARCODE_READINESS_COUNT; index++)
        {
            robot_packed_put_signed(&writer, p_packet->readiness[index],
                                    DIAG_BITS_DIST);
        }
        robot_packed_put(&writer, p_packet->rejected_steps, DIAG_BITS_SAT16);
    }

    return robot_packed_length(&writer);
}

/* The eleven reader settings (robot_barcode_trace_configure order). */
static void barcode_put_config (robot_packed_writer_t *        p_writer,
                                const robot_barcode_packet_t * p_packet)
{
    /* Raw endpoints, inversion and thresholds are never negative; the casts
     * keep the bit pattern of values already inside their fields. */
    robot_packed_put(p_writer, (uint32_t) p_packet->config[0], DIAG_BITS_RAW);
    robot_packed_put(p_writer, (uint32_t) p_packet->config[1], DIAG_BITS_RAW);
    robot_packed_put(p_writer, (uint32_t) p_packet->config[2], DIAG_BITS_FLAG);
    robot_packed_put(p_writer, (uint32_t) p_packet->config[3], DIAG_BITS_NORM);
    robot_packed_put(p_writer, (uint32_t) p_packet->config[4], DIAG_BITS_NORM);
    /* Minimum element x100: 0 (unconfigured) up to 100000 (1000 mm). */
    robot_packed_put(p_writer, (uint32_t) p_packet->config[5],
                     DIAG_BITS_ELEMENT);
    robot_packed_put_signed(p_writer, p_packet->config[6], DIAG_BITS_OFFSET);
    robot_packed_put_signed(p_writer, p_packet->config[7], DIAG_BITS_GEOM);
    robot_packed_put_signed(p_writer, p_packet->config[8], DIAG_BITS_GEOM);
    robot_packed_put_signed(p_writer, p_packet->config[9], DIAG_BITS_GEOM);
    robot_packed_put_signed(p_writer, p_packet->config[10], DIAG_BITS_GEOM);
}

/* Edge or sample rows; row numbers are the offset plus the row index. */
static void barcode_put_rows (robot_packed_writer_t *        p_writer,
                              const robot_barcode_packet_t * p_packet)
{
    uint32_t row = 0u;

    robot_packed_put(p_writer, p_packet->count, DIAG_BITS_COUNT);
    if (BC_PACKET_EDGES == p_packet->type)
    {
        robot_packed_put(p_writer, p_packet->offset, DIAG_BITS_EDGE);
        for (row = 0u; row < p_packet->count; ++row)
        {
            const robot_barcode_edge_t * p_edge = &p_packet->edges[row];

            robot_packed_put(p_writer, p_edge->ts_ms, DIAG_BITS_U32);
            robot_packed_put_signed(p_writer, p_edge->width_x100,
                                    DIAG_BITS_DIST);
            robot_packed_put_signed(p_writer, p_edge->left_x10, DIAG_BITS_DIST);
            robot_packed_put_signed(p_writer, p_edge->right_x10,
                                    DIAG_BITS_DIST);
        }
    }
    else
    {
        robot_packed_put(p_writer, p_packet->offset, DIAG_BITS_SAMPLE);
        for (row = 0u; row < p_packet->count; ++row)
        {
            const robot_barcode_sample_t * p_sample = &p_packet->samples[row];

            robot_packed_put(p_writer, p_sample->ts_ms, DIAG_BITS_U32);
            robot_packed_put(p_writer, p_sample->raw, DIAG_BITS_RAW);
            robot_packed_put(p_writer, p_sample->normalized, DIAG_BITS_NORM);
            robot_packed_put_signed(p_writer, p_sample->left_x10,
                                    DIAG_BITS_DIST);
            robot_packed_put_signed(p_writer, p_sample->right_x10,
                                    DIAG_BITS_DIST);
            robot_packed_put_signed(p_writer, p_sample->position_x100,
                                    DIAG_BITS_DIST);
            robot_packed_put(p_writer, p_sample->black, DIAG_BITS_FLAG);
        }
    }
}

/* Scan summary for progress, end and state packets. */
static void barcode_put_summary (robot_packed_writer_t *        p_writer,
                                 const robot_barcode_packet_t * p_packet)
{
    robot_packed_put(p_writer, p_packet->state, DIAG_BITS_STATE);
    robot_packed_put(p_writer, p_packet->total_edges, DIAG_BITS_EDGES);
    robot_packed_put(p_writer, p_packet->total_samples, DIAG_BITS_SAMPLES);
    robot_packed_put(p_writer, p_packet->sample_drops, DIAG_BITS_SAT16);
    robot_packed_put_signed(p_writer, p_packet->partial, DIAG_BITS_DIST);
    robot_packed_put(p_writer, p_packet->blocked, DIAG_BITS_BLOCKED);

    if (BC_PACKET_END == p_packet->type)
    {
        robot_packed_put(p_writer, p_packet->reason, DIAG_BITS_REASON);
        robot_packed_put(p_writer, p_packet->format, DIAG_BITS_FORMAT);
        robot_packed_put(p_writer, p_packet->symbol, DIAG_BITS_SYMBOL);
        robot_packed_put(p_writer, p_packet->reversed, DIAG_BITS_FLAG);
        robot_packed_put_signed(p_writer, p_packet->leading, DIAG_BITS_DIST);
        robot_packed_put_signed(p_writer, p_packet->trailing, DIAG_BITS_DIST);
        robot_packed_put_signed(p_writer, p_packet->narrow, DIAG_BITS_DIST);
        robot_packed_put_signed(p_writer, p_packet->ratio, DIAG_BITS_DIST);
        robot_packed_put_signed(p_writer, p_packet->max_gap, DIAG_BITS_DIST);
    }
}

/* ---- rover/ir: type 11, 75 bytes for four rows ---- */
/* Rows are consecutive samples: the first carries its full sequence number
 * and time, later rows only the time step since the row before. */
static uint32_t ir_pack (uint8_t *                       p_payload,
                         const robot_ir_stream_batch_t * p_batch, uint32_t now)
{
    const robot_ir_stream_sample_t * p_last =
        &p_batch->rows[p_batch->count - 1u];
    robot_packed_writer_t writer = {NULL, 0u, 0u, false};
    uint32_t              row    = 0u;

    diag_writer_start(&writer, p_payload, ROBOT_WIRE_TYPE_IR);
    robot_packed_put(&writer, robot_diagnostics_session(), DIAG_BITS_U32);
    robot_packed_put(&writer, diag_lag(now, p_last->ts_ms), DIAG_BITS_SAT16);
    robot_packed_put(&writer, p_batch->dropped, DIAG_BITS_SAT16);
    robot_packed_put(&writer, p_batch->count, DIAG_BITS_COUNT);
    robot_packed_put(&writer, p_batch->rows[0].seq, DIAG_BITS_U32);
    robot_packed_put(&writer, p_batch->rows[0].ts_ms, DIAG_BITS_U32);
    for (row = 0u; row < p_batch->count; ++row)
    {
        const robot_ir_stream_sample_t * p_row = &p_batch->rows[row];

        if (0u != row)
        {
            robot_packed_put(&writer,
                             p_row->ts_ms - p_batch->rows[row - 1u].ts_ms,
                             DIAG_BITS_DT);
        }
        robot_packed_put(&writer, p_row->raw, DIAG_BITS_RAW);
        robot_packed_put(&writer, p_row->normalized, DIAG_BITS_NORM);
        robot_packed_put(&writer, p_row->black, DIAG_BITS_FLAG);
        robot_packed_put_signed(&writer, p_row->left_x10, DIAG_BITS_DIST);
        robot_packed_put_signed(&writer, p_row->right_x10, DIAG_BITS_DIST);
        robot_packed_put(&writer, p_row->blocked, DIAG_BITS_BLOCKED);
        robot_packed_put(&writer, p_row->minimum, DIAG_BITS_RAW);
        robot_packed_put(&writer, p_row->maximum, DIAG_BITS_RAW);
        robot_packed_put(&writer, p_row->inverted, DIAG_BITS_FLAG);
    }
    /* Optional trailing field: older firmware omitted the nominal cadence. */
    robot_packed_put(&writer, ROBOT_IR_STREAM_PERIOD_MS, 8u);

    return robot_packed_length(&writer);
}

/* Age of a record in ms.  "now" is read before the record is peeked, so a
 * higher-priority task can stamp one just after it: that record is 0 ms
 * old, not 4 billion. */
static uint32_t diag_lag (uint32_t now, uint32_t ts_ms)
{
    uint32_t lag = now - ts_ms;

    return (lag > (UINT32_MAX / 2u)) ? 0u : lag;
}

/* Start a diagnostic message: clear the buffer and write its header byte. */
static void diag_writer_start (robot_packed_writer_t * p_writer,
                               uint8_t * p_payload, uint32_t type)
{
    robot_packed_writer_init(p_writer, p_payload, ROBOT_WIRE_DIAG_MESSAGE_MAX);
    robot_packed_put(p_writer, robot_wire_header(type), DIAG_BITS_HEADER);
}

/*** end of file ***/

