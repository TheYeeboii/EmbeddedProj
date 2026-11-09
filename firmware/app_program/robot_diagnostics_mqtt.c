/** @file robot_diagnostics_mqtt.c
 *
 * @brief Buddy 1: publishes the four diagnostic streams as JSON.
 *
 *   rover/actions   Buddy 2 motor actions        (robot_action_log)
 *   rover/steering  steering decisions           (robot_steering_trace)
 *   rover/barcode   barcode scans                (robot_barcode_trace)
 *   rover/ir        continuous IR samples        (robot_ir_stream)
 *
 * Each publisher sends at most two queued records per call, from the
 * controller task, and stops early when the outbox is full or offline.
 * Built only with DIAGNOSTICS=1 (pico_rp2040.mk); the default build links
 * robot_diagnostics_off.c instead.  The streams are tuning tools, so they
 * stay JSON text (docs/MQTT_BINARY_FORMAT.md).
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "comms.h"
#include "mqtt_config.h"
#include "robot_action_log.h"
#include "robot_barcode_trace.h"
#include "robot_diagnostics_mqtt.h"
#include "robot_ir_stream.h"
#include "robot_steering_trace.h"

/* ---- rover/actions: Buddy 2 motor actions (robot_action_log) ---- */
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

/* ---- rover/steering: steering decisions (robot_steering_trace) ---- */
#define STEERING_PACKETS_PER_CALL 2u
#define STEERING_SNAPSHOT_MS      1000u

static void steering_format (char *                         p_payload,
                             const robot_steering_event_t * p_event,
                             bool b_queued, uint32_t now);

/* ---- rover/barcode: barcode scans (robot_barcode_trace) ---- */
#define BARCODE_PACKETS_PER_CALL 2u

/* tm_sprintf takes UB pointers; every value below is cast to the UINT/INT
 * its %u/%d conversion expects.  Up to four bounded rows per message; the
 * host test exercises the integer limits. */

static INT  barcode_append_config (char *                         p_out,
                                   const robot_barcode_packet_t * p_packet);
static INT  barcode_append_rows (char *                         p_out,
                                 const robot_barcode_packet_t * p_packet);
static INT  barcode_append_summary (char *                         p_out,
                                    const robot_barcode_packet_t * p_packet);
static void barcode_format (char *                         p_payload,
                            const robot_barcode_packet_t * p_packet,
                            uint32_t                       now);

/* Names indexed by robot_barcode_packet_type_t. */
static const char * const g_type_names[] = {
    "barcode_start",    "barcode_edges", "barcode_samples",
    "barcode_progress", "barcode_end",   "barcode_state"};

/* ---- rover/ir: continuous IR samples (robot_ir_stream) ---- */
/* At most this many packets per call, so the network task is not starved. */
#define IR_MQTT_PACKETS_PER_CALL 2u
/* A short final batch is flushed once its oldest row is this old. */
#define IR_MQTT_FLUSH_MS 40u

static INT ir_mqtt_format (char *                          p_payload,
                           const robot_ir_stream_batch_t * p_batch,
                           uint32_t                        now);

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

void robot_barcode_publish (uint32_t now_ms)
{
    static char            payload[COMMS_PAYLOAD_MAX];
    robot_barcode_packet_t packet = {0};
    uint32_t               sent   = 0u;
    bool b_more = (COMMS_LINK_MQTT_UP == comms_get_link_state());

    for (sent = 0u; b_more && (sent < BARCODE_PACKETS_PER_CALL); ++sent)
    {
        if (!robot_barcode_packet_peek(&packet, now_ms))
        {
            b_more = false;
        }
        else
        {
            barcode_format(payload, &packet, now_ms);
            if (comms_publish_text(COMMS_TOPIC_BARCODE, payload))
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

/* ---- rover/actions: Buddy 2 motor actions (robot_action_log) ---- */
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
            if (comms_publish_text(COMMS_TOPIC_ACTIONS, p_payload))
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
        if (comms_publish_text(COMMS_TOPIC_ACTIONS, p_payload))
        {
            last_snapshot_ms = now_ms;
            b_have_snapshot  = true;
        }
    }
}

/* ---- rover/steering: steering decisions (robot_steering_trace) ---- */
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

/* ---- rover/barcode: barcode scans (robot_barcode_trace) ---- */
/* Build the whole JSON object for one packet. */
static void barcode_format (char *                         p_payload,
                            const robot_barcode_packet_t * p_packet,
                            uint32_t                       now)
{
    INT length = tm_sprintf(
        (UB *) p_payload,
        (UB *) "{\"v\":1,\"type\":\"%s\",\"boot\":%u,\"seq\":%u,"
               "\"scan\":%u,\"ts_ms\":%u,\"dropped\":%u,\"lag_ms\":%u",
        g_type_names[p_packet->type], (UINT) robot_diagnostics_session(),
        (UINT) p_packet->seq, (UINT) p_packet->scan, (UINT) p_packet->ts_ms,
        (UINT) p_packet->dropped, (UINT) (now - p_packet->ts_ms));

    if (BC_PACKET_START == p_packet->type)
    {
        length += tm_sprintf((UB *) p_payload + length,
                             (UB *) ",\"leading\":%d", (INT) p_packet->leading);
        length += barcode_append_config(p_payload + length, p_packet);
    }
    else if ((BC_PACKET_EDGES == p_packet->type) ||
             (BC_PACKET_SAMPLES == p_packet->type))
    {
        length += barcode_append_rows(p_payload + length, p_packet);
    }
    else
    {
        length += barcode_append_summary(p_payload + length, p_packet);
    }

    if (BC_PACKET_STATE == p_packet->type)
    {
        length += barcode_append_config(p_payload + length, p_packet);
        length += tm_sprintf(
            (UB *) p_payload + length,
            (UB *) ",\"readiness\":[%d,%d,%d,%d,%d,%d,%d,%u]",
            (INT) p_packet->readiness[0], (INT) p_packet->readiness[1],
            (INT) p_packet->readiness[2], (INT) p_packet->readiness[3],
            (INT) p_packet->readiness[4], (INT) p_packet->readiness[5],
            (INT) p_packet->readiness[6], (UINT) p_packet->rejected_steps);
    }
    (void) tm_sprintf((UB *) p_payload + length, (UB *) "}");
}

/* ,"config":[...] with the eleven reader settings. */
static INT barcode_append_config (char *                         p_out,
                                  const robot_barcode_packet_t * p_packet)
{
    return tm_sprintf((UB *) p_out,
                      (UB *) ",\"config\":[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]",
                      (INT) p_packet->config[0], (INT) p_packet->config[1],
                      (INT) p_packet->config[2], (INT) p_packet->config[3],
                      (INT) p_packet->config[4], (INT) p_packet->config[5],
                      (INT) p_packet->config[6], (INT) p_packet->config[7],
                      (INT) p_packet->config[8], (INT) p_packet->config[9],
                      (INT) p_packet->config[10]);
}

/* ,"rows":[...] with the edges or samples of the packet. */
static INT barcode_append_rows (char *                         p_out,
                                const robot_barcode_packet_t * p_packet)
{
    INT      length = tm_sprintf((UB *) p_out, (UB *) ",\"rows\":[");
    uint32_t row    = 0u;

    for (row = 0u; row < p_packet->count; ++row)
    {
        const char * p_comma = (0u != row) ? "," : "";

        if (BC_PACKET_EDGES == p_packet->type)
        {
            const robot_barcode_edge_t * p_edge = &p_packet->edges[row];

            length +=
                tm_sprintf((UB *) p_out + length, (UB *) "%s[%u,%u,%d,%d,%d]",
                           p_comma, (UINT) (p_packet->offset + row),
                           (UINT) p_edge->ts_ms, (INT) p_edge->width_x100,
                           (INT) p_edge->left_x10, (INT) p_edge->right_x10);
        }
        else
        {
            const robot_barcode_sample_t * p_sample = &p_packet->samples[row];

            length += tm_sprintf(
                (UB *) p_out + length, (UB *) "%s[%u,%u,%u,%u,%d,%d,%d,%u]",
                p_comma, (UINT) (p_packet->offset + row),
                (UINT) p_sample->ts_ms, (UINT) p_sample->raw,
                (UINT) p_sample->normalized, (INT) p_sample->left_x10,
                (INT) p_sample->right_x10, (INT) p_sample->position_x100,
                (UINT) p_sample->black);
        }
    }
    length += tm_sprintf((UB *) p_out + length, (UB *) "]");

    return length;
}

/* Scan summary for progress, end and state packets. */
static INT barcode_append_summary (char *                         p_out,
                                   const robot_barcode_packet_t * p_packet)
{
    INT length = tm_sprintf(
        (UB *) p_out,
        (UB *) ",\"state\":%u,\"count\":%u,\"samples\":%u,"
               "\"sample_drops\":%u,\"partial\":%d,\"blocked\":%u",
        (UINT) p_packet->state, (UINT) p_packet->total_edges,
        (UINT) p_packet->total_samples, (UINT) p_packet->sample_drops,
        (INT) p_packet->partial, (UINT) p_packet->blocked);

    if (BC_PACKET_END == p_packet->type)
    {
        length +=
            tm_sprintf((UB *) p_out + length,
                       (UB *) ",\"reason\":%u,\"format\":%u,\"symbol\":%u,"
                              "\"reversed\":%u,\"leading\":%d,\"trailing\":%d,"
                              "\"narrow\":%d,\"ratio\":%d,\"max_gap\":%d",
                       (UINT) p_packet->reason, (UINT) p_packet->format,
                       (UINT) p_packet->symbol, (UINT) p_packet->reversed,
                       (INT) p_packet->leading, (INT) p_packet->trailing,
                       (INT) p_packet->narrow, (INT) p_packet->ratio,
                       (INT) p_packet->max_gap);
    }

    return length;
}

/* ---- rover/ir: continuous IR samples (robot_ir_stream) ---- */
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

