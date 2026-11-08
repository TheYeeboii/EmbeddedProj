/** @file robot_barcode_mqtt.c
 *
 * @brief Publishes barcode scan packets (robot_barcode_trace) as JSON on the
 *        rover/barcode MQTT topic.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "comms.h"
#include "robot_barcode_trace.h"
#include "robot_barcode_mqtt.h"
#include "robot_steering_trace.h"

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

/*** end of file ***/

