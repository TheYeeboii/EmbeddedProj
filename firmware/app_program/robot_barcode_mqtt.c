#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "comms.h"
#include "robot_barcode_trace.h"
#include "robot_steering_trace.h"

static INT barcode_append_config(char *out, const robot_barcode_packet_t *p)
{
    return tm_sprintf((UB *)out, (UB *)
        ",\"config\":[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]",
        (INT)p->config[0], (INT)p->config[1], (INT)p->config[2], (INT)p->config[3],
        (INT)p->config[4], (INT)p->config[5], (INT)p->config[6], (INT)p->config[7],
        (INT)p->config[8], (INT)p->config[9], (INT)p->config[10]);
}

void robot_barcode_publish(uint32_t now)
{
    static char payload[COMMS_PAYLOAD_MAX];
    static const char *const types[] = {"barcode_start", "barcode_edges", "barcode_samples", "barcode_progress", "barcode_end", "barcode_state"};
    robot_barcode_packet_t p;
    unsigned sent, i;
    if (comms_get_link_state() != COMMS_LINK_MQTT_UP) return;
    for (sent = 0; sent < 2; ++sent) {
        INT length;
        if (!robot_barcode_packet_peek(&p,now)) return;
        /* Up to four bounded rows per message; tested at integer limits. */
        length = tm_sprintf((UB *)payload, (UB *)
            "{\"v\":1,\"type\":\"%s\",\"boot\":%u,\"seq\":%u,\"scan\":%u,\"ts_ms\":%u,\"dropped\":%u,\"lag_ms\":%u",
            types[p.type], (UINT)robot_diagnostics_session(), (UINT)p.seq, (UINT)p.scan,
            (UINT)p.ts_ms, (UINT)p.dropped, (UINT)(now - p.ts_ms));
        if (p.type == BC_PACKET_START) {
            length += tm_sprintf((UB *)payload+length, (UB *)",\"leading\":%d", (INT)p.leading);
            length += barcode_append_config(payload+length,&p);
        } else if (p.type == BC_PACKET_EDGES || p.type == BC_PACKET_SAMPLES) {
            length += tm_sprintf((UB *)payload+length, (UB *)",\"rows\":[");
            for (i = 0; i < p.count; ++i) {
                if (p.type == BC_PACKET_EDGES) {
                    robot_barcode_edge_t *e = &p.edges[i];
                    length += tm_sprintf((UB *)payload+length, (UB *)"%s[%u,%u,%d,%d,%d]",
                        i ? "," : "", (UINT)(p.offset+i), (UINT)e->ts_ms,
                        (INT)e->width_x100, (INT)e->left_x10, (INT)e->right_x10);
                } else {
                    robot_barcode_sample_t *s = &p.samples[i];
                    length += tm_sprintf((UB *)payload+length, (UB *)"%s[%u,%u,%u,%u,%d,%d,%d,%u]",
                        i ? "," : "", (UINT)(p.offset+i), (UINT)s->ts_ms,
                        (UINT)s->raw, (UINT)s->normalized, (INT)s->left_x10,
                        (INT)s->right_x10, (INT)s->position_x100, (UINT)s->black);
                }
            }
            length += tm_sprintf((UB *)payload+length, (UB *)"]");
        } else {
            length += tm_sprintf((UB *)payload+length, (UB *)
                ",\"state\":%u,\"count\":%u,\"samples\":%u,\"sample_drops\":%u,\"partial\":%d,\"blocked\":%u",
                (UINT)p.state, (UINT)p.total_edges, (UINT)p.total_samples,
                (UINT)p.sample_drops, (INT)p.partial, (UINT)p.blocked);
            if (p.type == BC_PACKET_END) length += tm_sprintf((UB *)payload+length, (UB *)
                ",\"reason\":%u,\"format\":%u,\"symbol\":%u,\"reversed\":%u,\"leading\":%d,\"trailing\":%d,\"narrow\":%d,\"ratio\":%d,\"max_gap\":%d",
                (UINT)p.reason, (UINT)p.format, (UINT)p.symbol, (UINT)p.reversed,
                (INT)p.leading, (INT)p.trailing, (INT)p.narrow, (INT)p.ratio, (INT)p.max_gap);
        }
        if (p.type == BC_PACKET_STATE) {
            length += barcode_append_config(payload+length,&p);
            length += tm_sprintf((UB *)payload+length, (UB *)
                ",\"readiness\":[%d,%d,%d,%d,%d,%d,%d,%u]",
                (INT)p.readiness[0], (INT)p.readiness[1], (INT)p.readiness[2],
                (INT)p.readiness[3], (INT)p.readiness[4], (INT)p.readiness[5],
                (INT)p.readiness[6], (UINT)p.rejected_steps);
        }
        (void)tm_sprintf((UB *)payload+length,(UB *)"}");
        if (!comms_publish(COMMS_TOPIC_BARCODE, payload)) return;
        robot_barcode_packet_ack(&p);
    }
}
