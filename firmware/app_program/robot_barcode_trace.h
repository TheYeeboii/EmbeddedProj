#ifndef ROBOT_BARCODE_TRACE_H
#define ROBOT_BARCODE_TRACE_H
#include <stdbool.h>
#include <stdint.h>
#include "buddy3_barcode.h"

#define BARCODE_TRACE_FRAMES  2u
#define BARCODE_TRACE_SAMPLES 512u
#define BARCODE_TRACE_BATCH   4u
typedef struct
{
    uint32_t ts_ms;
    int32_t  left_x10, right_x10;
    uint16_t raw, normalized;
    uint8_t  black, blocked;
    int32_t  travel_x100;
} robot_barcode_observation_t;
typedef struct
{
    uint32_t ts_ms;
    int32_t  left_x10, right_x10, position_x100;
    uint16_t raw, normalized;
    uint8_t  black;
} robot_barcode_sample_t;
typedef struct
{
    uint32_t ts_ms;
    int32_t  width_x100, left_x10, right_x10;
} robot_barcode_edge_t;
enum
{
    BC_PACKET_START,
    BC_PACKET_EDGES,
    BC_PACKET_SAMPLES,
    BC_PACKET_PROGRESS,
    BC_PACKET_END,
    BC_PACKET_STATE
};
typedef struct
{
    uint32_t scan, seq, ts_ms, dropped, sample_drops, rejected_steps;
    uint16_t offset, total_samples;
    uint8_t  type, count, state, reason, format, symbol, reversed, blocked,
        total_edges;
    int32_t readiness[7];
    int32_t config[11], leading, trailing, partial, narrow, ratio, max_gap;
    robot_barcode_edge_t   edges[BARCODE_TRACE_BATCH];
    robot_barcode_sample_t samples[BARCODE_TRACE_BATCH];
} robot_barcode_packet_t;

int32_t robot_barcode_scaled (float value, float scale);
void    robot_barcode_trace_configure (const int32_t config[11]);
void    robot_barcode_trace_enable (bool enabled);
void    robot_barcode_trace_cancel (uint32_t now);
/* Call after the pure reader, retaining pre-step active/unfinished information.
 * Copies only new authoritative edges; never infers a missed transition. */
void robot_barcode_trace_observe (
    const buddy3_barcode_decoder_t *    decoder,
    const robot_barcode_observation_t * observation, bool was_active,
    float unfinished_before, const buddy3_barcode_event_t * result);
bool robot_barcode_packet_peek (robot_barcode_packet_t * packet, uint32_t now);
void robot_barcode_packet_ack (const robot_barcode_packet_t * packet);
void robot_barcode_publish (uint32_t now);
#endif

/*** end of file ***/

