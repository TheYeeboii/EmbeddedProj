/** @file robot_barcode_trace.h
 *
 * @brief Capture of barcode scans (samples, element edges and the decode
 *        result) for the dashboard barcode view, split into small packets.
 */
#ifndef ROBOT_BARCODE_TRACE_H
#define ROBOT_BARCODE_TRACE_H

#include <stdbool.h>
#include <stdint.h>
#include "buddy3_barcode.h"

#define BARCODE_TRACE_FRAMES  2u
#define BARCODE_TRACE_SAMPLES 512u
#define BARCODE_TRACE_BATCH   4u

/* Lengths of the configuration and readiness arrays sent to the dashboard. */
#define BARCODE_CONFIG_COUNT    11u
#define BARCODE_READINESS_COUNT 7u

/* Reasons the reader is suspended (observation.blocked bit mask). */
#define BARCODE_BLOCKED_STOPPED    1u  /* Tracking stopped or unconfigured. */
#define BARCODE_BLOCKED_ENCODER    2u  /* Encoder fault. */
#define BARCODE_BLOCKED_TURN       4u  /* Excessive heading change. */
#define BARCODE_BLOCKED_REVERSE    8u  /* Reverse wheel travel. */
#define BARCODE_BLOCKED_JUNCTION   16u /* Junction or recovery phase. */
#define BARCODE_BLOCKED_SAMPLE_GAP 32u /* Sample spacing too large. */

/** Reader state reported to the dashboard (barcode-model.js "states"). */
typedef enum
{
    BARCODE_STATE_OFF,
    BARCODE_STATE_WAITING,
    BARCODE_STATE_CAPTURING,
    BARCODE_STATE_TRAILING,
    BARCODE_STATE_ACCEPTED,
    BARCODE_STATE_REJECTED,
    BARCODE_STATE_SUSPENDED
} robot_barcode_state_t;

/** One 10 ms reader input, as seen by the trace. */
typedef struct
{
    uint32_t ts_ms;
    int32_t  left_x10;
    int32_t  right_x10;
    uint16_t raw;
    uint16_t normalized;
    uint8_t  black;
    uint8_t  blocked;
    int32_t  travel_x100;
} robot_barcode_observation_t;

/** One captured sample inside a scan. */
typedef struct
{
    uint32_t ts_ms;
    int32_t  left_x10;
    int32_t  right_x10;
    int32_t  position_x100;
    uint16_t raw;
    uint16_t normalized;
    uint8_t  black;
} robot_barcode_sample_t;

/** One element edge inside a scan. */
typedef struct
{
    uint32_t ts_ms;
    int32_t  width_x100;
    int32_t  left_x10;
    int32_t  right_x10;
} robot_barcode_edge_t;

/** Packet type (robot_barcode_packet_t.type). */
typedef enum
{
    BC_PACKET_START,
    BC_PACKET_EDGES,
    BC_PACKET_SAMPLES,
    BC_PACKET_PROGRESS,
    BC_PACKET_END,
    BC_PACKET_STATE
} robot_barcode_packet_type_t;

/** One packet ready for JSON formatting. */
typedef struct
{
    uint32_t               scan;
    uint32_t               seq;
    uint32_t               ts_ms;
    uint32_t               dropped;
    uint32_t               sample_drops;
    uint32_t               rejected_steps;
    uint16_t               offset;
    uint16_t               total_samples;
    uint8_t                type;
    uint8_t                count;
    uint8_t                state;
    uint8_t                reason;
    uint8_t                format;
    uint8_t                symbol;
    uint8_t                reversed;
    uint8_t                blocked;
    uint8_t                total_edges;
    int32_t                readiness[BARCODE_READINESS_COUNT];
    int32_t                config[BARCODE_CONFIG_COUNT];
    int32_t                leading;
    int32_t                trailing;
    int32_t                partial;
    int32_t                narrow;
    int32_t                ratio;
    int32_t                max_gap;
    robot_barcode_edge_t   edges[BARCODE_TRACE_BATCH];
    robot_barcode_sample_t samples[BARCODE_TRACE_BATCH];
} robot_barcode_packet_t;

/**
 * @brief Scale a float to a saturated integer (NaN gives 0).
 * @param value Value to scale.
 * @param scale Multiplier, e.g. 100 for hundredths.
 * @return value * scale, clamped to +/-INT32_MAX.
 */
int32_t robot_barcode_scaled (float value, float scale);

/**
 * @brief Store the reader configuration sent with every scan and enable it.
 * @param config BARCODE_CONFIG_COUNT scaled settings.
 */
void robot_barcode_trace_configure (const int32_t config[BARCODE_CONFIG_COUNT]);

/**
 * @brief Enable or disable the reader trace; disabling clears the config.
 * @param enabled True to enable.
 */
void robot_barcode_trace_enable (bool enabled);

/**
 * @brief End the scan in progress as interrupted.
 * @param now Current time in milliseconds.
 */
void robot_barcode_trace_cancel (uint32_t now);

/**
 * @brief Record one reader step.  Call after the pure reader, passing the
 *        pre-step active and unfinished-width information.  Copies only new
 *        authoritative edges and never infers a missed transition.
 * @param p_decoder         Reader state after the step.
 * @param p_observation     Input of this step.
 * @param was_active        Reader active flag before the step.
 * @param unfinished_before Unfinished width before the step.
 * @param p_result          Event of this step, or NULL if none.
 */
void robot_barcode_trace_observe (
    const buddy3_barcode_decoder_t * p_decoder,
    const robot_barcode_observation_t * p_observation, bool was_active,
    float unfinished_before, const buddy3_barcode_event_t * p_result);

/**
 * @brief Build the next packet to publish without consuming it.
 * @param p_packet Receives the packet.
 * @param now      Current time in milliseconds.
 * @return True when a packet is ready.
 */
bool robot_barcode_packet_peek (robot_barcode_packet_t * p_packet,
                                uint32_t now);

/**
 * @brief Advance the transport cursors after a successful publish.  A stale
 *        packet (from an overwritten scan or an old peek) is ignored.
 * @param p_packet Packet returned by robot_barcode_packet_peek().
 */
void robot_barcode_packet_ack (const robot_barcode_packet_t * p_packet);

#endif /* ROBOT_BARCODE_TRACE_H */

/*** end of file ***/

