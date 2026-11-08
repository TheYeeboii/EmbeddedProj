/** @file robot_barcode_trace.c
 *
 * @brief Capture of barcode scans (samples, element edges and the decode
 *        result) for the dashboard barcode view, split into small packets.
 */
#include <tk/tkernel.h>
#include "robot_barcode_trace.h"

/* robot_barcode_scaled() saturates just inside the int32_t range. */
#define SCALED_LIMIT 2147480000.0f
/* Progress packets at most every 100 ms; reader state once a second. */
#define PROGRESS_MS 100u
#define STATE_MS    1000u
/* A spacing failure stays visible this long before a scan starts. */
#define GAP_HOLD_MS 1000u
/* Readiness slots: raw, normalized, black, quiet, travel, limit, gap. */
#define READY_RAW       0u
#define READY_NORM      1u
#define READY_BLACK     2u
#define READY_QUIET     3u
#define READY_TRAVEL    4u
#define READY_STEP_MAX  5u
#define READY_GAP       6u
/* Config slot holding the minimum element width (x100). */
#define CONFIG_MIN_ELEMENT 5u
/* Values are sent in hundredths of a millimetre. */
#define X100 100.0f
#define NO_SLOT (-1)

/* Capture storage and transport cursors are private to this module. */
typedef struct
{
    uint32_t               id;
    uint32_t               started_ms;
    uint32_t               updated_ms;
    uint32_t               sample_drops;
    int32_t                config[BARCODE_CONFIG_COUNT];
    int32_t                leading_x100;
    int32_t                trailing_x100;
    int32_t                partial_x100;
    int32_t                narrow_x100;
    int32_t                ratio_x100;
    int32_t                max_gap_x100;
    robot_barcode_edge_t   edges[CODE39_FRAME_ELEMENTS];
    robot_barcode_sample_t samples[BARCODE_TRACE_SAMPLES];
    uint16_t               sample_count;
    uint8_t                count;
    uint8_t                state;
    uint8_t                reason;
    uint8_t                format;
    uint8_t                symbol;
    uint8_t                reversed;
    uint8_t                blocked;
    bool                   done;
    bool                   occupied;
    /* Publisher cursors: only advance after the mailbox accepts the packet. */
    uint16_t sent_samples;
    uint8_t  sent_edges;
    uint32_t sent_progress;
    bool     sent_start;
} robot_barcode_frame_t;

static robot_barcode_frame_t g_frames[BARCODE_TRACE_FRAMES];
static int32_t               g_active_slot = NO_SLOT;
static int32_t               g_configuration[BARCODE_CONFIG_COUNT];
static uint32_t              g_scan_sequence   = 0u;
static uint32_t              g_packet_sequence = 0u;
static uint32_t              g_dropped_scans   = 0u;
static uint32_t              g_last_state_ms   = 0u;
static bool                  g_b_enabled       = false;
static bool                  g_b_have_state    = false;
static uint8_t               g_reader_state    = BARCODE_STATE_OFF;
static uint8_t               g_reader_blocked  = 0u;
static int32_t               g_readiness[BARCODE_READINESS_COUNT];
static uint32_t              g_rejected_steps = 0u;
static uint32_t              g_last_gap_ms    = 0u;
static bool                  g_b_have_gap     = false;

static void trace_update_reader (const buddy3_barcode_decoder_t * p_decoder,
                                 const robot_barcode_observation_t * p_obs);
static void trace_start_frame (const buddy3_barcode_decoder_t * p_decoder,
                               const robot_barcode_observation_t * p_obs);
static void trace_record_step (robot_barcode_frame_t * p_frame,
                               const buddy3_barcode_decoder_t * p_decoder,
                               const robot_barcode_observation_t * p_obs,
                               float unfinished_before,
                               const buddy3_barcode_event_t * p_result);
static bool trace_state_packet (robot_barcode_packet_t * p_packet,
                                uint32_t now);
static bool trace_frame_packet (robot_barcode_packet_t * p_packet,
                                const robot_barcode_frame_t * p_frame);

int32_t robot_barcode_scaled (float value, float scale)
{
    float   scaled = value * scale;
    int32_t result = 0;

    /* NaN fails every comparison and falls through to 0. This avoids libc
     * headers that conflict with T-Kernel. */
    if (scaled > SCALED_LIMIT)
    {
        result = INT32_MAX;
    }
    else if (scaled < -SCALED_LIMIT)
    {
        result = -INT32_MAX;
    }
    else if (scaled >= -SCALED_LIMIT)
    {
        /* In range, so the conversion is defined. */
        result = (int32_t) scaled;
    }
    else
    {
        result = 0;
    }

    return result;
}

void robot_barcode_trace_enable (bool enabled)
{
    UINT     lock  = 0u;
    uint32_t index = 0u;

    DI(lock);
    g_b_enabled    = enabled;
    g_reader_state = enabled ? BARCODE_STATE_WAITING : BARCODE_STATE_OFF;
    if (!enabled)
    {
        for (index = 0u; index < BARCODE_CONFIG_COUNT; ++index)
        {
            g_configuration[index] = 0;
        }
    }
    EI(lock);
}

void robot_barcode_trace_configure (const int32_t config[BARCODE_CONFIG_COUNT])
{
    UINT     lock  = 0u;
    uint32_t index = 0u;

    DI(lock);
    for (index = 0u; index < BARCODE_CONFIG_COUNT; ++index)
    {
        g_configuration[index] = config[index];
    }
    g_b_enabled            = true;
    g_reader_state         = BARCODE_STATE_WAITING;
    g_reader_blocked       = BARCODE_BLOCKED_STOPPED;
    g_rejected_steps       = 0u;
    g_b_have_gap           = false;
    g_readiness[READY_GAP] = 0;
    EI(lock);
}

void robot_barcode_trace_cancel (uint32_t now)
{
    UINT lock = 0u;

    DI(lock);
    if (g_active_slot >= 0)
    {
        robot_barcode_frame_t * p_frame = &g_frames[g_active_slot];

        p_frame->done       = true;
        p_frame->state      = BARCODE_STATE_REJECTED;
        p_frame->reason     = B3BC_INTERRUPTED;
        p_frame->updated_ms = now;
        p_frame->blocked |= BARCODE_BLOCKED_STOPPED;
        g_active_slot = NO_SLOT;
    }
    g_reader_state = g_b_enabled ? BARCODE_STATE_WAITING : BARCODE_STATE_OFF;
    EI(lock);
}

void robot_barcode_trace_observe (
    const buddy3_barcode_decoder_t * p_decoder,
    const robot_barcode_observation_t * p_observation, bool was_active,
    float unfinished_before, const buddy3_barcode_event_t * p_result)
{
    UINT lock = 0u;

    DI(lock);
    trace_update_reader(p_decoder, p_observation);
    if (!was_active && p_decoder->active)
    {
        trace_start_frame(p_decoder, p_observation);
    }
    if (g_active_slot >= 0)
    {
        trace_record_step(&g_frames[g_active_slot], p_decoder, p_observation,
                          unfinished_before, p_result);
    }
    EI(lock);
}

bool robot_barcode_packet_peek (robot_barcode_packet_t * p_packet,
                                uint32_t now)
{
    UINT                   lock    = 0u;
    uint32_t               index   = 0u;
    uint32_t               slot    = BARCODE_TRACE_FRAMES;
    bool                   b_ready = false;
    robot_barcode_packet_t empty   = {0};

    *p_packet = empty;
    DI(lock);

    /* Oldest occupied scan first (wrap-safe id comparison). */
    for (index = 0u; index < BARCODE_TRACE_FRAMES; ++index)
    {
        if (g_frames[index].occupied &&
            ((BARCODE_TRACE_FRAMES == slot) ||
             ((int32_t) (g_frames[index].id - g_frames[slot].id) < 0)))
        {
            slot = index;
        }
    }
    p_packet->seq     = g_packet_sequence + 1u;
    p_packet->dropped = g_dropped_scans;

    if (BARCODE_TRACE_FRAMES == slot)
    {
        b_ready = trace_state_packet(p_packet, now);
    }
    else
    {
        b_ready = trace_frame_packet(p_packet, &g_frames[slot]);
    }
    EI(lock);

    return b_ready;
}

void robot_barcode_packet_ack (const robot_barcode_packet_t * p_packet)
{
    UINT     lock  = 0u;
    uint32_t index = 0u;

    DI(lock);
    /* Never advance a cursor based on an overwritten frame or stale peek. */
    if (p_packet->seq == (g_packet_sequence + 1u))
    {
        ++g_packet_sequence;
        if (BC_PACKET_STATE == p_packet->type)
        {
            g_last_state_ms = p_packet->ts_ms;
            g_b_have_state  = true;
        }
        else
        {
            for (index = 0u; index < BARCODE_TRACE_FRAMES; ++index)
            {
                robot_barcode_frame_t * p_frame = &g_frames[index];

                if (p_frame->occupied && (p_frame->id == p_packet->scan))
                {
                    if (BC_PACKET_START == p_packet->type)
                    {
                        p_frame->sent_start = true;
                    }
                    /* offset + count never exceeds the 29 edges or 512
                     * samples, so each cursor fits its type. */
                    if (BC_PACKET_EDGES == p_packet->type)
                    {
                        p_frame->sent_edges =
                            (uint8_t) (p_packet->offset + p_packet->count);
                    }
                    if (BC_PACKET_SAMPLES == p_packet->type)
                    {
                        p_frame->sent_samples =
                            (uint16_t) (p_packet->offset + p_packet->count);
                    }
                    if (BC_PACKET_PROGRESS == p_packet->type)
                    {
                        p_frame->sent_progress = p_packet->ts_ms;
                    }
                    if (BC_PACKET_END == p_packet->type)
                    {
                        p_frame->occupied = false;
                    }
                }
            }
        }
    }
    EI(lock);
}

/* Update the readiness evidence and reader state.  Caller holds the lock. */
static void trace_update_reader (const buddy3_barcode_decoder_t * p_decoder,
                                 const robot_barcode_observation_t * p_obs)
{
    g_readiness[READY_RAW]   = p_obs->raw;
    g_readiness[READY_NORM]  = p_obs->normalized;
    g_readiness[READY_BLACK] = p_obs->black;
    g_readiness[READY_QUIET] = robot_barcode_scaled(
        p_decoder->active ? p_decoder->leading_quiet
                          : p_decoder->white_distance,
        X100);
    g_readiness[READY_TRAVEL] = p_obs->travel_x100;
    /* A step may move at most a quarter of the minimum element. */
    g_readiness[READY_STEP_MAX] = g_configuration[CONFIG_MIN_ELEMENT] / 4;

    if (0u != (p_obs->blocked & BARCODE_BLOCKED_SAMPLE_GAP))
    {
        ++g_rejected_steps;
        g_last_gap_ms          = p_obs->ts_ms;
        g_b_have_gap           = true;
        g_readiness[READY_GAP] = p_obs->travel_x100;
    }
    if (!g_b_enabled || (0u != (p_obs->blocked & BARCODE_BLOCKED_STOPPED)))
    {
        g_b_have_gap = false;
    }
    g_reader_blocked = p_obs->blocked;

    /* A 1 Hz dashboard snapshot must not alias away a reset every other IR
     * sample. Retain a recent pre-capture spacing failure for one second. */
    if (!p_decoder->active && (0u == g_reader_blocked) && g_b_have_gap &&
        ((p_obs->ts_ms - g_last_gap_ms) < GAP_HOLD_MS))
    {
        g_reader_blocked |= BARCODE_BLOCKED_SAMPLE_GAP;
    }

    if (!g_b_enabled)
    {
        g_reader_state = BARCODE_STATE_OFF;
    }
    else if (0u != g_reader_blocked)
    {
        g_reader_state = BARCODE_STATE_SUSPENDED;
    }
    else if (!p_decoder->active)
    {
        g_reader_state = BARCODE_STATE_WAITING;
    }
    else if (CODE39_FRAME_ELEMENTS == p_decoder->count)
    {
        g_reader_state = BARCODE_STATE_TRAILING;
    }
    else
    {
        g_reader_state = BARCODE_STATE_CAPTURING;
    }
}

/* Claim a free slot (or overwrite the older scan) for a new frame.  Caller
 * holds the lock. */
static void trace_start_frame (const buddy3_barcode_decoder_t * p_decoder,
                               const robot_barcode_observation_t * p_obs)
{
    robot_barcode_frame_t * p_frame = NULL;
    uint32_t                slot    = 0u;
    uint32_t                index   = 0u;

    while ((slot < BARCODE_TRACE_FRAMES) && g_frames[slot].occupied)
    {
        ++slot;
    }
    if (BARCODE_TRACE_FRAMES == slot)
    {
        /* Both busy: drop the older scan (wrap-safe id comparison). */
        slot = ((int32_t) (g_frames[0].id - g_frames[1].id) < 0) ? 0u : 1u;
        ++g_dropped_scans;
    }

    /* Do not zero a large sample buffer in the control path. Counts make
     * old storage inaccessible; only fixed metadata is initialized. */
    p_frame                = &g_frames[slot];
    p_frame->id            = ++g_scan_sequence;
    p_frame->started_ms    = p_obs->ts_ms;
    p_frame->count         = 0u;
    p_frame->sample_count  = 0u;
    p_frame->sample_drops  = 0u;
    p_frame->sent_edges    = 0u;
    p_frame->sent_samples  = 0u;
    p_frame->sent_progress = 0u;
    p_frame->sent_start    = false;
    p_frame->done          = false;
    p_frame->occupied      = true;
    p_frame->reason        = 0u;
    p_frame->format        = 0u;
    p_frame->symbol        = 0u;
    p_frame->reversed      = 0u;
    p_frame->partial_x100  = 0;
    p_frame->trailing_x100 = 0;
    p_frame->narrow_x100   = 0;
    p_frame->ratio_x100    = 0;
    p_frame->max_gap_x100  = 0;
    p_frame->leading_x100  = robot_barcode_scaled(p_decoder->leading_quiet,
                                                  X100);
    for (index = 0u; index < BARCODE_CONFIG_COUNT; ++index)
    {
        p_frame->config[index] = g_configuration[index];
    }
    /* slot < BARCODE_TRACE_FRAMES, so it fits in int32_t. */
    g_active_slot = (int32_t) slot;
}

/* Copy new edges, the sample and any result into the active frame.  Caller
 * holds the lock. */
static void trace_record_step (robot_barcode_frame_t * p_frame,
                               const buddy3_barcode_decoder_t * p_decoder,
                               const robot_barcode_observation_t * p_obs,
                               float unfinished_before,
                               const buddy3_barcode_event_t * p_result)
{
    uint32_t index   = 0u;
    int32_t  max_gap = robot_barcode_scaled(p_decoder->max_sample_gap, X100);

    p_frame->updated_ms = p_obs->ts_ms;
    p_frame->state      = g_reader_state;
    p_frame->blocked    = p_obs->blocked;

    /* On guard abort the core may already have reset. Previously captured
     * edges remain intact, and unfinished width is explicitly unconfirmed. */
    if ((p_decoder->count > p_frame->count) &&
        (p_decoder->count <= CODE39_FRAME_ELEMENTS))
    {
        for (index = p_frame->count; index < p_decoder->count; ++index)
        {
            p_frame->edges[index].width_x100 =
                robot_barcode_scaled(p_decoder->widths[index], X100);
            p_frame->edges[index].ts_ms     = p_decoder->edge_ms[index];
            p_frame->edges[index].left_x10  = p_obs->left_x10;
            p_frame->edges[index].right_x10 = p_obs->right_x10;
        }
        p_frame->count = p_decoder->count;
    }
    p_frame->partial_x100 = robot_barcode_scaled(
        p_decoder->active
            ? (p_decoder->position - p_decoder->edge_position)
            : unfinished_before,
        X100);
    if (max_gap > p_frame->max_gap_x100)
    {
        p_frame->max_gap_x100 = max_gap;
    }

    if (p_frame->sample_count < BARCODE_TRACE_SAMPLES)
    {
        robot_barcode_sample_t * p_sample =
            &p_frame->samples[p_frame->sample_count];

        p_frame->sample_count++;
        p_sample->ts_ms         = p_obs->ts_ms;
        p_sample->raw           = p_obs->raw;
        p_sample->normalized    = p_obs->normalized;
        p_sample->left_x10      = p_obs->left_x10;
        p_sample->right_x10     = p_obs->right_x10;
        p_sample->position_x100 = 0;
        for (index = 0u; index < p_frame->count; ++index)
        {
            p_sample->position_x100 += p_frame->edges[index].width_x100;
        }
        p_sample->position_x100 += p_frame->partial_x100;
        p_sample->black = p_obs->black;
    }
    else
    {
        ++p_frame->sample_drops;
    }

    if (NULL != p_result)
    {
        p_frame->done   = true;
        p_frame->reason = (uint8_t) p_result->kind;   /* Enum fits a byte. */
        p_frame->format = (uint8_t) p_result->format_status;
        if (B3BC_VALID == p_result->kind)
        {
            p_frame->state = BARCODE_STATE_ACCEPTED;
            /* An accepted symbol is an uppercase ASCII letter. */
            p_frame->symbol   = (uint8_t) p_result->decoded.symbol;
            p_frame->reversed = p_result->decoded.reversed;
            p_frame->narrow_x100 =
                robot_barcode_scaled(p_result->decoded.narrow_width, X100);
            p_frame->ratio_x100 =
                robot_barcode_scaled(p_result->decoded.wide_ratio, X100);
            p_frame->trailing_x100 =
                robot_barcode_scaled(p_decoder->white_distance, X100);
            p_frame->partial_x100 = 0;
        }
        else
        {
            p_frame->state = BARCODE_STATE_REJECTED;
            if (CODE39_FRAME_ELEMENTS == p_decoder->count)
            {
                p_frame->trailing_x100 =
                    robot_barcode_scaled(p_decoder->white_distance, X100);
            }
        }
        g_active_slot = NO_SLOT;
    }
}

/* Reader-state packet, at most once a second while no scan is queued.
 * Returns true when the packet is due.  Caller holds the lock. */
static bool trace_state_packet (robot_barcode_packet_t * p_packet,
                                uint32_t now)
{
    uint32_t index   = 0u;
    bool     b_ready = !(g_b_have_state &&
                         ((now - g_last_state_ms) < STATE_MS));

    if (b_ready)
    {
        p_packet->type    = BC_PACKET_STATE;
        p_packet->state   = g_reader_state;
        p_packet->ts_ms   = now;
        p_packet->blocked = g_reader_blocked;
        for (index = 0u; index < BARCODE_CONFIG_COUNT; ++index)
        {
            p_packet->config[index] = g_configuration[index];
        }
        for (index = 0u; index < BARCODE_READINESS_COUNT; ++index)
        {
            p_packet->readiness[index] = g_readiness[index];
        }
        p_packet->rejected_steps = g_rejected_steps;
    }

    return b_ready;
}

/* Next packet of a scan: start, edges, progress, samples, then end.
 * Returns true when there is something to send.  Caller holds the lock. */
static bool trace_frame_packet (robot_barcode_packet_t * p_packet,
                                const robot_barcode_frame_t * p_frame)
{
    uint32_t index     = 0u;
    uint32_t remaining = 0u;
    bool     b_ready   = true;

    p_packet->scan          = p_frame->id;
    p_packet->ts_ms         = p_frame->updated_ms;
    p_packet->state         = p_frame->state;
    p_packet->total_edges   = p_frame->count;
    p_packet->total_samples = p_frame->sample_count;
    p_packet->sample_drops  = p_frame->sample_drops;
    p_packet->partial       = p_frame->partial_x100;
    p_packet->leading       = p_frame->leading_x100;
    p_packet->trailing      = p_frame->trailing_x100;
    p_packet->blocked       = p_frame->blocked;
    p_packet->reason        = p_frame->reason;
    p_packet->format        = p_frame->format;
    p_packet->symbol        = p_frame->symbol;
    p_packet->reversed      = p_frame->reversed;
    p_packet->narrow        = p_frame->narrow_x100;
    p_packet->ratio         = p_frame->ratio_x100;
    p_packet->max_gap       = p_frame->max_gap_x100;

    if (!p_frame->sent_start)
    {
        p_packet->type  = BC_PACKET_START;
        p_packet->ts_ms = p_frame->started_ms;
        for (index = 0u; index < BARCODE_CONFIG_COUNT; ++index)
        {
            p_packet->config[index] = p_frame->config[index];
        }
    }
    else if (p_frame->sent_edges < p_frame->count)
    {
        p_packet->type   = BC_PACKET_EDGES;
        p_packet->offset = p_frame->sent_edges;
        remaining        = (uint32_t) p_frame->count - p_frame->sent_edges;
        if (remaining > BARCODE_TRACE_BATCH)
        {
            remaining = BARCODE_TRACE_BATCH;
        }
        p_packet->count = (uint8_t) remaining; /* At most the batch size. */
        for (index = 0u; index < p_packet->count; ++index)
        {
            p_packet->edges[index] = p_frame->edges[p_packet->offset + index];
        }
    }
    else if (!p_frame->done &&
             ((0u == p_frame->sent_progress) ||
              ((p_frame->updated_ms - p_frame->sent_progress) >= PROGRESS_MS)))
    {
        p_packet->type = BC_PACKET_PROGRESS;
    }
    else if (p_frame->sent_samples < p_frame->sample_count)
    {
        p_packet->type   = BC_PACKET_SAMPLES;
        p_packet->offset = p_frame->sent_samples;
        /* sent_samples < sample_count here, so the difference cannot wrap. */
        remaining = (uint32_t) p_frame->sample_count - p_frame->sent_samples;
        if (remaining > BARCODE_TRACE_BATCH)
        {
            remaining = BARCODE_TRACE_BATCH;
        }
        p_packet->count = (uint8_t) remaining; /* At most the batch size. */
        for (index = 0u; index < p_packet->count; ++index)
        {
            p_packet->samples[index] =
                p_frame->samples[p_packet->offset + index];
        }
    }
    else if (p_frame->done)
    {
        p_packet->type = BC_PACKET_END;
    }
    else
    {
        b_ready = false;
    }

    return b_ready;
}

/*** end of file ***/

