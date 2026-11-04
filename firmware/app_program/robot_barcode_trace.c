#include <tk/tkernel.h>
#include "robot_barcode_trace.h"

/* Capture storage and transport cursors are private to this module. */
typedef struct {
    uint32_t id, started_ms, updated_ms, sample_drops;
    int32_t config[11], leading_x100, trailing_x100, partial_x100;
    int32_t narrow_x100, ratio_x100, max_gap_x100;
    robot_barcode_edge_t edges[CODE39_FRAME_ELEMENTS];
    robot_barcode_sample_t samples[BARCODE_TRACE_SAMPLES];
    uint16_t sample_count;
    uint8_t count, state, reason, format, symbol, reversed, blocked;
    bool done, occupied;
    /* Publisher cursors: only advance after the mailbox accepts the packet. */
    uint16_t sent_samples;
    uint8_t sent_edges;
    uint32_t sent_progress;
    bool sent_start;
} robot_barcode_frame_t;

static robot_barcode_frame_t frames[BARCODE_TRACE_FRAMES];
static int active_slot = -1;
static int32_t configuration[11];
static uint32_t scan_sequence, packet_sequence, dropped_scans, last_state;
static bool enabled, have_state;
static uint8_t reader_state, reader_blocked;
static int32_t readiness[7];
static uint32_t rejected_steps, last_gap_ms;
static bool have_gap;

int32_t robot_barcode_scaled(float value, float scale)
{
    float v = value * scale;
    if (!(v == v)) return 0; /* NaN; avoid libc headers conflicting with T-Kernel. */
    if (v > 2147480000.0f) return INT32_MAX;
    if (v < -2147480000.0f) return -INT32_MAX;
    return (int32_t)v;
}
void robot_barcode_trace_enable(bool value)
{
    UINT lock;
    unsigned i;
    DI(lock); enabled = value; reader_state = value ? 1u : 0u;
    if (!value) for (i = 0; i < 11; ++i) configuration[i] = 0;
    EI(lock);
}
void robot_barcode_trace_configure(const int32_t config[11])
{
    UINT lock;
    unsigned i;
    DI(lock);
    for (i = 0; i < 11; ++i) configuration[i] = config[i];
    enabled = true; reader_state = 1; reader_blocked = 1;
    rejected_steps = 0; have_gap = false; readiness[6] = 0;
    EI(lock);
}
void robot_barcode_trace_cancel(uint32_t now)
{
    UINT lock;
    DI(lock);
    if (active_slot >= 0) {
        robot_barcode_frame_t *f = &frames[active_slot];
        f->done = true; f->state = 5; f->reason = B3BC_INTERRUPTED;
        f->updated_ms = now; f->blocked |= 1u;
        active_slot = -1;
    }
    reader_state = enabled ? 1 : 0;
    EI(lock);
}

void robot_barcode_trace_observe(const buddy3_barcode_decoder_t *s,
    const robot_barcode_observation_t *o, bool was_active,
    float unfinished_before, const buddy3_barcode_event_t *result)
{
    UINT lock;
    robot_barcode_frame_t *f;
    unsigned i;
    DI(lock);
    readiness[0] = o->raw; readiness[1] = o->normalized; readiness[2] = o->black;
    readiness[3] = robot_barcode_scaled(s->active ? s->leading_quiet : s->white_distance,100);
    readiness[4] = o->travel_x100; readiness[5] = configuration[5] / 4;
    if (o->blocked & 32u) {
        ++rejected_steps; last_gap_ms = o->ts_ms; have_gap = true;
        readiness[6] = o->travel_x100;
    }
    if (!enabled || (o->blocked & 1u)) have_gap = false;
    reader_blocked = o->blocked;
    /* A 1 Hz dashboard snapshot must not alias away a reset every other IR
     * sample. Retain a recent pre-capture spacing failure for one second. */
    if (!s->active && !reader_blocked && have_gap && o->ts_ms - last_gap_ms < 1000u)
        reader_blocked |= 32u;
    reader_state = !enabled ? 0 : reader_blocked ? 6 : s->active ? (s->count == 29 ? 3 : 2) : 1;
    if (!was_active && s->active) {
        unsigned slot = 0;
        for (i = 0; i < BARCODE_TRACE_FRAMES; ++i)
            if (!frames[i].occupied) { slot = i; break; }
        if (i == BARCODE_TRACE_FRAMES) {
            slot = (int32_t)(frames[0].id - frames[1].id) < 0 ? 0 : 1;
            ++dropped_scans;
        }
        /* Do not zero a large sample buffer in the control path. Counts make
         * old storage inaccessible; only fixed metadata is initialized. */
        f = &frames[slot];
        f->id = ++scan_sequence; f->started_ms = o->ts_ms;
        f->count = 0; f->sample_count = 0; f->sample_drops = 0;
        f->sent_edges = 0; f->sent_samples = 0; f->sent_progress = 0; f->sent_start = false;
        f->done = false; f->occupied = true; f->reason = 0; f->format = 0;
        f->symbol = 0; f->reversed = 0; f->partial_x100 = 0; f->trailing_x100 = 0;
        f->narrow_x100 = 0; f->ratio_x100 = 0; f->max_gap_x100 = 0;
        f->leading_x100 = robot_barcode_scaled(s->leading_quiet,100);
        for (i = 0; i < 11; ++i) f->config[i] = configuration[i];
        active_slot = (int)slot;
    }
    if (active_slot < 0) { EI(lock); return; }
    f = &frames[active_slot];
    f->updated_ms = o->ts_ms; f->state = reader_state; f->blocked = o->blocked;
    /* On guard abort the core may already have reset. Previously captured
     * edges remain intact, and unfinished width is explicitly unconfirmed. */
    if (s->count > f->count && s->count <= CODE39_FRAME_ELEMENTS) {
        for (i = f->count; i < s->count; ++i) {
            f->edges[i].width_x100 = robot_barcode_scaled(s->widths[i],100);
            f->edges[i].ts_ms = s->edge_ms[i];
            f->edges[i].left_x10 = o->left_x10; f->edges[i].right_x10 = o->right_x10;
        }
        f->count = s->count;
    }
    f->partial_x100 = robot_barcode_scaled(s->active ? s->position - s->edge_position : unfinished_before,100);
    {
        int32_t gap = robot_barcode_scaled(s->max_sample_gap,100);
        if (gap > f->max_gap_x100) f->max_gap_x100 = gap;
    }
    if (f->sample_count < BARCODE_TRACE_SAMPLES) {
        robot_barcode_sample_t *sample = &f->samples[f->sample_count++];
        sample->ts_ms = o->ts_ms; sample->raw = o->raw; sample->normalized = o->normalized;
        sample->left_x10 = o->left_x10; sample->right_x10 = o->right_x10;
        sample->position_x100 = 0;
        for (i = 0; i < f->count; ++i) sample->position_x100 += f->edges[i].width_x100;
        sample->position_x100 += f->partial_x100;
        sample->black = o->black;
    } else ++f->sample_drops;
    if (result) {
        f->done = true; f->reason = result->kind; f->format = result->format_status;
        f->state = result->kind == B3BC_VALID ? 4 : 5;
        if (result->kind == B3BC_VALID) {
            f->symbol = (uint8_t)result->decoded.symbol; f->reversed = result->decoded.reversed;
            f->narrow_x100 = robot_barcode_scaled(result->decoded.narrow_width,100);
            f->ratio_x100 = robot_barcode_scaled(result->decoded.wide_ratio,100);
            f->trailing_x100 = robot_barcode_scaled(s->white_distance,100);
            f->partial_x100 = 0;
        } else if (s->count == 29)
            f->trailing_x100 = robot_barcode_scaled(s->white_distance,100);
        active_slot = -1;
    }
    EI(lock);
}

bool robot_barcode_packet_peek(robot_barcode_packet_t *p, uint32_t now)
{
    UINT lock;
    unsigned i, slot = BARCODE_TRACE_FRAMES;
    robot_barcode_packet_t empty = {0};
    *p = empty;
    DI(lock);
    for (i = 0; i < BARCODE_TRACE_FRAMES; ++i)
        if (frames[i].occupied && (slot == BARCODE_TRACE_FRAMES || (int32_t)(frames[i].id - frames[slot].id) < 0)) slot = i;
    p->seq = packet_sequence + 1; p->dropped = dropped_scans;
    if (slot == BARCODE_TRACE_FRAMES) {
        if (have_state && now - last_state < 1000u) { EI(lock); return false; }
        p->type = BC_PACKET_STATE; p->state = reader_state; p->ts_ms = now; p->blocked = reader_blocked;
        for (i = 0; i < 11; ++i) p->config[i] = configuration[i];
        for (i = 0; i < 7; ++i) p->readiness[i] = readiness[i];
        p->rejected_steps = rejected_steps;
        EI(lock); return true;
    }
    {
        robot_barcode_frame_t *f = &frames[slot];
        p->scan = f->id; p->ts_ms = f->updated_ms; p->state = f->state;
        p->total_edges = f->count; p->total_samples = f->sample_count; p->sample_drops = f->sample_drops;
        p->partial = f->partial_x100; p->leading = f->leading_x100; p->trailing = f->trailing_x100;
        p->blocked = f->blocked; p->reason = f->reason; p->format = f->format;
        p->symbol = f->symbol; p->reversed = f->reversed;
        p->narrow = f->narrow_x100; p->ratio = f->ratio_x100; p->max_gap = f->max_gap_x100;
        if (!f->sent_start) {
            p->type = BC_PACKET_START; p->ts_ms = f->started_ms;
            for (i = 0; i < 11; ++i) p->config[i] = f->config[i];
        } else if (f->sent_edges < f->count) {
            p->type = BC_PACKET_EDGES; p->offset = f->sent_edges;
            p->count = (uint8_t)(f->count - f->sent_edges);
            if (p->count > BARCODE_TRACE_BATCH) p->count = BARCODE_TRACE_BATCH;
            for (i = 0; i < p->count; ++i) p->edges[i] = f->edges[p->offset + i];
        } else if (!f->done && (f->sent_progress == 0 || f->updated_ms - f->sent_progress >= 100u)) {
            p->type = BC_PACKET_PROGRESS;
        } else if (f->sent_samples < f->sample_count) {
            p->type = BC_PACKET_SAMPLES; p->offset = f->sent_samples;
            p->count = (uint8_t)((f->sample_count - f->sent_samples) > BARCODE_TRACE_BATCH ? BARCODE_TRACE_BATCH : f->sample_count - f->sent_samples);
            for (i = 0; i < p->count; ++i) p->samples[i] = f->samples[p->offset + i];
        } else if (f->done) p->type = BC_PACKET_END;
        else { EI(lock); return false; }
    }
    EI(lock); return true;
}
void robot_barcode_packet_ack(const robot_barcode_packet_t *p)
{
    UINT lock;
    unsigned i;
    DI(lock);
    /* Never advance a cursor based on an overwritten frame or stale peek. */
    if (p->seq != packet_sequence + 1) { EI(lock); return; }
    ++packet_sequence;
    if (p->type == BC_PACKET_STATE) { last_state = p->ts_ms; have_state = true; }
    else for (i = 0; i < BARCODE_TRACE_FRAMES; ++i) {
        robot_barcode_frame_t *f = &frames[i];
        if (!f->occupied || f->id != p->scan) continue;
        if (p->type == BC_PACKET_START) f->sent_start = true;
        if (p->type == BC_PACKET_EDGES) f->sent_edges = p->offset + p->count;
        if (p->type == BC_PACKET_SAMPLES) f->sent_samples = p->offset + p->count;
        if (p->type == BC_PACKET_PROGRESS) f->sent_progress = p->ts_ms;
        if (p->type == BC_PACKET_END) f->occupied = false;
    }
    EI(lock);
}
