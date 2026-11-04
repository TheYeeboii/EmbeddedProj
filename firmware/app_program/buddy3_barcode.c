#include "buddy3_barcode.h"
#include <math.h>

void buddy3_barcode_reset(buddy3_barcode_decoder_t *s)
{
    buddy3_barcode_decoder_t empty = {0};
    *s = empty;
}

float buddy3_barcode_travel(float left_mm, float right_mm,
                           float track_mm, float offset_left_mm)
{
    if (!(track_mm > 0.0f)) return 0.0f;
    return 0.5f * (left_mm + right_mm) -
           offset_left_mm * (right_mm - left_mm) / track_mm;
}

static bool abort_frame(buddy3_barcode_decoder_t *s,
    buddy3_barcode_event_t *event, buddy3_barcode_event_kind_t kind)
{
    bool was_active = s->active;
    buddy3_barcode_reset(s);
    if (was_active) event->kind = kind;
    return was_active;
}

bool buddy3_barcode_step(buddy3_barcode_decoder_t *s,
    const buddy3_barcode_config_t *cfg, bool black, float travel_mm,
    uint32_t now, bool permitted, buddy3_barcode_event_t *event)
{
    buddy3_barcode_event_t empty = {0};
    float edge;
    *event = empty;
    event->ts_ms = now;
    if (!permitted || !isfinite(cfg->min_element_mm) || !(cfg->min_element_mm > 0.0f))
        return abort_frame(s, event, B3BC_INTERRUPTED);
    if (!isfinite(travel_mm) || travel_mm < 0.0f)
        return abort_frame(s, event, B3BC_REVERSE);
    if (travel_mm > cfg->min_element_mm * 0.25f)
        return abort_frame(s, event, B3BC_SAMPLE_GAP);
    if (!s->have_sample) {
        s->have_sample = true;
        s->previous_black = black;
        s->last_progress_ms = now;
        return false;
    }
    if (s->active && now - s->last_progress_ms >= 1500u)
        return abort_frame(s, event, B3BC_STALL);
    if (s->active && now - s->started_ms >= 30000u)
        return abort_frame(s, event, B3BC_TIMEOUT);
    /* Do not interpret noise while stationary as new spatial elements. */
    if (travel_mm == 0.0f) return false;
    s->last_progress_ms = now;
    s->position += travel_mm;
    if (travel_mm > s->max_sample_gap) s->max_sample_gap = travel_mm;
    edge = s->position - travel_mm * 0.5f;
    if (!s->active) {
        if (black && !s->previous_black) {
            s->leading_quiet = s->white_distance + travel_mm * 0.5f;
            if (s->leading_quiet >= 10.0f * cfg->min_element_mm) {
                s->active = true;
                s->count = 0;
                s->position -= edge; /* Local coordinates avoid growing floats. */
                s->edge_position = 0.0f;
                s->started_ms = now;
                s->max_sample_gap = travel_mm;
            }
        }
        if (black) s->white_distance = 0.0f;
        else {
            s->white_distance += travel_mm;
            /* Bounded lead-in; values beyond this cannot affect recognition. */
            if (s->white_distance > 10000.0f) s->white_distance = 10000.0f;
        }
        if (!s->active) s->position = 0.0f;
        s->previous_black = black;
        return false;
    }
    if (black != s->previous_black) {
        if (s->count == CODE39_FRAME_ELEMENTS) {
            /* New black before a sufficient trailing quiet zone. */
            event->kind = B3BC_FORMAT;
            event->format_status = CODE39_BAD_QUIET;
            buddy3_barcode_reset(s);
            return true;
        }
        s->widths[s->count] = edge - s->edge_position;
        s->edge_ms[s->count] = now;
        ++s->count;
        s->edge_position = edge;
        s->previous_black = black;
    }
    if (s->count == CODE39_FRAME_ELEMENTS) {
        code39_result_t result;
        code39_status_t status = code39_decode(s->widths, s->count,
            s->leading_quiet, 100000.0f, &result);
        if (status == CODE39_OK &&
            (result.narrow_width < cfg->min_element_mm ||
             s->max_sample_gap > result.narrow_width * 0.25f))
            status = CODE39_BAD_WIDTH;
        if (status == CODE39_OK) {
            status = code39_decode(s->widths, s->count, s->leading_quiet,
                s->position - s->edge_position, &result);
            if (status == CODE39_BAD_QUIET) return false; /* Still crossing white. */
        }
        event->format_status = status;
        event->kind = status == CODE39_OK ? B3BC_VALID : B3BC_FORMAT;
        if (status == CODE39_OK) event->decoded = result;
        s->active = false;
        s->white_distance = s->position - s->edge_position;
        s->position = 0.0f;
        return true;
    }
    /* Bound a missing edge even while the wheels keep moving. */
    if (s->position > 10000.0f)
        return abort_frame(s, event, B3BC_TIMEOUT);
    return false;
}

const char *buddy3_barcode_event_name(buddy3_barcode_event_kind_t kind)
{
    static const char *const names[] = {
        "none", "valid", "format", "interrupted", "reverse", "sample_gap", "stall", "timeout"
    };
    return (unsigned)kind < sizeof(names) / sizeof(names[0]) ? names[kind] : "unknown";
}
