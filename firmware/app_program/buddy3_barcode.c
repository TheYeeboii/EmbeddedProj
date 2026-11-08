/** @file buddy3_barcode.c
 *
 * @brief Buddy 3 barcode reader: turns black/white samples and wheel travel
 *        into Code 39 element widths and decodes the framed letter.
 */
#include "buddy3_barcode.h"
#include <math.h>

/* A sample may move at most a quarter of the smallest element. */
#define SAMPLE_GAP_FRACTION 0.25f
/* The leading quiet zone must be ten minimum elements of white. */
#define QUIET_ELEMENTS 10.0f
#define STALL_MS       1500u
#define FRAME_MAX_MS   30000u
/* Lead-in and frame lengths beyond this cannot affect recognition. */
#define MAX_TRACK_MM 10000.0f
/* Trailing quiet used for the first decode, before the real one is known. */
#define UNBOUNDED_QUIET_MM 100000.0f

static bool barcode_abort (buddy3_barcode_decoder_t *  p_state,
                           buddy3_barcode_event_t *    p_event,
                           buddy3_barcode_event_kind_t kind);
static void barcode_wait_for_start (buddy3_barcode_decoder_t *      p_state,
                                    const buddy3_barcode_config_t * p_cfg,
                                    bool black, float32_t travel_mm,
                                    float32_t edge, uint32_t now);
static bool barcode_collect (buddy3_barcode_decoder_t *      p_state,
                             const buddy3_barcode_config_t * p_cfg, bool black,
                             float32_t edge, uint32_t now,
                             buddy3_barcode_event_t * p_event);
static bool barcode_finish (buddy3_barcode_decoder_t *      p_state,
                            const buddy3_barcode_config_t * p_cfg,
                            buddy3_barcode_event_t *        p_event);

void buddy3_barcode_reset (buddy3_barcode_decoder_t * p_state)
{
    buddy3_barcode_decoder_t empty = {0};

    *p_state = empty;
}

float32_t buddy3_barcode_travel (float32_t left_mm, float32_t right_mm,
                                 float32_t track_mm, float32_t offset_left_mm)
{
    float32_t travel = 0.0f;

    /* The sensor is offset sideways, so on a curve it moves a little more or
     * less than the centre of the axle. */
    if (track_mm > 0.0f)
    {
        travel = (0.5f * (left_mm + right_mm)) -
                 ((offset_left_mm * (right_mm - left_mm)) / track_mm);
    }

    return travel;
}

bool buddy3_barcode_step (buddy3_barcode_decoder_t *      p_state,
                          const buddy3_barcode_config_t * p_cfg, bool black,
                          float32_t travel_mm, uint32_t now, bool permitted,
                          buddy3_barcode_event_t * p_event)
{
    buddy3_barcode_event_t empty   = {0};
    bool                   b_event = false;
    float32_t              edge    = 0.0f;

    *p_event       = empty;
    p_event->ts_ms = now;

    if (!permitted || !isfinite(p_cfg->min_element_mm) ||
        !(p_cfg->min_element_mm > 0.0f))
    {
        b_event = barcode_abort(p_state, p_event, B3BC_INTERRUPTED);
    }
    else if (!isfinite(travel_mm) || (travel_mm < 0.0f))
    {
        b_event = barcode_abort(p_state, p_event, B3BC_REVERSE);
    }
    else if (travel_mm > (p_cfg->min_element_mm * SAMPLE_GAP_FRACTION))
    {
        b_event = barcode_abort(p_state, p_event, B3BC_SAMPLE_GAP);
    }
    else if (!p_state->have_sample)
    {
        p_state->have_sample      = true;
        p_state->previous_black   = black;
        p_state->last_progress_ms = now;
    }
    else if (p_state->active && ((now - p_state->last_progress_ms) >= STALL_MS))
    {
        b_event = barcode_abort(p_state, p_event, B3BC_STALL);
    }
    else if (p_state->active && ((now - p_state->started_ms) >= FRAME_MAX_MS))
    {
        b_event = barcode_abort(p_state, p_event, B3BC_TIMEOUT);
    }
    else if (!(travel_mm > 0.0f))
    {
        /* Do not interpret noise while stationary as new spatial elements. */
    }
    else
    {
        p_state->last_progress_ms = now;
        p_state->position += travel_mm;
        if (travel_mm > p_state->max_sample_gap)
        {
            p_state->max_sample_gap = travel_mm;
        }
        /* The colour changed somewhere within this sample: use its middle. */
        edge = p_state->position - (travel_mm * 0.5f);
        if (!p_state->active)
        {
            barcode_wait_for_start(p_state, p_cfg, black, travel_mm, edge, now);
        }
        else
        {
            b_event =
                barcode_collect(p_state, p_cfg, black, edge, now, p_event);
        }
    }

    return b_event;
}

const char * buddy3_barcode_event_name (buddy3_barcode_event_kind_t kind)
{
    static const char * const names[] = {"none",        "valid",   "format",
                                         "interrupted", "reverse", "sample_gap",
                                         "stall",       "timeout"};
    const char *              p_name  = "unknown";

    /* The enum is non-negative, so the unsigned compare is a range check. */
    if ((uint32_t) kind < (sizeof(names) / sizeof(names[0])))
    {
        p_name = names[kind];
    }

    return p_name;
}

/* Reset the decoder; report kind only if a frame was in progress. */
static bool barcode_abort (buddy3_barcode_decoder_t *  p_state,
                           buddy3_barcode_event_t *    p_event,
                           buddy3_barcode_event_kind_t kind)
{
    bool b_was_active = p_state->active;

    buddy3_barcode_reset(p_state);
    if (b_was_active)
    {
        p_event->kind = kind;
    }

    return b_was_active;
}

/* Measure the white lead-in and start a frame on the first black after a
 * long enough quiet zone. */
static void barcode_wait_for_start (buddy3_barcode_decoder_t *      p_state,
                                    const buddy3_barcode_config_t * p_cfg,
                                    bool black, float32_t travel_mm,
                                    float32_t edge, uint32_t now)
{
    if (black && !p_state->previous_black)
    {
        p_state->leading_quiet = p_state->white_distance + (travel_mm * 0.5f);
        if (p_state->leading_quiet >= (QUIET_ELEMENTS * p_cfg->min_element_mm))
        {
            p_state->active = true;
            p_state->count  = 0u;
            /* Local coordinates avoid growing floats. */
            p_state->position -= edge;
            p_state->edge_position  = 0.0f;
            p_state->started_ms     = now;
            p_state->max_sample_gap = travel_mm;
        }
    }

    if (black)
    {
        p_state->white_distance = 0.0f;
    }
    else
    {
        p_state->white_distance += travel_mm;
        if (p_state->white_distance > MAX_TRACK_MM)
        {
            p_state->white_distance = MAX_TRACK_MM;
        }
    }

    if (!p_state->active)
    {
        p_state->position = 0.0f;
    }
    p_state->previous_black = black;
}

/* Record element edges of an active frame and decode once all 29 elements
 * are known.  Returns true when an event was produced. */
static bool barcode_collect (buddy3_barcode_decoder_t *      p_state,
                             const buddy3_barcode_config_t * p_cfg, bool black,
                             float32_t edge, uint32_t now,
                             buddy3_barcode_event_t * p_event)
{
    bool b_event = false;

    if ((black != p_state->previous_black) &&
        (CODE39_FRAME_ELEMENTS == p_state->count))
    {
        /* New black before a sufficient trailing quiet zone. */
        p_event->kind          = B3BC_FORMAT;
        p_event->format_status = CODE39_BAD_QUIET;
        buddy3_barcode_reset(p_state);
        b_event = true;
    }
    else
    {
        if (black != p_state->previous_black)
        {
            p_state->widths[p_state->count]  = edge - p_state->edge_position;
            p_state->edge_ms[p_state->count] = now;
            ++p_state->count;
            p_state->edge_position  = edge;
            p_state->previous_black = black;
        }

        if (CODE39_FRAME_ELEMENTS == p_state->count)
        {
            b_event = barcode_finish(p_state, p_cfg, p_event);
        }
        else if (p_state->position > MAX_TRACK_MM)
        {
            /* Bound a missing edge even while the wheels keep moving. */
            b_event = barcode_abort(p_state, p_event, B3BC_TIMEOUT);
        }
        else
        {
            /* Frame still in progress. */
        }
    }

    return b_event;
}

/* Decode a complete frame.  The geometry is checked first with an
 * unbounded trailing quiet zone; the real quiet zone is then checked so a
 * frame waits while the sensor is still crossing white.  Returns true when
 * an event was produced. */
static bool barcode_finish (buddy3_barcode_decoder_t *      p_state,
                            const buddy3_barcode_config_t * p_cfg,
                            buddy3_barcode_event_t *        p_event)
{
    code39_result_t result     = {0};
    code39_status_t status     = CODE39_OK;
    bool            b_event    = true;
    bool            b_crossing = false;

    status = code39_decode(p_state->widths, p_state->count,
                           p_state->leading_quiet, UNBOUNDED_QUIET_MM, &result);
    if ((CODE39_OK == status) &&
        ((result.narrow_width < p_cfg->min_element_mm) ||
         (p_state->max_sample_gap >
          (result.narrow_width * SAMPLE_GAP_FRACTION))))
    {
        status = CODE39_BAD_WIDTH;
    }
    if (CODE39_OK == status)
    {
        status = code39_decode(
            p_state->widths, p_state->count, p_state->leading_quiet,
            p_state->position - p_state->edge_position, &result);
        /* Only the trailing quiet zone can fail here: not enough white yet. */
        b_crossing = (CODE39_BAD_QUIET == status);
    }

    if (b_crossing)
    {
        b_event = false; /* Still crossing white. */
    }
    else
    {
        p_event->format_status = status;
        if (CODE39_OK == status)
        {
            p_event->kind    = B3BC_VALID;
            p_event->decoded = result;
        }
        else
        {
            p_event->kind = B3BC_FORMAT;
        }
        p_state->active         = false;
        p_state->white_distance = p_state->position - p_state->edge_position;
        p_state->position       = 0.0f;
    }

    return b_event;
}

/*** end of file ***/

