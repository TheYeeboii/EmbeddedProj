#include "buddy3_barcode.h"

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

uint8_t buddy3_barcode_step(buddy3_barcode_decoder_t *s,
    const buddy3_barcode_config_t *cfg, bool black, float travel_mm,
    uint32_t now, bool permitted)
{
    float previous_position = s->position;
    uint8_t command;
    if (!permitted || !(cfg->cell_mm > 0.0f) ||
        !(travel_mm >= 0.0f) || travel_mm > cfg->cell_mm * 0.25f)
    {
        buddy3_barcode_reset(s);
        return 0u;
    }
    if (!s->have_sample)
    {
        s->have_sample = true;
        s->previous_black = black;
        s->last_progress_ms = now;
        return 0u;
    }
    s->position += travel_mm;
    if (travel_mm > 0.0f) s->last_progress_ms = now;
    if (s->active && (now - s->last_progress_ms) >= 1500u)
    {
        buddy3_barcode_reset(s);
        return 0u;
    }
    if (!s->active)
    {
        if (!black)
        {
            s->white_distance += travel_mm;
            if (s->white_distance >= 2.0f * cfg->cell_mm) s->armed = true;
        }
        else
        {
            if (s->armed && !s->previous_black)
            {
                /* Estimate edge halfway between the two observations. */
                s->active = true;
                s->frame = 0u;
                s->bits = 0u;
                s->next_sample = previous_position + travel_mm * 0.5f +
                                 cfg->cell_mm * 0.5f;
            }
            s->white_distance = 0.0f;
            s->armed = false;
        }
    }
    s->previous_black = black;
    if (!s->active || s->position < s->next_sample) return 0u;

    s->frame = (uint16_t)((s->frame << 1u) | (black ? 1u : 0u));
    s->bits++;
    s->next_sample += cfg->cell_mm;
    if (s->bits == 8u && s->frame != 0xA5u)
    {
        buddy3_barcode_reset(s);
        return 0u;
    }
    if (s->bits != 16u) return 0u;
    command = (uint8_t)(s->frame & 0xffu);
    buddy3_barcode_reset(s);
    /* Quiet lead-in must be earned again; repeated samples cannot retrigger. */
    if (command >= 1u && command <= 3u) return command;
    return 0u;
}
