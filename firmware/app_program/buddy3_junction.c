#include "buddy3_junction.h"

void buddy3_junction_reset(buddy3_junction_t *s)
{
    buddy3_junction_t empty = {0};
    *s = empty;
}
bool buddy3_junction_active(const buddy3_junction_t *s)
{
    return s->phase == B3J_CONFIRM || s->phase == B3J_CROSS;
}
buddy3_junction_output_t buddy3_junction_step(buddy3_junction_t *s,
    uint8_t side, uint32_t now, float left_mm, float right_mm,
    bool encoders_healthy, bool entry_allowed, int16_t base)
{
    buddy3_junction_output_t out = {base, base, B3J_IDLE, false, false};
    float left, right, centre, error, correction;
    if (s->phase == B3J_STOP) goto stopped;
    if (s->phase == B3J_CLEAR) buddy3_junction_reset(s);
    if (s->phase == B3J_IDLE) {
        if (side != 3u || !entry_allowed) return out;
        s->phase = B3J_CONFIRM;
        s->started_ms = now;
        s->start_left = left_mm;
        s->start_right = right_mm;
        s->clear_pending = false;
    }

    if (!encoders_healthy) goto stopped;
    left = left_mm - s->start_left;
    right = right_mm - s->start_right;
    /* Negative/NaN progress includes reversing or odometry reset. */
    if (!(left >= 0.0f) || !(right >= 0.0f)) goto stopped;
    if ((now - s->started_ms) >= B3_JUNCTION_MAX_MS ||
        left >= B3_JUNCTION_MAX_MM || right >= B3_JUNCTION_MAX_MM)
        goto stopped;

    if (s->phase == B3J_CONFIRM) {
        if (side != 3u) {
            buddy3_junction_reset(s);
            return out; /* Brief black/noise: resume ordinary correction. */
        }
        if ((now - s->started_ms) >= B3_JUNCTION_CONFIRM_MS)
            s->phase = B3J_CROSS;
    }

    centre = 0.5f * (left + right);
    error = right - left;
    if (error > B3_JUNCTION_MAX_HEADING_ERROR_MM ||
        error < -B3_JUNCTION_MAX_HEADING_ERROR_MM) goto stopped;
    /* Slow whichever wheel has travelled farther. Never reverse/boost. */
    correction = (error < 0.0f ? -error : error) * B3_JUNCTION_HEADING_PWM_PER_MM;
    if (correction > (float)base / 3.0f) correction = (float)base / 3.0f;
    if (error > 0.0f) out.right -= (int16_t)correction;
    else out.left -= (int16_t)correction;

    if (s->phase == B3J_CROSS) {
        if (side == 0u) {
            if (!s->clear_pending) {
                s->clear_pending = true;
                s->clear_start = centre;
            }
            if (centre >= B3_JUNCTION_MIN_MM &&
                centre - s->clear_start >= B3_JUNCTION_CLEAR_MM)
                s->phase = B3J_CLEAR;
        } else {
            s->clear_pending = false;
        }
    }
    out.phase = s->phase;
    out.handled = true;
    return out;

stopped:
    s->phase = B3J_STOP;
    out.left = 0;
    out.right = 0;
    out.phase = B3J_STOP;
    out.handled = true;
    out.stopped = true;
    return out;
}
