#include "buddy3_steering.h"

#include <limits.h>

void buddy3_steering_reset(buddy3_steering_t *s)
{
    buddy3_steering_t empty = {0};
    *s = empty;
}

buddy3_steering_output_t buddy3_steering_step(buddy3_steering_t *s,
    uint8_t stage, uint8_t side, uint32_t now, int16_t base, int16_t correction)
{
    buddy3_steering_output_t out = {base, base, B3_TRACK};
    int16_t inner;
    int16_t outer;

    /* Keep both wheels moving forward, while applying the requested wheel
       differential immediately.  The old base/5 cap meant the default
       stage-1 controller ignored most of the configured curve power. */
    if (correction < 0) correction = 0;
    if (correction > base) correction = base;
    inner = base - correction;
    outer = (correction > (INT16_MAX - base)) ? INT16_MAX : base + correction;
    if (side != s->side) { s->side = side; s->side_since = now; }
    if (s->stopped) goto stopped;

    /* Both-black is ambiguous. Hold previous correction briefly, then stop.
       This is not junction classification. A stop requires an explicit restart. */
    if (side == 3u) {
        if (!s->black_pending) { s->black_pending = true; s->black_since = now; }
        if ((now - s->black_since) >= B3_BOTH_BLACK_MS) goto stopped;
        out.phase = B3_AMBIGUOUS;
        if (s->direction == 0u) { out.left = 0; out.right = 0; }
        else if (s->direction == 1u) { out.left = inner; out.right = outer; }
        else { out.left = outer; out.right = inner; }
        return out;
    }
    s->black_pending = false;

    if (s->recovering) {
        if ((now - s->recovery_since) >= B3_RECOVERY_MAX_MS) goto stopped;
        if (side == 1u || side == 2u) {
            s->touched = true;
            s->white_pending = false;
        }
        if (s->touched && side == 0u) {
            if (!s->white_pending) { s->white_pending = true; s->white_since = now; }
            out.phase = B3_ALIGN;
            out.left = base / 2;
            out.right = base / 2;
            if ((now - s->white_since) >= B3_ALIGN_WHITE_MS) {
                s->recovering = false;
                s->corner_active = false;
                s->direction = 0u;
            }
            return out;
        }
        out.phase = B3_SEEK;
        if (s->direction == 1u) out.left = 0;
        else out.right = 0;
        return out;
    }

    if (side == 1u || side == 2u) {
        if (s->direction != side) {
            s->corner_active = false;
            s->direction = side;
        }
        out.phase = B3_MICRO;
        if (stage >= 2u && ((now - s->side_since) >= B3_CORRECTION_MS ||
                            s->corner_active)) {
            if (!s->corner_active) { s->corner_since = now; s->corner_active = true; }
            if ((now - s->corner_since) >= B3_CORNER_MAX_MS) goto stopped;
            inner = 0;
            outer = base;
            out.phase = B3_CORNER;
        }
        if (side == 1u) { out.left = inner; out.right = outer; }
        else { out.left = outer; out.right = inner; }
    } else if (s->corner_active) {
        if (stage < 3u) goto stopped;
        s->recovering = true;
        s->recovery_since = now;
        s->touched = false;
        s->white_pending = false;
        out.phase = B3_SEEK;
        if (s->direction == 1u) out.left = 0;
        else out.right = 0;
    } else {
        /* Both-white after micro correction is normal: no blind timeout spin. */
        s->direction = 0u;
    }
    return out;
stopped:
    s->stopped = true;
    out.left = 0;
    out.right = 0;
    out.phase = B3_STOP;
    return out;
}
