/** @file buddy3_junction.c
 *
 * @brief Buddy 3 cross-junction handling: go straight across a both-black
 *        junction while holding the entry heading with the encoders.
 */
#include "buddy3_junction.h"

#define SIDE_BOTH_WHITE 0u
#define SIDE_BOTH_BLACK 3u
/* The heading correction may slow a wheel by at most a third of base. */
#define MAX_CORRECTION_DIVISOR 3.0f

static bool junction_out_of_limits (const buddy3_junction_t * p_state,
                                    uint32_t now, float left, float right,
                                    bool encoders_healthy);
static bool junction_steer (buddy3_junction_t * p_state, uint8_t side,
                            float left, float right, int16_t base,
                            buddy3_junction_output_t * p_out);

void buddy3_junction_reset (buddy3_junction_t * p_state)
{
    buddy3_junction_t empty = {0};

    *p_state = empty;
}

bool buddy3_junction_active (const buddy3_junction_t * p_state)
{
    return ((B3J_CONFIRM == p_state->phase) || (B3J_CROSS == p_state->phase));
}

buddy3_junction_output_t buddy3_junction_step (buddy3_junction_t * p_state,
                                               uint8_t side, uint32_t now,
                                               float left_mm, float right_mm,
                                               bool encoders_healthy,
                                               bool entry_allowed, int16_t base)
{
    buddy3_junction_output_t out    = {base, base, B3J_IDLE, false, false};
    float                    left   = 0.0f;
    float                    right  = 0.0f;
    bool                     b_stop = (B3J_STOP == p_state->phase);
    bool                     b_idle = false;

    if (!b_stop)
    {
        if (B3J_CLEAR == p_state->phase)
        {
            buddy3_junction_reset(p_state);
        }
        if (B3J_IDLE == p_state->phase)
        {
            if ((SIDE_BOTH_BLACK != side) || !entry_allowed)
            {
                b_idle = true;
            }
            else
            {
                p_state->phase         = B3J_CONFIRM;
                p_state->started_ms    = now;
                p_state->start_left    = left_mm;
                p_state->start_right   = right_mm;
                p_state->clear_pending = false;
            }
        }
    }

    if (!b_stop && !b_idle)
    {
        left   = left_mm - p_state->start_left;
        right  = right_mm - p_state->start_right;
        b_stop = junction_out_of_limits(p_state, now, left, right,
                                        encoders_healthy);
    }

    if (!b_stop && !b_idle && (B3J_CONFIRM == p_state->phase))
    {
        if (SIDE_BOTH_BLACK != side)
        {
            /* Brief black/noise: resume ordinary correction. */
            buddy3_junction_reset(p_state);
            b_idle = true;
        }
        else if ((now - p_state->started_ms) >= B3_JUNCTION_CONFIRM_MS)
        {
            p_state->phase = B3J_CROSS;
        }
        else
        {
            /* Still debouncing: keep driving straight. */
        }
    }

    if (!b_stop && !b_idle)
    {
        b_stop = junction_steer(p_state, side, left, right, base, &out);
    }

    if (b_stop)
    {
        p_state->phase = B3J_STOP;
        out.left       = 0;
        out.right      = 0;
        out.phase      = B3J_STOP;
        out.handled    = true;
        out.stopped    = true;
    }

    return out;
}

/* True when the crossing must stop: unhealthy encoders, negative or NaN
 * progress (reversing or an odometry reset), or too much time or travel. */
static bool junction_out_of_limits (const buddy3_junction_t * p_state,
                                    uint32_t now, float left, float right,
                                    bool encoders_healthy)
{
    return (!encoders_healthy || !(left >= 0.0f) || !(right >= 0.0f) ||
            ((now - p_state->started_ms) >= B3_JUNCTION_MAX_MS) ||
            (left >= B3_JUNCTION_MAX_MM) || (right >= B3_JUNCTION_MAX_MM));
}

/* Hold the entry heading and watch for a distance-qualified white exit.
 * Returns true when the heading error is too large to continue. */
static bool junction_steer (buddy3_junction_t * p_state, uint8_t side,
                            float left, float right, int16_t base,
                            buddy3_junction_output_t * p_out)
{
    float centre     = 0.5f * (left + right);
    float error      = right - left;
    float correction = 0.0f;
    bool  b_stop     = false;

    if ((error > B3_JUNCTION_MAX_HEADING_ERROR_MM) ||
        (error < -B3_JUNCTION_MAX_HEADING_ERROR_MM))
    {
        b_stop = true;
    }
    else
    {
        /* Slow whichever wheel has travelled farther. Never reverse/boost. */
        correction = ((error < 0.0f) ? -error : error) *
                     B3_JUNCTION_HEADING_PWM_PER_MM;
        if (correction > ((float) base / MAX_CORRECTION_DIVISOR))
        {
            correction = (float) base / MAX_CORRECTION_DIVISOR;
        }
        /* 0 <= correction <= base / 3, so it fits in int16_t. */
        if (error > 0.0f)
        {
            p_out->right -= (int16_t) correction;
        }
        else
        {
            p_out->left -= (int16_t) correction;
        }

        if (B3J_CROSS == p_state->phase)
        {
            if (SIDE_BOTH_WHITE == side)
            {
                if (!p_state->clear_pending)
                {
                    p_state->clear_pending = true;
                    p_state->clear_start   = centre;
                }
                if ((centre >= B3_JUNCTION_MIN_MM) &&
                    ((centre - p_state->clear_start) >= B3_JUNCTION_CLEAR_MM))
                {
                    p_state->phase = B3J_CLEAR;
                }
            }
            else
            {
                p_state->clear_pending = false;
            }
        }
        p_out->phase   = p_state->phase;
        p_out->handled = true;
    }

    return b_stop;
}

/*** end of file ***/

