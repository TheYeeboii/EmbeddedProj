/** @file buddy3_steering.c
 *
 * @brief Buddy 3 line-steering state machine: micro correction, corners,
 *        recovery and the both-black stop.
 */
#include "buddy3_steering.h"

#include <limits.h>

/* Sensor patterns passed as "side". */
#define SIDE_BOTH_WHITE 0u
#define SIDE_LEFT       1u
#define SIDE_RIGHT      2u
#define SIDE_BOTH_BLACK 3u

static void steer_seek_output (uint8_t                    direction,
                               buddy3_steering_output_t * p_out);
static bool steer_ambiguous (buddy3_steering_t * p_state, uint32_t now,
                             int16_t inner, int16_t outer,
                             buddy3_steering_output_t * p_out);
static bool steer_recovery (buddy3_steering_t * p_state, uint8_t side,
                            uint32_t now, int16_t base,
                            buddy3_steering_output_t * p_out);
static bool steer_tracking (buddy3_steering_t * p_state, uint8_t side,
                            uint32_t now, int16_t base, int16_t inner,
                            int16_t outer, buddy3_steering_output_t * p_out);

void buddy3_steering_reset (buddy3_steering_t * p_state)
{
    buddy3_steering_t empty = {0};

    *p_state = empty;
}

buddy3_steering_output_t buddy3_steering_step (buddy3_steering_t * p_state,
                                               uint8_t side,
                                               uint32_t now, int16_t base,
                                               int16_t correction)
{
    buddy3_steering_output_t out    = {base, base, B3_TRACK};
    int16_t                  inner  = 0;
    int16_t                  outer  = 0;
    bool                     b_stop = false;

    /* Keep both wheels moving forward, while applying the requested wheel
     * differential immediately.  The old base/5 cap meant the default
     * controller ignored most of the configured curve power. */
    if (correction < 0)
    {
        correction = 0;
    }
    if (correction > base)
    {
        correction = base;
    }
    inner = base - correction;
    outer = (correction > (INT16_MAX - base)) ? INT16_MAX : base + correction;

    if (side != p_state->side)
    {
        p_state->side       = side;
        p_state->side_since = now;
    }

    b_stop = p_state->stopped;
    if (!b_stop)
    {
        if (SIDE_BOTH_BLACK == side)
        {
            b_stop = steer_ambiguous(p_state, now, inner, outer, &out);
        }
        else
        {
            p_state->black_pending = false;
            if (p_state->recovering)
            {
                b_stop = steer_recovery(p_state, side, now, base, &out);
            }
            else
            {
                b_stop = steer_tracking(p_state, side, now, base, inner,
                                        outer, &out);
            }
        }
    }

    /* A stop is latched and requires an explicit buddy3_steering_reset(). */
    if (b_stop)
    {
        p_state->stopped = true;
        out.left         = 0;
        out.right        = 0;
        out.phase        = B3_STOP;
    }

    return out;
}

/* Turn back towards the line: hold the wheel on the side we last saw it. */
static void steer_seek_output (uint8_t                    direction,
                               buddy3_steering_output_t * p_out)
{
    p_out->phase = B3_SEEK;
    if (SIDE_LEFT == direction)
    {
        p_out->left = 0;
    }
    else
    {
        p_out->right = 0;
    }
}

/* Both-black is ambiguous. Hold the previous correction briefly, then stop.
 * This is not junction classification. Returns true when the car must stop. */
static bool steer_ambiguous (buddy3_steering_t * p_state, uint32_t now,
                             int16_t inner, int16_t outer,
                             buddy3_steering_output_t * p_out)
{
    bool b_stop = false;

    if (!p_state->black_pending)
    {
        p_state->black_pending = true;
        p_state->black_since   = now;
    }

    if ((now - p_state->black_since) >= B3_BOTH_BLACK_MS)
    {
        b_stop = true;
    }
    else
    {
        p_out->phase = B3_AMBIGUOUS;
        if (SIDE_BOTH_WHITE == p_state->direction)
        {
            p_out->left  = 0;
            p_out->right = 0;
        }
        else if (SIDE_LEFT == p_state->direction)
        {
            p_out->left  = inner;
            p_out->right = outer;
        }
        else
        {
            p_out->left  = outer;
            p_out->right = inner;
        }
    }

    return b_stop;
}

/* After a lost corner: seek until an edge is touched, then align once the
 * sensors read white for long enough. Returns true when the car must stop. */
static bool steer_recovery (buddy3_steering_t * p_state, uint8_t side,
                            uint32_t now, int16_t base,
                            buddy3_steering_output_t * p_out)
{
    bool b_stop = false;

    if ((now - p_state->recovery_since) >= B3_RECOVERY_MAX_MS)
    {
        b_stop = true;
    }
    else
    {
        if ((SIDE_LEFT == side) || (SIDE_RIGHT == side))
        {
            p_state->touched       = true;
            p_state->white_pending = false;
        }

        if (p_state->touched && (SIDE_BOTH_WHITE == side))
        {
            if (!p_state->white_pending)
            {
                p_state->white_pending = true;
                p_state->white_since   = now;
            }
            p_out->phase = B3_ALIGN;
            p_out->left  = base / 2;
            p_out->right = base / 2;
            if ((now - p_state->white_since) >= B3_ALIGN_WHITE_MS)
            {
                p_state->recovering    = false;
                p_state->corner_active = false;
                p_state->direction     = SIDE_BOTH_WHITE;
            }
        }
        else
        {
            steer_seek_output(p_state->direction, p_out);
        }
    }

    return b_stop;
}

/* Normal tracking: micro correction, corner holding and the start of
 * recovery when a corner is lost. Returns true when the car must stop. */
static bool steer_tracking (buddy3_steering_t * p_state, uint8_t side,
                            uint32_t now, int16_t base, int16_t inner,
                            int16_t outer, buddy3_steering_output_t * p_out)
{
    bool b_stop = false;

    if ((SIDE_LEFT == side) || (SIDE_RIGHT == side))
    {
        if (p_state->direction != side)
        {
            p_state->corner_active = false;
            p_state->direction     = side;
        }
        p_out->phase = B3_MICRO;
        if (((now - p_state->side_since) >= B3_CORRECTION_MS) ||
            p_state->corner_active)
        {
            /* A continuously seen edge can be a long circular section of the
             * track. It is evidence to keep turning, not a reason to stop. */
            p_state->corner_active = true;
            inner                  = 0;
            outer                  = base;
            p_out->phase           = B3_CORNER;
        }
        if (SIDE_LEFT == side)
        {
            p_out->left  = inner;
            p_out->right = outer;
        }
        else
        {
            p_out->left  = outer;
            p_out->right = inner;
        }
    }
    else if (p_state->corner_active)
    {
        p_state->recovering     = true;
        p_state->recovery_since = now;
        p_state->touched        = false;
        p_state->white_pending  = false;
        steer_seek_output(p_state->direction, p_out);
    }
    else
    {
        /* Both-white after micro correction is normal: no blind timeout. */
        p_state->direction = SIDE_BOTH_WHITE;
    }

    return b_stop;
}

/*** end of file ***/

