#include "barcode_slowdown.h"

bool barcode_slowdown_update (barcode_slowdown_t * state, bool black,
                               uint32_t now_ms)
{
    if (black)
    {
        state->active = true;
        state->timing_white = false;
    }
    else if (state->active)
    {
        if (!state->timing_white)
        {
            state->white_since_ms = now_ms;
            state->timing_white = true;
        }
        else if ((now_ms - state->white_since_ms) > BARCODE_WHITE_RELEASE_MS)
        {
            state->active = false;
            state->timing_white = false;
        }
    }
    return state->active;
}
