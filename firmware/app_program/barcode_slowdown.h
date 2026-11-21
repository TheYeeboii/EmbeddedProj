#ifndef BARCODE_SLOWDOWN_H
#define BARCODE_SLOWDOWN_H

#include <stdbool.h>
#include <stdint.h>

#define BARCODE_WHITE_RELEASE_MS (3000u)

typedef struct
{
    bool active;
    bool timing_white;
    uint32_t white_since_ms;
} barcode_slowdown_t;

/* Any black sample latches slowdown. Release after continuous white >3 s. */
bool barcode_slowdown_update (barcode_slowdown_t * state, bool black,
                               uint32_t now_ms);

#endif
