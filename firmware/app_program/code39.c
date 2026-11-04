#include "code39.h"
#include <math.h>
#include <stdint.h>

/* Code 39 encodings: wide=1, nine elements, first bar in bit 8.
 * Reference: ZXing Code39Reader CHARACTER_ENCODINGS (A through Z). */
static const uint16_t letters[26] = {
    0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00d, 0x10c, 0x04c,
    0x01c, 0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106,
    0x046, 0x016, 0x181, 0x0c1, 0x1c0, 0x091, 0x190, 0x0d0
};
#define STAR 0x094u
#define REVERSE_STAR 0x052u

static bool classify(const float *widths, uint16_t *pattern,
                     float *narrow, float *wide)
{
    float sorted[9];
    unsigned i, j;
    for (i = 0; i < 9; ++i) {
        if (!isfinite(widths[i]) || !(widths[i] > 0.0f)) return false;
        sorted[i] = widths[i];
    }
    for (i = 1; i < 9; ++i) {
        float value = sorted[i];
        for (j = i; j > 0 && sorted[j - 1] > value; --j)
            sorted[j] = sorted[j - 1];
        sorted[j] = value;
    }
    *narrow = *wide = 0.0f;
    for (i = 0; i < 6; ++i) *narrow += sorted[i] / 6.0f;
    for (i = 6; i < 9; ++i) *wide += sorted[i] / 3.0f;
    /* Allow measurement noise around nominal 2:1 and 3:1 print ratios. */
    if (*wide < 1.6f * *narrow || *wide > 3.8f * *narrow) return false;
    *pattern = 0;
    for (i = 0; i < 9; ++i) {
        bool is_wide = widths[i] > (sorted[5] + sorted[6]) * 0.5f;
        float expected = is_wide ? *wide : *narrow;
        if (fabsf(widths[i] - expected) > 0.25f * expected) return false;
        *pattern = (uint16_t)((*pattern << 1) | is_wide);
    }
    return true;
}

code39_status_t code39_decode(const float *widths, unsigned count,
    float leading_quiet, float trailing_quiet, code39_result_t *result)
{
    code39_result_t decoded = {0};
    uint16_t patterns[3];
    float narrow[3], wide[3], canonical[CODE39_FRAME_ELEMENTS];
    float max_narrow = 0.0f, min_narrow = INFINITY;
    unsigned i, letter;
    if (!result) return CODE39_BAD_FRAME;
    *result = decoded;
    if (!widths || count != CODE39_FRAME_ELEMENTS) return CODE39_BAD_COUNT;
    for (i = 0; i < 3; ++i) {
        if (!classify(widths + i * 10, &patterns[i], &narrow[i], &wide[i]))
            return CODE39_BAD_WIDTH;
    }
    if (patterns[0] == REVERSE_STAR && patterns[2] == REVERSE_STAR) {
        decoded.reversed = true;
        for (i = 0; i < CODE39_FRAME_ELEMENTS; ++i)
            canonical[i] = widths[CODE39_FRAME_ELEMENTS - 1 - i];
        widths = canonical;
        for (i = 0; i < 3; ++i)
            (void)classify(widths + i * 10, &patterns[i], &narrow[i], &wide[i]);
    }
    if (patterns[0] != STAR || patterns[2] != STAR) return CODE39_BAD_FRAME;
    for (i = 0; i < 3; ++i) {
        if (narrow[i] > max_narrow) max_narrow = narrow[i];
        if (narrow[i] < min_narrow) min_narrow = narrow[i];
        decoded.narrow_width += narrow[i] / 3.0f;
        decoded.wide_ratio += (wide[i] / narrow[i]) / 3.0f;
    }
    if (max_narrow > 1.3f * min_narrow) return CODE39_BAD_WIDTH;
    for (i = 9; i <= 19; i += 10)
        if (!isfinite(widths[i]) ||
            fabsf(widths[i] - decoded.narrow_width) > 0.3f * decoded.narrow_width)
            return CODE39_BAD_WIDTH;
    if (!isfinite(leading_quiet) || !isfinite(trailing_quiet) ||
        leading_quiet < 10.0f * max_narrow || trailing_quiet < 10.0f * max_narrow)
        return CODE39_BAD_QUIET;
    for (letter = 0; letter < 26; ++letter)
        if (letters[letter] == patterns[1]) break;
    if (letter == 26) return CODE39_BAD_SYMBOL;
    decoded.symbol = (char)('A' + letter);
    for (i = 0; i < CODE39_FRAME_ELEMENTS; ++i) decoded.frame_width += widths[i];
    if (!isfinite(decoded.frame_width)) return CODE39_BAD_WIDTH;
    *result = decoded;
    return CODE39_OK;
}

const char *code39_status_name(code39_status_t status)
{
    static const char *const names[] = {
        "ok", "element_count", "width", "markers", "symbol", "quiet_zone"
    };
    return (unsigned)status < sizeof(names) / sizeof(names[0]) ?
           names[status] : "unknown";
}
