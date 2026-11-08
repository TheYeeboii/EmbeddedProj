/** @file code39.c
 *
 * @brief Decoder for a single framed Code 39 letter (*A* to *Z*) from a list
 *        of measured element widths.
 */
#include "code39.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>

/* Each character is nine elements: six narrow and three wide. Characters
 * are separated by one narrow gap, so they start every ten elements. */
#define CHAR_ELEMENTS   9u
#define CHAR_NARROW     6u
#define CHAR_WIDE       3u
#define CHAR_STRIDE     10u
#define FRAME_CHARS     3u
#define LETTER_COUNT    26u
#define STAR            0x094u
#define REVERSE_STAR    0x052u
#define QUIET_NARROWS   10.0f
#define GAP_TOLERANCE   0.3f
#define NARROW_SPREAD   1.3f
#define ELEMENT_TOLERANCE 0.25f
#define MIN_WIDE_RATIO  1.6f
#define MAX_WIDE_RATIO  3.8f

/* Code 39 encodings: wide=1, nine elements, first bar in bit 8.
 * Reference: ZXing Code39Reader CHARACTER_ENCODINGS (A through Z). */
static const uint16_t g_letters[LETTER_COUNT] = {
    0x109u, 0x049u, 0x148u, 0x019u, 0x118u, 0x058u, 0x00du, 0x10cu, 0x04cu,
    0x01cu, 0x103u, 0x043u, 0x142u, 0x013u, 0x112u, 0x052u, 0x007u, 0x106u,
    0x046u, 0x016u, 0x181u, 0x0c1u, 0x1c0u, 0x091u, 0x190u, 0x0d0u};

static bool code39_classify (const float * p_widths, uint16_t * p_pattern,
                             float * p_narrow, float * p_wide);
static code39_status_t code39_check_frame (const float * p_widths,
                                           const float * p_narrow,
                                           const float * p_wide,
                                           float leading_quiet,
                                           float trailing_quiet,
                                           code39_result_t * p_decoded);

code39_status_t code39_decode (const float * p_widths, uint32_t count,
                               float leading_quiet, float trailing_quiet,
                               code39_result_t * p_result)
{
    code39_result_t decoded = {0};
    code39_status_t status  = CODE39_OK;
    uint16_t        patterns[FRAME_CHARS] = {0u};
    float           narrow[FRAME_CHARS]   = {0.0f};
    float           wide[FRAME_CHARS]     = {0.0f};
    float           canonical[CODE39_FRAME_ELEMENTS];
    uint32_t        index  = 0u;
    uint32_t        letter = 0u;

    if (NULL == p_result)
    {
        status = CODE39_BAD_FRAME;
    }
    else if ((NULL == p_widths) || (CODE39_FRAME_ELEMENTS != count))
    {
        status = CODE39_BAD_COUNT;
    }
    else
    {
        for (index = 0u; (CODE39_OK == status) && (index < FRAME_CHARS);
             ++index)
        {
            if (!code39_classify(p_widths + (index * CHAR_STRIDE),
                                 &patterns[index], &narrow[index],
                                 &wide[index]))
            {
                status = CODE39_BAD_WIDTH;
            }
        }
    }

    /* A frame read right-to-left starts and ends with a reversed star. */
    if ((CODE39_OK == status) && (REVERSE_STAR == patterns[0]) &&
        (REVERSE_STAR == patterns[2]))
    {
        decoded.reversed = true;
        for (index = 0u; index < CODE39_FRAME_ELEMENTS; ++index)
        {
            canonical[index] = p_widths[CODE39_FRAME_ELEMENTS - 1u - index];
        }
        p_widths = canonical;
        for (index = 0u; index < FRAME_CHARS; ++index)
        {
            (void) code39_classify(p_widths + (index * CHAR_STRIDE),
                                   &patterns[index], &narrow[index],
                                   &wide[index]);
        }
    }

    if ((CODE39_OK == status) &&
        ((STAR != patterns[0]) || (STAR != patterns[2])))
    {
        status = CODE39_BAD_FRAME;
    }

    if (CODE39_OK == status)
    {
        status = code39_check_frame(p_widths, narrow, wide, leading_quiet,
                                    trailing_quiet, &decoded);
    }

    if (CODE39_OK == status)
    {
        letter = 0u;
        while ((letter < LETTER_COUNT) && (g_letters[letter] != patterns[1]))
        {
            ++letter;
        }
        if (LETTER_COUNT == letter)
        {
            status = CODE39_BAD_SYMBOL;
        }
        else
        {
            /* letter < 26, so 'A' + letter is an uppercase ASCII letter. */
            decoded.symbol = (char) ('A' + letter);
        }
    }

    if (CODE39_OK == status)
    {
        for (index = 0u; index < CODE39_FRAME_ELEMENTS; ++index)
        {
            decoded.frame_width += p_widths[index];
        }
        if (!isfinite(decoded.frame_width))
        {
            status = CODE39_BAD_WIDTH;
        }
    }

    /* The output is cleared on rejection and only filled on success. */
    if (NULL != p_result)
    {
        if (CODE39_OK == status)
        {
            *p_result = decoded;
        }
        else
        {
            code39_result_t empty = {0};

            *p_result = empty;
        }
    }

    return status;
}

const char * code39_status_name (code39_status_t status)
{
    static const char * const names[] = {
        "ok", "element_count", "width", "markers", "symbol", "quiet_zone"};
    const char * p_name = "unknown";

    /* The enum is non-negative, so the unsigned compare is a range check. */
    if ((uint32_t) status < (sizeof(names) / sizeof(names[0])))
    {
        p_name = names[status];
    }

    return p_name;
}

/* Split nine element widths into narrow and wide and build the 9-bit
 * pattern (wide=1). Returns false when the widths are not a clean 2:1 to
 * 3:1 character. */
static bool code39_classify (const float * p_widths, uint16_t * p_pattern,
                             float * p_narrow, float * p_wide)
{
    float    sorted[CHAR_ELEMENTS];
    uint32_t index   = 0u;
    uint32_t slot    = 0u;
    bool     b_valid = true;

    for (index = 0u; b_valid && (index < CHAR_ELEMENTS); ++index)
    {
        if (!isfinite(p_widths[index]) || !(p_widths[index] > 0.0f))
        {
            b_valid = false;
        }
        else
        {
            sorted[index] = p_widths[index];
        }
    }

    if (b_valid)
    {
        /* Insertion sort: nine elements, ascending. */
        for (index = 1u; index < CHAR_ELEMENTS; ++index)
        {
            float value = sorted[index];

            for (slot = index; (slot > 0u) && (sorted[slot - 1u] > value);
                 --slot)
            {
                sorted[slot] = sorted[slot - 1u];
            }
            sorted[slot] = value;
        }

        *p_narrow = 0.0f;
        *p_wide   = 0.0f;
        for (index = 0u; index < CHAR_NARROW; ++index)
        {
            *p_narrow += sorted[index] / (float) CHAR_NARROW;
        }
        for (index = CHAR_NARROW; index < CHAR_ELEMENTS; ++index)
        {
            *p_wide += sorted[index] / (float) CHAR_WIDE;
        }

        /* Allow measurement noise around nominal 2:1 and 3:1 print ratios. */
        if ((*p_wide < (MIN_WIDE_RATIO * *p_narrow)) ||
            (*p_wide > (MAX_WIDE_RATIO * *p_narrow)))
        {
            b_valid = false;
        }
    }

    if (b_valid)
    {
        float threshold = (sorted[CHAR_NARROW - 1u] + sorted[CHAR_NARROW]) *
                          0.5f;

        *p_pattern = 0u;
        for (index = 0u; b_valid && (index < CHAR_ELEMENTS); ++index)
        {
            bool  b_wide   = (p_widths[index] > threshold);
            float expected = b_wide ? *p_wide : *p_narrow;

            if (fabsf(p_widths[index] - expected) >
                (ELEMENT_TOLERANCE * expected))
            {
                b_valid = false;
            }
            else
            {
                /* At most nine bits are ever set, so the pattern fits. */
                *p_pattern = (uint16_t) ((uint32_t) (*p_pattern << 1u) |
                                         (b_wide ? 1u : 0u));
            }
        }
    }

    return b_valid;
}

/* Frame checks that need all three characters: consistent narrow width,
 * narrow inter-character gaps and quiet zones of at least ten narrows.
 * Fills the averaged widths in p_decoded. */
static code39_status_t code39_check_frame (const float * p_widths,
                                           const float * p_narrow,
                                           const float * p_wide,
                                           float leading_quiet,
                                           float trailing_quiet,
                                           code39_result_t * p_decoded)
{
    code39_status_t status     = CODE39_OK;
    float           max_narrow = 0.0f;
    float           min_narrow = INFINITY;
    uint32_t        index      = 0u;

    for (index = 0u; index < FRAME_CHARS; ++index)
    {
        if (p_narrow[index] > max_narrow)
        {
            max_narrow = p_narrow[index];
        }
        if (p_narrow[index] < min_narrow)
        {
            min_narrow = p_narrow[index];
        }
        p_decoded->narrow_width += p_narrow[index] / (float) FRAME_CHARS;
        p_decoded->wide_ratio +=
            (p_wide[index] / p_narrow[index]) / (float) FRAME_CHARS;
    }

    if (max_narrow > (NARROW_SPREAD * min_narrow))
    {
        status = CODE39_BAD_WIDTH;
    }

    /* The two gaps sit after the first and second characters. */
    for (index = CHAR_ELEMENTS;
         (CODE39_OK == status) && (index < (CHAR_STRIDE * 2u));
         index += CHAR_STRIDE)
    {
        if (!isfinite(p_widths[index]) ||
            (fabsf(p_widths[index] - p_decoded->narrow_width) >
             (GAP_TOLERANCE * p_decoded->narrow_width)))
        {
            status = CODE39_BAD_WIDTH;
        }
    }

    if ((CODE39_OK == status) &&
        (!isfinite(leading_quiet) || !isfinite(trailing_quiet) ||
         (leading_quiet < (QUIET_NARROWS * max_narrow)) ||
         (trailing_quiet < (QUIET_NARROWS * max_narrow))))
    {
        status = CODE39_BAD_QUIET;
    }

    return status;
}

/*** end of file ***/

