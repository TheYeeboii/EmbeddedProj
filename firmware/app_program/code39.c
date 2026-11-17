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
#define CHAR_ELEMENTS 9u
#define CHAR_NARROW   6u
#define CHAR_WIDE     3u
#define CHAR_STRIDE   10u
#define FRAME_CHARS   3u
#define LETTER_COUNT  26u
#define STAR          0x094u
#define REVERSE_STAR  0x052u
#define QUIET_NARROWS 10.0f
/* Every letter and the '*' has two wide bars and one wide space.  Bars are
 * at even positions (a character starts on black), spaces at odd ones. */
#define KIND_BAR    0u
#define KIND_SPACE  1u
#define KIND_COUNT  2u
#define KIND_NARROW 3u /* narrow elements of each kind in a character */
/* Each wide element must be at least this much wider than the mean narrow
 * element of its own kind; no fixed print ratio or per-element tolerance,
 * so blurred edges still decode. */
#define MIN_WIDE_RATIO 1.4f

/* Wide elements of each kind (bars, spaces) in one character. */
static const uint32_t g_wide_per_kind[KIND_COUNT] = {2u, 1u};

/* Code 39 encodings: wide=1, nine elements, first bar in bit 8.
 * Reference: ZXing Code39Reader CHARACTER_ENCODINGS (A through Z). */
static const uint16_t g_letters[LETTER_COUNT] = {
    0x109u, 0x049u, 0x148u, 0x019u, 0x118u, 0x058u, 0x00du, 0x10cu, 0x04cu,
    0x01cu, 0x103u, 0x043u, 0x142u, 0x013u, 0x112u, 0x052u, 0x007u, 0x106u,
    0x046u, 0x016u, 0x181u, 0x0c1u, 0x1c0u, 0x091u, 0x190u, 0x0d0u};

static void code39_pick_wide (const float32_t * p_widths, uint32_t kind,
                              bool * p_wide);
static bool code39_classify (const float32_t * p_widths, uint16_t * p_pattern,
                             float32_t * p_narrow, float32_t * p_wide,
                             float32_t * p_threshold);
static code39_status_t
code39_check_frame (const float32_t * p_widths, const float32_t * p_narrow,
                    const float32_t * p_wide, const float32_t * p_threshold,
                    float32_t leading_quiet, float32_t trailing_quiet,
                    code39_result_t * p_decoded);
static code39_status_t code39_finish (const float32_t * p_widths,
                                      uint16_t          pattern,
                                      code39_result_t * p_decoded);

code39_status_t code39_decode (const float32_t * p_widths, uint32_t count,
                               float32_t         leading_quiet,
                               float32_t         trailing_quiet,
                               code39_result_t * p_result)
{
    code39_result_t decoded                = {0};
    code39_status_t status                 = CODE39_OK;
    uint16_t        patterns[FRAME_CHARS]  = {0u};
    float32_t       narrow[FRAME_CHARS]    = {0.0f};
    float32_t       wide[FRAME_CHARS]      = {0.0f};
    float32_t       threshold[FRAME_CHARS] = {0.0f};
    float32_t       canonical[CODE39_FRAME_ELEMENTS];
    uint32_t        index = 0u;

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
                                 &patterns[index], &narrow[index], &wide[index],
                                 &threshold[index]))
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
                                   &wide[index], &threshold[index]);
        }
    }

    if ((CODE39_OK == status) &&
        ((STAR != patterns[0]) || (STAR != patterns[2])))
    {
        status = CODE39_BAD_FRAME;
    }

    if (CODE39_OK == status)
    {
        status = code39_check_frame(p_widths, narrow, wide, threshold,
                                    leading_quiet, trailing_quiet, &decoded);
    }

    if (CODE39_OK == status)
    {
        status = code39_finish(p_widths, patterns[1], &decoded);
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

/* Mark the widest elements of one kind (KIND_BAR or KIND_SPACE) as wide. */
static void code39_pick_wide (const float32_t * p_widths, uint32_t kind,
                              bool * p_wide)
{
    uint32_t pick  = 0u;
    uint32_t index = 0u;

    for (pick = 0u; pick < g_wide_per_kind[kind]; ++pick)
    {
        uint32_t best = CHAR_ELEMENTS;

        for (index = kind; index < CHAR_ELEMENTS; index += KIND_COUNT)
        {
            if (!p_wide[index] &&
                ((CHAR_ELEMENTS == best) || (p_widths[index] > p_widths[best])))
            {
                best = index;
            }
        }
        p_wide[best] = true;
    }
}

/* Split nine element widths into narrow and wide and build the 9-bit
 * pattern (wide=1): the two widest bars and the widest space are wide.
 * Bars are only compared with bars and spaces with spaces: the IR spot makes
 * black read wider than white, and the first bar of a pass often wider
 * still, so a narrow bar can outgrow a wide space.  Returns false when a
 * width is invalid or a wide element is not clearly wider than the narrow
 * ones of its kind.  The gap threshold is between the spaces' classes, as
 * the gaps between characters are spaces. */
static bool code39_classify (const float32_t * p_widths, uint16_t * p_pattern,
                             float32_t * p_narrow, float32_t * p_wide,
                             float32_t * p_threshold)
{
    bool      wide[CHAR_ELEMENTS]    = {false};
    float32_t min_wide[KIND_COUNT]   = {INFINITY, INFINITY};
    float32_t narrow_sum[KIND_COUNT] = {0.0f, 0.0f};
    float32_t max_narrow_space       = 0.0f;
    uint32_t  index                  = 0u;
    uint32_t  kind                   = 0u;
    bool      b_valid                = true;

    for (index = 0u; b_valid && (index < CHAR_ELEMENTS); ++index)
    {
        if (!isfinite(p_widths[index]) || !(p_widths[index] > 0.0f))
        {
            b_valid = false;
        }
    }

    if (b_valid)
    {
        code39_pick_wide(p_widths, KIND_BAR, wide);
        code39_pick_wide(p_widths, KIND_SPACE, wide);
        *p_narrow  = 0.0f;
        *p_wide    = 0.0f;
        *p_pattern = 0u;
        for (index = 0u; index < CHAR_ELEMENTS; ++index)
        {
            kind = index % KIND_COUNT;
            if (wide[index])
            {
                *p_wide += p_widths[index] / (float32_t) CHAR_WIDE;
                if (p_widths[index] < min_wide[kind])
                {
                    min_wide[kind] = p_widths[index];
                }
            }
            else
            {
                *p_narrow += p_widths[index] / (float32_t) CHAR_NARROW;
                narrow_sum[kind] += p_widths[index];
                if ((KIND_SPACE == kind) &&
                    (p_widths[index] > max_narrow_space))
                {
                    max_narrow_space = p_widths[index];
                }
            }
            /* At most nine bits are ever set, so the pattern fits. */
            *p_pattern = (uint16_t) ((uint32_t) (*p_pattern << 1u) |
                                     (wide[index] ? 1u : 0u));
        }
        *p_threshold = (min_wide[KIND_SPACE] + max_narrow_space) * 0.5f;
        for (kind = 0u; kind < KIND_COUNT; ++kind)
        {
            b_valid = b_valid &&
                      (min_wide[kind] >= ((MIN_WIDE_RATIO * narrow_sum[kind]) /
                                          (float32_t) KIND_NARROW));
        }
    }

    return b_valid;
}

bool code39_is_star (const float32_t * p_widths)
{
    uint16_t  pattern   = 0u;
    float32_t narrow    = 0.0f;
    float32_t wide      = 0.0f;
    float32_t threshold = 0.0f;

    return (NULL != p_widths) &&
           code39_classify(p_widths, &pattern, &narrow, &wide, &threshold) &&
           ((STAR == pattern) || (REVERSE_STAR == pattern));
}

/* Frame checks that need all three characters: inter-character gaps
 * narrower than both neighbours' wide/narrow threshold and quiet zones of
 * at least ten narrows. Fills the averaged widths in p_decoded. */
static code39_status_t
code39_check_frame (const float32_t * p_widths, const float32_t * p_narrow,
                    const float32_t * p_wide, const float32_t * p_threshold,
                    float32_t leading_quiet, float32_t trailing_quiet,
                    code39_result_t * p_decoded)
{
    code39_status_t status     = CODE39_OK;
    float32_t       max_narrow = 0.0f;
    uint32_t        index      = 0u;

    for (index = 0u; index < FRAME_CHARS; ++index)
    {
        if (p_narrow[index] > max_narrow)
        {
            max_narrow = p_narrow[index];
        }
        p_decoded->narrow_width += p_narrow[index] / (float32_t) FRAME_CHARS;
        p_decoded->wide_ratio +=
            (p_wide[index] / p_narrow[index]) / (float32_t) FRAME_CHARS;
    }

    /* The two gaps sit after the first and second characters. */
    for (index = 0u; (CODE39_OK == status) && (index < (FRAME_CHARS - 1u));
         ++index)
    {
        float32_t gap = p_widths[(index * CHAR_STRIDE) + CHAR_ELEMENTS];

        if (!isfinite(gap) || !(gap > 0.0f) || (gap >= p_threshold[index]) ||
            (gap >= p_threshold[index + 1u]))
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

/* Look up the middle character as a letter and total the frame width.
 * Returns CODE39_OK, CODE39_BAD_SYMBOL or CODE39_BAD_WIDTH. */
static code39_status_t code39_finish (const float32_t * p_widths,
                                      uint16_t          pattern,
                                      code39_result_t * p_decoded)
{
    code39_status_t status = CODE39_OK;
    uint32_t        letter = 0u;
    uint32_t        index  = 0u;

    while ((letter < LETTER_COUNT) && (g_letters[letter] != pattern))
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
        p_decoded->symbol = (char) ('A' + letter);
        for (index = 0u; index < CODE39_FRAME_ELEMENTS; ++index)
        {
            p_decoded->frame_width += p_widths[index];
        }
        if (!isfinite(p_decoded->frame_width))
        {
            status = CODE39_BAD_WIDTH;
        }
    }

    return status;
}

/*** end of file ***/

