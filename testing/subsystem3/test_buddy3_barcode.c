#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "buddy3_barcode.h"

/* Independent, human-readable Code 39 fixtures: 1=wide, 0=narrow. */
static const char * const alphabet[] = {
    "100001001", "001001001", "101001000", "000011001", "100011000",
    "001011000", "000001101", "100001100", "001001100", "000011100",
    "100000011", "001000011", "101000010", "000010011", "100010010",
    "001010010", "000000111", "100000110", "001000110", "000010110",
    "110000001", "011000001", "111000000", "010010001", "110010000",
    "011010000"};
static void frame (char symbol, float32_t narrow, float32_t ratio, bool reverse,
                   float32_t * out)
{
    const char * groups[] = {"010010100", alphabet[symbol - 'A'], "010010100"};
    float32_t    forward[29];
    for (unsigned group = 0; group < 3; ++group)
    {
        for (unsigned index = 0; index < 9; ++index)
        {
            forward[group * 10 + index] =
                narrow * (('1' == groups[group][index]) ? ratio : 1.0f);
        }
    }
    forward[9] = forward[19] = narrow;
    for (unsigned index = 0; index < 29; ++index)
    {
        out[index] = forward[reverse ? 28 - index : index];
    }
}

static void test_parser (void)
{
    const float32_t scales[] = {0.5f, 0.75f, 1.0f, 1.5f, 2.0f};
    float32_t       widths[29];
    code39_result_t result;
    for (char letter = 'A'; letter <= 'Z'; ++letter)
    {
        for (unsigned scale = 0; scale < 5; ++scale)
        {
            for (unsigned reverse = 0; reverse < 2; ++reverse)
            {
                for (unsigned ratio = 2; ratio <= 3; ++ratio)
                {
                    frame(letter, scales[scale], (float32_t) ratio,
                          reverse != 0, widths);
                    for (unsigned index = 0; index < 29; ++index)
                    {
                        widths[index] *=
                            1.0f + ((0u == (index % 3u)) ? 0.08f : -0.08f);
                    }
                    assert(code39_decode(widths, 29, 30, 30, &result) ==
                           CODE39_OK);
                    assert(result.symbol == letter &&
                           result.reversed == (reverse != 0));
                }
            }
        }
    }
    /* Pixel widths from the supplied photos, top to bottom. Quiet zones here
     * are synthetic: these are image-pattern fixtures, not physical IR traces.
     */
    float32_t photo_f[] = {14, 16, 46, 16, 46, 16, 15, 47, 15, 17,
                           15, 16, 15, 48, 47, 17, 47, 16, 16, 17,
                           15, 17, 48, 17, 48, 17, 16, 50, 16};
    float32_t photo_y[] = {13, 15, 42, 15, 42, 15, 14, 44, 14, 15,
                           14, 16, 14, 15, 44, 16, 14, 46, 45, 16,
                           15, 16, 46, 16, 47, 16, 16, 49, 15};
    assert(CODE39_OK == code39_decode(photo_f, 29, 200, 200, &result));
    assert(('F' == result.symbol) && result.reversed);
    assert(CODE39_OK == code39_decode(photo_y, 29, 200, 200, &result));
    assert(('Y' == result.symbol) && result.reversed);
    frame('A', 1, 3, false, widths);
    assert(CODE39_BAD_COUNT == code39_decode(widths, 28, 20, 20, &result));
    assert(!result.symbol);
    assert(CODE39_BAD_QUIET == code39_decode(widths, 29, 9, 20, &result));
    assert(CODE39_BAD_QUIET == code39_decode(widths, 29, 20, 9, &result));
    widths[9] = 3;
    assert(CODE39_BAD_WIDTH == code39_decode(widths, 29, 20, 20, &result));
    frame('A', 1, 3, false, widths);
    widths[4] = NAN;
    assert(CODE39_BAD_WIDTH == code39_decode(widths, 29, 20, 20, &result));
    frame('A', 1, 3, false, widths);
    widths[4] = 0;
    assert(CODE39_BAD_WIDTH == code39_decode(widths, 29, 20, 20, &result));
    frame('A', 1, 3, false, widths);
    for (unsigned index = 0; index < 9; ++index)
    {
        widths[20 + index] = widths[8 - index];
    }
    assert(CODE39_BAD_FRAME == code39_decode(widths, 29, 20, 20, &result));
    frame('A', 1, 3, false, widths);
    for (unsigned index = 0; index < 9; ++index)
    {
        widths[10 + index] = widths[index]; /* * as payload */
    }
    assert(CODE39_BAD_SYMBOL == code39_decode(widths, 29, 20, 20, &result));
    for (unsigned index = 0; index < 29; ++index)
    {
        widths[index] = 1;
    }
    assert(CODE39_BAD_WIDTH == code39_decode(widths, 29, 20, 20, &result));
}

static unsigned replay (char symbol, float32_t scale, bool reversed,
                        bool variable_speed)
{
    buddy3_barcode_decoder_t decoder = {0};
    buddy3_barcode_config_t  cfg     = {0.25f};
    buddy3_barcode_event_t   event;
    float32_t                widths[29];
    float32_t                boundaries[30];
    float32_t                pos     = 0;
    float32_t                step_mm = 0;
    unsigned                 valid   = 0;
    frame(symbol, scale, 3, reversed, widths);
    boundaries[0] = 15 * scale;
    for (unsigned index = 0; index < 29; ++index)
    {
        boundaries[index + 1] = boundaries[index] + widths[index];
    }
    for (uint32_t now = 0; pos < boundaries[29] + 30 * scale; ++now)
    {
        bool black = false;
        for (unsigned index = 0; index < 29; ++index)
        {
            if (pos >= boundaries[index] && pos < boundaries[index + 1])
            {
                black = (0u == (index % 2u));
            }
        }
        if (buddy3_barcode_step(&decoder, &cfg, black, step_mm, now, true,
                                &event))
        {
            assert(B3BC_VALID == event.kind);
            assert(event.decoded.symbol == symbol &&
                   event.decoded.reversed == reversed);
            assert(29 == decoder.count);
            for (unsigned index = 1; index < 29; ++index)
            {
                assert(decoder.edge_ms[index] > decoder.edge_ms[index - 1]);
            }
            ++valid;
        }
        step_mm = variable_speed && now % 7 < 3 ? 0.025f : 0.05f;
        pos += step_mm;
    }
    return valid;
}

/* Drive a run of widths (black first) under the sensor at 0.05 mm/sample,
 * starting after `lead` mm of white and stopping `tail` mm after the last
 * element. Returns the first valid letter, or 0. */
static char drive (const float32_t * widths, unsigned count, float32_t lead,
                   float32_t tail)
{
    buddy3_barcode_decoder_t decoder = {0};
    buddy3_barcode_config_t  cfg     = {0.25f};
    buddy3_barcode_event_t   event;
    float32_t                end = lead;
    char                     got = 0;
    for (unsigned index = 0; index < count; ++index)
    {
        end += widths[index];
    }
    for (uint32_t now = 0; (now * 0.05f) < (end + tail) && !got; ++now)
    {
        float32_t pos   = now * 0.05f;
        float32_t edge  = lead;
        bool      black = false;
        for (unsigned index = 0; index < count; ++index)
        {
            if (pos >= edge && pos < edge + widths[index])
            {
                black = (0u == (index % 2u));
            }
            edge += widths[index];
        }
        if (buddy3_barcode_step(&decoder, &cfg, black, now ? 0.05f : 0.0f, now,
                                true, &event) &&
            B3BC_VALID == event.kind)
        {
            got = event.decoded.symbol;
        }
    }
    return got;
}


static void test_no_quiet_and_resync (void)
{
    float32_t widths[29 + 4];
    frame('F', 2, 3, false, widths);
    assert('F' == drive(widths, 29, 0.5f, 0.2f)); /* stop at the last bar */
    frame('Y', 2, 3, true, widths);
    assert('Y' == drive(widths, 29, 0.5f, 0.2f));
    /* A stray black mark and gap first. */
    widths[0] = 3;
    widths[1] = 2;
    frame('A', 2, 3, false, widths + 2);
    assert('A' == drive(widths, 31, 0.5f, 0.2f));
    /* Blurred narrow bars (narrows 1.6..2.6, wides 5..6.5) still decode. */
    frame('C', 2, 3, false, widths);
    for (unsigned index = 0; index < 29; ++index)
    {
        widths[index] *=
            (1u == (index % 4u)) ? 1.3f : ((3u == (index % 4u)) ? 0.8f : 1.0f);
    }
    assert('C' == drive(widths, 29, 0.5f, 0.2f));
}

static void test_abort (void)
{
    buddy3_barcode_decoder_t decoder = {0};
    buddy3_barcode_config_t  cfg     = {4};
    buddy3_barcode_event_t   event;
    for (unsigned index = 0; index < 100; ++index)
    {
        assert(!buddy3_barcode_step(&decoder, &cfg, index % 2, 0, index, true,
                                    &event));
    }
    assert(!decoder.active && !decoder.count);
    decoder.active      = true;
    decoder.have_sample = true;
    assert(buddy3_barcode_step(&decoder, &cfg, true, 0, 1500, true, &event));
    assert(B3BC_STALL == event.kind && !decoder.active);
    decoder.active = true;
    assert(buddy3_barcode_step(&decoder, &cfg, true, -1, 1510, true, &event));
    assert(B3BC_REVERSE == event.kind && !decoder.active);
    decoder.active = true;
    assert(buddy3_barcode_step(&decoder, &cfg, true, 2, 1520, true, &event));
    assert(B3BC_SAMPLE_GAP == event.kind);
    decoder.active = true;
    assert(buddy3_barcode_step(&decoder, &cfg, true, 1, 1530, false, &event));
    assert(B3BC_INTERRUPTED == event.kind);
    decoder.active           = true;
    decoder.have_sample      = true;
    decoder.last_progress_ms = 30000;
    assert(buddy3_barcode_step(&decoder, &cfg, true, 1, 30000, true, &event));
    assert(B3BC_TIMEOUT == event.kind);
    assert(9 == buddy3_barcode_travel(9, 11, 100, 50));
    assert(11 == buddy3_barcode_travel(9, 11, 100, -50));
    assert(10 == buddy3_barcode_travel(10, 10, 100, 50));
}
/* A real pass over a small printed *Y* (17 Nov 2026): narrow about 4.5 mm,
 * wide about 14 mm, but the first bar blurred to 10 mm.  Taking the three
 * widest of nine elements made that bar wide and lost the opening '*';
 * comparing bars with bars and spaces with spaces reads the letter. */
static void test_blurred_first_bar (void)
{
    static const float32_t recorded[29] = {
        10.0f, 8.2f,  5.4f, 3.6f, 14.2f, 5.3f, 13.2f, 4.7f, 3.7f, 4.6f,
        14.5f, 13.6f, 5.1f, 4.1f, 14.8f, 5.3f, 4.0f,  4.5f, 5.1f, 4.9f,
        4.8f,  14.2f, 6.3f, 3.7f, 14.8f, 4.5f, 14.5f, 5.0f, 3.4f};
    code39_result_t result = {0};

    assert(code39_is_star(recorded));
    assert(CODE39_OK == code39_decode(recorded, 29, 1000.0f, 1000.0f, &result));
    assert(('Y' == result.symbol) && !result.reversed);
}

int main (void)
{
    test_blurred_first_bar();
    test_parser();
    for (char letter = 'A'; letter <= 'Z'; ++letter)
    {
        for (unsigned reverse = 0; reverse < 2; ++reverse)
        {
            assert(1 == replay(letter, 0.5f, reverse != 0, false));
            assert(1 == replay(letter, 2.0f, reverse != 0, true));
        }
    }
    test_abort();
    test_no_quiet_and_resync();
    puts("PASS: adaptive A-Z Code39, both orientations, photo F/Y, noise, "
         "malformed frames, distance replay, changing speed, abort guards, "
         "blurred first bar (real *Y* pass)");
    return 0;
}

/*** end of file ***/

