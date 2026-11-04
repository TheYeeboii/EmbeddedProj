#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "buddy3_barcode.h"

/* Independent, human-readable Code 39 fixtures: 1=wide, 0=narrow. */
static const char *const alphabet[] = {
    "100001001", "001001001", "101001000", "000011001", "100011000",
    "001011000", "000001101", "100001100", "001001100", "000011100",
    "100000011", "001000011", "101000010", "000010011", "100010010",
    "001010010", "000000111", "100000110", "001000110", "000010110",
    "110000001", "011000001", "111000000", "010010001", "110010000",
    "011010000"
};
static void frame(char symbol, float narrow, float ratio, bool reverse, float *out)
{
    const char *groups[] = { "010010100", alphabet[symbol - 'A'], "010010100" };
    float forward[29];
    for (unsigned g = 0; g < 3; ++g)
        for (unsigned i = 0; i < 9; ++i)
            forward[g * 10 + i] = narrow * (groups[g][i] == '1' ? ratio : 1.0f);
    forward[9] = forward[19] = narrow;
    for (unsigned i = 0; i < 29; ++i) out[i] = forward[reverse ? 28 - i : i];
}

static void test_parser(void)
{
    const float scales[] = {0.5f, 0.75f, 1.0f, 1.5f, 2.0f};
    float widths[29];
    code39_result_t result;
    for (char c = 'A'; c <= 'Z'; ++c)
        for (unsigned scale = 0; scale < 5; ++scale)
            for (unsigned reverse = 0; reverse < 2; ++reverse)
                for (unsigned ratio = 2; ratio <= 3; ++ratio) {
                    frame(c, scales[scale], (float)ratio, reverse != 0, widths);
                    for (unsigned i = 0; i < 29; ++i)
                        widths[i] *= 1.0f + (i % 3 == 0 ? 0.08f : -0.08f);
                    assert(code39_decode(widths,29,30,30,&result) == CODE39_OK);
                    assert(result.symbol == c && result.reversed == (reverse != 0));
                }
    /* Pixel widths from the supplied photos, top to bottom. Quiet zones here
     * are synthetic: these are image-pattern fixtures, not physical IR traces. */
    float photo_f[] = {14,16,46,16,46,16,15,47,15,17,15,16,15,48,47,17,47,16,16,17,15,17,48,17,48,17,16,50,16};
    float photo_y[] = {13,15,42,15,42,15,14,44,14,15,14,16,14,15,44,16,14,46,45,16,15,16,46,16,47,16,16,49,15};
    assert(code39_decode(photo_f,29,200,200,&result) == CODE39_OK);
    assert(result.symbol == 'F' && result.reversed);
    assert(code39_decode(photo_y,29,200,200,&result) == CODE39_OK);
    assert(result.symbol == 'Y' && result.reversed);
    frame('A',1,3,false,widths);
    assert(code39_decode(widths,28,20,20,&result) == CODE39_BAD_COUNT);
    assert(!result.symbol);
    assert(code39_decode(widths,29,9,20,&result) == CODE39_BAD_QUIET);
    assert(code39_decode(widths,29,20,9,&result) == CODE39_BAD_QUIET);
    widths[9] = 3;
    assert(code39_decode(widths,29,20,20,&result) == CODE39_BAD_WIDTH);
    frame('A',1,3,false,widths); widths[4] = NAN;
    assert(code39_decode(widths,29,20,20,&result) == CODE39_BAD_WIDTH);
    frame('A',1,3,false,widths); widths[4] = 0;
    assert(code39_decode(widths,29,20,20,&result) == CODE39_BAD_WIDTH);
    frame('A',1,3,false,widths);
    for (unsigned i=0;i<9;++i) widths[20+i] = widths[8-i];
    assert(code39_decode(widths,29,20,20,&result) == CODE39_BAD_FRAME);
    frame('A',1,3,false,widths);
    for (unsigned i=0;i<9;++i) widths[10+i] = widths[i]; /* * as payload */
    assert(code39_decode(widths,29,20,20,&result) == CODE39_BAD_SYMBOL);
    for (unsigned i=0;i<29;++i) widths[i] = 1;
    assert(code39_decode(widths,29,20,20,&result) == CODE39_BAD_WIDTH);
}

static unsigned replay(char symbol, float scale, bool reversed, bool variable_speed)
{
    buddy3_barcode_decoder_t s = {0};
    buddy3_barcode_config_t cfg = {0.25f};
    buddy3_barcode_event_t event;
    float widths[29], boundaries[30], pos = 0, ds = 0;
    unsigned valid = 0;
    frame(symbol,scale,3,reversed,widths);
    boundaries[0] = 15 * scale;
    for (unsigned i=0;i<29;++i) boundaries[i+1] = boundaries[i]+widths[i];
    for (uint32_t now=0;pos<boundaries[29]+30*scale;++now) {
        bool black = false;
        for (unsigned i=0;i<29;++i)
            if (pos >= boundaries[i] && pos < boundaries[i+1]) black = i%2 == 0;
        if (buddy3_barcode_step(&s,&cfg,black,ds,now,true,&event)) {
            assert(event.kind == B3BC_VALID);
            assert(event.decoded.symbol == symbol && event.decoded.reversed == reversed);
            assert(s.count == 29);
            for (unsigned i=1;i<29;++i) assert(s.edge_ms[i] > s.edge_ms[i-1]);
            ++valid;
        }
        ds = variable_speed && now%7<3 ? 0.025f : 0.05f;
        pos += ds;
    }
    return valid;
}

static void test_abort(void)
{
    buddy3_barcode_decoder_t s = {0};
    buddy3_barcode_config_t cfg = {4};
    buddy3_barcode_event_t event;
    for (unsigned i=0;i<100;++i)
        assert(!buddy3_barcode_step(&s,&cfg,i%2,0,i,true,&event));
    assert(!s.active && !s.count);
    s.active=true; s.have_sample=true;
    assert(buddy3_barcode_step(&s,&cfg,true,0,1500,true,&event));
    assert(event.kind==B3BC_STALL && !s.active);
    s.active=true;
    assert(buddy3_barcode_step(&s,&cfg,true,-1,1510,true,&event));
    assert(event.kind==B3BC_REVERSE && !s.active);
    s.active=true;
    assert(buddy3_barcode_step(&s,&cfg,true,2,1520,true,&event));
    assert(event.kind==B3BC_SAMPLE_GAP);
    s.active=true;
    assert(buddy3_barcode_step(&s,&cfg,true,1,1530,false,&event));
    assert(event.kind==B3BC_INTERRUPTED);
    s.active=true; s.have_sample=true; s.last_progress_ms=30000;
    assert(buddy3_barcode_step(&s,&cfg,true,1,30000,true,&event));
    assert(event.kind==B3BC_TIMEOUT);
    assert(buddy3_barcode_travel(9,11,100,50)==9);
    assert(buddy3_barcode_travel(9,11,100,-50)==11);
    assert(buddy3_barcode_travel(10,10,100,50)==10);
}
int main(void)
{
    test_parser();
    for (char c='A';c<='Z';++c)
        for (unsigned reverse=0;reverse<2;++reverse) {
            assert(replay(c,0.5f,reverse!=0,false)==1);
            assert(replay(c,2.0f,reverse!=0,true)==1);
        }
    test_abort();
    puts("PASS: adaptive A-Z Code39, both orientations, photo F/Y, noise, malformed frames, distance replay, changing speed, abort guards");
    return 0;
}
