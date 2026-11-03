#include <assert.h>
#include <stdio.h>
#include "buddy3_barcode.h"

static unsigned run_frame(unsigned command, unsigned speed_pattern,
                          unsigned marker, bool permitted)
{
    buddy3_barcode_decoder_t s;
    buddy3_barcode_config_t cfg = {10.0f};
    float pos = 0.0f, ds = 0.0f;
    unsigned count = 0u;
    uint32_t now = 0u;
    buddy3_barcode_reset(&s);
    while (pos < 230.0f) {
        bool black = false;
        if (pos >= 30.0f && pos < 190.0f) {
            unsigned bit = (unsigned)((pos - 30.0f) / 10.0f);
            unsigned frame = (marker << 8u) | command;
            black = ((frame >> (15u - bit)) & 1u) != 0u;
        }
        unsigned got = buddy3_barcode_step(&s,&cfg,black,ds,now,permitted);
        if (got) { assert(got == command); ++count; }
        ds = speed_pattern ? ((now % 70u) < 30u ? 0.5f : 1.5f) : 1.0f;
        pos += ds;
        now += 10u;
    }
    return count;
}

int main(void)
{
    buddy3_barcode_decoder_t s;
    buddy3_barcode_config_t cfg = {10.0f};
    for (unsigned cmd=1;cmd<=3;++cmd) {
        assert(run_frame(cmd,0,0xA5,true)==1);
        assert(run_frame(cmd,1,0xA5,true)==1);
    }
    assert(run_frame(4,0,0xA5,true)==0);
    assert(run_frame(1,0,0x5A,true)==0);
    assert(run_frame(1,0,0xA5,false)==0);
    buddy3_barcode_reset(&s);
    for (unsigned n=0;n<100;++n)
        assert(buddy3_barcode_step(&s,&cfg,n%2,0,10*n,true)==0);
    assert(!s.active); /* Stationary flicker cannot earn a quiet lead-in. */
    s.active=true; s.have_sample=true; s.last_progress_ms=0;
    buddy3_barcode_step(&s,&cfg,true,0,1500,true);
    assert(!s.active);
    s.active=true;
    buddy3_barcode_step(&s,&cfg,true,3,1510,true);
    assert(!s.active); /* Reject skipped cells rather than invent samples. */
    s.active=true;
    buddy3_barcode_step(&s,&cfg,true,-1,1520,true);
    assert(!s.active);
    cfg.cell_mm=0;
    s.active=true;
    buddy3_barcode_step(&s,&cfg,true,1,1530,true);
    assert(!s.active);
    assert(buddy3_barcode_travel(9,11,100,50)==9.0f);
    assert(buddy3_barcode_travel(9,11,100,-50)==11.0f);
    assert(buddy3_barcode_travel(10,10,100,50)==10.0f);
    puts("PASS: all commands, changing speed, invalid frames, no-motion, stall, gaps, reverse, disabled input, sensor-offset geometry");
    return 0;
}
