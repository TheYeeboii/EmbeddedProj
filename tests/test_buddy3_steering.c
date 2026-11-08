#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "buddy3_steering.h"

static buddy3_steering_output_t step (buddy3_steering_t * s, unsigned stage,
                                      unsigned side, uint32_t now)
{
    return buddy3_steering_step(s, stage, side, now, 12000, 2000);
}
int main (void)
{
    buddy3_steering_t        s;
    buddy3_steering_output_t o;
    buddy3_steering_reset(&s);
    o = step(&s, 1, 1, 0);
    assert(10000 == o.left && 14000 == o.right && B3_MICRO == o.phase);
    o = step(&s, 1, 1, 1000);
    assert(B3_MICRO == o.phase);
    o = step(&s, 1, 0, 1010);
    assert(12000 == o.left && 12000 == o.right);
    o = step(&s, 1, 0, 9000);
    assert(B3_TRACK == o.phase);
    buddy3_steering_reset(&s);
    o = step(&s, 1, 2, 0);
    assert(10000 == o.right && 14000 == o.left);
    o = step(&s, 1, 3, 10);
    assert(B3_AMBIGUOUS == o.phase && o.right < o.left);
    o = step(&s, 1, 3, 160);
    assert(B3_STOP == o.phase && 0 == o.left && 0 == o.right);
    o = step(&s, 1, 0, 200);
    assert(B3_STOP == o.phase);
    buddy3_steering_reset(&s);
    step(&s, 2, 1, 0);
    o = step(&s, 2, 1, 300);
    assert(B3_CORNER == o.phase && 0 == o.left && 12000 == o.right);
    o = step(&s, 2, 0, 310);
    assert(B3_STOP == o.phase);
    buddy3_steering_reset(&s);
    step(&s, 3, 2, 0);
    step(&s, 3, 2, 300);
    o = step(&s, 3, 0, 310);
    assert(B3_SEEK == o.phase && 0 == o.right);
    o = step(&s, 3, 0, 800);
    assert(B3_SEEK == o.phase);
    step(&s, 3, 2, 810);
    o = step(&s, 3, 0, 820);
    assert(B3_ALIGN == o.phase && 6000 == o.left && 6000 == o.right);
    step(&s, 3, 0, 920);
    o = step(&s, 3, 0, 930);
    assert(B3_TRACK == o.phase);
    buddy3_steering_reset(&s);
    step(&s, 3, 1, 0);
    step(&s, 3, 1, 300);
    step(&s, 3, 0, 310);
    o = step(&s, 3, 0, 1510);
    assert(B3_STOP == o.phase);
    buddy3_steering_reset(&s);
    step(&s, 2, 2, 0);
    step(&s, 2, 2, 300);
    o = step(&s, 2, 2, 2100);
    assert(B3_CORNER == o.phase && 0 == o.right && 12000 == o.left);
    buddy3_steering_reset(&s);
    step(&s, 2, 1, UINT32_MAX - 100);
    o = step(&s, 2, 1, 199);
    assert(B3_CORNER == o.phase);
    /* Requested stronger correction must not hit the old 20%-of-base cap. */
    buddy3_steering_reset(&s);
    o = buddy3_steering_step(&s, 1, 1, 0, 12000, 4800);
    assert(7200 == o.left && 16800 == o.right);
    buddy3_steering_reset(&s);
    o = buddy3_steering_step(&s, 1, 2, 0, 12000, 4800);
    assert(7200 == o.right && 16800 == o.left);
    /* Preserve main's outer boost while clamping forward PWM safely. */
    buddy3_steering_reset(&s);
    o = buddy3_steering_step(&s, 1, 1, 0, 12000, 20000);
    assert(0 == o.left && 24000 == o.right);
    buddy3_steering_reset(&s);
    o = buddy3_steering_step(&s, 1, 2, 0, 25000, 20000);
    assert(32767 == o.left && 5000 == o.right);
    buddy3_steering_reset(&s);
    o = buddy3_steering_step(&s, 1, 1, 0, 12000, -2000);
    assert(12000 == o.left && 12000 == o.right);
    o = step(&s, 1, 0, 16000);
    assert(B3_TRACK == o.phase);
    o = step(&s, 1, 0, 120000);
    assert(B3_TRACK == o.phase);
    puts("PASS: stronger symmetric correction, nonnegative clamp, continuous "
         "tracking, corner/recovery/ambiguous stops, timer wrap");
    return 0;
}

/*** end of file ***/

