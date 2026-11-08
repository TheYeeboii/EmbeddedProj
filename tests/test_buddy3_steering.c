#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "buddy3_steering.h"

static buddy3_steering_output_t
step (buddy3_steering_t * p_state, unsigned stage, unsigned side, uint32_t now)
{
    return buddy3_steering_step(p_state, stage, side, now, 12000, 2000);
}
int main (void)
{
    buddy3_steering_t        state;
    buddy3_steering_output_t output;
    buddy3_steering_reset(&state);
    output = step(&state, 1, 1, 0);
    assert(10000 == output.left && 14000 == output.right &&
           B3_MICRO == output.phase);
    output = step(&state, 1, 1, 1000);
    assert(B3_MICRO == output.phase);
    output = step(&state, 1, 0, 1010);
    assert(12000 == output.left && 12000 == output.right);
    output = step(&state, 1, 0, 9000);
    assert(B3_TRACK == output.phase);
    buddy3_steering_reset(&state);
    output = step(&state, 1, 2, 0);
    assert(10000 == output.right && 14000 == output.left);
    output = step(&state, 1, 3, 10);
    assert(B3_AMBIGUOUS == output.phase && output.right < output.left);
    output = step(&state, 1, 3, 160);
    assert(B3_STOP == output.phase && 0 == output.left && 0 == output.right);
    output = step(&state, 1, 0, 200);
    assert(B3_STOP == output.phase);
    buddy3_steering_reset(&state);
    step(&state, 2, 1, 0);
    output = step(&state, 2, 1, 300);
    assert(B3_CORNER == output.phase && 0 == output.left &&
           12000 == output.right);
    output = step(&state, 2, 0, 310);
    assert(B3_STOP == output.phase);
    buddy3_steering_reset(&state);
    step(&state, 3, 2, 0);
    step(&state, 3, 2, 300);
    output = step(&state, 3, 0, 310);
    assert(B3_SEEK == output.phase && 0 == output.right);
    output = step(&state, 3, 0, 800);
    assert(B3_SEEK == output.phase);
    step(&state, 3, 2, 810);
    output = step(&state, 3, 0, 820);
    assert(B3_ALIGN == output.phase && 6000 == output.left &&
           6000 == output.right);
    step(&state, 3, 0, 920);
    output = step(&state, 3, 0, 930);
    assert(B3_TRACK == output.phase);
    buddy3_steering_reset(&state);
    step(&state, 3, 1, 0);
    step(&state, 3, 1, 300);
    step(&state, 3, 0, 310);
    output = step(&state, 3, 0, 1510);
    assert(B3_STOP == output.phase);
    buddy3_steering_reset(&state);
    step(&state, 2, 2, 0);
    step(&state, 2, 2, 300);
    output = step(&state, 2, 2, 2100);
    assert(B3_CORNER == output.phase && 0 == output.right &&
           12000 == output.left);
    buddy3_steering_reset(&state);
    step(&state, 2, 1, UINT32_MAX - 100);
    output = step(&state, 2, 1, 199);
    assert(B3_CORNER == output.phase);
    /* Requested stronger correction must not hit the old 20%-of-base cap. */
    buddy3_steering_reset(&state);
    output = buddy3_steering_step(&state, 1, 1, 0, 12000, 4800);
    assert(7200 == output.left && 16800 == output.right);
    buddy3_steering_reset(&state);
    output = buddy3_steering_step(&state, 1, 2, 0, 12000, 4800);
    assert(7200 == output.right && 16800 == output.left);
    /* Preserve main's outer boost while clamping forward PWM safely. */
    buddy3_steering_reset(&state);
    output = buddy3_steering_step(&state, 1, 1, 0, 12000, 20000);
    assert(0 == output.left && 24000 == output.right);
    buddy3_steering_reset(&state);
    output = buddy3_steering_step(&state, 1, 2, 0, 25000, 20000);
    assert(32767 == output.left && 5000 == output.right);
    buddy3_steering_reset(&state);
    output = buddy3_steering_step(&state, 1, 1, 0, 12000, -2000);
    assert(12000 == output.left && 12000 == output.right);
    output = step(&state, 1, 0, 16000);
    assert(B3_TRACK == output.phase);
    output = step(&state, 1, 0, 120000);
    assert(B3_TRACK == output.phase);
    puts("PASS: stronger symmetric correction, nonnegative clamp, continuous "
         "tracking, corner/recovery/ambiguous stops, timer wrap");
    return 0;
}

/*** end of file ***/

