#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "buddy3_steering.h"

static buddy3_steering_output_t
step (buddy3_steering_t * p_state, unsigned side, uint32_t now)
{
    return buddy3_steering_step(p_state, side, now, 12000, 2000);
}
int main (void)
{
    buddy3_steering_t        state;
    buddy3_steering_output_t output;
    /* Micro correction, then straight tracking on both-white. */
    buddy3_steering_reset(&state);
    output = step(&state, 1, 0);
    assert(10000 == output.left && 14000 == output.right &&
           B3_MICRO == output.phase);
    output = step(&state, 1, 100);
    assert(B3_MICRO == output.phase);
    output = step(&state, 0, 110);
    assert(12000 == output.left && 12000 == output.right);
    output = step(&state, 0, 9000);
    assert(B3_TRACK == output.phase);
    /* Both-black that persists is ambiguous, then a latched stop. */
    buddy3_steering_reset(&state);
    output = step(&state, 2, 0);
    assert(10000 == output.right && 14000 == output.left);
    output = step(&state, 3, 10);
    assert(B3_AMBIGUOUS == output.phase && output.right < output.left);
    output = step(&state, 3, 160);
    assert(B3_STOP == output.phase && 0 == output.left && 0 == output.right);
    output = step(&state, 0, 200);
    assert(B3_STOP == output.phase);
    /* An edge held for B3_CORRECTION_MS becomes a corner; losing it starts
     * a seek towards that side instead of stopping. */
    buddy3_steering_reset(&state);
    step(&state, 1, 0);
    output = step(&state, 1, 300);
    assert(B3_CORNER == output.phase && 0 == output.left &&
           12000 == output.right);
    output = step(&state, 0, 310);
    assert(B3_SEEK == output.phase && 0 == output.left);
    /* Recovery: seek, touch, align, then normal tracking. */
    buddy3_steering_reset(&state);
    step(&state, 2, 0);
    step(&state, 2, 300);
    output = step(&state, 0, 310);
    assert(B3_SEEK == output.phase && 0 == output.right);
    output = step(&state, 0, 800);
    assert(B3_SEEK == output.phase);
    step(&state, 2, 810);
    output = step(&state, 0, 820);
    assert(B3_ALIGN == output.phase && 6000 == output.left &&
           6000 == output.right);
    step(&state, 0, 920);
    output = step(&state, 0, 930);
    assert(B3_TRACK == output.phase);
    /* Recovery is bounded: nothing found in B3_RECOVERY_MAX_MS = stop. */
    buddy3_steering_reset(&state);
    step(&state, 1, 0);
    step(&state, 1, 300);
    step(&state, 0, 310);
    output = step(&state, 0, 1510);
    assert(B3_STOP == output.phase);
    /* A long continuous edge keeps turning (circular track section). */
    buddy3_steering_reset(&state);
    step(&state, 2, 0);
    step(&state, 2, 300);
    output = step(&state, 2, 2100);
    assert(B3_CORNER == output.phase && 0 == output.right &&
           12000 == output.left);
    buddy3_steering_reset(&state);
    step(&state, 1, UINT32_MAX - 100);
    output = step(&state, 1, 199);
    assert(B3_CORNER == output.phase);
    /* Requested stronger correction must not hit the old 20%-of-base cap. */
    buddy3_steering_reset(&state);
    output = buddy3_steering_step(&state, 1, 0, 12000, 4800);
    assert(7200 == output.left && 16800 == output.right);
    buddy3_steering_reset(&state);
    output = buddy3_steering_step(&state, 2, 0, 12000, 4800);
    assert(7200 == output.right && 16800 == output.left);
    /* Preserve main's outer boost while clamping forward PWM safely. */
    buddy3_steering_reset(&state);
    output = buddy3_steering_step(&state, 1, 0, 12000, 20000);
    assert(0 == output.left && 24000 == output.right);
    buddy3_steering_reset(&state);
    output = buddy3_steering_step(&state, 2, 0, 25000, 20000);
    assert(32767 == output.left && 5000 == output.right);
    buddy3_steering_reset(&state);
    output = buddy3_steering_step(&state, 1, 0, 12000, -2000);
    assert(12000 == output.left && 12000 == output.right);
    output = step(&state, 0, 16000);
    assert(B3_TRACK == output.phase);
    output = step(&state, 0, 120000);
    assert(B3_TRACK == output.phase);
    puts("PASS: stronger symmetric correction, nonnegative clamp, continuous "
         "tracking, corner/recovery/ambiguous stops, timer wrap");
    return 0;
}

/*** end of file ***/
