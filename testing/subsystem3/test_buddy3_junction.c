#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "buddy3_junction.h"
static buddy3_junction_output_t step (buddy3_junction_t * p_state,
                                      unsigned side, uint32_t now,
                                      float32_t left_mm, float32_t right_mm)
{
    return buddy3_junction_step(p_state, side, now, left_mm, right_mm, true,
                                true, 12000);
}
int main (void)
{
    buddy3_junction_t        state;
    buddy3_junction_output_t output;
    buddy3_junction_reset(&state);
    output = step(&state, 0, 0, 0, 0);
    assert(!output.handled);
    output = step(&state, 3, 10, 0, 0);
    assert(output.handled && B3J_CONFIRM == output.phase &&
           output.left == output.right);
    output = step(&state, 0, 20, 1, 1);
    assert(!output.handled && B3J_IDLE == state.phase);
    output = step(&state, 3, 100, 0, 0);
    assert(B3J_CONFIRM == output.phase);
    output = step(&state, 3, 130, 2, 2);
    assert(B3J_CROSS == output.phase);
    output = step(&state, 1, 150, 5, 5);
    assert(B3J_CROSS == output.phase && output.left == output.right);
    output = step(&state, 2, 170, 10, 10);
    assert(B3J_CROSS == output.phase && output.left == output.right);
    output = step(&state, 0, 180, 20, 20);
    assert(B3J_CROSS == output.phase);
    output = step(&state, 0, 190, 24, 24);
    assert(B3J_CROSS == output.phase);
    output = step(&state, 0, 200, 25, 25);
    assert(B3J_CLEAR == output.phase);
    output = step(&state, 1, 210, 26, 26);
    assert(!output.handled);
    buddy3_junction_reset(&state);
    step(&state, 3, 0, 0, 0);
    output = step(&state, 3, 30, 5, 7);
    assert(output.right < output.left && output.right >= 8000);
    buddy3_junction_reset(&state);
    step(&state, 3, 0, 0, 0);
    output = step(&state, 3, 30, 7, 5);
    assert(output.left < output.right && output.left >= 8000);
    buddy3_junction_reset(&state);
    step(&state, 3, 0, 0, 0);
    step(&state, 3, 30, 2, 2);
    step(&state, 0, 50, 20, 20);
    step(&state, 1, 60, 23,
         23); /* A new branch edge invalidates early white clearance. */
    output = step(&state, 0, 70, 25, 25);
    assert(B3J_CROSS == output.phase);
    output = step(&state, 0, 80, 30, 30);
    assert(B3J_CLEAR == output.phase);
    buddy3_junction_reset(&state);
    step(&state, 3, 0, 0, 0);
    output = step(&state, 3, B3_JUNCTION_MAX_MS, 0, 0);
    assert(output.stopped && 0 == output.left && 0 == output.right);
    output = step(&state, 0, B3_JUNCTION_MAX_MS + 10, 2, 2);
    assert(output.stopped);
    buddy3_junction_reset(&state);
    step(&state, 3, 0, 0, 0);
    output = step(&state, 3, 100, 100, 100);
    assert(output.stopped);
    buddy3_junction_reset(&state);
    step(&state, 3, 0, 0, 0);
    output = buddy3_junction_step(&state, 3, 10, 0, 0, false, true, 12000);
    assert(output.stopped);
    buddy3_junction_reset(&state);
    step(&state, 3, 0, 10, 10);
    output = step(&state, 3, 10, 0, 0);
    assert(output.stopped); /* Odometry reset/reverse. */
    buddy3_junction_reset(&state);
    step(&state, 3, 0, 0, 0);
    output = step(&state, 3, 30, 1, 22);
    assert(output.stopped); /* Excessive heading deviation. */
    buddy3_junction_reset(&state);
    output = buddy3_junction_step(&state, 3, 0, 0, 0, true, false, 12000);
    assert(!output.handled); /* Existing corner/recovery keeps ownership. */
    buddy3_junction_reset(&state);
    step(&state, 3, UINT32_MAX - 20, 0, 0);
    output = step(&state, 3, 9, 1, 1);
    assert(B3J_CROSS == output.phase);
    puts("PASS: junction debounce, straight commitment, branch suppression, "
         "distance exit, heading correction, fault/distance/time stops, wrap");
    return 0;
}

/*** end of file ***/

