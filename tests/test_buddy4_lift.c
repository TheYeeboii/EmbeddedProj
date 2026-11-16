/* Buddy 4 lifted / overturned detection on the portable processing code. */
#include <assert.h>
#include <stdio.h>

#include "buddy4_processing.h"

static const int16_t g_mag[3] = {200, 0, 0};

/* Feed one accelerometer sample (mg, rover axes) at the 50 ms period. */
static void sample (buddy4_processor_t * p_proc, uint32_t * p_now,
                    int16_t x, int16_t y, int16_t z)
{
    const int16_t accel[3] = {x, y, z};

    *p_now += BUDDY4_PERIOD_MS;
    buddy4_process(p_proc, *p_now, true, g_mag, true, accel, 0.0f, 0.0f,
                   false, false);
}

static void settle (buddy4_processor_t * p_proc, uint32_t * p_now)
{
    for (int index = 0; index < 20; index++)
    {
        sample(p_proc, p_now, 0, 0, 1000);
    }
    assert(!p_proc->state.lifted && (0u == p_proc->state.lift_count));
}

int main (void)
{
    buddy4_processor_t proc;
    uint32_t           now = 0u;

    /* Flat floor with vibration, and one upward spike: not lifted. */
    buddy4_processor_init(&proc, NULL);
    settle(&proc, &now);
    sample(&proc, &now, 0, 0, 1400);
    sample(&proc, &now, 0, 0, 1000);
    sample(&proc, &now, 30, -20, 1150);
    assert(!proc.state.lifted && (0u == proc.state.lift_count));

    /* Picked up: Z above 1.25 g for two samples, held 500 ms. */
    sample(&proc, &now, 0, 0, 1350);
    assert(!proc.state.lifted);
    sample(&proc, &now, 0, 0, 1400);
    assert(proc.state.lifted && (1u == proc.state.lift_count));
    assert(BUDDY4_MOTION_LIFTED == proc.state.motion_class);
    for (int index = 0; index < 9; index++)
    {
        sample(&proc, &now, 0, 0, 1000); /* held still in the air */
    }
    assert(proc.state.lifted);
    sample(&proc, &now, 0, 0, 1000);
    assert(!proc.state.lifted && (1u == proc.state.lift_count));

    /* Overturned on its back: Z near -1 g for four samples (200 ms). */
    buddy4_processor_init(&proc, NULL);
    settle(&proc, &now);
    for (int index = 0; index < 3; index++)
    {
        sample(&proc, &now, 0, 0, -1000);
    }
    assert(!proc.state.lifted);
    sample(&proc, &now, 0, 0, -1000);
    assert(proc.state.lifted && (1u == proc.state.lift_count));
    for (int index = 0; index < 40; index++)
    {
        sample(&proc, &now, 0, 0, -1000); /* still upside down */
    }
    assert(proc.state.lifted && (1u == proc.state.lift_count));

    /* On its side (90 deg) is overturned; a 30 deg slope is not. */
    buddy4_processor_init(&proc, NULL);
    settle(&proc, &now);
    for (int index = 0; index < 10; index++)
    {
        sample(&proc, &now, 0, 500, 866);
    }
    assert(!proc.state.lifted);
    for (int index = 0; index < 4; index++)
    {
        sample(&proc, &now, 0, 1000, 0);
    }
    assert(proc.state.lifted);

    /* A run reset (processor init) clears the count. */
    buddy4_processor_init(&proc, NULL);
    assert(!proc.state.lifted && (0u == proc.state.lift_count));

    puts("PASS: lift needs 100 ms of upward push and holds 500 ms; "
         "overturned past 45 deg for 200 ms; slope and spikes ignored");
    return 0;
}
