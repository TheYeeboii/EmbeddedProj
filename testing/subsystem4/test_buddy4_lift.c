/* Buddy 4 lifted / overturned detection on the portable processing code. */
#include <assert.h>
#include <stdio.h>

#include "buddy4_processing.h"

static const int16_t g_mag[3] = {200, 0, 0};
/* Any IR sensor sees the floor (in the air all three read black). */
static bool g_b_floor = true;

/* Feed one accelerometer sample (mg, rover axes) at the 50 ms period. */
static void sample (buddy4_processor_t * p_proc, uint32_t * p_now,
                    int16_t accel_x, int16_t accel_y, int16_t accel_z)
{
    const int16_t accel[3] = {accel_x, accel_y, accel_z};

    *p_now += BUDDY4_PERIOD_MS;
    buddy4_processor_set_floor(p_proc, g_b_floor);
    buddy4_process(p_proc, *p_now, true, g_mag, true, accel, 0.0f, 0.0f, false,
                   false);
}

static void settle (buddy4_processor_t * p_proc, uint32_t * p_now)
{
    for (int index = 0; index < 20; index++)
    {
        sample(p_proc, p_now, 0, 0, 1000);
    }
    assert(!p_proc->state.lifted && (0u == p_proc->state.lift_count));
}

/* A pick-up needs 100 ms of upward push and stays lifted while held, until
 * the IR sensors see the floor again or a new start ends it. */
static void test_lift_and_put_down (void)
{
    buddy4_processor_t proc;
    uint32_t           now = 0u;

    /* Flat floor with vibration, and one upward spike: not lifted. */
    g_b_floor = true;
    buddy4_processor_init(&proc, NULL);
    settle(&proc, &now);
    sample(&proc, &now, 0, 0, 1400);
    sample(&proc, &now, 0, 0, 1000);
    sample(&proc, &now, 30, -20, 1150);
    assert(!proc.state.lifted && (0u == proc.state.lift_count));

    /* Picked up: Z above 1.25 g for two samples.  It stays lifted while
     * held (all IR black), even still and even when jostled in the hand. */
    sample(&proc, &now, 0, 0, 1350);
    assert(!proc.state.lifted);
    sample(&proc, &now, 0, 0, 1400);
    assert(proc.state.lifted && (1u == proc.state.lift_count));
    assert(BUDDY4_MOTION_LIFTED == proc.state.motion_class);
    g_b_floor = false;
    for (int index = 0; index < 60; index++)
    {
        sample(&proc, &now, 0, 0, 1000); /* held still in the air, 3 s */
    }
    sample(&proc, &now, 0, 0, 1500); /* a jolt in the hand */
    sample(&proc, &now, 0, 0, 1400);
    sample(&proc, &now, 0, 0, 1000);
    assert(proc.state.lifted && (1u == proc.state.lift_count));

    /* Put down: IR sees the floor for 100 ms; the landing jolt is not a new
     * lift. */
    g_b_floor = true;
    sample(&proc, &now, 0, 0, 1400);
    assert(proc.state.lifted);
    sample(&proc, &now, 0, 0, 1350);
    assert(!proc.state.lifted);
    for (int index = 0; index < 20; index++)
    {
        sample(&proc, &now, 0, 0, 1000);
    }
    assert(!proc.state.lifted && (1u == proc.state.lift_count));

    /* Lifted again later: counted once more. */
    sample(&proc, &now, 0, 0, 1350);
    sample(&proc, &now, 0, 0, 1400);
    assert(proc.state.lifted && (2u == proc.state.lift_count));

    /* IR never saw the air (barely lifted, or set down on a black patch):
     * it stays lifted until a new start puts it down. */
    for (int index = 0; index < 40; index++)
    {
        sample(&proc, &now, 0, 0, 1000);
    }
    assert(proc.state.lifted);
    buddy4_processor_put_down(&proc);
    sample(&proc, &now, 0, 0, 1000);
    assert(!proc.state.lifted && (2u == proc.state.lift_count));
}

/* Tipped past 45 degrees for 200 ms reads as lifted and stays so; a slope
 * does not, and a run reset clears the count. */
static void test_overturned (void)
{
    buddy4_processor_t proc;
    uint32_t           now = 0u;

    /* Overturned on its back: Z near -1 g for four samples (200 ms). */
    g_b_floor = true;
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
    /* A new start cannot clear a rover that is still upside down. */
    buddy4_processor_put_down(&proc);
    sample(&proc, &now, 0, 0, -1000);
    assert(proc.state.lifted);

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
}

int main (void)
{
    test_lift_and_put_down();
    test_overturned();
    puts("PASS: lift needs 100 ms of upward push and stays lifted while held "
         "(IR black) until IR sees the floor or a new start; put-down is not "
         "a new lift; overturned past 45 deg for 200 ms; slope and spikes "
         "ignored");
    return 0;
}

/*** end of file ***/

