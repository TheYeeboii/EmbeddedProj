/** @file robot_steering_trace.c
 *
 * @brief Queue of steering decisions (wheel commands plus the IR evidence
 *        behind them) for the dashboard steering timeline.
 */
#include <tk/tkernel.h>
#include "robot_steering_trace.h"

/* Wheel-command difference that counts as a left or right turn. */
#define TRACE_TURN_THRESHOLD 256
/* Unchanged steady state is still saved at 10 Hz as evidence. */
#define TRACE_STEADY_MS 100u

static robot_steering_event_t g_queue[ROBOT_STEERING_QUEUE_SIZE];
static robot_steering_event_t g_current;
static uint32_t               g_head           = 0u;
static uint32_t               g_count          = 0u;
static uint32_t               g_sequence       = 0u;
static uint32_t               g_dropped        = 0u;
static uint32_t               g_session_id     = 0u;
static uint8_t                g_owner          = TRACE_MANUAL;
static bool                   g_b_have_current = false;

static uint8_t trace_direction (const robot_steering_event_t * p_event);
static uint8_t trace_owner (const robot_steering_event_t * p_event);
static uint8_t trace_side (const robot_steering_event_t * p_event);
static bool    trace_is_repeat (const robot_steering_event_t * p_event);

void robot_diagnostics_init (uint32_t session)
{
    g_session_id = session;
}

uint32_t robot_diagnostics_session (void)
{
    return g_session_id;
}

void robot_steering_owner (uint8_t owner)
{
    UINT state = 0u;

    if (owner <= TRACE_MOTION)
    {
        DI(state);
        g_owner = owner;
        EI(state);
    }
}

void robot_steering_record (const robot_steering_event_t * p_input)
{
    UINT                   state = 0u;
    robot_steering_event_t event = *p_input;

    event.direction = trace_direction(&event);
    DI(state);
    event.owner = trace_owner(&event);

    /* Save transitions at the command boundary, plus 10 Hz steady evidence.
     * Never coalesce a pulse or sensor/phase/owner transition. */
    if (!trace_is_repeat(&event))
    {
        if (ROBOT_STEERING_QUEUE_SIZE == g_count)
        {
            g_head = (g_head + 1u) % ROBOT_STEERING_QUEUE_SIZE;
            --g_count;
            ++g_dropped;
        }
        event.seq                                               = ++g_sequence;
        event.dropped                                           = g_dropped;
        g_current                                               = event;
        g_b_have_current                                        = true;
        g_queue[(g_head + g_count) % ROBOT_STEERING_QUEUE_SIZE] = event;
        ++g_count;
    }
    EI(state);
}

bool robot_steering_peek (robot_steering_event_t * p_event)
{
    UINT state       = 0u;
    bool b_available = false;

    DI(state);
    b_available = (0u != g_count);
    if (b_available)
    {
        *p_event         = g_queue[g_head];
        p_event->dropped = g_dropped;
    }
    EI(state);

    return b_available;
}

void robot_steering_ack (uint32_t seq)
{
    UINT state = 0u;

    DI(state);
    if ((0u != g_count) && (g_queue[g_head].seq == seq))
    {
        g_head = (g_head + 1u) % ROBOT_STEERING_QUEUE_SIZE;
        --g_count;
    }
    EI(state);
}

bool robot_steering_snapshot (robot_steering_event_t * p_event)
{
    UINT state   = 0u;
    bool b_ready = false;

    DI(state);
    b_ready = ((0u == g_count) && g_b_have_current);
    if (b_ready)
    {
        *p_event         = g_current;
        p_event->dropped = g_dropped;
    }
    EI(state);

    return b_ready;
}

/* 1 = turning left, 2 = turning right, 0 = straight. */
static uint8_t trace_direction (const robot_steering_event_t * p_event)
{
    int32_t difference =
        (int32_t) p_event->requested[1] - (int32_t) p_event->requested[0];
    uint8_t direction = 0u;

    if (difference > TRACE_TURN_THRESHOLD)
    {
        direction = 1u;
    }
    else if (difference < -TRACE_TURN_THRESHOLD)
    {
        direction = 2u;
    }
    else
    {
        direction = 0u;
    }

    return direction;
}

/* Line-following events are owned by the phase that made them; others by
 * the current owner.  A zero command is always reported as stopped. Caller
 * holds the interrupt lock. */
static uint8_t trace_owner (const robot_steering_event_t * p_event)
{
    uint8_t owner = g_owner;

    if ((0 == p_event->requested[0]) && (0 == p_event->requested[1]))
    {
        owner = TRACE_STOPPED;
    }
    else if (!p_event->has_input)
    {
        owner = g_owner;
    }
    else if (TRACE_CROSS == p_event->input.phase)
    {
        owner = TRACE_JUNCTION;
    }
    else if ((TRACE_SEEK == p_event->input.phase) ||
             (TRACE_ALIGN == p_event->input.phase))
    {
        owner = TRACE_SEARCH;
    }
    else
    {
        owner = TRACE_TRACKING;
    }

    return owner;
}

/* Sensor pattern: bit 0 left black, bit 1 right black; 0 without input. */
static uint8_t trace_side (const robot_steering_event_t * p_event)
{
    uint8_t side = 0u;

    if (p_event->has_input)
    {
        if (p_event->input.norm[0] >= p_event->input.threshold)
        {
            side |= 1u;
        }
        if (p_event->input.norm[1] >= p_event->input.threshold)
        {
            side |= 2u;
        }
    }

    return side;
}

/* True when the event only repeats the last saved one within 100 ms.
 * Caller holds the interrupt lock. */
static bool trace_is_repeat (const robot_steering_event_t * p_event)
{
    return (g_b_have_current && (p_event->direction == g_current.direction) &&
            (p_event->owner == g_current.owner) &&
            (p_event->has_input == g_current.has_input) &&
            (p_event->input.phase == g_current.input.phase) &&
            (trace_side(p_event) == trace_side(&g_current)) &&
            (p_event->faults == g_current.faults) &&
            ((p_event->ts_ms - g_current.ts_ms) < TRACE_STEADY_MS));
}

/*** end of file ***/

