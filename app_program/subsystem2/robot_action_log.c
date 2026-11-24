/** @file robot_action_log.c
 *
 * @brief Queue of steering-direction changes (turn left/right and their
 *        stops) taken at the accepted motor-command boundary.
 */
#include <tk/tkernel.h>
#include "robot_action_log.h"

static robot_action_event_t g_queue[ROBOT_ACTION_QUEUE_SIZE];
static uint32_t             g_head     = 0u;
static uint32_t             g_count    = 0u;
static uint32_t             g_sequence = 0u;
static uint32_t             g_dropped  = 0u;
static robot_action_event_t g_current;

static void action_enqueue (uint8_t action);

uint8_t robot_action_direction_for_pwm (int16_t left, int16_t right)
{
    int32_t difference = (int32_t) right - (int32_t) left;
    uint8_t direction  = ROBOT_ACTION_NONE;

    if (difference > ROBOT_ACTION_PWM_THRESHOLD)
    {
        direction = ROBOT_ACTION_LEFT;
    }
    else if (difference < -ROBOT_ACTION_PWM_THRESHOLD)
    {
        direction = ROBOT_ACTION_RIGHT;
    }
    else
    {
        direction = ROBOT_ACTION_NONE;
    }

    return direction;
}

void robot_action_record (uint8_t direction, uint8_t source, int16_t left,
                          int16_t right, uint32_t ts_ms)
{
    UINT    interrupt_state = 0u;
    uint8_t previous        = ROBOT_ACTION_NONE;

    if ((direction <= ROBOT_ACTION_RIGHT) && (source <= ROBOT_ACTION_STOP))
    {
        DI(interrupt_state);
        previous            = g_current.direction;
        g_current.direction = direction;
        g_current.source    = source;
        g_current.left_pwm  = left;
        g_current.right_pwm = right;
        g_current.ts_ms     = ts_ms;
        if (previous != direction)
        {
            if (ROBOT_ACTION_LEFT == previous)
            {
                action_enqueue(ROBOT_ACTION_STOP_LEFT);
            }
            if (ROBOT_ACTION_RIGHT == previous)
            {
                action_enqueue(ROBOT_ACTION_STOP_RIGHT);
            }
            if (ROBOT_ACTION_LEFT == direction)
            {
                action_enqueue(ROBOT_ACTION_TURN_LEFT);
            }
            if (ROBOT_ACTION_RIGHT == direction)
            {
                action_enqueue(ROBOT_ACTION_TURN_RIGHT);
            }
        }
        EI(interrupt_state);
    }
}

bool robot_action_peek (robot_action_event_t * p_event)
{
    UINT interrupt_state = 0u;
    bool b_available     = false;

    if (NULL != p_event)
    {
        DI(interrupt_state);
        b_available = (0u != g_count);
        if (b_available)
        {
            *p_event         = g_queue[g_head];
            p_event->dropped = g_dropped;
        }
        EI(interrupt_state);
    }

    return b_available;
}

void robot_action_ack (uint32_t seq)
{
    UINT interrupt_state = 0u;

    DI(interrupt_state);
    /* A producer may have overflowed the queue after the peek. */
    if ((0u != g_count) && (g_queue[g_head].seq == seq))
    {
        g_head = (g_head + 1u) % ROBOT_ACTION_QUEUE_SIZE;
        g_count--;
    }
    EI(interrupt_state);
}

void robot_action_snapshot (robot_action_event_t * p_state)
{
    UINT interrupt_state = 0u;

    if (NULL != p_state)
    {
        DI(interrupt_state);
        *p_state         = g_current;
        p_state->seq     = g_sequence;
        p_state->dropped = g_dropped;
        EI(interrupt_state);
    }
}

bool robot_action_snapshot_if_empty (robot_action_event_t * p_state)
{
    UINT interrupt_state = 0u;
    bool b_empty         = false;

    if (NULL != p_state)
    {
        DI(interrupt_state);
        b_empty = (0u == g_count);
        if (b_empty)
        {
            *p_state         = g_current;
            p_state->seq     = g_sequence;
            p_state->dropped = g_dropped;
        }
        EI(interrupt_state);
    }

    return b_empty;
}

/* Caller holds the short interrupt lock. Keep newest events on overflow. */
static void action_enqueue (uint8_t action)
{
    robot_action_event_t event = g_current;

    if (ROBOT_ACTION_QUEUE_SIZE == g_count)
    {
        g_head = (g_head + 1u) % ROBOT_ACTION_QUEUE_SIZE;
        g_count--;
        g_dropped++;
    }
    event.action                                          = action;
    event.seq                                             = ++g_sequence;
    event.dropped                                         = g_dropped;
    g_queue[(g_head + g_count) % ROBOT_ACTION_QUEUE_SIZE] = event;
    g_count++;
}

/*** end of file ***/

