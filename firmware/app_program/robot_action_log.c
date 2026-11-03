#include <tk/tkernel.h>
#include "robot_action_log.h"

static robot_action_event_t g_queue[ROBOT_ACTION_QUEUE_SIZE];
static uint32_t g_head, g_count, g_sequence, g_dropped;
static robot_action_event_t g_current;

uint8_t robot_action_direction_for_pwm(int16_t left, int16_t right)
{
    int32_t difference = (int32_t)right - (int32_t)left;
    if (difference > ROBOT_ACTION_PWM_THRESHOLD) return ROBOT_ACTION_LEFT;
    if (difference < -ROBOT_ACTION_PWM_THRESHOLD) return ROBOT_ACTION_RIGHT;
    return ROBOT_ACTION_NONE;
}

/* Caller holds the short interrupt lock. Keep newest events on overflow. */
static void enqueue(uint8_t action)
{
    robot_action_event_t event = g_current;
    if (g_count == ROBOT_ACTION_QUEUE_SIZE) {
        g_head = (g_head + 1u) % ROBOT_ACTION_QUEUE_SIZE;
        g_count--;
        g_dropped++;
    }
    event.action = action;
    event.seq = ++g_sequence;
    event.dropped = g_dropped;
    g_queue[(g_head + g_count) % ROBOT_ACTION_QUEUE_SIZE] = event;
    g_count++;
}
void robot_action_record(uint8_t direction, uint8_t source,
                         int16_t left, int16_t right, uint32_t ts_ms)
{
    UINT interrupt_state;
    uint8_t previous;
    if (direction > ROBOT_ACTION_RIGHT || source > ROBOT_ACTION_STOP) return;
    DI(interrupt_state);
    previous = g_current.direction;
    g_current.direction = direction;
    g_current.source = source;
    g_current.left_pwm = left;
    g_current.right_pwm = right;
    g_current.ts_ms = ts_ms;
    if (previous != direction) {
        if (previous == ROBOT_ACTION_LEFT) enqueue(ROBOT_ACTION_STOP_LEFT);
        if (previous == ROBOT_ACTION_RIGHT) enqueue(ROBOT_ACTION_STOP_RIGHT);
        if (direction == ROBOT_ACTION_LEFT) enqueue(ROBOT_ACTION_TURN_LEFT);
        if (direction == ROBOT_ACTION_RIGHT) enqueue(ROBOT_ACTION_TURN_RIGHT);
    }
    EI(interrupt_state);
}
bool robot_action_peek(robot_action_event_t *event)
{
    UINT interrupt_state;
    bool available;
    if (event == NULL) return false;
    DI(interrupt_state);
    available = g_count != 0u;
    if (available) {
        *event = g_queue[g_head];
        event->dropped = g_dropped;
    }
    EI(interrupt_state);
    return available;
}
void robot_action_ack(uint32_t seq)
{
    UINT interrupt_state;
    DI(interrupt_state);
    /* A producer may have overflowed the queue after the peek. */
    if (g_count != 0u && g_queue[g_head].seq == seq) {
        g_head = (g_head + 1u) % ROBOT_ACTION_QUEUE_SIZE;
        g_count--;
    }
    EI(interrupt_state);
}
void robot_action_snapshot(robot_action_event_t *state)
{
    UINT interrupt_state;
    if (state == NULL) return;
    DI(interrupt_state);
    *state = g_current;
    state->seq = g_sequence;
    state->dropped = g_dropped;
    EI(interrupt_state);
}

bool robot_action_snapshot_if_empty(robot_action_event_t *state)
{
    UINT interrupt_state;
    bool empty;
    if (state == NULL) return false;
    DI(interrupt_state);
    empty = g_count == 0u;
    if (empty) {
        *state = g_current;
        state->seq = g_sequence;
        state->dropped = g_dropped;
    }
    EI(interrupt_state);
    return empty;
}
