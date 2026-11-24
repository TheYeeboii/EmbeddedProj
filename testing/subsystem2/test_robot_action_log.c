#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "../../app_program/subsystem2/robot_action_log.c"
#include "../../app_program/subsystem1/robot_diagnostics_mqtt.c"
#include "../../app_program/subsystem1/robot_packed.c"
#include "../../app_program/subsystem1/robot_wire.c"

/* Bit positions in the packed action message (robot_diagnostics_mqtt.c). */
#define ACTION_BIT_STATE     8u
#define ACTION_BIT_ACTION    89u
#define ACTION_BIT_DIRECTION 91u
#define ACTION_LENGTH        16u

static uint32_t g_link_state;
static bool     g_b_accept_publish;
static uint8_t  g_sent[16][ROBOT_WIRE_DIAG_MESSAGE_MAX];
static uint32_t g_sent_length[16];
static unsigned g_sent_count;
uint8_t         comms_get_link_state (void)
{
    return g_link_state;
}
bool comms_publish (uint32_t topic, const uint8_t * p_data, uint32_t length)
{
    assert(COMMS_TOPIC_ACTIONS == topic);
    if (!g_b_accept_publish)
    {
        return false;
    }
    assert(g_sent_count < 16 && ACTION_LENGTH == length);
    memcpy(g_sent[g_sent_count], p_data, length);
    g_sent_length[g_sent_count++] = length;
    return true;
}
/* Read one field of a captured message. */
static uint32_t sent_field (unsigned message, uint32_t bit, uint32_t width)
{
    robot_packed_reader_t reader;

    robot_packed_reader_init(&reader, g_sent[message], g_sent_length[message]);
    for (; bit > 0u; bit -= (bit > 32u) ? 32u : bit)
    {
        (void) robot_packed_get(&reader, (bit > 32u) ? 32u : bit);
    }
    return robot_packed_get(&reader, width);
}
/* True when message is a packed action event (or state) with this action. */
static bool sent_action (unsigned message, bool b_state, uint32_t action)
{
    return (0x81u == g_sent[message][0]) &&
           ((b_state ? 1u : 0u) == sent_field(message, ACTION_BIT_STATE, 1u)) &&
           (action == sent_field(message, ACTION_BIT_ACTION, 2u));
}
int main (void)
{
    robot_action_event_t action_event;
    assert(ROBOT_ACTION_LEFT == robot_action_direction_for_pwm(10000, 12000));
    assert(ROBOT_ACTION_RIGHT == robot_action_direction_for_pwm(12000, 10000));
    assert(ROBOT_ACTION_NONE == robot_action_direction_for_pwm(0, 256));
    assert(ROBOT_ACTION_NONE == robot_action_direction_for_pwm(0, 0));
    robot_action_record(ROBOT_ACTION_NONE, ROBOT_ACTION_STOP, 0, 0, 1);
    assert(!robot_action_peek(&action_event));
    robot_action_record(ROBOT_ACTION_LEFT, ROBOT_ACTION_PWM, 0, 12000, 10);
    robot_action_record(ROBOT_ACTION_LEFT, ROBOT_ACTION_PWM, 100, 12000, 20);
    assert(1 == g_count); /* Deduplicate repeated intent, not PWM values. */
    assert(!robot_action_snapshot_if_empty(&action_event));
    robot_action_record(ROBOT_ACTION_RIGHT, ROBOT_ACTION_MOTION, 12000, -12000,
                        30);
    robot_action_record(ROBOT_ACTION_NONE, ROBOT_ACTION_STOP, 0, 0, 40);
    assert(4 == g_count);
    assert(robot_action_peek(&action_event) && 1 == action_event.seq &&
           10 == action_event.ts_ms &&
           ROBOT_ACTION_TURN_LEFT == action_event.action);
    robot_action_publish(100);
    assert(4 == g_count && 0 == g_sent_count);
    g_link_state = COMMS_LINK_MQTT_UP;
    robot_action_publish(101);
    assert(4 == g_count && 0 == g_sent_count);
    g_b_accept_publish = true;
    robot_action_publish(102);
    assert(2 == g_count && 2 == g_sent_count);
    assert(sent_action(0, false, ROBOT_ACTION_TURN_LEFT));
    assert(sent_action(1, false, ROBOT_ACTION_STOP_LEFT));
    robot_action_publish(103);
    assert(0 == g_count && 5 == g_sent_count);
    assert(sent_action(2, false, ROBOT_ACTION_TURN_RIGHT));
    assert(sent_action(3, false, ROBOT_ACTION_STOP_RIGHT));
    assert(sent_action(4, true, 0u)); /* current state */
    assert(ROBOT_ACTION_NONE == sent_field(4, ACTION_BIT_DIRECTION, 2u));
    robot_action_publish(104);
    assert(5 == g_sent_count); /* No snapshot spam. */
    robot_action_publish(1103);
    assert(6 == g_sent_count);
    for (unsigned index = 0; index < 50; ++index)
    {
        robot_action_record((0u == (index % 2u)) ? ROBOT_ACTION_LEFT
                                                 : ROBOT_ACTION_NONE,
                            ROBOT_ACTION_PWM, 0, 0, 2000 + index);
    }
    assert(32 == g_count && 18 == g_dropped);
    assert(robot_action_peek(&action_event) && 23 == action_event.seq &&
           18 == action_event.dropped);
    robot_action_ack(1);
    assert(robot_action_peek(&action_event) &&
           23 == action_event.seq); /* Stale ack cannot pop a newer event. */
    robot_action_snapshot(&action_event);
    assert(ROBOT_ACTION_NONE == action_event.direction &&
           54 == action_event.seq && 18 == action_event.dropped);
    while (robot_action_peek(&action_event))
    {
        robot_action_ack(action_event.seq);
    }
    assert(robot_action_snapshot_if_empty(&action_event));
    assert(!robot_action_peek(NULL));
    puts("PASS: PWM direction, start/stop order, rapid reversal, packed "
         "messages, "
         "deduplication, offline/backpressure retry, ordered snapshots, "
         "overflow counters, stale ack");
    return 0;
}

/*** end of file ***/

