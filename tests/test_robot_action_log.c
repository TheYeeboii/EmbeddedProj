#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "../firmware/app_program/robot_action_log.c"
#include "../firmware/app_program/robot_action_mqtt.c"

static uint32_t g_link_state;
static bool     g_b_accept_publish;
static char     g_sent[16][256];
static unsigned g_sent_count;
int             tm_sprintf (unsigned char * out, unsigned char * fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int written = vsnprintf((char *) out, 256, (const char *) fmt, args);
    va_end(args);
    assert(written >= 0 && written < 256);
    return written;
}
uint8_t comms_get_link_state (void)
{
    return g_link_state;
}
bool comms_publish_text (uint32_t topic, const char * payload)
{
    assert(COMMS_TOPIC_ACTIONS == topic);
    if (!g_b_accept_publish)
    {
        return false;
    }
    assert(g_sent_count < 16 && strlen(payload) < 256);
    strcpy(g_sent[g_sent_count++], payload);
    return true;
}
int main (void)
{
    robot_action_event_t action_event;
    assert(robot_action_direction_for_pwm(10000, 12000) == ROBOT_ACTION_LEFT);
    assert(robot_action_direction_for_pwm(12000, 10000) == ROBOT_ACTION_RIGHT);
    assert(robot_action_direction_for_pwm(0, 256) == ROBOT_ACTION_NONE);
    assert(robot_action_direction_for_pwm(0, 0) == ROBOT_ACTION_NONE);
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
    assert(strstr(g_sent[0], "\"action\":\"turn_left\""));
    assert(strstr(g_sent[1], "\"action\":\"stop_turn_left\""));
    robot_action_publish(103);
    assert(0 == g_count && 5 == g_sent_count);
    assert(strstr(g_sent[2], "\"action\":\"turn_right\""));
    assert(strstr(g_sent[3], "\"action\":\"stop_turn_right\""));
    assert(strstr(g_sent[4], "\"type\":\"action_state\""));
    assert(strstr(g_sent[4], "\"direction\":\"none\""));
    robot_action_publish(104);
    assert(5 == g_sent_count); /* No snapshot spam. */
    robot_action_publish(1103);
    assert(6 == g_sent_count);
    for (unsigned index = 0; index < 50; ++index)
    {
        robot_action_record((index % 2) == 0 ? ROBOT_ACTION_LEFT
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
    puts("PASS: PWM direction, start/stop order, rapid reversal, "
         "deduplication, offline/backpressure retry, ordered snapshots, "
         "overflow counters, stale ack");
    return 0;
}

/*** end of file ***/

