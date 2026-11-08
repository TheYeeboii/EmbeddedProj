#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "../firmware/app_program/robot_action_log.c"
#include "../firmware/app_program/robot_action_mqtt.c"

static uint32_t link_state;
static bool     accept_publish;
static char     sent[16][256];
static unsigned sent_count;
int             tm_sprintf (unsigned char * out, unsigned char * fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf((char *) out, 256, (const char *) fmt, ap);
    va_end(ap);
    assert(n >= 0 && n < 256);
    return n;
}
uint32_t comms_get_link_state (void)
{
    return link_state;
}
bool comms_publish (uint32_t topic, const char * payload)
{
    assert(COMMS_TOPIC_ACTIONS == topic);
    if (!accept_publish)
    {
        return false;
    }
    assert(sent_count < 16 && strlen(payload) < 256);
    strcpy(sent[sent_count++], payload);
    return true;
}
int main (void)
{
    robot_action_event_t e;
    assert(robot_action_direction_for_pwm(10000, 12000) == ROBOT_ACTION_LEFT);
    assert(robot_action_direction_for_pwm(12000, 10000) == ROBOT_ACTION_RIGHT);
    assert(robot_action_direction_for_pwm(0, 256) == ROBOT_ACTION_NONE);
    assert(robot_action_direction_for_pwm(0, 0) == ROBOT_ACTION_NONE);
    robot_action_record(ROBOT_ACTION_NONE, ROBOT_ACTION_STOP, 0, 0, 1);
    assert(!robot_action_peek(&e));
    robot_action_record(ROBOT_ACTION_LEFT, ROBOT_ACTION_PWM, 0, 12000, 10);
    robot_action_record(ROBOT_ACTION_LEFT, ROBOT_ACTION_PWM, 100, 12000, 20);
    assert(1 == g_count); /* Deduplicate repeated intent, not PWM values. */
    assert(!robot_action_snapshot_if_empty(&e));
    robot_action_record(ROBOT_ACTION_RIGHT, ROBOT_ACTION_MOTION, 12000, -12000,
                        30);
    robot_action_record(ROBOT_ACTION_NONE, ROBOT_ACTION_STOP, 0, 0, 40);
    assert(4 == g_count);
    assert(robot_action_peek(&e) && 1 == e.seq && 10 == e.ts_ms &&
           ROBOT_ACTION_TURN_LEFT == e.action);
    robot_action_publish(100);
    assert(4 == g_count && 0 == sent_count);
    link_state = COMMS_LINK_MQTT_UP;
    robot_action_publish(101);
    assert(4 == g_count && 0 == sent_count);
    accept_publish = true;
    robot_action_publish(102);
    assert(2 == g_count && 2 == sent_count);
    assert(strstr(sent[0], "\"action\":\"turn_left\""));
    assert(strstr(sent[1], "\"action\":\"stop_turn_left\""));
    robot_action_publish(103);
    assert(0 == g_count && 5 == sent_count);
    assert(strstr(sent[2], "\"action\":\"turn_right\""));
    assert(strstr(sent[3], "\"action\":\"stop_turn_right\""));
    assert(strstr(sent[4], "\"type\":\"action_state\""));
    assert(strstr(sent[4], "\"direction\":\"none\""));
    robot_action_publish(104);
    assert(5 == sent_count); /* No snapshot spam. */
    robot_action_publish(1103);
    assert(6 == sent_count);
    for (unsigned i = 0; i < 50; ++i)
    {
        robot_action_record((i % 2) == 0 ? ROBOT_ACTION_LEFT
                                         : ROBOT_ACTION_NONE,
                            ROBOT_ACTION_PWM, 0, 0, 2000 + i);
    }
    assert(32 == g_count && 18 == g_dropped);
    assert(robot_action_peek(&e) && 23 == e.seq && 18 == e.dropped);
    robot_action_ack(1);
    assert(robot_action_peek(&e) &&
           23 == e.seq); /* Stale ack cannot pop a newer event. */
    robot_action_snapshot(&e);
    assert(ROBOT_ACTION_NONE == e.direction && 54 == e.seq && 18 == e.dropped);
    while (robot_action_peek(&e))
    {
        robot_action_ack(e.seq);
    }
    assert(robot_action_snapshot_if_empty(&e));
    assert(!robot_action_peek(NULL));
    puts("PASS: PWM direction, start/stop order, rapid reversal, "
         "deduplication, offline/backpressure retry, ordered snapshots, "
         "overflow counters, stale ack");
    return 0;
}

/*** end of file ***/

