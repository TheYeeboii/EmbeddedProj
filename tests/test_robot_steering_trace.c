#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "../firmware/app_program/robot_steering_trace.c"
#include "../firmware/app_program/robot_steering_mqtt.c"

static bool     g_b_online;
static bool     g_b_writable;
static unsigned g_published;
static char     g_last_payload[512];
int             tm_sprintf (unsigned char * out, unsigned char * fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int size = vsnprintf((char *) out, 512, (char *) fmt, args);
    va_end(args);
    assert(size > 0 && size < 512);
    return size;
}
uint32_t comms_get_link_state (void)
{
    return g_b_online ? COMMS_LINK_MQTT_UP : 0;
}
bool comms_publish (uint32_t topic, const char * payload)
{
    assert(COMMS_TOPIC_STEERING == topic);
    if (!g_b_writable)
    {
        return false;
    }
    strcpy(g_last_payload, payload);
    ++g_published;
    return true;
}
int main (void)
{
    robot_steering_event_t event = {0}, out;
    robot_diagnostics_init(123);
    robot_steering_owner(TRACE_MANUAL);
    event.requested[1] = 2000;
    event.applied[1]   = 800;
    event.ts_ms        = 10;
    robot_steering_record(&event);
    event.ts_ms = 20;
    robot_steering_record(&event); // steady evidence coalesces
    assert(1 == g_count);
    event.ts_ms        = 30;
    event.requested[0] = 3000;
    robot_steering_record(&event);
    assert(2 == g_count); // 20ms left pulse retained
    assert(robot_steering_peek(&out) && 1 == out.direction &&
           800 == out.applied[1]);
    robot_steering_publish(40);
    assert(2 == g_count && !g_published);
    g_b_online = true;
    robot_steering_publish(40);
    assert(2 == g_count && !g_published);
    g_b_writable = true;
    robot_steering_publish(40);
    assert(0 == g_count && 2 == g_published);
    robot_steering_publish(50);
    assert(strstr(g_last_payload, "steering_state"));
    assert(strstr(g_last_payload, "\"boot\":123"));
    unsigned before = g_published;
    robot_steering_publish(60);
    assert(g_published == before);
    event.has_input       = true;
    event.input.threshold = 550;
    event.input.phase     = TRACE_CROSS;
    event.input.norm[0]   = 900;
    event.ts_ms           = 61;
    robot_steering_record(&event);
    assert(robot_steering_peek(&out) && TRACE_JUNCTION == out.owner);
    uint32_t stale = out.seq;
    for (unsigned index = 0; index < 80; ++index)
    {
        event.ts_ms += 100;
        robot_steering_record(&event);
    }
    assert(64 == g_count && g_dropped > 0);
    robot_steering_ack(stale);
    assert(64 == g_count);
    while (robot_steering_peek(&out))
    {
        robot_steering_ack(out.seq);
    }
    /* Exercise maximum wire length, including all unsigned/signed bounds. */
    memset(&event, 0, sizeof(event));
    event.has_input    = true;
    event.ts_ms        = UINT32_MAX;
    event.encoder_ms   = UINT32_MAX;
    event.input.ts_ms  = UINT32_MAX;
    event.requested[0] = event.applied[0] = event.speed_x10[0] = -32767;
    event.requested[1] = event.applied[1] = event.speed_x10[1] = 32767;
    event.distance_x10[0]                                      = -INT32_MAX;
    event.distance_x10[1]                                      = INT32_MAX;
    event.input.raw[0] = event.input.raw[1] = 4095;
    event.input.norm[0] = event.input.norm[1] = event.input.threshold = 1000;
    for (unsigned index = 0; index < 4; ++index)
    {
        event.input.calibration[index] = 4095;
    }
    event.input.dark  = 3;
    event.faults      = 3;
    event.input.phase = TRACE_DARK;
    g_sequence        = UINT32_MAX - 1;
    g_dropped         = UINT32_MAX;
    g_session_id      = UINT32_MAX;
    robot_steering_record(&event);
    robot_steering_publish(UINT32_MAX - 1);
    puts("PASS: steering pulse capture, coalescing, ownership, drops, stale "
         "ack, offline/backpressure retry and worst-case JSON bounds");
}

/*** end of file ***/

