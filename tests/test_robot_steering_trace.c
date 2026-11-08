#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "../firmware/app_program/robot_steering_trace.c"
#include "../firmware/app_program/robot_steering_mqtt.c"

static bool     online, writable;
static unsigned published;
static char     last_payload[512];
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
    return online ? COMMS_LINK_MQTT_UP : 0;
}
bool comms_publish (uint32_t topic, const char * payload)
{
    assert(COMMS_TOPIC_STEERING == topic);
    if (!writable)
    {
        return false;
    }
    strcpy(last_payload, payload);
    ++published;
    return true;
}
int main (void)
{
    robot_steering_event_t e = {0}, out;
    robot_diagnostics_init(123);
    robot_steering_owner(TRACE_MANUAL);
    e.requested[1] = 2000;
    e.applied[1]   = 800;
    e.ts_ms        = 10;
    robot_steering_record(&e);
    e.ts_ms = 20;
    robot_steering_record(&e); // steady evidence coalesces
    assert(1 == count);
    e.ts_ms        = 30;
    e.requested[0] = 3000;
    robot_steering_record(&e);
    assert(2 == count); // 20ms left pulse retained
    assert(robot_steering_peek(&out) && 1 == out.direction &&
           800 == out.applied[1]);
    robot_steering_publish(40);
    assert(2 == count && !published);
    online = true;
    robot_steering_publish(40);
    assert(2 == count && !published);
    writable = true;
    robot_steering_publish(40);
    assert(0 == count && 2 == published);
    robot_steering_publish(50);
    assert(strstr(last_payload, "steering_state"));
    assert(strstr(last_payload, "\"boot\":123"));
    unsigned before = published;
    robot_steering_publish(60);
    assert(published == before);
    e.has_input       = true;
    e.input.threshold = 550;
    e.input.phase     = TRACE_CROSS;
    e.input.norm[0]   = 900;
    e.ts_ms           = 61;
    robot_steering_record(&e);
    assert(robot_steering_peek(&out) && TRACE_JUNCTION == out.owner);
    uint32_t stale = out.seq;
    for (unsigned i = 0; i < 80; ++i)
    {
        e.ts_ms += 100;
        robot_steering_record(&e);
    }
    assert(64 == count && dropped > 0);
    robot_steering_ack(stale);
    assert(64 == count);
    while (robot_steering_peek(&out))
    {
        robot_steering_ack(out.seq);
    }
    /* Exercise maximum wire length, including all unsigned/signed bounds. */
    memset(&e, 0, sizeof(e));
    e.has_input    = true;
    e.ts_ms        = UINT32_MAX;
    e.encoder_ms   = UINT32_MAX;
    e.input.ts_ms  = UINT32_MAX;
    e.requested[0] = e.applied[0] = e.speed_x10[0] = -32767;
    e.requested[1] = e.applied[1] = e.speed_x10[1] = 32767;
    e.distance_x10[0]                              = -INT32_MAX;
    e.distance_x10[1]                              = INT32_MAX;
    e.input.raw[0] = e.input.raw[1] = 4095;
    e.input.norm[0] = e.input.norm[1] = e.input.threshold = 1000;
    for (unsigned i = 0; i < 4; ++i)
    {
        e.input.calibration[i] = 4095;
    }
    e.input.dark  = 3;
    e.faults      = 3;
    e.input.phase = TRACE_DARK;
    sequence      = UINT32_MAX - 1;
    dropped       = UINT32_MAX;
    session_id    = UINT32_MAX;
    robot_steering_record(&e);
    robot_steering_publish(UINT32_MAX - 1);
    puts("PASS: steering pulse capture, coalescing, ownership, drops, stale "
         "ack, offline/backpressure retry and worst-case JSON bounds");
}

/*** end of file ***/

