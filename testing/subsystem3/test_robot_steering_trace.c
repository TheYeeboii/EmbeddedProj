#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "../../app_program/subsystem3/robot_steering_trace.c"
#include "../../app_program/subsystem1/robot_diagnostics_mqtt.c"
#include "../../app_program/subsystem1/robot_packed.c"
#include "../../app_program/subsystem1/robot_wire.c"

/* Packed steering message (robot_diagnostics_mqtt.c): 461 bits. */
#define STEERING_LENGTH 58u

static bool     g_b_online;
static bool     g_b_writable;
static unsigned g_published;
static uint8_t  g_last_payload[ROBOT_WIRE_DIAG_MESSAGE_MAX];
static uint32_t g_last_length;
uint8_t         comms_get_link_state (void)
{
    return g_b_online ? COMMS_LINK_MQTT_UP : 0;
}
bool comms_publish (uint32_t topic, const uint8_t * p_data, uint32_t length)
{
    assert(COMMS_TOPIC_STEERING == topic);
    assert(STEERING_LENGTH == length);
    if (!g_b_writable)
    {
        return false;
    }
    memcpy(g_last_payload, p_data, length);
    g_last_length = length;
    ++g_published;
    return true;
}
/* Header, state flag and boot id of the last message. */
static void last_header (uint32_t * p_state, uint32_t * p_boot)
{
    robot_packed_reader_t reader;

    robot_packed_reader_init(&reader, g_last_payload, g_last_length);
    assert(0x91u == robot_packed_get(&reader, 8u)); /* type 9, version 1 */
    *p_state = robot_packed_get(&reader, 1u);
    *p_boot  = robot_packed_get(&reader, 32u);
}
int main (void)
{
    robot_steering_event_t event = {0}, out;
    robot_diagnostics_init(123);
    /* Lag: a record stamped just after "now" was read is 0 ms old (it must
     * not wrap to 65535 and blank the live view); the tick wrap still works. */
    assert(10u == diag_lag(100u, 90u) && 0u == diag_lag(100u, 100u));
    assert(0u == diag_lag(100u, 101u) && 0u == diag_lag(100u, 102u));
    assert(10u == diag_lag(5u, UINT32_MAX - 4u));
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
    uint32_t state = 0u, boot = 0u;
    last_header(&state, &boot);
    assert(1u == state && 123u == boot); /* steering_state of boot 123 */
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
    last_header(&state, &boot); /* still exactly 58 bytes at every bound */
    assert(UINT32_MAX == boot);
    puts("PASS: steering pulse capture, coalescing, ownership, drops, stale "
         "ack, offline/backpressure retry and packed 58-byte bounds");
}

/*** end of file ***/

