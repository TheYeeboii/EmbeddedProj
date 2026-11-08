#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "../firmware/app_program/robot_ir_stream.c"
#include "../firmware/app_program/robot_ir_mqtt.c"

static bool     online = true, writable = true;
static FILE *   fixture;
static char *   base;
static unsigned published, largest;
static uint32_t boot = 123;
uint32_t        robot_diagnostics_session (void)
{
    return boot;
}
uint32_t comms_get_link_state (void)
{
    return online ? COMMS_LINK_MQTT_UP : 0;
}
int tm_sprintf (unsigned char * out, unsigned char * fmt, ...)
{
    if (strstr((char *) fmt, "\"v\":1"))
    {
        base = (char *) out;
    }
    size_t  remaining = 512 - (size_t) ((char *) out - base);
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf((char *) out, remaining, (char *) fmt, args);
    va_end(args);
    assert(n >= 0 && (size_t) n < remaining);
    return n;
}
bool comms_publish (uint32_t topic, const char * payload)
{
    assert(COMMS_TOPIC_IR_STREAM == topic && strlen(payload) < 512);
    if (!writable)
    {
        return false;
    }
    ++published;
    if (strlen(payload) > largest)
    {
        largest = (unsigned) strlen(payload);
    }
    if (fixture)
    {
        fprintf(fixture, "%s\n", payload);
    }
    return true;
}
int main (int argc, char ** argv)
{
    robot_barcode_observation_t o = {0};
    o.blocked = 17; // Stopped/unconfigured and no valid decoder phase.
    if (argc > 1)
    {
        fixture = fopen(argv[1], "w");
        assert(fixture);
    }
    for (unsigned i = 0; i < 4; ++i)
    {
        o.ts_ms      = 10 + i * 10;
        o.black      = 1 == i || 2 == i;
        o.normalized = o.black ? 1000 : 0;
        o.raw        = o.black ? 2500 : 2000;
        robot_ir_stream_record(&o, 2000, 2500, false);
    }
    robot_ir_stream_batch_t batch;
    assert(robot_ir_stream_peek(&batch) && 4 == batch.count);
    assert(batch.rows[1].black && batch.rows[2].black && !batch.rows[3].black);
    assert(0 == batch.rows[1].left_x10 &&
           0 == batch.rows[1].right_x10); // Stationary pulse is retained.
    robot_ir_stream_publish(40);
    assert(!g_count && 1 == published);
    for (unsigned i = 0; i < 4; ++i)
    {
        o.ts_ms      = 1000 + i * 10;
        o.black      = 1;
        o.normalized = 1000;
        o.raw        = 2500;
        o.blocked    = 32;
        robot_ir_stream_record(&o, 2000, 2500, false);
    }
    robot_ir_stream_publish(1030);
    assert(!g_count && 2 == published);
    if (fixture)
    {
        fclose(fixture);
        fixture = NULL;
    }
    // A short final batch is flushed within 40ms, not held forever.
    o.ts_ms = 1100;
    robot_ir_stream_record(&o, 2000, 2500, false);
    robot_ir_stream_publish(1100);
    assert(1 == g_count);
    online = false;
    robot_ir_stream_publish(1200);
    assert(1 == g_count);
    online   = true;
    writable = false;
    robot_ir_stream_publish(1200);
    assert(1 == g_count);
    writable = true;
    robot_ir_stream_publish(1200);
    assert(0 == g_count);
    // Overflow does not permit a stale ack to consume unrelated newer rows.
    o.ts_ms = 1300;
    robot_ir_stream_record(&o, 2000, 2500, false);
    assert(robot_ir_stream_peek(&batch));
    for (unsigned i = 0; i < ROBOT_IR_STREAM_CAPACITY + 8; ++i)
    {
        o.ts_ms += 10;
        robot_ir_stream_record(&o, 2000, 2500, false);
    }
    assert(g_dropped > 0 && ROBOT_IR_STREAM_CAPACITY == g_count);
    robot_ir_stream_ack(&batch);
    assert(ROBOT_IR_STREAM_CAPACITY == g_count);
    while (robot_ir_stream_peek(&batch))
    {
        robot_ir_stream_ack(&batch);
    }
    // Worst-case four-row JSON, unsigned sequence wrap and calibration values.
    g_sequence     = UINT32_MAX - 3;
    g_dropped      = UINT32_MAX;
    boot         = UINT32_MAX;
    o.ts_ms      = UINT32_MAX;
    o.left_x10   = -INT32_MAX;
    o.right_x10  = INT32_MAX;
    o.raw        = 4095;
    o.normalized = 1000;
    o.black      = 1;
    o.blocked    = 63;
    for (unsigned i = 0; i < 4; ++i)
    {
        robot_ir_stream_record(&o, 4095, 4095, true);
    }
    robot_ir_stream_publish(UINT32_MAX - 1);
    assert(0 == g_count);
    printf("PASS: independent IR capture while disabled/stationary, short "
           "pulses, final batch, offline/retry, overflow/stale ack, wrap; max "
           "packet %u bytes\n",
           largest);
}

/*** end of file ***/

