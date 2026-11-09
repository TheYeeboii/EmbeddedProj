#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "../firmware/app_program/robot_ir_stream.c"
#include "../firmware/app_program/robot_diagnostics_mqtt.c"
#include "../firmware/app_program/robot_packed.c"
#include "../firmware/app_program/robot_wire.c"

static bool     g_b_online   = true;
static bool     g_b_writable = true;
static FILE *   g_p_fixture;
static unsigned g_published;
static unsigned g_largest;
uint8_t         comms_get_link_state (void)
{
    return g_b_online ? COMMS_LINK_MQTT_UP : 0;
}
bool comms_publish (uint32_t topic, const uint8_t * p_data, uint32_t length)
{
    /* Packed IR batch: type 11, version 1, within the outbox. */
    assert(COMMS_TOPIC_IR_STREAM == topic && 0xB1u == p_data[0] &&
           length <= ROBOT_WIRE_DIAG_MESSAGE_MAX);
    if (!g_b_writable)
    {
        return false;
    }
    ++g_published;
    if (length > g_largest)
    {
        g_largest = (unsigned) length;
    }
    if (g_p_fixture)
    {
        /* One message per line, as hex; tests/test_ir_stream_wire.cjs decodes
         * it with the dashboard's rover-wire.js. */
        for (uint32_t index = 0u; index < length; ++index)
        {
            fprintf(g_p_fixture, "%02x", (unsigned) p_data[index]);
        }
        fputc('\n', g_p_fixture);
    }
    return true;
}
int main (int argc, char ** argv)
{
    robot_barcode_observation_t observation = {0};

    robot_diagnostics_init(123); /* real steering-trace session id */
    observation.blocked =
        17; // Stopped/unconfigured and no valid decoder phase.
    if (argc > 1)
    {
        g_p_fixture = fopen(argv[1], "w");
        assert(g_p_fixture);
    }
    for (unsigned index = 0; index < 4; ++index)
    {
        observation.ts_ms      = 10 + index * 10;
        observation.black      = 1 == index || 2 == index;
        observation.normalized = observation.black ? 1000 : 0;
        observation.raw        = observation.black ? 2500 : 2000;
        robot_ir_stream_record(&observation, 2000, 2500, false);
    }
    robot_ir_stream_batch_t batch;
    assert(robot_ir_stream_peek(&batch) && 4 == batch.count);
    assert(batch.rows[1].black && batch.rows[2].black && !batch.rows[3].black);
    assert(0 == batch.rows[1].left_x10 &&
           0 == batch.rows[1].right_x10); // Stationary pulse is retained.
    robot_ir_stream_publish(40);
    assert(!g_count && 1 == g_published);
    for (unsigned index = 0; index < 4; ++index)
    {
        observation.ts_ms      = 1000 + index * 10;
        observation.black      = 1;
        observation.normalized = 1000;
        observation.raw        = 2500;
        observation.blocked    = 32;
        robot_ir_stream_record(&observation, 2000, 2500, false);
    }
    robot_ir_stream_publish(1030);
    assert(!g_count && 2 == g_published);
    if (g_p_fixture)
    {
        fclose(g_p_fixture);
        g_p_fixture = NULL;
    }
    // A short final batch is flushed within 40ms, not held forever.
    observation.ts_ms = 1100;
    robot_ir_stream_record(&observation, 2000, 2500, false);
    robot_ir_stream_publish(1100);
    assert(1 == g_count);
    g_b_online = false;
    robot_ir_stream_publish(1200);
    assert(1 == g_count);
    g_b_online   = true;
    g_b_writable = false;
    robot_ir_stream_publish(1200);
    assert(1 == g_count);
    g_b_writable = true;
    robot_ir_stream_publish(1200);
    assert(0 == g_count);
    // Overflow does not permit a stale ack to consume unrelated newer rows.
    observation.ts_ms = 1300;
    robot_ir_stream_record(&observation, 2000, 2500, false);
    assert(robot_ir_stream_peek(&batch));
    for (unsigned index = 0; index < ROBOT_IR_STREAM_CAPACITY + 8; ++index)
    {
        observation.ts_ms += 10;
        robot_ir_stream_record(&observation, 2000, 2500, false);
    }
    assert(g_dropped > 0 && ROBOT_IR_STREAM_CAPACITY == g_count);
    robot_ir_stream_ack(&batch);
    assert(ROBOT_IR_STREAM_CAPACITY == g_count);
    while (robot_ir_stream_peek(&batch))
    {
        robot_ir_stream_ack(&batch);
    }
    // Worst-case four-row JSON, unsigned sequence wrap and calibration values.
    g_sequence = UINT32_MAX - 3;
    g_dropped  = UINT32_MAX;
    robot_diagnostics_init(UINT32_MAX);
    observation.ts_ms      = UINT32_MAX;
    observation.left_x10   = -INT32_MAX;
    observation.right_x10  = INT32_MAX;
    observation.raw        = 4095;
    observation.normalized = 1000;
    observation.black      = 1;
    observation.blocked    = 63;
    for (unsigned index = 0; index < 4; ++index)
    {
        robot_ir_stream_record(&observation, 4095, 4095, true);
    }
    robot_ir_stream_publish(UINT32_MAX - 1);
    assert(0 == g_count);
    printf("PASS: independent IR capture while disabled/stationary, short "
           "pulses, final batch, offline/retry, overflow/stale ack, wrap; max "
           "packed batch %u bytes\n",
           g_largest);
}

/*** end of file ***/

