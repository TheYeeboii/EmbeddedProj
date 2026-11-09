#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "../firmware/app_program/robot_barcode_trace.c"
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
    assert(COMMS_TOPIC_BARCODE == topic);
    /* Packed barcode message: type 10, version 1, within the outbox. */
    assert(0xA1u == p_data[0] && length <= ROBOT_WIRE_DIAG_MESSAGE_MAX);
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
        /* One message per line, as hex; tests/test_barcode_wire.cjs decodes it
         * with the dashboard's rover-wire.js. */
        for (uint32_t index = 0u; index < length; ++index)
        {
            fprintf(g_p_fixture, "%02x", (unsigned) p_data[index]);
        }
        fputc('\n', g_p_fixture);
    }
    return true;
}
static void flush (uint32_t now)
{
    robot_barcode_packet_t peeked;
    for (unsigned index = 0;
         index < 1000 && robot_barcode_packet_peek(&peeked, now); ++index)
    {
        robot_barcode_publish(now);
    }
}
static void reset (void)
{
    memset(g_frames, 0, sizeof(g_frames));
    g_active_slot     = -1;
    g_scan_sequence   = 0;
    g_packet_sequence = 0;
    g_dropped_scans   = 0;
    g_b_have_state    = false;
    int32_t cfg[]     = {2000, 2500, 0, 400, 600, 25, 0, 1500, 650, 3870, 3870};
    robot_barcode_trace_configure(cfg);
}
static void replay (bool reverse, bool interrupt)
{
    const char * patterns[] = {"010010100", "001011000", "010010100"};
    float32_t    widths[29];
    float32_t    boundaries[30];
    boundaries[0] = 7.5f;
    for (unsigned index = 0; index < 29; ++index)
    {
        widths[index] = (9 == index || 19 == index)               ? 0.5f
                        : patterns[index / 10][index % 10] == '1' ? 1.5f
                                                                  : 0.5f;
    }
    for (unsigned index = 0; index < 29; ++index)
    {
        boundaries[index + 1] =
            boundaries[index] + widths[reverse ? 28 - index : index];
    }
    buddy3_barcode_decoder_t decoder  = {0};
    buddy3_barcode_config_t  cfg      = {0.25f};
    bool                     accepted = false;
    bool                     aborted  = false;
    for (unsigned tick = 0; tick < 720; ++tick)
    {
        float32_t pos   = (float32_t) tick * 0.0625f;
        bool      black = false;
        for (unsigned index = 0; index < 29; ++index)
        {
            if (pos >= boundaries[index] && pos < boundaries[index + 1])
            {
                black = index % 2 == 0;
            }
        }
        bool      before  = decoder.active;
        float32_t partial = decoder.position - decoder.edge_position + 0.0625f;
        bool      permitted = !(interrupt && pos > 11);
        buddy3_barcode_event_t event;
        bool                   emitted =
            buddy3_barcode_step(&decoder, &cfg, black, tick ? 0.0625f : 0,
                                tick * 10, permitted, &event);
        robot_barcode_observation_t observation = {tick * 10,
                                                   (int32_t) (pos * 10),
                                                   (int32_t) (pos * 10),
                                                   black ? 2500 : 2000,
                                                   black ? 1000 : 0,
                                                   black,
                                                   permitted ? 0 : 2,
                                                   6};
        robot_barcode_trace_observe(&decoder, &observation, before, partial,
                                    emitted ? &event : NULL);
        if (emitted)
        {
            if (interrupt)
            {
                assert(B3BC_INTERRUPTED == event.kind);
                aborted = true;
            }
            else
            {
                assert(B3BC_VALID == event.kind &&
                       event.decoded.symbol == 'F' &&
                       event.decoded.reversed == reverse);
                accepted = true;
            }
        }
        if (tick % 2 == 0)
        {
            robot_barcode_publish(tick * 10);
        }
    }
    assert(interrupt ? aborted : accepted);
    flush(8000);
}
/* The reader-state packet carries the configuration; disabling clears it. */
static void test_reader_state (void)
{
    robot_barcode_packet_t initial = {0};

    reset();
    assert(robot_barcode_packet_peek(&initial, 0));
    assert(BC_PACKET_STATE == initial.type && 25 == initial.config[5] &&
           400 == initial.config[3]);
    robot_barcode_trace_enable(false);
    assert(robot_barcode_packet_peek(&initial, 0) && 0 == initial.state &&
           0 == initial.config[5]);
}

/* Replay forward, reversed and aborted scans; optionally save them. */
static void test_replays (int argc, char ** argv)
{
    reset();
    if (argc > 1)
    {
        g_p_fixture = fopen(argv[1], "w");
        assert(g_p_fixture);
    }
    replay(false, false);
    replay(true, false);
    replay(false, true);
    if (g_p_fixture)
    {
        fclose(g_p_fixture);
        g_p_fixture = NULL;
    }
}

/* Aborts, raw noise, overwrite pressure and pre-frame spacing failures. */
static void test_capture_edge_cases (void)
{
    buddy3_barcode_decoder_t    decoder     = {0};
    robot_barcode_observation_t observation = {20, 10, 10, 2500, 1000, 1, 0, 0};
    buddy3_barcode_event_t      failure     = {0};
    robot_barcode_packet_t      packet      = {0};

    /* The abort is preserved even though the core reset its own arrays. */
    reset();
    decoder.active        = true;
    decoder.leading_quiet = 5;
    decoder.count         = 1;
    decoder.widths[0]     = 1;
    decoder.edge_ms[0]    = 20;
    decoder.position      = 1.5f;
    decoder.edge_position = 1;
    robot_barcode_trace_observe(&decoder, &observation, false, 0, NULL);
    buddy3_barcode_reset(&decoder);
    failure.kind      = B3BC_STALL;
    observation.ts_ms = 1600;
    robot_barcode_trace_observe(&decoder, &observation, true, 0.5f, &failure);
    assert(g_frames[0].done && 1 == g_frames[0].count &&
           100 == g_frames[0].edges[0].width_x100 &&
           B3BC_STALL == g_frames[0].reason);
    assert(robot_barcode_packet_peek(&packet, 1600));
    g_b_writable = false;
    robot_barcode_publish(1600);
    assert(!g_frames[0].sent_start);
    g_b_writable = true;
    flush(1600);
    assert(!g_frames[0].occupied);
    /* Stationary raw noise adds observations but cannot create widths. */
    reset();
    decoder.active        = true;
    decoder.leading_quiet = 5;
    decoder.count         = 0;
    decoder.position      = 0;
    decoder.edge_position = 0;
    for (unsigned index = 0; index < 600; ++index)
    {
        observation.ts_ms      = index * 10;
        observation.normalized = (0u != (index % 2u)) ? 1000u : 0u;
        robot_barcode_trace_observe(&decoder, &observation, index != 0, 0,
                                    NULL);
    }
    assert(0 == g_frames[0].count && 512 == g_frames[0].sample_count &&
           88 == g_frames[0].sample_drops);
    robot_barcode_trace_cancel(6000);
    assert(g_frames[0].done && B3BC_INTERRUPTED == g_frames[0].reason);
    /* Overwrite pressure and stale acknowledgement cannot consume a new scan.
     */
    assert(robot_barcode_packet_peek(&packet, 6000));
    for (unsigned index = 0; index < 3; ++index)
    {
        observation.ts_ms += 100;
        robot_barcode_trace_observe(&decoder, &observation, false, 0, NULL);
        robot_barcode_trace_cancel(observation.ts_ms);
    }
    robot_barcode_packet_ack(&packet);
    assert(g_dropped_scans >= 2);
    /* An oversized step before a frame stays visible across the next zero
     * tick; a 1 Hz status snapshot must not hide the rejection. */
    reset();
    buddy3_barcode_reset(&decoder);
    observation.ts_ms       = 100;
    observation.blocked     = 32;
    observation.travel_x100 = 100;
    robot_barcode_trace_observe(&decoder, &observation, false, 0, NULL);
    observation.ts_ms       = 110;
    observation.blocked     = 0;
    observation.travel_x100 = 0;
    robot_barcode_trace_observe(&decoder, &observation, false, 0, NULL);
    assert(robot_barcode_packet_peek(&packet, 110));
    assert(6 == packet.state && (packet.blocked & 32) &&
           1 == packet.rejected_steps);
    assert(100 == packet.readiness[6] &&
           packet.readiness[0] == observation.raw);
    observation.ts_ms = 1200;
    robot_barcode_trace_observe(&decoder, &observation, false, 0, NULL);
    assert(robot_barcode_packet_peek(&packet, 1200) && 1 == packet.state);
}

/* All packet kinds serialize safely at the largest numeric bounds. */
static void test_numeric_bounds (void)
{
    robot_barcode_frame_t * p_frame = &g_frames[0];

    reset();
    p_frame->occupied     = true;
    p_frame->scan_id      = UINT32_MAX;
    p_frame->started_ms   = UINT32_MAX;
    p_frame->updated_ms   = UINT32_MAX;
    p_frame->leading_x100 = INT32_MAX;
    for (unsigned index = 0; index < 11; ++index)
    {
        p_frame->config[index] = INT32_MAX;
    }
    robot_barcode_publish(UINT32_MAX);
    p_frame->count = 4;
    for (unsigned index = 0; index < 4; ++index)
    {
        p_frame->edges[index].ts_ms      = UINT32_MAX;
        p_frame->edges[index].width_x100 = INT32_MAX;
        p_frame->edges[index].left_x10   = -INT32_MAX;
        p_frame->edges[index].right_x10  = INT32_MAX;
    }
    robot_barcode_publish(UINT32_MAX);
    p_frame->sample_count = 4;
    for (unsigned index = 0; index < 4; ++index)
    {
        p_frame->samples[index].ts_ms         = UINT32_MAX;
        p_frame->samples[index].raw           = 4095;
        p_frame->samples[index].normalized    = 1000;
        p_frame->samples[index].left_x10      = -INT32_MAX;
        p_frame->samples[index].right_x10     = INT32_MAX;
        p_frame->samples[index].position_x100 = INT32_MAX;
        p_frame->samples[index].black         = 1;
    }
    robot_barcode_publish(UINT32_MAX);
    p_frame->done          = true;
    p_frame->state         = 5;
    p_frame->reason        = B3BC_FORMAT;
    p_frame->format        = CODE39_BAD_WIDTH;
    p_frame->partial_x100  = INT32_MAX;
    p_frame->trailing_x100 = INT32_MAX;
    p_frame->narrow_x100   = INT32_MAX;
    p_frame->ratio_x100    = INT32_MAX;
    p_frame->max_gap_x100  = INT32_MAX;
    p_frame->sample_drops  = UINT32_MAX;
    g_dropped_scans        = UINT32_MAX;
    robot_barcode_publish(UINT32_MAX);
    for (unsigned index = 0; index < 11; ++index)
    {
        g_configuration[index] = INT32_MAX;
    }
    g_b_have_state    = false;
    g_packet_sequence = UINT32_MAX - 1;
    g_reader_state    = 6;
    g_reader_blocked  = 63;
    g_readiness[0]    = 4095;
    g_readiness[1]    = 1000;
    g_readiness[2]    = 1;
    for (unsigned index = 3; index < 7; ++index)
    {
        g_readiness[index] = INT32_MAX;
    }
    g_rejected_steps = UINT32_MAX;
    robot_barcode_publish(UINT32_MAX);
}

int main (int argc, char ** argv)
{
    robot_diagnostics_init(123); /* real steering-trace session id */
    test_reader_state();
    test_replays(argc, argv);
    test_capture_edge_cases();
    test_numeric_bounds();
    printf("PASS: barcode capture-to-MQTT replay, forward/reverse F, aborted "
           "evidence, raw bounds, pressure/retry, stale ack; max packed "
           "payload %u bytes\n",
           g_largest);

    return 0;
}

/*** end of file ***/

