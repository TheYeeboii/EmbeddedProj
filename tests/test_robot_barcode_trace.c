#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "../firmware/app_program/robot_barcode_trace.c"
#include "../firmware/app_program/robot_barcode_mqtt.c"

static bool     online = true, writable = true;
static FILE *   fixture;
static char *   format_base;
static unsigned published, largest;
uint32_t        robot_diagnostics_session (void)
{
    return 123;
}
uint32_t comms_get_link_state (void)
{
    return online ? COMMS_LINK_MQTT_UP : 0;
}
int tm_sprintf (unsigned char * out, unsigned char * fmt, ...)
{
    /* The serializer appends only fixed-format, bounded rows. Check every
     * append against the TOTAL buffer, not just the current formatting call. */
    if (strstr((char *) fmt, "\"v\":1"))
    {
        format_base = (char *) out;
    }
    size_t  remaining = 512 - (size_t) ((char *) out - format_base);
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf((char *) out, remaining, (char *) fmt, args);
    va_end(args);
    assert(n >= 0 && (size_t) n < remaining);
    return n;
}
bool comms_publish (uint32_t topic, const char * payload)
{
    assert(COMMS_TOPIC_BARCODE == topic);
    assert(strlen(payload) < 512);
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
static void flush (uint32_t now)
{
    robot_barcode_packet_t p;
    for (unsigned i = 0; i < 1000 && robot_barcode_packet_peek(&p, now); ++i)
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
    float32_t    widths[29], boundaries[30];
    boundaries[0] = 7.5f;
    for (unsigned i = 0; i < 29; ++i)
    {
        widths[i] = (9 == i || 19 == i)               ? 0.5f
                    : patterns[i / 10][i % 10] == '1' ? 1.5f
                                                      : 0.5f;
    }
    for (unsigned i = 0; i < 29; ++i)
    {
        boundaries[i + 1] = boundaries[i] + widths[reverse ? 28 - i : i];
    }
    buddy3_barcode_decoder_t s        = {0};
    buddy3_barcode_config_t  cfg      = {0.25f};
    bool                     accepted = false, aborted = false;
    for (unsigned tick = 0; tick < 720; ++tick)
    {
        float32_t pos   = (float32_t) tick * 0.0625f;
        bool      black = false;
        for (unsigned i = 0; i < 29; ++i)
        {
            if (pos >= boundaries[i] && pos < boundaries[i + 1])
            {
                black = i % 2 == 0;
            }
        }
        bool                   before  = s.active;
        float32_t              partial = s.position - s.edge_position + 0.0625f;
        bool                   permitted = !(interrupt && pos > 11);
        buddy3_barcode_event_t event;
        bool emitted = buddy3_barcode_step(&s, &cfg, black, tick ? 0.0625f : 0,
                                           tick * 10, permitted, &event);
        robot_barcode_observation_t o = {tick * 10,
                                         (int32_t) (pos * 10),
                                         (int32_t) (pos * 10),
                                         black ? 2500 : 2000,
                                         black ? 1000 : 0,
                                         black,
                                         permitted ? 0 : 2,
                                         6};
        robot_barcode_trace_observe(&s, &o, before, partial,
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
int main (int argc, char ** argv)
{
    reset();
    robot_barcode_packet_t initial;
    assert(robot_barcode_packet_peek(&initial, 0));
    assert(BC_PACKET_STATE == initial.type && 25 == initial.config[5] &&
           400 == initial.config[3]);
    robot_barcode_trace_enable(false);
    assert(robot_barcode_packet_peek(&initial, 0) && 0 == initial.state &&
           0 == initial.config[5]);
    reset();
    if (argc > 1)
    {
        fixture = fopen(argv[1], "w");
        assert(fixture);
    }
    replay(false, false);
    replay(true, false);
    replay(false, true);
    if (fixture)
    {
        fclose(fixture);
        fixture = NULL;
    }
    /* The abort is preserved even though the core reset its own arrays. */
    reset();
    buddy3_barcode_decoder_t s    = {0};
    s.active                      = true;
    s.leading_quiet               = 5;
    s.count                       = 1;
    s.widths[0]                   = 1;
    s.edge_ms[0]                  = 20;
    s.position                    = 1.5f;
    s.edge_position               = 1;
    robot_barcode_observation_t o = {20, 10, 10, 2500, 1000, 1, 0, 0};
    robot_barcode_trace_observe(&s, &o, false, 0, NULL);
    buddy3_barcode_reset(&s);
    buddy3_barcode_event_t failure = {0};
    failure.kind                   = B3BC_STALL;
    o.ts_ms                        = 1600;
    robot_barcode_trace_observe(&s, &o, true, 0.5f, &failure);
    assert(g_frames[0].done && 1 == g_frames[0].count &&
           100 == g_frames[0].edges[0].width_x100 &&
           B3BC_STALL == g_frames[0].reason);
    robot_barcode_packet_t packet;
    assert(robot_barcode_packet_peek(&packet, 1600));
    writable = false;
    robot_barcode_publish(1600);
    assert(!g_frames[0].sent_start);
    writable = true;
    flush(1600);
    assert(!g_frames[0].occupied);
    /* Stationary raw noise adds observations but cannot create widths. */
    reset();
    s.active        = true;
    s.leading_quiet = 5;
    s.count         = 0;
    s.position      = 0;
    s.edge_position = 0;
    for (unsigned i = 0; i < 600; ++i)
    {
        o.ts_ms      = i * 10;
        o.normalized = i % 2 ? 1000 : 0;
        robot_barcode_trace_observe(&s, &o, i != 0, 0, NULL);
    }
    assert(0 == g_frames[0].count && 512 == g_frames[0].sample_count &&
           88 == g_frames[0].sample_drops);
    robot_barcode_trace_cancel(6000);
    assert(g_frames[0].done && B3BC_INTERRUPTED == g_frames[0].reason);
    /* Overwrite pressure and stale acknowledgement cannot consume a new scan.
     */
    assert(robot_barcode_packet_peek(&packet, 6000));
    for (unsigned i = 0; i < 3; ++i)
    {
        o.ts_ms += 100;
        robot_barcode_trace_observe(&s, &o, false, 0, NULL);
        robot_barcode_trace_cancel(o.ts_ms);
    }
    robot_barcode_packet_ack(&packet);
    assert(g_dropped_scans >= 2);
    /* An oversized step before a frame stays visible across the next zero
     * tick; a 1 Hz status snapshot must not hide the rejection. */
    reset();
    buddy3_barcode_reset(&s);
    o.ts_ms       = 100;
    o.blocked     = 32;
    o.travel_x100 = 100;
    robot_barcode_trace_observe(&s, &o, false, 0, NULL);
    o.ts_ms       = 110;
    o.blocked     = 0;
    o.travel_x100 = 0;
    robot_barcode_trace_observe(&s, &o, false, 0, NULL);
    assert(robot_barcode_packet_peek(&packet, 110));
    assert(6 == packet.state && (packet.blocked & 32) &&
           1 == packet.rejected_steps);
    assert(100 == packet.readiness[6] && packet.readiness[0] == o.raw);
    o.ts_ms = 1200;
    robot_barcode_trace_observe(&s, &o, false, 0, NULL);
    assert(robot_barcode_packet_peek(&packet, 1200) && 1 == packet.state);
    /* All packet kinds serialize safely at the largest numeric bounds. */
    reset();
    robot_barcode_frame_t * f = &g_frames[0];
    f->occupied               = true;
    f->id                     = UINT32_MAX;
    f->started_ms = f->updated_ms = UINT32_MAX;
    f->leading_x100               = INT32_MAX;
    for (unsigned i = 0; i < 11; ++i)
    {
        f->config[i] = INT32_MAX;
    }
    robot_barcode_publish(UINT32_MAX);
    f->count = 4;
    for (unsigned i = 0; i < 4; ++i)
    {
        f->edges[i].ts_ms      = UINT32_MAX;
        f->edges[i].width_x100 = INT32_MAX;
        f->edges[i].left_x10   = -INT32_MAX;
        f->edges[i].right_x10  = INT32_MAX;
    }
    robot_barcode_publish(UINT32_MAX);
    f->sample_count = 4;
    for (unsigned i = 0; i < 4; ++i)
    {
        f->samples[i].ts_ms         = UINT32_MAX;
        f->samples[i].raw           = 4095;
        f->samples[i].normalized    = 1000;
        f->samples[i].left_x10      = -INT32_MAX;
        f->samples[i].right_x10     = INT32_MAX;
        f->samples[i].position_x100 = INT32_MAX;
        f->samples[i].black         = 1;
    }
    robot_barcode_publish(UINT32_MAX);
    f->done         = true;
    f->state        = 5;
    f->reason       = B3BC_FORMAT;
    f->format       = CODE39_BAD_WIDTH;
    f->partial_x100 = f->trailing_x100 = f->narrow_x100 = f->ratio_x100 =
        f->max_gap_x100                                 = INT32_MAX;
    f->sample_drops                                     = UINT32_MAX;
    g_dropped_scans                                     = UINT32_MAX;
    robot_barcode_publish(UINT32_MAX);
    for (unsigned i = 0; i < 11; ++i)
    {
        g_configuration[i] = INT32_MAX;
    }
    g_b_have_state    = false;
    g_packet_sequence = UINT32_MAX - 1;
    g_reader_state    = 6;
    g_reader_blocked  = 63;
    g_readiness[0]    = 4095;
    g_readiness[1]    = 1000;
    g_readiness[2]    = 1;
    for (unsigned i = 3; i < 7; ++i)
    {
        g_readiness[i] = INT32_MAX;
    }
    g_rejected_steps = UINT32_MAX;
    robot_barcode_publish(UINT32_MAX);
    printf("PASS: barcode capture-to-MQTT replay, forward/reverse F, aborted "
           "evidence, raw bounds, pressure/retry, stale ack; max payload %u "
           "bytes\n",
           largest);
}

/*** end of file ***/

