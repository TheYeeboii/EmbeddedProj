/** @file robot_ir_stream.c
 *
 * @brief Fixed-size queue of continuous barcode-sensor (GPIO) samples, filled
 *        by the line task and drained by the MQTT publisher.
 */
#include <tk/tkernel.h>
#include "robot_ir_stream.h"

static robot_ir_stream_sample_t g_queue[ROBOT_IR_STREAM_CAPACITY];
static uint32_t                 g_head     = 0u;
static uint32_t                 g_count    = 0u;
static uint32_t                 g_sequence = 0u;
static uint32_t                 g_dropped  = 0u;

int32_t robot_ir_stream_position (float32_t distance_mm)
{
    const float32_t scaled = distance_mm * 10.0f;
    if (scaled >= 2147480000.0f)
    {
        return INT32_MAX;
    }
    if (scaled <= -2147480000.0f)
    {
        return -INT32_MAX;
    }
    /* NaN also fails this comparison. */
    return scaled >= -2147480000.0f ? (int32_t) scaled : 0;
}

void robot_ir_stream_record (const robot_ir_stream_sample_t * p_observation)
{
    UINT lock;
    robot_ir_stream_sample_t sample = *p_observation;

    DI(lock);
    if (ROBOT_IR_STREAM_CAPACITY == g_count)
    {
        /* Full: overwrite the oldest sample and count it as dropped. */
        g_head = (g_head + 1u) % ROBOT_IR_STREAM_CAPACITY;
        --g_count;
        ++g_dropped;
    }
    sample.seq                                             = ++g_sequence;
    g_queue[(g_head + g_count) % ROBOT_IR_STREAM_CAPACITY] = sample;
    ++g_count;
    EI(lock);
}

bool robot_ir_stream_peek (robot_ir_stream_batch_t * p_batch)
{
    UINT     lock = 0u;
    uint32_t row  = 0u;

    DI(lock);
    /* The ternary result is at most ROBOT_IR_STREAM_BATCH, so it fits. */
    p_batch->count =
        (uint8_t) ((g_count > ROBOT_IR_STREAM_BATCH) ? ROBOT_IR_STREAM_BATCH
                                                     : g_count);
    p_batch->dropped = g_dropped;
    for (row = 0u; row < p_batch->count; ++row)
    {
        p_batch->rows[row] = g_queue[(g_head + row) % ROBOT_IR_STREAM_CAPACITY];
    }
    EI(lock);

    return (0u != p_batch->count);
}

void robot_ir_stream_ack (const robot_ir_stream_batch_t * p_batch)
{
    UINT     lock = 0u;
    uint32_t row  = 0u;

    DI(lock);
    /* Producer overflow may remove some/all peeked rows before publish. Only
     * consume rows that were actually included, never a newer head. */
    for (row = 0u; row < p_batch->count; ++row)
    {
        if ((0u != g_count) && (g_queue[g_head].seq == p_batch->rows[row].seq))
        {
            g_head = (g_head + 1u) % ROBOT_IR_STREAM_CAPACITY;
            --g_count;
        }
    }
    EI(lock);
}

/*** end of file ***/

