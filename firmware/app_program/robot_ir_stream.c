/** @file robot_ir_stream.c
 *
 * @brief Fixed-size queue of continuous barcode-sensor (ADC2) samples, filled
 *        by the line task and drained by the MQTT publisher.
 */
#include <tk/tkernel.h>
#include "robot_ir_stream.h"

static robot_ir_stream_sample_t g_queue[ROBOT_IR_STREAM_CAPACITY];
static uint32_t                 g_head     = 0u;
static uint32_t                 g_count    = 0u;
static uint32_t                 g_sequence = 0u;
static uint32_t                 g_dropped  = 0u;

void robot_ir_stream_record (const robot_barcode_observation_t * p_observation,
                             uint16_t minimum, uint16_t maximum, bool inverted)
{
    UINT                     lock   = 0u;
    robot_ir_stream_sample_t sample = {0};

    sample.ts_ms      = p_observation->ts_ms;
    sample.raw        = p_observation->raw;
    sample.normalized = p_observation->normalized;
    sample.black      = p_observation->black;
    sample.blocked    = p_observation->blocked;
    sample.left_x10   = p_observation->left_x10;
    sample.right_x10  = p_observation->right_x10;
    sample.minimum    = minimum;
    sample.maximum    = maximum;
    sample.inverted   = inverted;

    DI(lock);
    if (ROBOT_IR_STREAM_CAPACITY == g_count)
    {
        /* Full: overwrite the oldest sample and count it as dropped. */
        g_head = (g_head + 1u) % ROBOT_IR_STREAM_CAPACITY;
        --g_count;
        ++g_dropped;
    }
    sample.seq = ++g_sequence;
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
    p_batch->count = (uint8_t) ((g_count > ROBOT_IR_STREAM_BATCH)
                                    ? ROBOT_IR_STREAM_BATCH
                                    : g_count);
    p_batch->dropped = g_dropped;
    for (row = 0u; row < p_batch->count; ++row)
    {
        p_batch->rows[row] =
            g_queue[(g_head + row) % ROBOT_IR_STREAM_CAPACITY];
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

