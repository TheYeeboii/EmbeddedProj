#include <tk/tkernel.h>
#include "robot_ir_stream.h"

static robot_ir_stream_sample_t queue[ROBOT_IR_STREAM_CAPACITY];
static uint32_t head, count, sequence, dropped;

void robot_ir_stream_record(const robot_barcode_observation_t *o,
    uint16_t minimum, uint16_t maximum, bool inverted)
{
    UINT lock;
    robot_ir_stream_sample_t sample = {0};
    sample.ts_ms = o->ts_ms; sample.raw = o->raw; sample.normalized = o->normalized;
    sample.black = o->black; sample.blocked = o->blocked;
    sample.left_x10 = o->left_x10; sample.right_x10 = o->right_x10;
    sample.minimum = minimum; sample.maximum = maximum; sample.inverted = inverted;
    DI(lock);
    if (count == ROBOT_IR_STREAM_CAPACITY) {
        head = (head + 1u) % ROBOT_IR_STREAM_CAPACITY; --count; ++dropped;
    }
    sample.seq = ++sequence;
    queue[(head + count) % ROBOT_IR_STREAM_CAPACITY] = sample;
    ++count;
    EI(lock);
}
bool robot_ir_stream_peek(robot_ir_stream_batch_t *batch)
{
    UINT lock;
    unsigned i;
    DI(lock);
    batch->count = (uint8_t)(count > ROBOT_IR_STREAM_BATCH ? ROBOT_IR_STREAM_BATCH : count);
    batch->dropped = dropped;
    for (i = 0; i < batch->count; ++i) batch->rows[i] = queue[(head + i) % ROBOT_IR_STREAM_CAPACITY];
    EI(lock); return batch->count != 0;
}
void robot_ir_stream_ack(const robot_ir_stream_batch_t *batch)
{
    UINT lock;
    unsigned i;
    DI(lock);
    /* Producer overflow may remove some/all peeked rows before publish. Only
     * consume rows that were actually included, never a newer head. */
    for (i = 0; i < batch->count; ++i)
        if (count && queue[head].seq == batch->rows[i].seq) {
            head = (head + 1u) % ROBOT_IR_STREAM_CAPACITY; --count;
        }
    EI(lock);
}
