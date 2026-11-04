#ifndef ROBOT_IR_STREAM_H
#define ROBOT_IR_STREAM_H
#include <stdbool.h>
#include <stdint.h>
#include "robot_barcode_trace.h"

#define ROBOT_IR_STREAM_CAPACITY 256u
#define ROBOT_IR_STREAM_BATCH 4u
#define ROBOT_IR_STREAM_PERIOD_MS 10u
typedef struct {
    uint32_t seq, ts_ms;
    int32_t left_x10, right_x10;
    uint16_t raw, normalized, minimum, maximum;
    uint8_t black, blocked, inverted;
} robot_ir_stream_sample_t;
typedef struct {
    robot_ir_stream_sample_t rows[ROBOT_IR_STREAM_BATCH];
    uint32_t dropped;
    uint8_t count;
} robot_ir_stream_batch_t;

/* Always observes ADC2, independently of decoder enable/framing/movement.
 * Fixed-size queue only; no formatting, allocation or network in capture. */
void robot_ir_stream_record(const robot_barcode_observation_t *observation,
    uint16_t minimum, uint16_t maximum, bool inverted);
bool robot_ir_stream_peek(robot_ir_stream_batch_t *batch);
void robot_ir_stream_ack(const robot_ir_stream_batch_t *batch);
void robot_ir_stream_publish(uint32_t now_ms);
#endif
