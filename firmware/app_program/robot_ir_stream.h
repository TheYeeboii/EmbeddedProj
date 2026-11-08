/** @file robot_ir_stream.h
 *
 * @brief Fixed-size queue of continuous barcode-sensor (ADC2) samples, filled
 *        by the line task and drained by the MQTT publisher.
 */
#ifndef ROBOT_IR_STREAM_H
#define ROBOT_IR_STREAM_H

#include <stdbool.h>
#include <stdint.h>
#include "robot_barcode_trace.h"

#define ROBOT_IR_STREAM_CAPACITY  256u
#define ROBOT_IR_STREAM_BATCH     4u
#define ROBOT_IR_STREAM_PERIOD_MS 10u

/** One 10 ms observation of the barcode sensor. */
typedef struct
{
    uint32_t seq;
    uint32_t ts_ms;
    int32_t  left_x10;
    int32_t  right_x10;
    uint16_t raw;
    uint16_t normalized;
    uint16_t minimum;
    uint16_t maximum;
    uint8_t  black;
    uint8_t  blocked;
    uint8_t  inverted;
} robot_ir_stream_sample_t;

/** Oldest queued samples, copied out for one MQTT packet. */
typedef struct
{
    robot_ir_stream_sample_t rows[ROBOT_IR_STREAM_BATCH];
    uint32_t                 dropped;
    uint8_t                  count;
} robot_ir_stream_batch_t;

/**
 * @brief Queue one observation.  Always observes ADC2, independently of
 *        decoder enable, framing or movement.  Fixed-size queue only; no
 *        formatting, allocation or network work happens here.
 * @param p_observation Decoder observation to copy.
 * @param minimum       Current calibration minimum.
 * @param maximum       Current calibration maximum.
 * @param inverted      True when the calibration is inverted.
 */
void robot_ir_stream_record (const robot_barcode_observation_t * p_observation,
                             uint16_t minimum, uint16_t maximum, bool inverted);

/**
 * @brief Copy the oldest samples without removing them.
 * @param p_batch Receives up to ROBOT_IR_STREAM_BATCH rows.
 * @return True when at least one row was copied.
 */
bool robot_ir_stream_peek (robot_ir_stream_batch_t * p_batch);

/**
 * @brief Remove rows that were published.  Rows already overwritten by an
 *        overflow are skipped, so a stale batch never removes newer rows.
 * @param p_batch Batch returned by robot_ir_stream_peek().
 */
void robot_ir_stream_ack (const robot_ir_stream_batch_t * p_batch);

#endif /* ROBOT_IR_STREAM_H */

/*** end of file ***/

