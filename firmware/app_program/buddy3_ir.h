/**
 * @file    buddy3_ir.h
 * @brief   Buddy 3: IR line following and barcode decoding.
 *
 * micro T-Kernel usage:
 *   - buddy3_init() once from usermain().
 *   - buddy3_update() every BUDDY3_SAMPLE_PERIOD_MS from the line task.
 *   - The other functions may be called from any task.
 */

#ifndef BUDDY3_IR_H
#define BUDDY3_IR_H

#include <stdbool.h>
#include <stdint.h>

#define BUDDY3_SAMPLE_PERIOD_MS  (10u)

/* Barcode commands (see docs/BUDDY3_LINE_AND_BARCODE.md). */
#define BUDDY3_BARCODE_LEFT      (1u)
#define BUDDY3_BARCODE_RIGHT     (2u)
#define BUDDY3_BARCODE_STRAIGHT  (3u)

typedef struct
{
    uint16_t minimum[3];
    uint16_t maximum[3];
    bool     line_is_dark;
    float    proportional_gain;
    float    derivative_gain;
    int16_t  base_pwm;
} buddy3_calibration_t;

typedef struct
{
    uint16_t raw[3];
    uint16_t normalized[3];
    int16_t  line_position;
    bool     line_seen;
    bool     junction_seen;
    bool     barcode_ready;
    uint8_t  barcode_command;
} buddy3_ir_state_t;

void buddy3_init(void);
void buddy3_set_calibration(const buddy3_calibration_t *p_calibration);
void buddy3_get_calibration(buddy3_calibration_t *p_calibration);
void buddy3_update(uint32_t now_ms);
void buddy3_start_line_follow(void);
void buddy3_stop_line_follow(void);
bool buddy3_line_following(void);
void buddy3_get_state(buddy3_ir_state_t *p_state);
bool buddy3_take_barcode(uint8_t *p_command);
/** true = that sensor sees black: front-left, front-right, side (barcode). */
void buddy3_get_colors(bool *p_left_black, bool *p_right_black, bool *p_side_black);

#endif /* BUDDY3_IR_H */
