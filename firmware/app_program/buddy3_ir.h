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

/*  TODO:
    1. Rename structs to appropriate name
    2. Split the struct into IR Calibration Params and Steering Config
    3. Split the struct again into 1 calibration param struct per IR sensor instead of for all 3
*/

// Contains IR sensor calibration parameters and line-following steering configurations.
typedef struct
{
    uint16_t minimum[3]; // Minimum raw readings of 3 IR sensors when NOT on track
    uint16_t maximum[3]; // Maximum of 3 IR sensors when on track
    bool     line_is_dark; // Option to invert readings if sensor reads higher value on dark instead of white
    float    proportional_gain; // Steering strength: Too high cause overcorrection, too low causes missing the curve
    float    derivative_gain; // CURRENTLY UNUSED - Multiplies the proportional gain
    int16_t  base_pwm; // Base power of the motor: 32767 = full duty
} buddy3_calibration_t; // TODO: Rename to something meaningful and refactor

// Snapshot of IR readings and interpreted line/barcode status.
// Sensor order: front left, front right, side barcode.
typedef struct
{
    uint16_t raw[3]; // Original ADC readings: 0-4095
    uint16_t normalized[3]; // Calibrated readings: 0-1000
    int16_t  line_position; // -1000 to +1000: negative = left, positive = right; zero at junctions
    bool     line_seen; // Either front sensor detects black; false can also mean centred between sensors
    bool     junction_seen; // Both front sensors meet the junction threshold; not proof of a physical junction
    bool     barcode_ready; // A decoded command is waiting to be consumed
    uint8_t  barcode_command; // 1 = left, 2 = right, 3 = straight; pending only when barcode_ready is true
} buddy3_ir_state_t;

void buddy3_init(void);
void buddy3_set_calibration(const buddy3_calibration_t *p_calibration);
void buddy3_get_calibration(buddy3_calibration_t *p_calibration);
void buddy3_update(uint32_t now_ms);
void buddy3_start_line_follow(void);
void buddy3_stop_line_follow(void);
bool buddy3_line_following(void);
void buddy3_get_state(buddy3_ir_state_t *p_state);
/** Clear the latched front-line observation used by Buddy 5's final return. */
void buddy3_clear_line_seen_event(void);
/** Return and clear whether either front sensor has seen black since the last clear. */
bool buddy3_take_line_seen_event(void);
bool buddy3_take_barcode(uint8_t *p_command);
/** true = that sensor sees black: front-left, front-right, side (barcode). */
void buddy3_get_colors(bool *p_left_black, bool *p_right_black, bool *p_side_black);

#endif /* BUDDY3_IR_H */
