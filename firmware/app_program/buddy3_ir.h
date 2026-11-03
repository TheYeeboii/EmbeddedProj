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
*/

/* Compatibility interface for existing callers; internal settings are split below. */
typedef struct
{
    uint16_t minimum[3]; // Raw endpoints mapped to 0 before inversion
    uint16_t maximum[3]; // Raw endpoints mapped to 1000 before inversion
    bool     line_is_dark; // Invert when black produces lower raw readings than floor
    float    proportional_gain; // Steering strength: Too high cause overcorrection, too low causes missing the curve
    float    derivative_gain; // Multiplies position change; unused in current straddle mode
    int16_t  base_pwm; // Base power of the motor: 32767 = full duty
} buddy3_calibration_t; // TODO: Rename to something meaningful and refactor

/* Calibration parameters for one IR sensor. */
typedef struct
{
    uint16_t minimum; // Raw endpoint mapped to 0 before inversion
    uint16_t maximum; // Raw endpoint mapped to 1000 before inversion
    bool     line_is_dark; // Invert this sensor's reading when black produces lower values
} ir_calibration_params_t;

typedef struct
{
    float   proportional_gain; // Steering strength: Too high cause overcorrection, too low causes missing the curve
    float   derivative_gain; // Multiplies position change; unused in current straddle mode
    int16_t base_pwm; // Base power of the motor: 32767 = full duty
} line_follow_configs_t;

/* Raw and normalized values from one IR sensor. */
typedef struct
{
    uint16_t raw; // Original ADC reading: 0-4095
    uint16_t normalized; // Calibrated reading: 0-1000
} ir_sensor_reading_t;

// Compatibility snapshot of IR readings and interpreted line/barcode status.
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

// Calibration struct utility
void buddy3_set_calibration(const buddy3_calibration_t *p_calibration);
void buddy3_get_calibration(buddy3_calibration_t *p_calibration);
void buddy3_update(uint32_t now_ms);

// Line following logic
void buddy3_start_line_follow(void);
void buddy3_stop_line_follow(void);
bool buddy3_line_following(void);
/* 0 = original integration, 1 = micro, 2 = corner, 3 = bounded recovery.
 * Change only while stopped. Stages 1..3 are limited to 15-second runs. */
void buddy3_set_test_stage(uint8_t stage);
uint8_t buddy3_test_stage(void);

// IR struct utility
void buddy3_get_state(buddy3_ir_state_t *p_state);
/** Print a numerical sensor snapshot when BUDDY3_DEBUG=1; call from telemetry. */
void buddy3_print_debug(void);

/** Clear the latched front-line observation used by Buddy 5's final return. */
void buddy3_clear_line_seen_event(void);
/** Return and clear whether either front sensor has seen black since the last clear. */
bool buddy3_take_line_seen_event(void);
bool buddy3_take_barcode(uint8_t *p_command);
/** true = that sensor sees black: front-left, front-right, side (barcode). */
void buddy3_get_colors(bool *p_left_black, bool *p_right_black, bool *p_side_black);

#endif /* BUDDY3_IR_H */
