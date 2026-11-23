/**
 * @file    buddy3_ir.h
 * @brief   Buddy 3: IR line following and barcode decoding.
 *
 * micro T-Kernel usage:
 *   - buddy3_init() once from usermain().
 *   - buddy3_update() from the line task at the selected sensor cadence.
 *   - The other functions may be called from any task.
 */

#ifndef BUDDY3_IR_H
#define BUDDY3_IR_H

#include <stdbool.h>
#include <stdint.h>
#include "robot_types.h"

/* Front sensors, line following and IR diagnostics share this interval. */
#define BUDDY3_SAMPLE_PERIOD_MS (10u)

/* TODO: Give the structures below names that start with buddy3_. */

/** Snapshot of the sensor calibration and steering settings
 *  (buddy3_get_calibration()); the module keeps them split into
 *  ir_calibration_params_t and line_follow_configs_t. */
typedef struct
{
    /* Raw endpoints mapped to 0 and 1000 before inversion. */
    uint16_t minimum[3];
    uint16_t maximum[3];
    /* Invert when black produces lower raw readings than the floor. */
    bool line_is_dark;
    /* Steering strength: too high overcorrects, too low misses curves. */
    float32_t proportional_gain;
    /* Multiplies the position change; unused in the straddle mode. */
    float32_t derivative_gain;
    /* Base motor power: 32767 = full duty. */
    int16_t base_pwm;
} buddy3_calibration_t;

/** Calibration parameters for one IR sensor. */
typedef struct
{
    uint16_t minimum;      /* Raw endpoint mapped to 0 before inversion. */
    uint16_t maximum;      /* Raw endpoint mapped to 1000 before inversion. */
    bool     line_is_dark; /* Invert when black reads lower than the floor. */
} ir_calibration_params_t;

/** Line-following steering settings. */
typedef struct
{
    float32_t proportional_gain; /* Steering strength. */
    float32_t derivative_gain;   /* Unused in the straddle mode. */
    int16_t   base_pwm;          /* Base motor power: 32767 = full duty. */
} line_follow_configs_t;

/** Raw and normalized values from one IR sensor. */
typedef struct
{
    uint16_t raw;        /* Original ADC reading: 0-4095. */
    uint16_t normalized; /* Calibrated reading: 0-1000. */
} ir_sensor_reading_t;

/** Compatibility snapshot of IR readings and the interpreted line/barcode
 *  status.  Arrays are in ADC order; index 2 is the side barcode sensor. */
typedef struct
{
    uint16_t raw[3];        /* Original ADC readings: 0-4095. */
    uint16_t normalized[3]; /* Calibrated readings: 0-1000. */
    /* -1000 to +1000: negative = left, positive = right; 0 at junctions. */
    int16_t line_position;
    /* Either front sensor sees black; false can also mean centred. */
    bool line_seen;
    /* Both front sensors pass the junction threshold (not proof of one). */
    bool junction_seen;
} buddy3_ir_state_t;

/** @brief Set up the front ADC pins and clear the line sensor state. */
void buddy3_init (void);

/**
 * @brief Set the line-following speed.  The sensor calibration is not
 *        changed.
 * @param base_pwm          Straight-ahead power, 0..BUDDY2_PWM_MAX.
 * @param proportional_gain Steering gain; times 1000 it is the extra power
 *                          (PWM) on the outer wheel in a curve.
 */
void buddy3_set_line_speed (int16_t base_pwm, float32_t proportional_gain);

/**
 * @brief Copy the sensor calibration and steering settings.
 * @param p_calibration Receives the settings; NULL is ignored.
 */
void buddy3_get_calibration (buddy3_calibration_t * p_calibration);

/**
 * @brief Read the sensors and update steering every BUDDY3_SAMPLE_PERIOD_MS.
 * @param now_ms Current time in milliseconds.
 */
void buddy3_update (uint32_t now_ms);

/** @brief Start line following and reset steering and junction state. */
void buddy3_start_line_follow (void);

/** @brief Stop line following and the motors. */
void buddy3_stop_line_follow (void);

/** @brief Tell whether line following is on. */
bool buddy3_line_following (void);

/**
 * @brief Select the steering stage: 0 = original integration, 1 = micro,
 *        2 = corner, 3 = bounded recovery.  Ignored while running; runs
 *        continue until stopped or a fault occurs.
 * @param stage 0..3.
 */
void buddy3_set_test_stage (uint8_t stage);

/** @brief Read the selected steering stage (0..3). */
uint8_t buddy3_test_stage (void);

/**
 * @brief Copy the latest IR snapshot.
 * @param p_state Receives the snapshot; NULL is ignored.
 */
void buddy3_get_state (buddy3_ir_state_t * p_state);

/** @brief Print a numerical sensor snapshot when BUDDY3_DEBUG=1; call from
 *         the telemetry task. */
void buddy3_print_debug (void);

/** @brief Clear the latched front-line observation used by Buddy 5's final
 *         return to the line. */
void buddy3_clear_line_seen_event (void);

/**
 * @brief Return and clear the latched front-line observation.
 * @return True if either front sensor has seen black since the last clear.
 */
bool buddy3_take_line_seen_event (void);

/* Sensors buddy3_set_ir_levels() can set (dashboard numbering). */
#define BUDDY3_LEVELS_LEFT    (1u)
#define BUDDY3_LEVELS_RIGHT   (2u)

/**
 * @brief Set one front sensor's white and black raw levels at the next line step.
 * @param sensor BUDDY3_LEVELS_LEFT or BUDDY3_LEVELS_RIGHT.
 * @param white  Raw reading of white paper, 0..4095.
 * @param black  Raw reading of black, up to 4095 and at least
 *               BUDDY3_LEVEL_MIN_SPAN above white.
 * @return True when the levels were accepted.
 */
bool buddy3_set_ir_levels (uint8_t sensor, uint16_t white, uint16_t black);

/* Least raw difference between the white and black levels. */
#define BUDDY3_LEVEL_MIN_SPAN (200u)
/* Largest 12-bit ADC reading. */
#define BUDDY3_RAW_MAX (4095u)

/**
 * @brief Report which sensors see black.  A NULL pointer skips that sensor.
 * @param p_left_black  Front-left sensor.
 * @param p_right_black Front-right sensor.
 * @param p_side_black  Side (barcode) sensor.
 */
void buddy3_get_colors (bool * p_left_black, bool * p_right_black,
                        bool * p_side_black);

#endif /* BUDDY3_IR_H */

/*** end of file ***/

