/**
 * @file    buddy3_ir.h
 * @brief   Buddy 3: IR line following and barcode decoding.
 *
 * micro T-Kernel usage:
 *   - buddy3_init() once from usermain().
 *   - buddy3_update() every BUDDY3_BARCODE_PERIOD_MS from the line task.
 *   - The other functions may be called from any task.
 */

#ifndef BUDDY3_IR_H
#define BUDDY3_IR_H

#include <stdbool.h>
#include <stdint.h>
#include "robot_types.h"

/* Line following, junctions and the IR stream run every 10 ms. */
#define BUDDY3_SAMPLE_PERIOD_MS (10u)
/* The side (barcode) sensor is read five times as often: the reader needs
 * at least four samples per 3 mm narrow bar (0.75 mm of travel per sample),
 * which at 10 ms limited driving to about 75 mm/s.  At 2 ms it allows about
 * 375 mm/s, so a manual W drive can read barcodes.  Needs the kernel tick
 * (CNF_TIMER_PERIOD) at 1 or 2 ms. */
#define BUDDY3_BARCODE_PERIOD_MS (2u)

#include "buddy3_barcode.h"

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
    /* A reader event is waiting to be consumed. */
    bool barcode_ready;
    /* Compatibility field: ASCII letter on a valid read, otherwise 0. */
    uint8_t barcode_command;
} buddy3_ir_state_t;

/** @brief Set up the ADC pins, clear the state and configure the reader. */
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
 * @brief Feed the barcode reader from the side sensor and, every
 *        BUDDY3_SAMPLE_PERIOD_MS, read all sensors and (while line following
 *        is on) steer.  Call every BUDDY3_BARCODE_PERIOD_MS.
 * @param now_ms Current time in milliseconds.
 */
void buddy3_update (uint32_t now_ms);

/** @brief Start line following (resets steering, junction and reader). */
void buddy3_start_line_follow (void);

/** @brief Stop line following and the motors. */
void buddy3_stop_line_follow (void);

/** @brief Tell whether line following is on. */
bool buddy3_line_following (void);

/**
 * @brief Let the barcode reader run while the robot is driven by hand.
 *
 * Without line following the reader is normally suspended.  The controller
 * calls this with true while a manual drive goes straight forward, so a
 * barcode can be read by driving over it.  The same checks still apply:
 * forward travel, healthy encoders and a small heading change.  The change
 * takes effect at the IR task's next sample, which also restarts the reader,
 * so only the IR task ever resets it.
 *
 * @param b_enabled True while a manual forward drive is running.
 */
void buddy3_set_manual_barcode (bool b_enabled);

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

/**
 * @brief Take the side (barcode) sensor's present reading as white.  The IR
 *        task averages its next samples and makes them the sensor's white
 *        level, so the surface under it reads 0.  Ignored when that reading
 *        is too close to black to be paper.  Runs at power-up as well.
 */
void buddy3_zero_barcode_white (void);

/* Sensors buddy3_set_ir_levels() can set (dashboard numbering). */
#define BUDDY3_LEVELS_BARCODE (0u)
#define BUDDY3_LEVELS_LEFT    (1u)
#define BUDDY3_LEVELS_RIGHT   (2u)

/**
 * @brief Set one IR sensor's white and black raw levels by hand.  The IR
 *        task applies them at its next line step (the barcode sensor's not
 *        in the middle of reading a barcode).  For the barcode sensor this
 *        also replaces any pending white zero.
 * @param sensor BUDDY3_LEVELS_BARCODE, _LEFT or _RIGHT.
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

/* Turns a barcode letter can request at the next junction. */
#define BUDDY3_TURN_NONE  (0u)
#define BUDDY3_TURN_LEFT  (1u)
#define BUDDY3_TURN_RIGHT (2u)

/**
 * @brief Request a turn at the next junction while line following.  When the
 *        junction is confirmed, line following stops on it instead of
 *        crossing straight, and buddy3_take_junction_turn() reports the turn.
 *        A later call replaces the request; starting line following clears
 *        it.
 * @param turn BUDDY3_TURN_NONE, _LEFT or _RIGHT; other values are ignored.
 */
void buddy3_set_junction_turn (uint8_t turn);

/**
 * @brief Take the turn reached at a junction.
 * @param p_turn Receives BUDDY3_TURN_LEFT or BUDDY3_TURN_RIGHT.
 * @return True when line following stopped on a junction for a turn.
 */
bool buddy3_take_junction_turn (uint8_t * p_turn);

/**
 * @brief Take the oldest unread barcode event.
 * @param p_event Receives the event; its dropped field counts lost events.
 * @return True when an event was waiting.
 */
bool buddy3_take_barcode (buddy3_barcode_event_t * p_event);

/**
 * @brief Configure the barcode reader; only while stopped.  Rejects
 *        uncalibrated or low-resolution odometry.  The minimum element is the
 *        smallest resolvable stripe, not a fixed cell width; the reader
 *        infers the actual narrow/wide widths per frame.
 * @param min_element_mm Smallest stripe in mm; 0 disables the reader.
 * @param offset_left_mm Sideways sensor offset, left positive.
 * @return True when the reader is configured and enabled.
 */
bool buddy3_configure_barcode (uint16_t min_element_mm, int16_t offset_left_mm);

/** @brief Tell whether the barcode reader is configured. */
bool buddy3_barcode_calibrated (void);

/**
 * @brief Report which sensors see black.  All pointers must be non-NULL.
 * @param p_left_black  Front-left sensor.
 * @param p_right_black Front-right sensor.
 * @param p_side_black  Side (barcode) sensor.
 */
void buddy3_get_colors (bool * p_left_black, bool * p_right_black,
                        bool * p_side_black);

#endif /* BUDDY3_IR_H */

/*** end of file ***/

