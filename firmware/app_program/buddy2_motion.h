/**
 * @file    buddy2_motion.h
 * @brief   Buddy 2 motion-control library (motors, encoders, PID, moves).
 *
 * micro T-Kernel version.  How it is used:
 *
 *   1. usermain() calls buddy2_motion_init() once.
 *   2. A periodic motion task calls buddy2_update() every
 *      BUDDY2_CONTROL_PERIOD_MS (see app_main.c).
 *   3. Any other task may call the move / turn / stop / get-state
 *      functions below.  They are protected by a kernel mutex, so they are
 *      safe to call from any task (but NOT from an interrupt handler).
 *
 * PWM values in this API are signed, -BUDDY2_PWM_MAX .. +BUDDY2_PWM_MAX,
 * where +BUDDY2_PWM_MAX means full power forwards.  The library scales
 * them to the real PWM counter internally.
 */

#ifndef BUDDY2_MOTION_H
#define BUDDY2_MOTION_H

#include <stdbool.h>
#include <stdint.h>
#include "robot_types.h"

/** Full power in the public API (same range as the old CMake project). */
#define BUDDY2_PWM_MAX (32767)

/** Motor trim, percent of the commanded PWM.  Lower the FASTER motor until
 *  the robot drives straight.  Change at run time with buddy2_set_trim();
 *  copy the tuned values here so they survive a reboot. */
#define BUDDY2_TRIM_DEFAULT_LEFT  (83u) /* tuned: left motor is faster */
#define BUDDY2_TRIM_DEFAULT_RIGHT (100u)
#define BUDDY2_TRIM_MIN           (50u)

/** Global speed limit, percent.  Scales motor commands from arrows, raw
 *  drive, line following, distance/turn moves and barcode moves.  The
 *  *_at_speed() calls use their own percentage instead (obstacle avoidance
 *  uses them so the limit cannot starve its moves).
 *  Change at run time with buddy2_set_speed_limit(). */
#define BUDDY2_SPEED_DEFAULT (60u)
#define BUDDY2_SPEED_MIN     (20u)

/** How often buddy2_update() must be called. */
#define BUDDY2_CONTROL_PERIOD_MS (20u)

typedef struct
{
    float32_t wheel_diameter_mm;
    float32_t track_width_mm;
    float32_t left_ticks_per_wheel_rev;
    float32_t right_ticks_per_wheel_rev;
    uint16_t  pwm_limit;    /**< Max |PWM| accepted, in API units.  */
    uint16_t  pwm_deadband; /**< Min |PWM| that moves a wheel.      */
} buddy2_calibration_t;

typedef struct
{
    int32_t   left_ticks;
    int32_t   right_ticks;
    float32_t left_speed_mm_s;
    float32_t right_speed_mm_s;
    float32_t left_distance_mm;
    float32_t right_distance_mm;
    uint32_t  timestamp_ms;
    /** true = this wheel was powered for BUDDY2_ENCODER_FAULT_MS but its
     *  encoder gave no ticks (encoder/wiring fault, or the wheel is
     *  blocked).  Clears by itself as soon as ticks arrive again. */
    bool left_encoder_fault;
    bool right_encoder_fault;
} buddy2_motion_state_t;

/**
 * @brief Set up PWM, encoder interrupts and the library mutex.
 * @note  gpio_irq_init() must have been called first.
 * @return 0 on success, a negative micro T-Kernel error code otherwise.
 */
int32_t buddy2_motion_init (void);

/** @brief Run one control step.  Call every BUDDY2_CONTROL_PERIOD_MS. */
void buddy2_update (uint32_t now_ms);

struct robot_steering_input;

/**
 * @brief Drive both motors open-loop and record the front-IR decision
 *        snapshot that led to it.  Same motor behaviour as
 *        buddy2_set_motor_pwm().
 * @param left_pwm  Left command, -BUDDY2_PWM_MAX..BUDDY2_PWM_MAX.
 * @param right_pwm Right command, -BUDDY2_PWM_MAX..BUDDY2_PWM_MAX.
 * @param p_input   Decision evidence, or NULL.
 */
void buddy2_set_motor_pwm_observed (
    int16_t left_pwm, int16_t right_pwm,
    const struct robot_steering_input * p_input);

/**
 * @brief Stop both motors and record the decision evidence.
 * @param p_input Decision evidence, or NULL.
 */
void buddy2_stop_observed (const struct robot_steering_input * p_input);

/** @brief Drive both motors open-loop (trim and speed limit applied). */
void buddy2_set_motor_pwm (int16_t left_pwm, int16_t right_pwm);

/**
 * @brief Drive both motors open-loop at a fixed speed percentage instead of
 *        the global speed limit (trim still applied).
 * @param speed_pct Speed in percent, BUDDY2_SPEED_MIN..100.
 * @return False (motors unchanged) if not initialised or speed_pct is out
 *         of range.
 */
bool buddy2_set_motor_pwm_at_speed (int16_t left_pwm, int16_t right_pwm,
                                    uint8_t speed_pct);

/** @brief Stop both motors and cancel any distance/turn move. */
void buddy2_stop (void);

/** @brief Copy the latest odometry and encoder-fault state. */
void buddy2_get_motion_state (buddy2_motion_state_t * p_state);

/** Odometry including ticks since the last control update. */
typedef struct
{
    float32_t left_distance_mm;
    float32_t right_distance_mm;
    bool      left_encoder_fault;
    bool      right_encoder_fault;
} buddy2_encoder_position_t;

/**
 * @brief Task-only, non-mutating position snapshot including ticks since the
 *        last 20 ms control update.  Fault flags remain the last
 *        control-loop verdict.
 * @param p_position Receives the snapshot; NULL is ignored.
 */
void buddy2_get_encoder_position (buddy2_encoder_position_t * p_position);

/** @brief Zero the odometry and clear latched encoder faults. */
void buddy2_reset_encoders (void);

/** @brief Start a closed-loop straight move forward.
 *  @return False if not initialised, max_pwm <= 0 or distance is 0. */
bool buddy2_move_forward (float32_t distance_mm, int16_t max_pwm);

/** @brief Start a closed-loop straight move backward.
 *  @return False if not initialised, max_pwm <= 0 or distance is 0. */
bool buddy2_move_backward (float32_t distance_mm, int16_t max_pwm);

/** @brief buddy2_move_backward() at a fixed speed percentage
 *         (BUDDY2_SPEED_MIN..100) instead of the global speed limit.
 *  @return False if not initialised, max_pwm <= 0, distance is 0 or
 *          speed_pct is out of range. */
bool buddy2_move_backward_at_speed (float32_t distance_mm, int16_t max_pwm,
                                    uint8_t speed_pct);

/** @brief Start an in-place left (anticlockwise) turn.
 *  @return False if not initialised, max_pwm <= 0 or angle is 0. */
bool buddy2_turn_left (float32_t angle_deg, int16_t max_pwm);

/** @brief Start an in-place right (clockwise) turn.
 *  @return False if not initialised, max_pwm <= 0 or angle is 0. */
bool buddy2_turn_right (float32_t angle_deg, int16_t max_pwm);

/** @brief Tell whether a distance/turn move is still running. */
bool buddy2_motion_active (void);

/**
 * @brief Scale open-loop PWM per motor (raw drive, arrow pad, line following).
 * @return false if either value is outside BUDDY2_TRIM_MIN..100.
 */
bool buddy2_set_trim (uint8_t left_pct, uint8_t right_pct);

/** @brief Read the open-loop trim; both pointers must be non-NULL. */
void buddy2_get_trim (uint8_t * p_left_pct, uint8_t * p_right_pct);

/**
 * @brief Global speed limit for all motion (percent of the commanded PWM).
 * @return false if the value is outside BUDDY2_SPEED_MIN..100.
 */
bool buddy2_set_speed_limit (uint8_t percent);

/** @brief Read the global speed limit in percent. */
uint8_t buddy2_get_speed_limit (void);

/** @brief Replace the geometry; invalid values are ignored as a whole. */
void buddy2_set_calibration (const buddy2_calibration_t * p_calibration);

/** @brief Copy the geometry in use. */
void buddy2_get_calibration (buddy2_calibration_t * p_calibration);

#endif /* BUDDY2_MOTION_H */

/*** end of file ***/

