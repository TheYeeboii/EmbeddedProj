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

/** Full power in the public API (same range as the old CMake project). */
#define BUDDY2_PWM_MAX              (32767)

/** How often buddy2_update() must be called. */
#define BUDDY2_CONTROL_PERIOD_MS    (20u)

typedef struct
{
    float    wheel_diameter_mm;
    float    track_width_mm;
    float    left_ticks_per_wheel_rev;
    float    right_ticks_per_wheel_rev;
    uint16_t pwm_limit;       /**< Max |PWM| accepted, in API units.  */
    uint16_t pwm_deadband;    /**< Min |PWM| that moves a wheel.      */
} buddy2_calibration_t;

typedef struct
{
    int32_t  left_ticks;
    int32_t  right_ticks;
    float    left_speed_mm_s;
    float    right_speed_mm_s;
    float    left_distance_mm;
    float    right_distance_mm;
    uint32_t timestamp_ms;
} buddy2_motion_state_t;

/**
 * @brief Set up PWM, encoder interrupts and the library mutex.
 * @note  gpio_irq_init() must have been called first.
 * @return 0 on success, a negative micro T-Kernel error code otherwise.
 */
int32_t buddy2_motion_init(void);

/** @brief Run one control step.  Call every BUDDY2_CONTROL_PERIOD_MS. */
void buddy2_update(uint32_t now_ms);

void buddy2_set_motor_pwm(int16_t left_pwm, int16_t right_pwm);
void buddy2_stop(void);
void buddy2_get_motion_state(buddy2_motion_state_t *p_state);
void buddy2_reset_encoders(void);

bool buddy2_move_forward(float distance_mm, int16_t max_pwm);
bool buddy2_move_backward(float distance_mm, int16_t max_pwm);
bool buddy2_turn_left(float angle_deg, int16_t max_pwm);
bool buddy2_turn_right(float angle_deg, int16_t max_pwm);
bool buddy2_motion_active(void);

void buddy2_set_calibration(const buddy2_calibration_t *p_calibration);
void buddy2_get_calibration(buddy2_calibration_t *p_calibration);

#endif /* BUDDY2_MOTION_H */
