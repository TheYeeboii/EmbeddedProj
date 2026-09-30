/**
 * @file    buddy4_imu.h
 * @brief   Buddy 4: LSM303DLHC accelerometer + compass (IMU) readings.
 *
 * Ported from the team's compass test code, plus accelerometer reads and
 * tilt angles as the starting point for hump detection.
 *
 * micro T-Kernel usage:
 *   - buddy4_init() once from usermain().
 *   - buddy4_update() periodically from the sensor task (it does I2C).
 *   - buddy4_get_state() from any task.
 */

#ifndef BUDDY4_IMU_H
#define BUDDY4_IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool     mag_ok;              /**< Last compass read succeeded.       */
    bool     accel_ok;            /**< Last accelerometer read succeeded. */
    int16_t  mag_raw[3];          /**< X, Y, Z raw compass counts.        */
    int16_t  accel_mg[3];         /**< X, Y, Z acceleration in milli-g.   */
    int16_t  heading_deg_x10;     /**< Compass heading, 0..3599 (0.1 deg). */
    int16_t  pitch_deg_x10;       /**< Nose-up is positive (0.1 deg).      */
    int16_t  roll_deg_x10;        /**< (0.1 deg).                          */
    int16_t  max_pitch_deg_x10;   /**< Highest pitch since reset - a first
                                       step towards "highest hump".        */
    uint32_t timestamp_ms;
} buddy4_imu_state_t;

/** @return true if both sensors answered on I2C. */
bool buddy4_init(void);
void buddy4_update(uint32_t now_ms);
void buddy4_get_state(buddy4_imu_state_t *p_state);
void buddy4_reset_max_pitch(void);

#endif /* BUDDY4_IMU_H */
