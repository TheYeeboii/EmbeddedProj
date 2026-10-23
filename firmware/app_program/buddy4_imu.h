/**
 * @file    buddy4_imu.h
 * @brief   Buddy 4: LSM303DLHC accelerometer + compass (IMU) readings.
 *
 * Calibration, filtering, terrain events and estimated height. See
 * docs/BUDDY4_IMU.md for configuration and physical validation.
 *
 * micro T-Kernel usage:
 *   - buddy4_init() once from usermain().
 *   - buddy4_update() only from buddy4_task (it does I2C).
 *   - buddy4_get_state() from any task.
 */

#ifndef BUDDY4_IMU_H
#define BUDDY4_IMU_H

#include <stdbool.h>
#include <stdint.h>

#include "buddy4_processing.h"

/** @return true if both sensors answered on I2C. */
bool buddy4_init(void);
void buddy4_update(uint32_t now_ms);
void buddy4_get_state(buddy4_imu_state_t *p_state);
void buddy4_reset_max_pitch(void);
/* Queued to the IMU owner task; resets filters and run statistics.
 * Supply measured calibration; defaults deliberately mark height invalid. */
bool buddy4_set_calibration(const buddy4_calibration_t *cal);
void buddy4_task(int32_t stacd, void *exinf);

#endif /* BUDDY4_IMU_H */
