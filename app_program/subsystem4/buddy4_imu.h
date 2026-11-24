/**
 * @file    buddy4_imu.h
 * @brief   Buddy 4: LSM303DLHC accelerometer + compass (IMU) readings.
 *
 * Calibration, filtering, terrain and motion events, estimated height.  See
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
#include <tk/tkernel.h>

#include "buddy4_processing.h"

/** @return true if both sensors answered on I2C. */
bool buddy4_init (void);

/**
 * @brief Read both sensors and run the processing.  IMU task only (I2C).
 * @param now_ms Current time in milliseconds.
 */
void buddy4_update (uint32_t now_ms);

/**
 * @brief Copy the latest IMU state; stale readings are marked not OK.
 * @param p_state Receives the state; NULL is ignored.
 */
void buddy4_get_state (buddy4_imu_state_t * p_state);

/** Queued to the IMU owner task; resets filters and run statistics.
 *  Supply measured calibration; defaults deliberately mark height invalid. */
bool buddy4_set_calibration (const buddy4_calibration_t * p_cal);
/** Queued to the IMU owner task: clears humps, peaks, lifts and filters
 *  for a new run, keeping the calibration.  Do it with the rover on flat
 *  ground: the level reference for humps is learned again afterwards. */
void buddy4_reset_run (void);
/** The rover is driving again (a new start): end a lift latch whose
 *  landing gave no jolt.  Clears the shared lifted flag at once. */
void buddy4_clear_lift (void);
/** The IMU owner task: 50 ms period, priority set in app_main.c. */
void buddy4_task (INT stacd, void * p_exinf);

#endif /* BUDDY4_IMU_H */

/*** end of file ***/

