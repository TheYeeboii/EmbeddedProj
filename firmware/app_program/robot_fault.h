/** @file robot_fault.h
 *
 * @brief Buddy 1 (shared): one fault byte for the whole robot.
 *
 * Change Requirement 1: fault conditions are individual bits in one shared
 * mask, not separate variables.  Several tasks set and clear bits in the same
 * byte, so a plain "mask |= bit" could lose another task's change (read,
 * modify, write is three steps, and the Cortex-M0+ has no atomic bit set on
 * RAM).  Every change therefore goes through robot_fault_set() or
 * robot_fault_clear(), which do the read-modify-write with interrupts
 * disabled.  Reading the byte is a single aligned load, so it needs no lock.
 *
 * Each bit has one owner that sets and clears it; the controller only reads
 * the mask and decides the reaction.
 */
#ifndef ROBOT_FAULT_H
#define ROBOT_FAULT_H

#include <stdint.h>

/* Fault bits.  Owner and condition in docs/MQTT_BINARY_FORMAT.md. */
#define ROBOT_FAULT_WIFI_LOSS   (0x01u) /**< Buddy 1, radio task. */
#define ROBOT_FAULT_MQTT_LOSS   (0x02u) /**< Buddy 1, radio task. */
#define ROBOT_FAULT_IMU         (0x04u) /**< Buddy 4, sensor task. */
#define ROBOT_FAULT_NO_ECHO     (0x08u) /**< Buddy 5, ultrasonic task. */
#define ROBOT_FAULT_ENCODER     (0x10u) /**< Buddy 2, motion task. */
#define ROBOT_FAULT_MOTOR_STALL (0x20u) /**< Buddy 2, motion task. */
/** Every defined bit; bits 6 and 7 are spare. */
#define ROBOT_FAULT_ALL (0x3Fu)

/**
 * @brief Set fault bits, safely against other tasks and interrupts.
 * @param mask ROBOT_FAULT_* bits to set; undefined bits are ignored.
 */
void robot_fault_set (uint8_t mask);

/**
 * @brief Clear fault bits, safely against other tasks and interrupts.
 * @param mask ROBOT_FAULT_* bits to clear; undefined bits are ignored.
 */
void robot_fault_clear (uint8_t mask);

/**
 * @brief Read the whole fault byte.
 * @return The current ROBOT_FAULT_* bits.
 */
uint8_t robot_fault_get (void);

#endif /* ROBOT_FAULT_H */

/*** end of file ***/

