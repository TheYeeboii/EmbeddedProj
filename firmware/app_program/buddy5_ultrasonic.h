/**
 * @file    buddy5_ultrasonic.h
 * @brief   Buddy 5: ultrasonic distance sensor on a scanning servo.
 *
 * Ported from the team's test code.  The echo is now timed with a GPIO
 * interrupt and the calling task sleeps while waiting, instead of
 * busy-waiting up to 60 ms like the old superloop did.
 *
 * micro T-Kernel usage:
 *   - buddy5_init() once from usermain().
 *   - buddy5_measure_cm() from ONE task at a time (the sensor task).
 *   - buddy5_get_last() from any task.
 */

#ifndef BUDDY5_ULTRASONIC_H
#define BUDDY5_ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

#define BUDDY5_NO_ECHO  (0u)   /* returned when nothing answered in time */

typedef struct
{
    uint32_t distance_cm;      /**< Last reading, BUDDY5_NO_ECHO if none. */
    uint8_t  servo_angle_deg;  /**< Servo angle for that reading.         */
    uint32_t timestamp_ms;
} buddy5_reading_t;

void     buddy5_init(void);
void     buddy5_set_servo_angle(uint8_t angle_deg);
/** Trigger one ping and wait (sleeping) for the echo.  Blocks <= ~40 ms. */
uint32_t buddy5_measure_cm(uint32_t now_ms);
void     buddy5_get_last(buddy5_reading_t *p_reading);

#endif /* BUDDY5_ULTRASONIC_H */
