/**
 * @file    buddy5_ultrasonic.h
 * @brief   Buddy 5: ultrasonic distance sensor on a scanning servo.
 *
 * Ported from the team's test code. The echo is read directly from the
 * configured Echo pin with a bounded timeout, keeping ultrasonic bring-up
 * independent of the RTOS GPIO interrupt dispatcher.
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

typedef enum
{
    BUDDY5_PING_VALID = 0,
    BUDDY5_PING_TIMEOUT_RISE,
    BUDDY5_PING_TIMEOUT_FALL,
    BUDDY5_PING_HIGH_BEFORE_TRIGGER
} buddy5_ping_result_t;

/**
 * @brief Initialise the ultrasonic Trigger/Echo GPIO pins and scanning servo.
 *
 * The function resets the saved telemetry reading, configures Trigger as a
 * low output and Echo as an input, then centres the servo at 90 degrees.
 * Call once during application startup before any measurement.
 */
void buddy5_init(void);

/**
 * @brief Point the ultrasonic scanning servo at an absolute angle.
 * @param angle_deg Requested angle in the range 0..180 degrees.
 *
 * Values above 180 are safely clamped.  This changes only the servo position;
 * it does not take a distance measurement or change telemetry.
 */
void buddy5_set_servo_angle(uint8_t angle_deg);

/**
 * @brief Take a forward/telemetry ultrasonic measurement.
 * @param now_ms Current application time in milliseconds.
 * @return Distance in centimetres, or BUDDY5_NO_ECHO on timeout.
 *
 * This blocks for at most about 40 ms.  It updates the saved reading used by
 * telemetry with the measured distance, current servo angle and timestamp.
 */
uint32_t buddy5_measure_cm(uint32_t now_ms);

/**
 * @brief Take a temporary left/right scan measurement.
 * @return Distance in centimetres, or BUDDY5_NO_ECHO on timeout.
 *
 * This blocks for at most about 40 ms but deliberately does NOT update the
 * telemetry reading.  The avoidance task uses the returned value only for
 * left/right and opposite-side decisions.
 */
uint32_t buddy5_sample_cm(void);

/** Take one non-telemetry side-scan ping and identify a failure, if any. */
uint32_t buddy5_sample_with_result_cm(buddy5_ping_result_t *p_result);

/**
 * @brief Copy the latest forward/telemetry reading.
 * @param[out] p_reading Destination structure; NULL is ignored safely.
 *
 * The copy is protected from an interrupt-time update.  Side-scan samples do
 * not change this value, so telemetry remains the front-facing distance.
 */
void     buddy5_get_last(buddy5_reading_t *p_reading);

#endif /* BUDDY5_ULTRASONIC_H */
