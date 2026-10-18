/**
 * @file    buddy5_ultrasonic.c
 * @brief   Buddy 5: ultrasonic distance sensor on a scanning servo.
 *
 * How a measurement works:
 *   1. The task sends a 10 us trigger pulse.
 *   2. It waits, with a bounded timeout, for the echo pin to rise.
 *   3. It times the high pulse until the echo pin falls.
 *   4. distance_cm = echo_time_us / 58 (team's original formula).
 * If no echo arrives within BUDDY5_ECHO_TIMEOUT_US the reading is 0.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy5_ultrasonic.h"
#include "pin_config.h"
#include "robot_hal.h"

#define BUDDY5_TRIGGER_PULSE_US     (10u)
#define BUDDY5_ECHO_TIMEOUT_US      (40000u) /* ~6.8 m round trip */
#define BUDDY5_ECHO_IDLE_TIMEOUT_US (2000u)  /* Echo must be low before a ping */
#define BUDDY5_US_PER_CM            (58u)

/* Servo: 125 MHz / 64 / (39062 + 1) = 50 Hz.  One count = 0.512 us.
 * SG90: 0.5 ms = 0 deg, 2.5 ms = 180 deg -> 977 .. 4883 counts.
 * (The old code used 1000..3000 = 0.51..1.54 ms, which only reached
 * about 95 deg.) */
#define BUDDY5_SERVO_CLKDIV         (64u)
#define BUDDY5_SERVO_WRAP           (39062u)
#define BUDDY5_SERVO_MIN_COUNTS     (977u)    /* 0.5 ms */
#define BUDDY5_SERVO_SPAN_COUNTS    (3906u)   /* +2.0 ms for 180 deg */
#define BUDDY5_SERVO_MAX_DEG        (180u)

static buddy5_reading_t  g_last;
static uint8_t           g_servo_angle   = 90u;

/**
 * @brief Confirm that Echo is low before issuing a new trigger pulse.
 *
 * A normal HC-SR04+ leaves Echo low between measurements.  If GP16 is still
 * high, the previous pulse is incomplete or the line/sensor is faulty.  Do
 * not send another trigger into that condition: reset GP16 to input mode,
 * report no echo, and allow the next scheduled measurement to retry.
 */
static bool buddy5_wait_for_echo_idle(void)
{
    uint64_t wait_started_us;

    if (!hal_gpio_read(ULTRASONIC_ECHO_PIN))
    {
        return true;
    }

    wait_started_us = hal_time_us();
    while (hal_gpio_read(ULTRASONIC_ECHO_PIN))
    {
        if ((hal_time_us() - wait_started_us) >= BUDDY5_ECHO_IDLE_TIMEOUT_US)
        {
            /* Re-apply the intended GPIO direction in case another module or
             * a previous peripheral setup changed it.  This cannot pull down
             * a sensor that is electrically driving Echo high. */
            hal_gpio_input(ULTRASONIC_ECHO_PIN);
            tm_printf((UB *)"[sonar] Echo GP%u HIGH before trigger; skipped ping\n",
                      (UINT)ULTRASONIC_ECHO_PIN);
            return false;
        }
    }
    return true;
}

/**
 * @brief Configure the HC-SR04+ GPIO pins, PWM servo output and initial state.
 *
 * Trigger starts low to avoid an accidental ping.  The saved distance starts
 * as BUDDY5_NO_ECHO until the first successful forward measurement.
 */
void buddy5_init(void)
{
    buddy5_reading_t empty = { BUDDY5_NO_ECHO, 90u, 0u };

    g_last = empty;

    hal_gpio_output(ULTRASONIC_TRIG_PIN, false);
    hal_gpio_input(ULTRASONIC_ECHO_PIN);
    tm_printf((UB *)"[sonar] configured: TRIG=GP%u ECHO=GP%u timeout=%u us\n",
              (UINT)ULTRASONIC_TRIG_PIN, (UINT)ULTRASONIC_ECHO_PIN,
              (UINT)BUDDY5_ECHO_TIMEOUT_US);

    hal_pwm_pin_init_div(SERVO_PWM_PIN, BUDDY5_SERVO_WRAP, BUDDY5_SERVO_CLKDIV);
    buddy5_set_servo_angle(90u);
}

/**
 * @brief Convert a requested 0..180 degree angle into an SG90 PWM pulse.
 *
 * The PWM pulse is calculated from the calibrated 0.5..2.5 ms range and the
 * requested angle is retained so a later telemetry reading records the angle
 * at which it was taken.
 */
void buddy5_set_servo_angle(uint8_t angle_deg)
{
    uint32_t counts;

    if (angle_deg > BUDDY5_SERVO_MAX_DEG)
    {
        angle_deg = BUDDY5_SERVO_MAX_DEG;
    }
    counts = BUDDY5_SERVO_MIN_COUNTS +
             (((uint32_t)angle_deg * BUDDY5_SERVO_SPAN_COUNTS) / BUDDY5_SERVO_MAX_DEG);
    hal_pwm_set_level(SERVO_PWM_PIN, counts);
    g_servo_angle = angle_deg;
}

/**
 * @brief Send one ultrasonic trigger pulse and calculate the echo distance.
 * @param now_ms Timestamp stored only when @p store_for_telemetry is true.
 * @param store_for_telemetry True for the forward reading; false for a
 * temporary avoidance scan.
 * @return Distance in centimetres, or BUDDY5_NO_ECHO for either timeout.
 *
 * Echo must first be low, then its rising edge is awaited and its high-pulse
 * duration is measured.  The bounded waits prevent the sensor task from
 * waiting forever on a disconnected or faulty sensor.
 */
static uint32_t buddy5_ping_cm(uint32_t now_ms, bool store_for_telemetry,
                               buddy5_ping_result_t *p_result)
{
    uint32_t distance_cm = BUDDY5_NO_ECHO;
    uint32_t pulse_us;
    uint64_t wait_started_us;
    uint64_t echo_started_us;
    uint64_t echo_finished_us;
    buddy5_ping_result_t result = BUDDY5_PING_TIMEOUT_RISE;

    if (!buddy5_wait_for_echo_idle())
    {
        result = BUDDY5_PING_HIGH_BEFORE_TRIGGER;
        goto store_reading;
    }

    hal_gpio_write(ULTRASONIC_TRIG_PIN, true);
    hal_delay_us(BUDDY5_TRIGGER_PULSE_US);
    hal_gpio_write(ULTRASONIC_TRIG_PIN, false);

    wait_started_us = hal_time_us();
    while (!hal_gpio_read(ULTRASONIC_ECHO_PIN))
    {
        if ((hal_time_us() - wait_started_us) >= BUDDY5_ECHO_TIMEOUT_US)
        {
            tm_printf((UB *)"[sonar] timeout: Echo GP%u never rose within %u us\n",
                      (UINT)ULTRASONIC_ECHO_PIN, (UINT)BUDDY5_ECHO_TIMEOUT_US);
            result = BUDDY5_PING_TIMEOUT_RISE;
            goto store_reading;
        }
    }
    echo_started_us = hal_time_us();
    while (hal_gpio_read(ULTRASONIC_ECHO_PIN))
    {
        if ((hal_time_us() - echo_started_us) >= BUDDY5_ECHO_TIMEOUT_US)
        {
            tm_printf((UB *)"[sonar] timeout: Echo GP%u stayed HIGH for %u us\n",
                      (UINT)ULTRASONIC_ECHO_PIN, (UINT)BUDDY5_ECHO_TIMEOUT_US);
            result = BUDDY5_PING_TIMEOUT_FALL;
            goto store_reading;
        }
    }
    echo_finished_us = hal_time_us();
    pulse_us = (uint32_t)(echo_finished_us - echo_started_us);
    distance_cm = pulse_us / BUDDY5_US_PER_CM;
    result = BUDDY5_PING_VALID;
    tm_printf((UB *)"[sonar] pulse=%u us distance=%u cm\n",
              (UINT)pulse_us, (UINT)distance_cm);

store_reading:
    if (store_for_telemetry)
    {
        g_last.distance_cm     = distance_cm;
        g_last.servo_angle_deg = g_servo_angle;
        g_last.timestamp_ms    = now_ms;
    }
    if (NULL != p_result)
    {
        *p_result = result;
    }

    return distance_cm;
}

/**
 * @brief Measure the current forward-facing distance and update telemetry.
 * @param now_ms Current application time in milliseconds.
 * @return Measured distance in centimetres, or BUDDY5_NO_ECHO.
 */
uint32_t buddy5_measure_cm(uint32_t now_ms)
{
    return buddy5_ping_cm(now_ms, true, NULL);
}

/**
 * @brief Measure a temporary avoidance-scan distance without changing telemetry.
 * @return Measured distance in centimetres, or BUDDY5_NO_ECHO.
 */
uint32_t buddy5_sample_cm(void)
{
    /* Side-scan values belong to the avoidance task, not the dashboard's
     * forward-distance field, so deliberately leave g_last unchanged. */
    return buddy5_ping_cm(0u, false, NULL);
}

uint32_t buddy5_sample_with_result_cm(buddy5_ping_result_t *p_result)
{
    return buddy5_ping_cm(0u, false, p_result);
}

/**
 * @brief Safely copy the latest front-facing measurement for telemetry.
 * @param[out] p_reading Destination; NULL is ignored.
 */
void buddy5_get_last(buddy5_reading_t *p_reading)
{
    UINT interrupt_state;

    if (NULL != p_reading)
    {
        DI(interrupt_state);
        *p_reading = g_last;
        EI(interrupt_state);
    }
}
