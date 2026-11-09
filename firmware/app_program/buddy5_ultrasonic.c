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
#include "robot_debug.h"
#include "robot_fault.h"

#define BUDDY5_TRIGGER_PULSE_US (10u)
#define BUDDY5_ECHO_TIMEOUT_US  (40000u) /* ~6.8 m round trip */
/* Echo must be low before a ping. */
#define BUDDY5_ECHO_IDLE_TIMEOUT_US (2000u)
#define BUDDY5_US_PER_CM            (58u)

/* Servo: 125 MHz / 64 / (39062 + 1) = 50 Hz.  One count = 0.512 us.
 * SG90: 0.5 ms = 0 deg, 2.5 ms = 180 deg -> 977 .. 4883 counts.
 * (The old code used 1000..3000 = 0.51..1.54 ms, which only reached
 * about 95 deg.) */
#define BUDDY5_SERVO_CLKDIV      (64u)
#define BUDDY5_SERVO_WRAP        (39062u)
#define BUDDY5_SERVO_MIN_COUNTS  (977u)  /* 0.5 ms */
#define BUDDY5_SERVO_SPAN_COUNTS (3906u) /* +2.0 ms for 180 deg */
#define BUDDY5_SERVO_MAX_DEG     (180u)

/* Servo angle that points the sensor straight ahead. */
#define BUDDY5_SERVO_FORWARD_DEG (90u)
/* Forward pings in a row with no answer from the sensor before the no-echo
 * fault bit is set (robot_fault.h).  One good reading clears it. */
#define BUDDY5_NO_ECHO_FAULT_COUNT (3u)

static buddy5_reading_t g_last;
static uint8_t          g_servo_angle = BUDDY5_SERVO_FORWARD_DEG;
/* Forward pings in a row the sensor did not answer; only this task writes
 * it.  Saturates at BUDDY5_NO_ECHO_FAULT_COUNT (2 bits used). */
static uint8_t g_no_answer_count = 0u;

static bool     buddy5_wait_while (bool level, uint64_t started_us,
                                   uint32_t timeout_us);
static bool     buddy5_wait_for_echo_idle (void);
static uint32_t buddy5_ping_cm (uint32_t now_ms, bool store_for_telemetry,
                                buddy5_ping_result_t * p_result);
static void     buddy5_report_fault (buddy5_ping_result_t result);

/**
 * @brief Configure the HC-SR04+ GPIO pins, PWM servo output and initial state.
 *
 * Trigger starts low to avoid an accidental ping.  The saved distance starts
 * as BUDDY5_NO_ECHO until the first successful forward measurement.
 */
void buddy5_init (void)
{
    buddy5_reading_t empty = {BUDDY5_NO_ECHO, BUDDY5_SERVO_FORWARD_DEG, 0u};

    g_last = empty;

    hal_gpio_output(ULTRASONIC_TRIG_PIN, false);
    hal_gpio_input(ULTRASONIC_ECHO_PIN);
    tm_printf((UB *) "[sonar] configured: TRIG=GP%u ECHO=GP%u timeout=%u us\n",
              (UINT) ULTRASONIC_TRIG_PIN, (UINT) ULTRASONIC_ECHO_PIN,
              (UINT) BUDDY5_ECHO_TIMEOUT_US);

    hal_pwm_pin_init_div(SERVO_PWM_PIN, BUDDY5_SERVO_WRAP, BUDDY5_SERVO_CLKDIV);
    buddy5_set_servo_angle(BUDDY5_SERVO_FORWARD_DEG);
}

/**
 * @brief Convert a requested 0..180 degree angle into an SG90 PWM pulse.
 *
 * The PWM pulse is calculated from the calibrated 0.5..2.5 ms range and the
 * requested angle is retained so a later telemetry reading records the angle
 * at which it was taken.
 */
void buddy5_set_servo_angle (uint8_t angle_deg)
{
    uint32_t counts = 0u;

    if (angle_deg > BUDDY5_SERVO_MAX_DEG)
    {
        angle_deg = BUDDY5_SERVO_MAX_DEG;
    }
    counts = BUDDY5_SERVO_MIN_COUNTS +
             (((uint32_t) angle_deg * BUDDY5_SERVO_SPAN_COUNTS) /
              BUDDY5_SERVO_MAX_DEG);
    hal_pwm_set_level(SERVO_PWM_PIN, counts);
    g_servo_angle = angle_deg;
}

/**
 * @brief Measure the current forward-facing distance and update telemetry.
 * @param now_ms Current application time in milliseconds.
 * @return Measured distance in centimetres, or BUDDY5_NO_ECHO.
 */
uint32_t buddy5_measure_cm (uint32_t now_ms)
{
    return buddy5_ping_cm(now_ms, true, NULL);
}

/**
 * @brief Measure a temporary avoidance-scan distance without changing
 * telemetry.
 * @return Measured distance in centimetres, or BUDDY5_NO_ECHO.
 */
uint32_t buddy5_sample_cm (void)
{
    /* Side-scan values belong to the avoidance task, not the dashboard's
     * forward-distance field, so deliberately leave g_last unchanged. */
    return buddy5_ping_cm(0u, false, NULL);
}

uint32_t buddy5_sample_with_result_cm (buddy5_ping_result_t * p_result)
{
    return buddy5_ping_cm(0u, false, p_result);
}

/**
 * @brief Safely copy the latest front-facing measurement for telemetry.
 * @param[out] p_reading Destination; NULL is ignored.
 */
void buddy5_get_last (buddy5_reading_t * p_reading)
{
    UINT interrupt_state = 0u;

    if (NULL != p_reading)
    {
        DI(interrupt_state);
        *p_reading = g_last;
        EI(interrupt_state);
    }
}

/**
 * @brief Busy-wait while the Echo pin reads @p level, up to a timeout.
 * @param level      Pin level to wait through.
 * @param started_us Time the wait is measured from.
 * @param timeout_us Longest allowed wait.
 * @return True when the pin changed in time, false on timeout.
 */
static bool buddy5_wait_while (bool level, uint64_t started_us,
                               uint32_t timeout_us)
{
    bool b_changed = false;
    bool b_waiting = true;

    while (b_waiting)
    {
        if (hal_gpio_read(ULTRASONIC_ECHO_PIN) != level)
        {
            b_changed = true;
            b_waiting = false;
        }
        else if ((hal_time_us() - started_us) >= timeout_us)
        {
            b_waiting = false;
        }
        else
        {
            /* Keep polling: the waits are bounded by timeout_us. */
        }
    }

    return b_changed;
}

/**
 * @brief Confirm that Echo is low before issuing a new trigger pulse.
 *
 * A normal HC-SR04+ leaves Echo low between measurements.  If the Echo pin is
 * still high, the previous pulse is incomplete or the line/sensor is faulty.
 * Do not send another trigger into that condition: reset the Echo pin to
 * input mode, report no echo, and allow the next scheduled measurement to
 * retry.
 */
static bool buddy5_wait_for_echo_idle (void)
{
    bool b_idle =
        buddy5_wait_while(true, hal_time_us(), BUDDY5_ECHO_IDLE_TIMEOUT_US);

    if (!b_idle)
    {
        /* Re-apply the intended GPIO direction in case another module or
         * a previous peripheral setup changed it.  This cannot pull down
         * a sensor that is electrically driving Echo high. */
        hal_gpio_input(ULTRASONIC_ECHO_PIN);
        tm_printf(
            (UB *) "[sonar] Echo GP%u HIGH before trigger; skipped ping\n",
            (UINT) ULTRASONIC_ECHO_PIN);
    }

    return b_idle;
}

/**
 * @brief Send one ultrasonic trigger pulse and calculate the echo distance.
 * @param now_ms Timestamp stored only when @p store_for_telemetry is true.
 * @param store_for_telemetry True for the forward reading; false for a
 * temporary avoidance scan.
 * @param p_result Receives why the ping failed; NULL is ignored.
 * @return Distance in centimetres, or BUDDY5_NO_ECHO for either timeout.
 *
 * Echo must first be low, then its rising edge is awaited and its high-pulse
 * duration is measured.  The bounded waits prevent the sensor task from
 * waiting forever on a disconnected or faulty sensor.
 */
static uint32_t buddy5_ping_cm (uint32_t now_ms, bool store_for_telemetry,
                                buddy5_ping_result_t * p_result)
{
    uint32_t             distance_cm     = BUDDY5_NO_ECHO;
    uint32_t             pulse_us        = 0u;
    uint64_t             echo_started_us = 0u;
    buddy5_ping_result_t result          = BUDDY5_PING_TIMEOUT_RISE;

    if (!buddy5_wait_for_echo_idle())
    {
        result = BUDDY5_PING_HIGH_BEFORE_TRIGGER;
    }
    else
    {
        hal_gpio_write(ULTRASONIC_TRIG_PIN, true);
        hal_delay_us(BUDDY5_TRIGGER_PULSE_US);
        hal_gpio_write(ULTRASONIC_TRIG_PIN, false);

        if (!buddy5_wait_while(false, hal_time_us(), BUDDY5_ECHO_TIMEOUT_US))
        {
            tm_printf(
                (UB *) "[sonar] timeout: Echo GP%u never rose within %u us\n",
                (UINT) ULTRASONIC_ECHO_PIN, (UINT) BUDDY5_ECHO_TIMEOUT_US);
            result = BUDDY5_PING_TIMEOUT_RISE;
        }
        else
        {
            echo_started_us = hal_time_us();
            if (!buddy5_wait_while(true, echo_started_us,
                                   BUDDY5_ECHO_TIMEOUT_US))
            {
                tm_printf(
                    (UB *) "[sonar] timeout: Echo GP%u stayed HIGH for %u us\n",
                    (UINT) ULTRASONIC_ECHO_PIN, (UINT) BUDDY5_ECHO_TIMEOUT_US);
                result = BUDDY5_PING_TIMEOUT_FALL;
            }
            else
            {
                /* The pulse is under BUDDY5_ECHO_TIMEOUT_US, so it fits. */
                pulse_us    = (uint32_t) (hal_time_us() - echo_started_us);
                distance_cm = pulse_us / BUDDY5_US_PER_CM;
                result      = BUDDY5_PING_VALID;
                ROBOT_ROUTINE_LOG((UB *) "[sonar] pulse=%u us distance=%u cm\n",
                                  (UINT) pulse_us, (UINT) distance_cm);
            }
        }
    }

    if (store_for_telemetry)
    {
        g_last.distance_cm     = distance_cm;
        g_last.servo_angle_deg = g_servo_angle;
        g_last.timestamp_ms    = now_ms;
        buddy5_report_fault(result);
    }
    if (NULL != p_result)
    {
        *p_result = result;
    }

    return distance_cm;
}

/**
 * @brief Keep the no-echo bit of the shared fault byte up to date.
 *
 * Only a sensor that does not answer counts: Echo never rising after the
 * trigger, or stuck high before it.  Echo staying high for the whole 40 ms
 * is the HC-SR04's normal "nothing in range" answer (open space), so it
 * neither counts nor clears.  Forward readings only.
 *
 * @param result How the forward ping ended.
 */
static void buddy5_report_fault (buddy5_ping_result_t result)
{
    if (BUDDY5_PING_VALID == result)
    {
        if (g_no_answer_count >= BUDDY5_NO_ECHO_FAULT_COUNT)
        {
            robot_fault_clear(ROBOT_FAULT_NO_ECHO);
        }
        g_no_answer_count = 0u;
    }
    else if ((BUDDY5_PING_TIMEOUT_RISE == result) ||
             (BUDDY5_PING_HIGH_BEFORE_TRIGGER == result))
    {
        if (g_no_answer_count < BUDDY5_NO_ECHO_FAULT_COUNT)
        {
            g_no_answer_count++;
            if (BUDDY5_NO_ECHO_FAULT_COUNT == g_no_answer_count)
            {
                robot_fault_set(ROBOT_FAULT_NO_ECHO);
            }
        }
    }
    else
    {
        /* Nothing in range: the sensor answered, the fault state stays. */
    }
}

/*** end of file ***/

