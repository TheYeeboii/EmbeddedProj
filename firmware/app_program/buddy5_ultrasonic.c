/**
 * @file    buddy5_ultrasonic.c
 * @brief   Buddy 5: ultrasonic distance sensor on a scanning servo.
 *
 * How a measurement works:
 *   1. The task sends a 10 us trigger pulse and goes to sleep.
 *   2. The echo pin's rising edge (interrupt) stores the start time.
 *   3. The falling edge stores the end time and wakes the task.
 *   4. distance_cm = echo_time_us / 58 (team's original formula).
 * If no echo arrives within BUDDY5_ECHO_TIMEOUT_MS the reading is 0.
 */

#include <tk/tkernel.h>

#include "buddy5_ultrasonic.h"
#include "gpio_irq.h"
#include "pin_config.h"
#include "robot_hal.h"

#define BUDDY5_TRIGGER_PULSE_US     (10u)
#define BUDDY5_ECHO_TIMEOUT_MS      (40)     /* ~6.8 m round trip */
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

static volatile uint64_t g_echo_start_us = 0u;
static volatile uint64_t g_echo_end_us   = 0u;
static volatile ID       g_waiting_task  = 0;
static buddy5_reading_t  g_last;
static uint8_t           g_servo_angle   = 90u;

/* Interrupt context: time-stamp the echo edges, wake the waiting task. */
static void buddy5_echo_callback(uint32_t gpio, uint32_t events)
{
    uint64_t now = hal_time_us();

    (void)gpio;
    if (0u != (events & HAL_GPIO_EDGE_RISE))
    {
        g_echo_start_us = now;
    }
    if ((0u != (events & HAL_GPIO_EDGE_FALL)) && (0u != g_echo_start_us))
    {
        g_echo_end_us = now;
        if (g_waiting_task > 0)
        {
            (void)tk_wup_tsk(g_waiting_task);
        }
    }
}

void buddy5_init(void)
{
    buddy5_reading_t empty = { BUDDY5_NO_ECHO, 90u, 0u };

    g_last = empty;

    /* Takes GP0/GP1 back from the kernel's UART console. */
    hal_gpio_output(ULTRASONIC_TRIG_PIN, false);
    hal_gpio_input(ULTRASONIC_ECHO_PIN);
    (void)gpio_irq_register(ULTRASONIC_ECHO_PIN,
                            HAL_GPIO_EDGE_RISE | HAL_GPIO_EDGE_FALL,
                            buddy5_echo_callback);

    hal_pwm_pin_init_div(SERVO_PWM_PIN, BUDDY5_SERVO_WRAP, BUDDY5_SERVO_CLKDIV);
    buddy5_set_servo_angle(90u);
}

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

uint32_t buddy5_measure_cm(uint32_t now_ms)
{
    uint32_t distance_cm = BUDDY5_NO_ECHO;
    UINT     interrupt_state;
    ER       result;

    DI(interrupt_state);
    g_echo_start_us = 0u;
    g_echo_end_us   = 0u;
    g_waiting_task  = tk_get_tid();
    EI(interrupt_state);
    (void)tk_can_wup(TSK_SELF);             /* drop any stale wake-up */

    hal_gpio_write(ULTRASONIC_TRIG_PIN, true);
    hal_delay_us(BUDDY5_TRIGGER_PULSE_US);
    hal_gpio_write(ULTRASONIC_TRIG_PIN, false);

    result = tk_slp_tsk(BUDDY5_ECHO_TIMEOUT_MS);

    DI(interrupt_state);
    g_waiting_task = 0;
    if ((E_OK == result) && (g_echo_end_us > g_echo_start_us))
    {
        distance_cm = (uint32_t)((g_echo_end_us - g_echo_start_us) / BUDDY5_US_PER_CM);
    }
    g_last.distance_cm     = distance_cm;
    g_last.servo_angle_deg = g_servo_angle;
    g_last.timestamp_ms    = now_ms;
    EI(interrupt_state);

    return distance_cm;
}

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
