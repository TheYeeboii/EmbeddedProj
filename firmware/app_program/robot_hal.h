/**
 * @file    robot_hal.h
 * @brief   Thin wrapper around the Pico C SDK hardware APIs.
 *
 * WHY THIS FILE EXISTS
 * The Pico SDK headers and the micro T-Kernel headers cannot be included in
 * the same .c file: both define things like GPIO_IN, GPIO_OUT, SIO_BASE and
 * size_t, differently.  So the rule in this project is:
 *
 *   - robot_hal.c is the ONLY application file that includes the Pico SDK
 *     "hardware/...h" headers.
 *   - Every other file includes <tk/tkernel.h> and this header.
 *
 * This header therefore uses plain C types only.  When a subsystem needs a
 * new SDK feature, add a small hal_... function here and in robot_hal.c.
 */

#ifndef ROBOT_HAL_H
#define ROBOT_HAL_H

#include <stdbool.h>
#include <stdint.h>

/* GPIO interrupt event bits (same values as the SDK's GPIO_IRQ_EDGE_*). */
#define HAL_GPIO_EDGE_FALL      (0x4u)
#define HAL_GPIO_EDGE_RISE      (0x8u)

/* Interrupt number shared by all GPIO pins (SDK: IO_IRQ_BANK0). */
#define HAL_IRQ_IO_BANK0        (13u)

/* I2C result codes. */
#define HAL_I2C_OK              (0)
#define HAL_I2C_ERROR           (-1)

/* ---- Start-up ---- */
/** Call once, first thing in usermain(): makes the microsecond timer run. */
void     hal_init(void);

/* ---- Time ---- */
/** Microseconds since boot (1 MHz hardware timer). */
uint64_t hal_time_us(void);
/** Diagnostic boot identifier; non-cryptographic oscillator noise. */
uint32_t hal_diagnostic_boot_id(void);
/** Busy-wait.  Only for very short waits (a few us); use tk_dly_tsk() otherwise. */
void     hal_delay_us(uint32_t us);

/* ---- GPIO ---- */
void     hal_gpio_input_pullup(uint32_t pin);
void     hal_gpio_input_pulldown(uint32_t pin);
void     hal_gpio_input(uint32_t pin);
void     hal_gpio_output(uint32_t pin, bool initial_value);
void     hal_gpio_write(uint32_t pin, bool value);
bool     hal_gpio_read(uint32_t pin);

/* ---- GPIO interrupts (used by gpio_irq.c) ---- */
void     hal_gpio_irq_enable(uint32_t pin, uint32_t events);
/** Read the pending events of a pin (limited to mask) and acknowledge them. */
uint32_t hal_gpio_irq_take_events(uint32_t pin, uint32_t mask);

/* ---- PWM ---- */
/** micro T-Kernel leaves the PWM block in reset: call once before PWM use. */
void     hal_pwm_release_reset(void);
/** Route a pin to PWM with the given counter top (wrap), output 0. */
void     hal_pwm_pin_init(uint32_t pin, uint32_t wrap);
/** Same, with an integer clock divider (e.g. 64 for a 50 Hz servo). */
void     hal_pwm_pin_init_div(uint32_t pin, uint32_t wrap, uint32_t clkdiv);
void     hal_pwm_set_level(uint32_t pin, uint32_t level);

/* ---- ADC (IR sensors) ---- */
void     hal_adc_init(void);
void     hal_adc_pin_init(uint32_t pin);
/** One 12-bit conversion (0..4095) on channel 0..3. */
uint16_t hal_adc_read(uint32_t channel);

/* ---- I2C (IMU / compass) ---- */
/** port: 0 or 1.  Returns false if the port number is invalid. */
bool     hal_i2c_init(uint32_t port, uint32_t baudrate,
                      uint32_t sda_pin, uint32_t scl_pin);
/** Returns HAL_I2C_OK or HAL_I2C_ERROR (no ACK or timeout). */
int32_t  hal_i2c_write(uint32_t port, uint8_t address,
                       const uint8_t *p_data, uint32_t length, bool nostop);
int32_t  hal_i2c_read(uint32_t port, uint8_t address,
                      uint8_t *p_data, uint32_t length, bool nostop);

#endif /* ROBOT_HAL_H */
