/**
 * @file    gpio_irq.h
 * @brief   Shared GPIO interrupt dispatcher for micro T-Kernel.
 *
 * The RP2040 has ONE interrupt line for all GPIO pins (IO_IRQ_BANK0).
 * Under micro T-Kernel that line is registered with tk_def_int(), and it
 * can only have one handler.  This module owns that handler and forwards
 * each pin's events to the callback registered for that pin.
 *
 * Every subsystem that needs a GPIO interrupt (wheel encoders, ultrasonic
 * echo, bump switches...) must register through this module instead of
 * calling tk_def_int() or the Pico SDK's gpio_set_irq_enabled_with_callback().
 */

#ifndef GPIO_IRQ_H
#define GPIO_IRQ_H

#include <stdint.h>

#include "robot_hal.h"

/**
 * @brief Callback run in INTERRUPT context when a registered pin fires.
 *
 * Keep it short: count, timestamp, or wake a task.  No printing, no
 * blocking kernel calls.
 *
 * @param gpio    Pin number that fired.
 * @param events  HAL_GPIO_EDGE_RISE / HAL_GPIO_EDGE_FALL bits that fired.
 */
typedef void (*gpio_irq_cb_t)(uint32_t gpio, uint32_t events);

/**
 * @brief Install the shared IO_IRQ_BANK0 handler.  Call once at start-up,
 *        before any gpio_irq_register().
 * @return 0 on success, a negative micro T-Kernel error code otherwise.
 */
int32_t gpio_irq_init(void);

/**
 * @brief Enable interrupts on one pin and route them to a callback.
 * @param gpio      Pin number (0..29).
 * @param events    HAL_GPIO_EDGE_RISE and/or HAL_GPIO_EDGE_FALL.
 * @param callback  Function to run from the interrupt.
 * @return 0 on success, -1 if the arguments are invalid.
 */
int32_t gpio_irq_register(uint32_t gpio, uint32_t events,
                          gpio_irq_cb_t callback);

#endif /* GPIO_IRQ_H */
