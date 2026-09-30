/**
 * @file    gpio_irq.c
 * @brief   Shared GPIO interrupt dispatcher for micro T-Kernel.
 *
 * See gpio_irq.h for why this exists.
 */

#include <tk/tkernel.h>

#include "gpio_irq.h"

/* Number of GPIO pins on the RP2040 user bank. */
#define GPIO_IRQ_PIN_COUNT      (30u)

/* NVIC priority for the GPIO bank interrupt.  On this port 1 is the
 * highest level an application interrupt may use and 3 is the lowest. */
#define GPIO_IRQ_NVIC_LEVEL     (2)

static gpio_irq_cb_t g_callbacks[GPIO_IRQ_PIN_COUNT];
static uint32_t            g_event_masks[GPIO_IRQ_PIN_COUNT];

/**
 * @brief The one and only IO_IRQ_BANK0 handler.
 *
 * Walks the pins that have a callback, acknowledges their pending events
 * and forwards them.
 */
static void gpio_irq_bank0_handler(UINT intno)
{
    uint32_t gpio;

    (void)intno;

    for (gpio = 0u; gpio < GPIO_IRQ_PIN_COUNT; gpio++)
    {
        if (NULL != g_callbacks[gpio])
        {
            uint32_t events = hal_gpio_irq_take_events(gpio, g_event_masks[gpio]);

            if (0u != events)
            {
                g_callbacks[gpio](gpio, events);
            }
        }
    }
}

int32_t gpio_irq_init(void)
{
    T_DINT dint;
    ER     result;

    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)gpio_irq_bank0_handler;

    result = tk_def_int((UINT)HAL_IRQ_IO_BANK0, &dint);
    if (E_OK == result)
    {
        EnableInt((UINT)HAL_IRQ_IO_BANK0, GPIO_IRQ_NVIC_LEVEL);
    }

    return (int32_t)result;
}

int32_t gpio_irq_register(uint32_t gpio, uint32_t events,
                          gpio_irq_cb_t callback)
{
    UINT interrupt_state;

    if ((gpio >= GPIO_IRQ_PIN_COUNT) || (NULL == callback) || (0u == events))
    {
        return -1;
    }

    /* Publish the callback before the pin can fire. */
    DI(interrupt_state);
    g_callbacks[gpio]   = callback;
    g_event_masks[gpio] = events;
    EI(interrupt_state);

    hal_gpio_irq_enable(gpio, events);

    return 0;
}
