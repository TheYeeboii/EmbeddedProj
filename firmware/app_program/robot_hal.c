/**
 * @file    robot_hal.c
 * @brief   Thin wrapper around the Pico C SDK hardware APIs.
 *
 * The ONLY application file allowed to include Pico SDK "hardware/..."
 * headers.  Do NOT include <tk/tkernel.h> here - see robot_hal.h.
 */

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/resets.h"
#include "hardware/timer.h"
#include "hardware/structs/watchdog.h"

#include "robot_hal.h"

/* Make sure our plain-C copies of SDK constants never drift. */
_Static_assert(HAL_GPIO_EDGE_FALL == GPIO_IRQ_EDGE_FALL, "edge fall mismatch");
_Static_assert(HAL_GPIO_EDGE_RISE == GPIO_IRQ_EDGE_RISE, "edge rise mismatch");
_Static_assert(HAL_IRQ_IO_BANK0 == IO_IRQ_BANK0,         "IRQ number mismatch");

/* The crystal on the Pico is 12 MHz; micro T-Kernel runs clk_ref from it. */
#define HAL_XOSC_MHZ            (12u)

/* clk_sys as configured by micro T-Kernel (include/sys/.../sysdef.h). */
#define HAL_CLK_SYS_HZ          (125000000u)

/* Per-byte I2C timeout, so a missing or stuck sensor cannot hang a task. */
#define HAL_I2C_TIMEOUT_US      (2000u)

/* ------------------------------------------------------------------ *
 *  SDK functions that normally come from the SDK runtime.
 *  micro T-Kernel replaces that runtime, so we provide them here.
 *  They are "weak": the Wi-Fi build supplies its own identical copies
 *  (lib/libwifi/.../cyw43_utk_compat.c), which then take precedence.
 * ------------------------------------------------------------------ */

__attribute__((weak)) uint32_t clock_get_hz(clock_handle_t clock)
{
    (void)clock;
    return HAL_CLK_SYS_HZ;
}

__attribute__((weak)) uint64_t time_us_64(void)
{
    uint32_t hi0;
    uint32_t lo;
    uint32_t hi1;

    do
    {
        hi0 = timer_hw->timerawh;
        lo  = timer_hw->timerawl;
        hi1 = timer_hw->timerawh;
    } while (hi0 != hi1);

    return ((uint64_t)hi0 << 32) | lo;
}

/* ---------------- Start-up ---------------- */

void hal_init(void)
{
    /* The 1 MHz timer (used for ultrasonic timing and I2C timeouts) needs
     * the TIMER block out of reset and the watchdog "tick" generator
     * dividing the 12 MHz reference down to 1 MHz.  The Pico SDK runtime
     * normally does both; under micro T-Kernel we do it here. */
    unreset_block_mask_wait_blocking(RESETS_RESET_TIMER_BITS);
    unreset_block_mask_wait_blocking(RESETS_RESET_PWM_BITS);

    if ((watchdog_hw->tick & WATCHDOG_TICK_CYCLES_BITS) != HAL_XOSC_MHZ)
    {
        watchdog_hw->tick = WATCHDOG_TICK_ENABLE_BITS | HAL_XOSC_MHZ;
    }
}

/* ---------------- Time ---------------- */

uint64_t hal_time_us(void)
{
    return time_us_64();
}

void hal_delay_us(uint32_t us)
{
    uint64_t end   = time_us_64() + us;
    uint32_t guard = (us + 1u) * 200u;  /* never spin forever, even if the
                                          timer were not running */

    while ((time_us_64() < end) && (guard > 0u))
    {
        guard--;
    }
}

/* ---------------- GPIO ---------------- */

void hal_gpio_input_pullup(uint32_t pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

void hal_gpio_input_pulldown(uint32_t pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_down(pin);
}

void hal_gpio_input(uint32_t pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
}

void hal_gpio_output(uint32_t pin, bool initial_value)
{
    /* gpio_init() also takes the pin back from whatever function it had
     * (e.g. the kernel's UART on GP0/GP1). */
    gpio_init(pin);
    gpio_disable_pulls(pin);
    gpio_put(pin, initial_value);
    gpio_set_dir(pin, GPIO_OUT);
}

void hal_gpio_write(uint32_t pin, bool value)
{
    gpio_put(pin, value);
}

bool hal_gpio_read(uint32_t pin)
{
    return gpio_get(pin);
}

/* ---------------- GPIO interrupts ---------------- */

void hal_gpio_irq_enable(uint32_t pin, uint32_t events)
{
    /* Enables the event in IO_BANK0 only.  The NVIC side is owned by
     * micro T-Kernel (tk_def_int / EnableInt in gpio_irq.c).
     *
     * We do not use the SDK's gpio_set_irq_enabled(): it pulls in the
     * SDK interrupt manager, which micro T-Kernel replaces.  We also always
     * use the core-0 enable register, because on this port peripheral
     * interrupts are owned by core 0 (kernel "processor 1"). */
    io_bank0_irq_ctrl_hw_t *p_irq_ctrl = &io_bank0_hw->proc0_irq_ctrl;
    uint32_t                shift      = 4u * (pin % 8u);

    gpio_acknowledge_irq(pin, events);     /* drop any stale edge */
    hw_set_bits(&p_irq_ctrl->inte[pin / 8u], events << shift);
}

uint32_t hal_gpio_irq_take_events(uint32_t pin, uint32_t mask)
{
    /* Core-0 status register, for the same reason as above. */
    const io_bank0_irq_ctrl_hw_t *p_irq_ctrl = &io_bank0_hw->proc0_irq_ctrl;
    uint32_t events = ((p_irq_ctrl->ints[pin / 8u] >> (4u * (pin % 8u))) & 0xFu) & mask;

    if (0u != events)
    {
        gpio_acknowledge_irq(pin, events);
    }
    return events;
}

/* ---------------- PWM ---------------- */

void hal_pwm_release_reset(void)
{
    unreset_block_mask_wait_blocking(RESETS_RESET_PWM_BITS);
}

void hal_pwm_pin_init_div(uint32_t pin, uint32_t wrap, uint32_t clkdiv)
{
    pwm_config config = pwm_get_default_config();
    uint32_t   slice  = pwm_gpio_to_slice_num(pin);

    pwm_config_set_clkdiv_int(&config, clkdiv);
    pwm_config_set_wrap(&config, (uint16_t)wrap);
    pwm_init(slice, &config, true);
    pwm_set_gpio_level(pin, 0u);
    gpio_disable_pulls(pin);
    gpio_set_function(pin, GPIO_FUNC_PWM);
}

void hal_pwm_pin_init(uint32_t pin, uint32_t wrap)
{
    hal_pwm_pin_init_div(pin, wrap, 1u);
}

void hal_pwm_set_level(uint32_t pin, uint32_t level)
{
    pwm_set_gpio_level(pin, (uint16_t)level);
}

/* ---------------- ADC ---------------- */

void hal_adc_init(void)
{
    adc_init();             /* also takes the ADC block out of reset */
}

void hal_adc_pin_init(uint32_t pin)
{
    adc_gpio_init(pin);
}

uint16_t hal_adc_read(uint32_t channel)
{
    adc_select_input(channel);
    (void)adc_read();       /* discard the first sample after switching */
    return adc_read();
}

/* ---------------- I2C ---------------- */

static i2c_inst_t *hal_i2c_port(uint32_t port)
{
    if (0u == port)
    {
        return i2c0;
    }
    if (1u == port)
    {
        return i2c1;
    }
    return NULL;
}

bool hal_i2c_init(uint32_t port, uint32_t baudrate,
                  uint32_t sda_pin, uint32_t scl_pin)
{
    i2c_inst_t *p_i2c = hal_i2c_port(port);

    if (NULL == p_i2c)
    {
        return false;
    }
    (void)i2c_init(p_i2c, baudrate);    /* resets + configures the block */
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    return true;
}

int32_t hal_i2c_write(uint32_t port, uint8_t address,
                      const uint8_t *p_data, uint32_t length, bool nostop)
{
    i2c_inst_t *p_i2c = hal_i2c_port(port);
    int         result;

    if ((NULL == p_i2c) || (NULL == p_data))
    {
        return HAL_I2C_ERROR;
    }
    result = i2c_write_timeout_per_char_us(p_i2c, address, p_data, length,
                                           nostop, HAL_I2C_TIMEOUT_US);
    return (result == (int)length) ? HAL_I2C_OK : HAL_I2C_ERROR;
}

int32_t hal_i2c_read(uint32_t port, uint8_t address,
                     uint8_t *p_data, uint32_t length, bool nostop)
{
    i2c_inst_t *p_i2c = hal_i2c_port(port);
    int         result;

    if ((NULL == p_i2c) || (NULL == p_data))
    {
        return HAL_I2C_ERROR;
    }
    result = i2c_read_timeout_per_char_us(p_i2c, address, p_data, length,
                                          nostop, HAL_I2C_TIMEOUT_US);
    return (result == (int)length) ? HAL_I2C_OK : HAL_I2C_ERROR;
}
