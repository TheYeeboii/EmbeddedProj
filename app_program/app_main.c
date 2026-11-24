/**
 * @file    app_main.c
 * @brief   Robot car application entry point (micro T-Kernel).
 *
 * usermain() is where micro T-Kernel hands control to us.  It first calls
 * every subsystem's subsystemN_init() (hardware and kernel objects), then
 * every subsystemN_start() (tasks), starts the shared controller task, and
 * sleeps forever.  The task list and priorities are in app.h; owners, pins
 * and interfaces in TEAM.md.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "app.h"
#include "gpio_irq.h"
#include "robot_controller.h"
#include "robot_hal.h"
#include "subsystem1.h"
#include "subsystem2.h"
#include "subsystem3.h"
#include "subsystem4.h"
#include "subsystem5.h"

/* ------------------------------------------------------------------ *
 *  Configuration
 * ------------------------------------------------------------------ */

#define APP_STACK_SIZE (4096)

#define APP_CONTROLLER_PERIOD_MS (20)

/* ------------------------------------------------------------------ *
 *  Shared helpers (app.h)
 * ------------------------------------------------------------------ */

uint32_t app_now_ms (void)
{
    SYSTIM now = {0};

    (void) tk_get_otm(&now);

    /* The low word wraps every 49 days; callers use wrap-safe differences. */
    return (uint32_t) now.lo;
}

bool app_check (const char * p_what, INT result)
{
    bool b_ok = (result >= E_OK);

    if (!b_ok)
    {
        tm_printf((UB *) "[init] %s FAILED, error %d\n", p_what, result);
    }

    return b_ok;
}

/* The loop keeps the reason visible on the USB console instead of returning
 * from usermain(), which would otherwise stop the kernel silently. */
void app_fail_forever (const char * p_what)
{
    for (;;)
    {
        tm_printf((UB *) "[init] FATAL: %s failed - see error above. Robot "
                         "stopped.\n",
                  p_what);
        (void) tk_dly_tsk(2000);
    }
}

ID app_start_task (const char * p_name, void (*p_task)(INT, void *),
                   INT priority)
{
    T_CTSK ctsk    = {0};
    ID     task_id = 0;

    ctsk.tskatr  = TA_HLNG | TA_RNG3;
    ctsk.task    = (FP) p_task;
    ctsk.itskpri = (PRI) priority;
    ctsk.stksz   = APP_STACK_SIZE;
    task_id      = tk_cre_tsk(&ctsk);
    if (!app_check(p_name, task_id))
    {
        app_fail_forever(p_name);
    }
    (void) tk_sta_tsk(task_id, 0);

    return task_id;
}

/** @brief Cyclic-handler ISR: wake the task whose ID is stored in exinf. */
static void app_wake_isr (void * p_exinf)
{
    /* exinf always points at a subsystem's static task-ID variable. */
    (void) tk_wup_tsk(*(ID *) p_exinf);
}

void app_start_cyclic (const char * p_name, ID * p_task_id, UINT period_ms)
{
    T_CCYC ccyc = {0};

    ccyc.exinf  = p_task_id;
    ccyc.cycatr = TA_HLNG | TA_STA;
    ccyc.cychdr = app_wake_isr;
    ccyc.cyctim = (RELTIM) period_ms;
    ccyc.cycphs = (RELTIM) period_ms;
    if (!app_check(p_name, tk_cre_cyc(&ccyc)))
    {
        app_fail_forever(p_name);
    }
}

/* ------------------------------------------------------------------ *
 *  Controller: commands in, decisions out
 * ------------------------------------------------------------------ */

/**
 * @brief Process dashboard commands and advance the main robot controller.
 *
 * The task waits for an MQTT command for up to APP_CONTROLLER_PERIOD_MS, then
 * calls robot_controller_step() even when no command was received.
 */
static void controller_task (INT stacd, void * p_exinf)
{
    uint8_t  command[COMMS_COMMAND_MAX];
    uint32_t length = 0u;

    (void) stacd;
    (void) p_exinf;
    for (;;)
    {
        /* Sleeps until a command arrives or 20 ms pass, whichever first. */
        if (comms_wait_command(command, sizeof(command), &length,
                               APP_CONTROLLER_PERIOD_MS))
        {
            robot_controller_handle_command(command, length, app_now_ms());
        }
        robot_controller_step(app_now_ms());
        /* Small sensor-status reports get room before bulk trace batches. */
        barcode_tuning_publish(app_now_ms());
        robot_action_publish(app_now_ms());
        robot_steering_publish(app_now_ms());
        robot_barcode_publish(app_now_ms());
        robot_ir_stream_publish(app_now_ms());
    }
}

/* ------------------------------------------------------------------ *
 *  Entry point
 * ------------------------------------------------------------------ */

/**
 * @brief Initialise every subsystem, start their tasks, then sleep.
 *
 * This is the micro T-Kernel application entry point.  It must not return
 * while the rover is running.
 */
EXPORT INT usermain (void)
{
    /* 1. Shared hardware. */
    hal_init();

    /* Release every Grove signal pin (Grove 1: GP0/GP1, Grove 2: GP2/GP3,
     * Grove 4: GP16/GP17) as a harmless pulled-down input before anything
     * else.  The kernel claims some of them at boot (hw_setting.c): GP0/GP1
     * as UART0 - and even in the USB-console build it mirrors console text
     * out of GP0, so UART TX drives that pin - and GP16 as the board-LED
     * output, driven low.  Our console is USB and no LED is fitted.  The
     * modules below then set up only the pins they really use
     * (pin_config.h), so a pin nothing uses is never driven and a stray
     * wire on it is safe.  (Grove 3 is I2C and Grove 5-7 are ADC; their
     * pins are set up by their own drivers.) */
    {
        static const uint8_t grove_pins[] = {0u, 1u, 2u, 3u, 16u, 17u};
        uint32_t             pin_index    = 0u;

        for (pin_index = 0u;
             pin_index < (sizeof(grove_pins) / sizeof(grove_pins[0]));
             pin_index++)
        {
            hal_gpio_input_pulldown((uint32_t) grove_pins[pin_index]);
        }
    }

    if (!app_check("gpio_irq_init", (INT) gpio_irq_init()))
    {
        app_fail_forever("gpio_irq_init");
    }

    /* 2. Every subsystem's hardware and kernel objects (no tasks yet). */
    subsystem1_init();
    subsystem2_init();
    subsystem3_init();
    subsystem4_init();
    subsystem5_init();
    robot_controller_init();

    /* 3. Tasks.  Each subsystem creates its own task(s) and cyclic
     * handlers. */
    subsystem2_start();
    subsystem3_start();
    (void) app_start_task("controller task", controller_task,
                          APP_PRI_CONTROLLER);
    subsystem4_start();
    subsystem5_start();
    subsystem1_start();

    /* usermain must never return while the system is running. */
    (void) tk_slp_tsk(TMO_FEVR);
    return 0;
}

/*** end of file ***/
