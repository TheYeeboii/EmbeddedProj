/**
 * @file    app_main.c
 * @brief   Robot car application entry point (micro T-Kernel).
 *
 * usermain() is where micro T-Kernel hands control to us.  It initialises
 * every subsystem, creates the tasks, and then sleeps forever.
 *
 *   Task         Pri  Period            Job                          Buddy
 *   -----------  ---  ----------------  ---------------------------  -----
 *   (usb)          3  kernel            USB serial console           -
 *   (cyw43)        4  2 ms, kernel      Wi-Fi + lwIP + MQTT          1
 *   motion         5  20 ms (cyclic)    buddy2_update(): PID, moves  2
 *   line           6  10 ms (cyclic)    buddy3_update(): IR, barcode 3
 *   controller     7  20 ms             commands, barcodes, buttons  all
 *   imu            9  50 ms target      IMU + hump processing         4
 *   sensors       10  100 ms            ultrasonic + avoidance        5
 *   telemetry     15  1 s               console + MQTT reports       1
 *
 * Lower number = higher priority.  (usb) and (cyw43) are created by the
 * kernel itself; (cyw43) only exists in Wi-Fi builds.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "app_main.h"
#include "buddy2_motion.h"
#include "buddy3_ir.h"
#include "buddy4_imu.h"
#include "buddy5_avoidance.h"
#include "buddy5_ultrasonic.h"
#include "comms.h"
#include "gpio_irq.h"
#include "pin_config.h"
#include "robot_controller.h"
#include "robot_hal.h"
#include "robot_telemetry.h"
#include "usb_console_compat.h"

/* ------------------------------------------------------------------ *
 *  Configuration
 * ------------------------------------------------------------------ */

#define APP_STACK_SIZE              (4096)

#define APP_PRI_MOTION              (5)
#define APP_PRI_LINE                (6)
#define APP_PRI_CONTROLLER          (7)
#define APP_PRI_IMU                 (9)
#define APP_PRI_SENSORS             (10)
#define APP_PRI_TELEMETRY           (15)

#define APP_CONTROLLER_PERIOD_MS    (20)

#define APP_TELEMETRY_TICK_MS       (1000)
#define APP_TELEMETRY_EVERY_N       (1u)     /* rover/telemetry every 1 s */
#define APP_HEARTBEAT_EVERY_N       (10u)    /* rover/heartbeat every 10 s */
#define APP_USB_WAIT_MS             (3000)
#define APP_STATUS_EVERY_N          (2u)     /* [status] line every 2 s   */
/* 1 = also print the raw JSON (what the dashboard gets) on the console. */
#define APP_CONSOLE_JSON            (0)

/* ------------------------------------------------------------------ *
 *  Helpers
 * ------------------------------------------------------------------ */

static ID g_motion_task_id = 0;
static ID g_line_task_id   = 0;

/** @brief Return the micro T-Kernel uptime in milliseconds. */
uint32_t app_now_ms(void)
{
    SYSTIM now;

    (void)tk_get_otm(&now);
    return (uint32_t)now.lo;
}

/** @brief Print an initialisation error and report whether a kernel call succeeded. */
static bool app_check(const char *p_what, ER result)
{
    if (result < E_OK)
    {
        tm_printf((UB *)"[init] %s FAILED, error %d\n", p_what, (INT)result);
        return false;
    }
    return true;
}

/**
 * @brief Keep the rover stopped after a fatal start-up failure.
 *
 * The loop keeps the reason visible on the USB console instead of returning
 * from usermain(), which would otherwise stop the kernel silently.
 */
static void app_fail_forever(const char *p_what)
{
    while (1)
    {
        tm_printf((UB *)"[init] FATAL: %s failed - see error above. Robot stopped.\n", p_what);
        (void)tk_dly_tsk(2000);
    }
}

/* ------------------------------------------------------------------ *
 *  Periodic tasks woken by cyclic handlers (exact period)
 * ------------------------------------------------------------------ */

/** @brief Wake the task whose ID is stored in a cyclic handler's exinf field. */
static void app_wake_handler(void *p_exinf)
{
    /* exinf holds a pointer to the task id to wake. */
    (void)tk_wup_tsk(*(ID *)p_exinf);
}

/**
 * @brief Run Buddy 2's periodic motor/encoder update.
 *
 * A cyclic handler wakes this task at BUDDY2_CONTROL_PERIOD_MS; the task
 * then updates motor control, encoder distance, and active motion commands.
 */
static void motion_task(INT stacd, void *p_exinf)
{
    (void)stacd;
    (void)p_exinf;
    while (1)
    {
        (void)tk_slp_tsk(TMO_FEVR);
        buddy2_update(app_now_ms());
    }
}

/**
 * @brief Run Buddy 3's periodic line-sensor update.
 *
 * A cyclic handler wakes this task at BUDDY3_SAMPLE_PERIOD_MS so line,
 * junction, and barcode readings remain current for the controller.
 */
static void line_task(INT stacd, void *p_exinf)
{
    (void)stacd;
    (void)p_exinf;
    while (1)
    {
        (void)tk_slp_tsk(TMO_FEVR);
        buddy3_update(app_now_ms());
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
static void controller_task(INT stacd, void *p_exinf)
{
    char command[COMMS_COMMAND_MAX];

    (void)stacd;
    (void)p_exinf;
    while (1)
    {
        /* Sleeps until a command arrives or 20 ms pass, whichever first. */
        if (comms_wait_command(command, sizeof(command), APP_CONTROLLER_PERIOD_MS))
        {
            tm_printf((UB *)"[mqtt] command: %s\n", command);
            robot_controller_handle_command(command, app_now_ms());
        }
        robot_controller_step(app_now_ms());
    }
}

/* Buddy 5 ultrasonic monitoring and obstacle avoidance live in
 * buddy5_avoidance.c.  This file only creates that task. */
/* ------------------------------------------------------------------ *
 *  Telemetry: console + MQTT
 * ------------------------------------------------------------------ */

/** @brief Convert a communications-link state to console text. */
static const char *app_link_name(uint32_t state)
{
    if (COMMS_LINK_MQTT_UP == state)
    {
        return "MQTT connected";
    }
    if (COMMS_LINK_WIFI_UP == state)
    {
        return "Wi-Fi connected, looking for the MQTT broker...";
    }
    return "offline";
}

/** @brief Convert a line-sensor Boolean reading to the status-line colour label. */
static const char *app_color(bool black)
{
    return black ? "BLACK" : "white";
}

/**
 * @brief Print one human-readable line of current robot status to USB.
 *
 * Example: [status] idle | front L=white R=white | side=white |
 * sonar 23 cm | heading 163.4 | pitch 1.2
 */
static void app_print_status(void)
{
    buddy4_imu_state_t imu;
    buddy5_reading_t   sonar;
    bool               left;
    bool               right;
    bool               side;
    const char        *p_sign = "";
    int32_t            pitch;

    buddy3_get_colors(&left, &right, &side);
    buddy4_get_state(&imu);
    buddy5_get_last(&sonar);

    pitch = imu.pitch_deg_x10;
    if (pitch < 0)
    {
        p_sign = "-";
        pitch  = -pitch;
    }

    tm_printf((UB *)"[status] %-6s | front L=%s R=%s | side=%s | ",
              robot_controller_mode_name(), app_color(left), app_color(right), app_color(side));
    if (BUDDY5_NO_ECHO == sonar.distance_cm)
    {
        tm_printf((UB *)"sonar --    | ");
    }
    else
    {
        tm_printf((UB *)"sonar %u cm | ", (UINT)sonar.distance_cm);
    }
    if (imu.mag_ok)
    {
        tm_printf((UB *)"heading %u.%u | pitch %s%u.%u\n",
                  (UINT)(imu.heading_deg_x10 / 10), (UINT)(imu.heading_deg_x10 % 10),
                  p_sign, (UINT)(pitch / 10), (UINT)(pitch % 10));
    }
    else
    {
        tm_printf((UB *)"IMU not answering\n");
    }

    /* Wiring check: which pins this build reads, and the live tick counts. */
    {
        buddy2_motion_state_t motion;

        buddy2_get_motion_state(&motion);
        tm_printf((UB *)"[encoders] left GP%u = %d ticks | right GP%u = %d ticks | build %s %s\n",
                  (UINT)ENCODER_LEFT_A, (INT)motion.left_ticks,
                  (UINT)ENCODER_RIGHT_A, (INT)motion.right_ticks,
                  __DATE__, __TIME__);
    }
}

/**
 * @brief Publish dashboard telemetry and print periodic USB status.
 *
 * Debug JSON, full telemetry, and the MQTT heartbeat have independent rates
 * controlled by the APP_TELEMETRY_* configuration constants.
 */
static void telemetry_task(INT stacd, void *p_exinf)
{
    static char buffer[ROBOT_TELEMETRY_BUFFER_SIZE];
    uint32_t    tick      = 0u;
    uint32_t    last_link = 0xFFFFFFFFu;
    INT         i;

    (void)stacd;
    (void)p_exinf;

    /* Give the USB console a moment so the banner is not lost. */
    for (i = 0; (i < (APP_USB_WAIT_MS / 100)) && (tm_usb_state() < 2u); i++)
    {
        (void)tk_dly_tsk(100);
    }
    tm_printf((UB *)"\n==========================================================\n");
    tm_printf((UB *)" Robot car (micro T-Kernel), built %s %s\n", __DATE__, __TIME__);
    tm_printf((UB *)" Encoders: left GP%u, right GP%u\n",
              (UINT)ENCODER_LEFT_A, (UINT)ENCODER_RIGHT_A);
    tm_printf((UB *)" Buttons:  GP21 = line following on/off\n");
#if defined(TM_WIFI_MQTT) && (TM_WIFI_MQTT == 1)
    tm_printf((UB *)" Wi-Fi:    on (dashboard)\n");
#else
    tm_printf((UB *)" Wi-Fi:    not in this build\n");
#endif
    tm_printf((UB *)" [status] every %u s: front L/R = line sensors, side = barcode\n",
              (UINT)((APP_TELEMETRY_TICK_MS * APP_STATUS_EVERY_N) / 1000));
    tm_printf((UB *)"          sensor (BLACK = line, or nothing under it)\n");
    tm_printf((UB *)"==========================================================\n");

    while (1)
    {
        uint32_t link = comms_get_link_state();

#if defined(TM_WIFI_MQTT) && (TM_WIFI_MQTT == 1)
        if (link != last_link)
        {
            tm_printf((UB *)"[wifi] %s\n", app_link_name(link));
            last_link = link;
        }
#else
        (void)link;
        (void)last_link;
        (void)app_link_name;
#endif

        /* Human-readable status for the USB console. */
        if (0u == (tick % APP_STATUS_EVERY_N))
        {
            app_print_status();
        }

        /* JSON for the dashboard (MQTT).  Printed only if APP_CONSOLE_JSON. */
        robot_telemetry_build_debug(buffer);
        (void)comms_publish(COMMS_TOPIC_DEBUG, buffer);
#if APP_CONSOLE_JSON
        tm_printf((UB *)"[debug] %s\n", buffer);
#endif
        if (0u == (tick % APP_TELEMETRY_EVERY_N))
        {
            robot_telemetry_build(buffer, app_now_ms());
            (void)comms_publish(COMMS_TOPIC_TELEMETRY, buffer);
#if APP_CONSOLE_JSON
            tm_printf((UB *)"[telemetry] %s\n", buffer);
#endif
        }
        if (0u == (tick % APP_HEARTBEAT_EVERY_N))
        {
            robot_telemetry_build_heartbeat(buffer);
            (void)comms_publish(COMMS_TOPIC_HEARTBEAT, buffer);
        }

        tick++;
        (void)tk_dly_tsk(APP_TELEMETRY_TICK_MS);
    }
}

/* ------------------------------------------------------------------ *
 *  Kernel object definitions
 * ------------------------------------------------------------------ */

#define APP_TASK(fn, pri)  { .tskatr = TA_HLNG | TA_RNG3, .task = (fn), \
                             .itskpri = (pri), .stksz = APP_STACK_SIZE }

static T_CTSK g_ctsk_motion      = APP_TASK(motion_task,              APP_PRI_MOTION);
static T_CTSK g_ctsk_line        = APP_TASK(line_task,                APP_PRI_LINE);
static T_CTSK g_ctsk_controller  = APP_TASK(controller_task,          APP_PRI_CONTROLLER);
static T_CTSK g_ctsk_imu         = APP_TASK(buddy4_task, APP_PRI_IMU);
static T_CTSK g_ctsk_sensors     = APP_TASK(buddy5_avoidance_task,    APP_PRI_SENSORS);
static T_CTSK g_ctsk_telemetry   = APP_TASK(telemetry_task,           APP_PRI_TELEMETRY);

static T_CCYC g_ccyc_motion =
{
    .exinf  = &g_motion_task_id,
    .cycatr = TA_HLNG | TA_STA,
    .cychdr = app_wake_handler,
    .cyctim = BUDDY2_CONTROL_PERIOD_MS,
    .cycphs = BUDDY2_CONTROL_PERIOD_MS,
};

static T_CCYC g_ccyc_line =
{
    .exinf  = &g_line_task_id,
    .cycatr = TA_HLNG | TA_STA,
    .cychdr = app_wake_handler,
    .cyctim = BUDDY3_SAMPLE_PERIOD_MS,
    .cycphs = BUDDY3_SAMPLE_PERIOD_MS,
};

/**
 * @brief Create and start one micro T-Kernel task.
 *
 * A task-creation failure is fatal because the remaining application cannot
 * operate predictably without the requested subsystem task.
 */
static ID app_start_task(const char *p_name, T_CTSK *p_ctsk)
{
    ID id = tk_cre_tsk(p_ctsk);

    if (!app_check(p_name, id))
    {
        app_fail_forever(p_name);
    }
    (void)tk_sta_tsk(id, 0);
    return id;
}

/* ------------------------------------------------------------------ *
 *  Entry point
 * ------------------------------------------------------------------ */

/**
 * @brief Initialise hardware and all Buddies, create application tasks, then sleep.
 *
 * This is the micro T-Kernel application entry point.  It must not return
 * while the rover is running.
 */
EXPORT INT usermain(void)
{
    /* 1. Hardware and subsystems. */
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
        static const uint8_t grove_pins[] = { 0u, 1u, 2u, 3u, 16u, 17u };
        uint32_t             i;

        for (i = 0u; i < (sizeof(grove_pins) / sizeof(grove_pins[0])); i++)
        {
            hal_gpio_input_pulldown((uint32_t)grove_pins[i]);
        }
    }

    if (!app_check("gpio_irq_init", (ER)gpio_irq_init()))
    {
        app_fail_forever("gpio_irq_init");
    }
    if (!app_check("comms_init", (ER)comms_init()))
    {
        app_fail_forever("comms_init");
    }
    if (!app_check("buddy2_motion_init", (ER)buddy2_motion_init()))
    {
        app_fail_forever("buddy2_motion_init");
    }
    buddy3_init();
    buddy5_init();
    robot_controller_init();
    if (!buddy4_init())
    {
        /* Not fatal: telemetry will show "compass_error". */
        tm_printf((UB *)"[init] IMU (LSM303) not answering on I2C - check Grove 3.\n");
    }

    /* 2. Tasks.  The cyclic handlers are created after their tasks exist. */
    g_motion_task_id = app_start_task("motion task", &g_ctsk_motion);
    g_line_task_id   = app_start_task("line task", &g_ctsk_line);
    if (!app_check("motion cyclic", tk_cre_cyc(&g_ccyc_motion)))
    {
        app_fail_forever("motion cyclic");
    }
    if (!app_check("line cyclic", tk_cre_cyc(&g_ccyc_line)))
    {
        app_fail_forever("line cyclic");
    }
    (void)app_start_task("controller task", &g_ctsk_controller);
    (void)app_start_task("IMU task", &g_ctsk_imu);
    (void)app_start_task("sensor task", &g_ctsk_sensors);
    (void)app_start_task("telemetry task", &g_ctsk_telemetry);

    /* usermain must never return while the system is running. */
    (void)tk_slp_tsk(TMO_FEVR);
    return 0;
}
