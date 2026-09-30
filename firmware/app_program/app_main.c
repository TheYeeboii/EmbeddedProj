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
 *   sensors       10  100 ms            IMU; ultrasonic + servo scan 4, 5
 *   telemetry     15  1 s               console + MQTT reports       1
 *   motion_test   20  on GP20 button    calibration test             2
 *   blink         30  250 ms            GP16 "alive" LED             -
 *
 * Lower number = higher priority.  (usb) and (cyw43) are created by the
 * kernel itself; (cyw43) only exists in Wi-Fi builds.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "app_main.h"
#include "buddy2_motion.h"
#include "buddy2_motion_test.h"
#include "buddy3_ir.h"
#include "buddy4_imu.h"
#include "buddy5_ultrasonic.h"
#include "comms.h"
#include "demo_tasks.h"
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
#define APP_PRI_SENSORS             (10)
#define APP_PRI_TELEMETRY           (15)
#define APP_PRI_MOTION_TEST         (20)
#define APP_PRI_BLINK               (30)

#define APP_CONTROLLER_PERIOD_MS    (20)
#define APP_SENSOR_PERIOD_MS        (100)
#define APP_SONAR_EVERY_N_CYCLES    (5u)     /* ultrasonic every 500 ms  */
#define APP_SERVO_SETTLE_MS         (60)     /* let the servo move first */
/* Coarse scan from the brief: 30, 60, 90, 120, 150 deg, then back. */
#define APP_SERVO_STEP_DEG          (30u)
#define APP_SERVO_MIN_DEG           (30u)
#define APP_SERVO_MAX_DEG           (150u)

#define APP_TELEMETRY_TICK_MS       (1000)
#define APP_TELEMETRY_EVERY_N       (3u)     /* rover/telemetry every 3 s */
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

uint32_t app_now_ms(void)
{
    SYSTIM now;

    (void)tk_get_otm(&now);
    return (uint32_t)now.lo;
}

static bool app_check(const char *p_what, ER result)
{
    if (result < E_OK)
    {
        tm_printf((UB *)"[init] %s FAILED, error %d\n", p_what, (INT)result);
        return false;
    }
    return true;
}

/* Start-up failed: keep repeating the error instead of returning from
 * usermain() (which would shut the kernel down silently). */
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

static void app_wake_handler(void *p_exinf)
{
    /* exinf holds a pointer to the task id to wake. */
    (void)tk_wup_tsk(*(ID *)p_exinf);
}

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

/* ------------------------------------------------------------------ *
 *  Sensors: IMU every cycle, ultrasonic scan every few cycles
 * ------------------------------------------------------------------ */

static void sensor_task(INT stacd, void *p_exinf)
{
    uint32_t cycle = 0u;
    uint8_t  angle = APP_SERVO_MIN_DEG;
    int32_t  step  = (int32_t)APP_SERVO_STEP_DEG;

    (void)stacd;
    (void)p_exinf;
    while (1)
    {
        buddy4_update(app_now_ms());

        if (0u == (cycle % APP_SONAR_EVERY_N_CYCLES))
        {
            buddy5_set_servo_angle(angle);
            (void)tk_dly_tsk(APP_SERVO_SETTLE_MS);
            (void)buddy5_measure_cm(app_now_ms());
            if (((step > 0) && (angle >= APP_SERVO_MAX_DEG)) ||
                ((step < 0) && (angle <= APP_SERVO_MIN_DEG)))
            {
                step = -step;       /* sweep back and forth */
            }
            angle = (uint8_t)((int32_t)angle + step);
        }
        cycle++;
        (void)tk_dly_tsk(APP_SENSOR_PERIOD_MS);
    }
}

/* ------------------------------------------------------------------ *
 *  Telemetry: console + MQTT
 * ------------------------------------------------------------------ */

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

static const char *app_color(bool black)
{
    return black ? "BLACK" : "white";
}

/* One easy-to-read line, e.g.
 * [status] idle | front L=white R=white | side=white | sonar 23 cm | heading 163.4 | pitch 1.2 */
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
}

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
    tm_printf((UB *)" Robot car (micro T-Kernel)\n");
    tm_printf((UB *)" Buttons:  GP20 = motion test   GP21 = line following on/off\n");
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
static T_CTSK g_ctsk_sensors     = APP_TASK(sensor_task,              APP_PRI_SENSORS);
static T_CTSK g_ctsk_telemetry   = APP_TASK(telemetry_task,           APP_PRI_TELEMETRY);
static T_CTSK g_ctsk_motion_test = APP_TASK(buddy2_motion_test_task,  APP_PRI_MOTION_TEST);
static T_CTSK g_ctsk_blink       = APP_TASK(blink_task,               APP_PRI_BLINK);

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

/* Create and start a task; stop everything if it fails. */
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

EXPORT INT usermain(void)
{
    /* 1. Hardware and subsystems. */
    hal_init();
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
    hal_gpio_input_pullup(BUTTON_TEST_PIN);
    buddy3_init();
    buddy5_init();
    robot_controller_init();
    if (!buddy4_init())
    {
        /* Not fatal: telemetry will show "compass_error". */
        tm_printf((UB *)"[init] IMU (LSM303) not answering on I2C - check Grove 3.\n");
    }

    /* 2. Tasks.  The cyclic handlers are created after their tasks exist. */
    (void)app_start_task("blink task", &g_ctsk_blink);
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
    (void)app_start_task("sensor task", &g_ctsk_sensors);
    (void)app_start_task("telemetry task", &g_ctsk_telemetry);
    (void)app_start_task("motion test task", &g_ctsk_motion_test);

    /* usermain must never return while the system is running. */
    (void)tk_slp_tsk(TMO_FEVR);
    return 0;
}
