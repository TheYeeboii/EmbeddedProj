/**
 * @file    subsystem1.c
 * @brief   Subsystem 1 (Wi-Fi, MQTT bridge, telemetry): start-up and the
 *          telemetry task.
 *
 * The MQTT client itself runs inside the kernel's Wi-Fi service task
 * (lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c).  Application tasks only
 * use the message buffers in comms.c.  The telemetry task publishes the
 * dashboard messages and prints the USB status line.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "app.h"
#include "subsystem1.h"
#include "subsystem2.h"
#include "subsystem3.h"
#include "subsystem4.h"
#include "subsystem5.h"
#include "pin_config.h"
#include "robot_controller.h"
#include "robot_hal.h"
#include "usb_console_compat.h"

#define APP_TELEMETRY_TICK_MS (1000)
#define APP_TELEMETRY_EVERY_N (1u)  /* rover/telemetry every 1 s */
#define APP_HEARTBEAT_EVERY_N (10u) /* rover/heartbeat every 10 s */
#define APP_USB_WAIT_MS       (3000)
#define APP_USB_POLL_MS       (100)
#define APP_USB_ENUMERATED    (2u) /* tm_usb_state() once enumerated */
#define APP_STATUS_EVERY_N    (2u) /* [status] line every 2 s   */

/* ------------------------------------------------------------------ *
 *  Telemetry: console + MQTT
 * ------------------------------------------------------------------ */

/** @brief Convert a communications-link state to console text. */
static const char * app_link_name (uint32_t state)
{
    const char * p_name = "offline";

    if (COMMS_LINK_MQTT_UP == state)
    {
        p_name = "MQTT connected";
    }
    else if (COMMS_LINK_WIFI_UP == state)
    {
        p_name = "Wi-Fi connected, looking for the MQTT broker...";
    }
    else
    {
        p_name = "offline";
    }

    return p_name;
}

/** @brief Convert a line-sensor Boolean reading to the status-line colour
 * label. */
static const char * app_color (bool black)
{
    return black ? "BLACK" : "white";
}

/**
 * @brief Print one human-readable line of current robot status to USB.
 *
 * Example: [status] idle | front L=white R=white | side=white |
 * sonar 23 cm | heading 163.4 | pitch 1.2
 */
static void app_print_status (void)
{
    buddy4_imu_state_t imu     = {0};
    buddy5_reading_t   sonar   = {0};
    bool               b_left  = false;
    bool               b_right = false;
    bool               b_side  = false;
    const char *       p_sign  = "";
    int32_t            pitch   = 0;

    buddy3_get_colors(&b_left, &b_right, &b_side);
    buddy4_get_state(&imu);
    buddy5_get_last(&sonar);

    pitch = imu.pitch_deg_x10;
    if (pitch < 0)
    {
        p_sign = "-";
        pitch  = -pitch;
    }

    tm_printf((UB *) "[status] %-6s | front L=%s R=%s | side=%s | ",
              robot_controller_mode_name(), app_color(b_left),
              app_color(b_right), app_color(b_side));
    if (BUDDY5_NO_ECHO == sonar.distance_cm)
    {
        tm_printf((UB *) "sonar --    | ");
    }
    else
    {
        tm_printf((UB *) "sonar %u cm | ", (UINT) sonar.distance_cm);
    }
    if (imu.mag_ok)
    {
        tm_printf((UB *) "heading %u.%u | pitch %s%u.%u\n",
                  (UINT) (imu.heading_deg_x10 / 10),
                  (UINT) (imu.heading_deg_x10 % 10), p_sign,
                  (UINT) (pitch / 10), (UINT) (pitch % 10));
    }
    else
    {
        tm_printf((UB *) "IMU not answering\n");
    }

    /* Wiring check: which pins this build reads, and the live tick counts. */
    {
        buddy2_motion_state_t motion = {0};

        buddy2_get_motion_state(&motion);
        tm_printf((UB *) "[encoders] left GP%u = %d ticks %s | right GP%u = %d "
                         "ticks %s | build %s %s\n",
                  (UINT) ENCODER_LEFT_A, (INT) motion.left_ticks,
                  motion.left_encoder_fault ? "NO TICKS!" : "ok",
                  (UINT) ENCODER_RIGHT_A, (INT) motion.right_ticks,
                  motion.right_encoder_fault ? "NO TICKS!" : "ok", __DATE__,
                  __TIME__);
        if (motion.left_encoder_fault || motion.right_encoder_fault)
        {
            tm_printf((UB *) "[encoders] FAULT: a wheel is powered but its "
                             "encoder gives no ticks "
                             "(check its wiring, or the wheel is blocked)\n");
        }
    }
}

/**
 * @brief Print the start-up banner on the USB console.
 *
 * Waits up to APP_USB_WAIT_MS for the console to enumerate first, so the
 * banner is not lost.
 */
static void app_print_banner (void)
{
    INT wait_step = 0;

    for (wait_step = 0; (wait_step < (APP_USB_WAIT_MS / APP_USB_POLL_MS)) &&
                        (tm_usb_state() < APP_USB_ENUMERATED);
         wait_step++)
    {
        (void) tk_dly_tsk(APP_USB_POLL_MS);
    }
    tm_printf((
        UB *) "\n==========================================================\n");
    tm_printf((UB *) " Robot car (micro T-Kernel), built %s %s\n", __DATE__,
              __TIME__);
    tm_printf((UB *) " Encoders: left GP%u, right GP%u\n",
              (UINT) ENCODER_LEFT_A, (UINT) ENCODER_RIGHT_A);
    tm_printf((UB *) " Buttons:  GP21 = line following on/off\n");
#if defined(TM_WIFI_MQTT) && (TM_WIFI_MQTT == 1)
    tm_printf((UB *) " Wi-Fi:    on (dashboard)\n");
#else
    tm_printf((UB *) " Wi-Fi:    not in this build\n");
#endif
    tm_printf((UB *) " Diagnostic streams: on (rover/actions, steering, "
                     "barcode, ir)\n");
    tm_printf((UB *) " [status] every %u s: front L/R = line sensors, side "
                     "= barcode\n",
              (UINT) ((APP_TELEMETRY_TICK_MS * APP_STATUS_EVERY_N) / 1000));
    tm_printf((UB *) "          sensor (BLACK = line, or nothing under it)\n");
    tm_printf(
        (UB *) "==========================================================\n");
}

/**
 * @brief Publish dashboard telemetry and print periodic USB status.
 *
 * Debug messages, full telemetry, and the MQTT heartbeat have independent rates
 * controlled by the APP_TELEMETRY_* configuration constants.
 */
static void telemetry_task (INT stacd, void * p_exinf)
{
    static uint8_t buffer[ROBOT_TELEMETRY_BUFFER_SIZE];
    uint32_t       length    = 0u;
    uint32_t       tick      = 0u;
    uint32_t       last_link = 0xFFFFFFFFu;

    (void) stacd;
    (void) p_exinf;

    app_print_banner();

    for (;;)
    {
        uint32_t link = comms_get_link_state();

#if defined(TM_WIFI_MQTT) && (TM_WIFI_MQTT == 1)
        if (link != last_link)
        {
            tm_printf((UB *) "[wifi] %s\n", app_link_name(link));
            last_link = link;
        }
#else
        (void) link;
        (void) last_link;
        (void) app_link_name;
#endif

        /* Human-readable status for the USB console. */
        if (0u == (tick % APP_STATUS_EVERY_N))
        {
            app_print_status();
        }

        /* Packed messages for the dashboard (MQTT).  A length of 0 (the
         * message did not fit) is refused by comms_publish(). */
        length = robot_telemetry_build_debug(buffer, sizeof(buffer));
        (void) comms_publish(COMMS_TOPIC_DEBUG, buffer, length);
        if (0u == (tick % APP_TELEMETRY_EVERY_N))
        {
            length =
                robot_telemetry_build(buffer, sizeof(buffer), app_now_ms());
            (void) comms_publish(COMMS_TOPIC_TELEMETRY, buffer, length);
        }
        if (0u == (tick % APP_HEARTBEAT_EVERY_N))
        {
            length = robot_telemetry_build_heartbeat(buffer, sizeof(buffer));
            (void) comms_publish(COMMS_TOPIC_HEARTBEAT, buffer, length);
        }

        tick++;
        (void) tk_dly_tsk(APP_TELEMETRY_TICK_MS);
    }
}

/* ------------------------------------------------------------------ *
 *  Start-up
 * ------------------------------------------------------------------ */

void subsystem1_init (void)
{
    if (!app_check("comms_init", (INT) comms_init()))
    {
        app_fail_forever("comms_init");
    }
    robot_diagnostics_init(hal_diagnostic_boot_id());
}

void subsystem1_start (void)
{
    (void) app_start_task("telemetry task", telemetry_task,
                          APP_PRI_TELEMETRY);
}

/*** end of file ***/
