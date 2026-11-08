/**
 * @file    buddy4_imu.c
 * @brief   Buddy 4: LSM303DLHC accelerometer + compass (IMU) readings.
 *
 * The compass part is the team's original code (same registers and
 * settings), moved onto robot_hal.c.  Tilt filtering, hump detection and
 * motion classification are in buddy4_processing.c.  Physical calibration
 * and known-height track validation are still required.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "app_main.h"
#include "buddy2_motion.h"
#include "buddy4_imu.h"
#include "comms.h"
#include "pin_config.h"
#include "robot_hal.h"

/* LSM303DLHC compass registers (team's original settings). */
#define LSM303_MAG_CRA_REG       (0x00u)
#define LSM303_MAG_CRA_15HZ      (0x10u)
#define LSM303_MAG_MR_REG        (0x02u)
#define LSM303_MAG_MR_CONTINUOUS (0x00u)
#define LSM303_MAG_OUT_X_H       (0x03u)

/* LSM303DLHC accelerometer registers. */
#define LSM303_ACC_CTRL_REG1  (0x20u)
#define LSM303_ACC_100HZ_XYZ  (0x57u) /* 100 Hz, X/Y/Z enabled        */
#define LSM303_ACC_CTRL_REG4  (0x23u)
#define LSM303_ACC_HR_2G      (0x88u) /* BDU, high resolution, +/-2 g */
#define LSM303_ACC_OUT_X_L    (0x28u)
#define LSM303_AUTO_INCREMENT (0x80u) /* multi-byte read              */

#define BUDDY4_CONFIG_MS (1000u)
#define BUDDY4_REPORT_MS (1000u)

static buddy4_imu_state_t   g_state;
static buddy4_processor_t   g_processor;
static buddy4_calibration_t g_pending_cal;
static bool                 g_cal_pending      = false;
static bool                 g_reset_pending    = false;
static bool                 g_stats_cleared    = false; /* IMU task only */
static bool                 g_mag_configured   = false;
static bool                 g_accel_configured = false;
static uint32_t             g_config_ms        = 0u;

static int16_t buddy4_be16 (const uint8_t * p_bytes)
{
    return (int16_t) (((uint16_t) p_bytes[0] << 8) | (uint16_t) p_bytes[1]);
}

static int16_t buddy4_le16 (const uint8_t * p_bytes)
{
    return (int16_t) (((uint16_t) p_bytes[1] << 8) | (uint16_t) p_bytes[0]);
}

/* The LSM303 is mounted turned 90 degrees: tilting the nose moves the
 * sensor's Y axis and tilting sideways moves X (checked by tilting the
 * rover).  Rotate both sensors into the axes the processing expects, X
 * along the rover (negative = nose up) and Y across it.  A rotation, not a
 * plain swap, so the compass still turns the same way. */
static void buddy4_to_rover_axes (int16_t axes[3])
{
    int16_t sensor_x = axes[0];

    axes[0] = axes[1];
    axes[1] = (int16_t) -sensor_x;
}

static bool buddy4_write_reg (uint8_t address, uint8_t reg, uint8_t value)
{
    uint8_t buffer[2];

    buffer[0] = reg;
    buffer[1] = value;
    return (HAL_I2C_OK ==
            hal_i2c_write(IMU_I2C_PORT, address, buffer, 2u, false));
}

static bool buddy4_read_regs (uint8_t address, uint8_t reg, uint8_t * p_buffer,
                              uint32_t length)
{
    if (HAL_I2C_OK != hal_i2c_write(IMU_I2C_PORT, address, &reg, 1u, true))
    {
        return false;
    }
    return (HAL_I2C_OK ==
            hal_i2c_read(IMU_I2C_PORT, address, p_buffer, length, false));
}

static bool buddy4_configure_mag (void)
{
    return buddy4_write_reg(LSM303_MAG_ADDR, LSM303_MAG_CRA_REG,
                            LSM303_MAG_CRA_15HZ) &&
           buddy4_write_reg(LSM303_MAG_ADDR, LSM303_MAG_MR_REG,
                            LSM303_MAG_MR_CONTINUOUS);
}

static bool buddy4_configure_accel (void)
{
    return buddy4_write_reg(LSM303_ACCEL_ADDR, LSM303_ACC_CTRL_REG1,
                            LSM303_ACC_100HZ_XYZ) &&
           buddy4_write_reg(LSM303_ACCEL_ADDR, LSM303_ACC_CTRL_REG4,
                            LSM303_ACC_HR_2G);
}

bool buddy4_init (void)
{
    buddy4_processor_init(&g_processor, NULL);
    g_state = g_processor.state;
    if (!hal_i2c_init(IMU_I2C_PORT, IMU_I2C_BAUDRATE, IMU_SDA_PIN, IMU_SCL_PIN))
    {
        return false;
    }
    g_mag_configured   = buddy4_configure_mag();
    g_accel_configured = buddy4_configure_accel();
    return g_mag_configured && g_accel_configured;
}

void buddy4_update (uint32_t now_ms)
{
    uint8_t               buffer[6];
    int16_t               mag[3]   = {0, 0, 0};
    int16_t               accel[3] = {0, 0, 0};
    bool                  mag_read;
    bool                  accel_read;
    bool                  odometry_fresh;
    bool                  forward;
    bool                  apply_cal;
    bool                  apply_reset;
    buddy2_motion_state_t motion;
    buddy4_calibration_t  cal;
    UINT                  interrupt_state;
    uint32_t              index;

    DI(interrupt_state);
    apply_cal       = g_cal_pending;
    apply_reset     = g_reset_pending;
    cal             = g_pending_cal;
    g_cal_pending   = false;
    g_reset_pending = false;
    EI(interrupt_state);
    if (apply_cal)
    {
        buddy4_processor_init(&g_processor, &cal);
        g_stats_cleared = true;
    }
    else if (apply_reset)
    {
        /* New run: same calibration, everything else from scratch. */
        cal = g_processor.calibration;
        buddy4_processor_init(&g_processor, &cal);
        g_stats_cleared = true;
    }
    else
    {
        /* nothing queued */
    }

    /* Re-send the configuration every second, not only after a failed read:
     * the compass was seen stuck in its sleep default (even from boot) while
     * reads still succeeded, which froze the heading.  Stats are kept. */
    if ((now_ms - g_config_ms) >= BUDDY4_CONFIG_MS)
    {
        g_config_ms        = now_ms;
        g_mag_configured   = buddy4_configure_mag();
        g_accel_configured = buddy4_configure_accel();
    }

    mag_read =
        g_mag_configured &&
        buddy4_read_regs(LSM303_MAG_ADDR, LSM303_MAG_OUT_X_H, buffer, 6u);
    if (mag_read)
    {
        /* Register order is X, Z, Y. */
        mag[0] = buddy4_be16(&buffer[0]);
        mag[2] = buddy4_be16(&buffer[2]);
        mag[1] = buddy4_be16(&buffer[4]);
        buddy4_to_rover_axes(mag); /* overflow -4096 stays out of range */
    }
    else
    {
        g_mag_configured = false;
    }

    accel_read = g_accel_configured &&
                 buddy4_read_regs(LSM303_ACCEL_ADDR,
                                  LSM303_ACC_OUT_X_L | LSM303_AUTO_INCREMENT,
                                  buffer, 6u);
    if (accel_read)
    {
        for (index = 0u; index < 3u; index++)
        {
            /* 12-bit left-justified, 1 mg per count at +/-2 g. */
            accel[index] = (int16_t) (buddy4_le16(&buffer[2u * index]) / 16);
        }
        buddy4_to_rover_axes(accel);
    }
    else
    {
        g_accel_configured = false;
    }

    buddy2_get_motion_state(&motion);
    now_ms = app_now_ms();
    /* Buddy 2 signs odometry using commanded direction; slip and externally
     * pushed wheels still require physical validation. */
    odometry_fresh = ((now_ms - motion.timestamp_ms) <= BUDDY4_STALE_MS);
    forward        = odometry_fresh && (motion.left_speed_mm_s > 0.0f) &&
                     (motion.right_speed_mm_s > 0.0f);
    buddy4_process(&g_processor, now_ms, mag_read, mag, accel_read, accel,
                   (motion.left_distance_mm + motion.right_distance_mm) * 0.5f,
                   (motion.right_distance_mm - motion.left_distance_mm) * 0.5f,
                   odometry_fresh, forward);

    DI(interrupt_state);
    g_state = g_processor.state;
    EI(interrupt_state);
}

void buddy4_get_state (buddy4_imu_state_t * p_state)
{
    UINT     interrupt_state;
    uint32_t now_ms;

    if (NULL == p_state)
    {
        return;
    }
    DI(interrupt_state);
    *p_state = g_state;
    EI(interrupt_state);

    now_ms = app_now_ms();
    if ((now_ms - p_state->mag_timestamp_ms) > BUDDY4_STALE_MS)
    {
        p_state->mag_ok = 0u;
    }
    if ((now_ms - p_state->accel_timestamp_ms) > BUDDY4_STALE_MS)
    {
        p_state->accel_ok = 0u;
    }
    p_state->imu_fault =
        ((0u == p_state->mag_ok) || (0u == p_state->accel_ok)) ? 1u : 0u;
}

bool buddy4_set_calibration (const buddy4_calibration_t * p_cal)
{
    UINT     interrupt_state;
    uint32_t index;

    if ((NULL == p_cal) || (p_cal->pitch_zero_x10 < -900) ||
        (p_cal->pitch_zero_x10 > 900) || (p_cal->roll_zero_x10 < -1800) ||
        (p_cal->roll_zero_x10 > 1800))
    {
        return false;
    }
    for (index = 0u; index < 3u; index++)
    {
        /* Comparisons also reject NaN and infinities. */
        if (!((p_cal->mag_scale[index] >= 0.01f) &&
              (p_cal->mag_scale[index] <= 100.0f)) ||
            !((p_cal->accel_bias_mg[index] >= -2000.0f) &&
              (p_cal->accel_bias_mg[index] <= 2000.0f)) ||
            !((p_cal->mag_bias[index] >= -4095.0f) &&
              (p_cal->mag_bias[index] <= 4095.0f)))
        {
            return false;
        }
    }
    DI(interrupt_state);
    g_pending_cal = *p_cal;
    g_cal_pending = true;
    EI(interrupt_state);
    return true;
}

void buddy4_reset_run (void)
{
    UINT interrupt_state;

    DI(interrupt_state);
    g_reset_pending = true;
    EI(interrupt_state);
}

/* Publish the IMU/hump report: periodically, and at once after an event. */
static void buddy4_report (const buddy4_imu_state_t * p_state)
{
    static const char digits[] = "0123456789abcdef";
    char              message[COMMS_PAYLOAD_MAX];
    char              hex[(BUDDY4_PACKET_SIZE * 2u) + 1u];
    uint8_t           packet[BUDDY4_PACKET_SIZE];
    uint32_t          index;

    (void) buddy4_pack(p_state, packet);
    for (index = 0u; index < BUDDY4_PACKET_SIZE; index++)
    {
        hex[2u * index]        = digits[packet[index] >> 4];
        hex[(2u * index) + 1u] = digits[packet[index] & 0x0Fu];
    }
    hex[BUDDY4_PACKET_SIZE * 2u] = '\0';

    /* Separate small JSON report: keeps the main telemetry message unchanged.
     * The sequence numbers repeat in every report, so a dropped one is
     * recovered by the next. */
    (void) tm_sprintf(
        (UB *) message,
        (UB *) "{\"imu_fault\":%u,\"imu_calibrated\":%u,\"roll_x10\":%d,"
               "\"hump_state\":%u,\"hump_count\":%u,\"hump_seq\":%u,"
               "\"hump_peak_x10\":%d,\"highest_peak_x10\":%d,"
               "\"hump_height_mm\":%u,\"height_valid\":%u,\"highest_hump_mm\":%"
               "u,"
               "\"motion\":%u,\"turn_rate_x10\":%d,\"impacts\":%u,"
               "\"imu_interval_ms\":%u,\"imu_packed\":\"%s\"}",
        (UINT) p_state->imu_fault, (UINT) p_state->calibrated,
        (INT) p_state->roll_deg_x10, (UINT) p_state->hump_state,
        (UINT) p_state->hump_count, (UINT) p_state->hump_sequence,
        (INT) p_state->hump_peak_deg_x10, (INT) p_state->highest_peak_deg_x10,
        (UINT) p_state->hump_height_mm, (UINT) p_state->height_valid,
        (UINT) p_state->highest_hump_mm, (UINT) p_state->motion_class,
        (INT) p_state->turn_rate_dps_x10, (UINT) p_state->impact_count,
        (UINT) p_state->sample_interval_ms, hex);
    (void) comms_publish(COMMS_TOPIC_TELEMETRY, message);
}

void buddy4_task (int32_t stacd, void * p_exinf)
{
    uint32_t reported_hump   = 0u;
    uint32_t reported_impact = 0u;
    uint32_t last_report_ms  = 0u;

    (void) stacd;
    (void) p_exinf;
    for (;;)
    {
        uint32_t           start_ms = app_now_ms();
        uint32_t           elapsed_ms;
        buddy4_imu_state_t state;

        buddy4_update(start_ms);
        buddy4_get_state(&state);
        if (g_stats_cleared)
        {
            /* Counters went back to 0: not a new hump or impact.  Report
             * at once so the dashboard clears. */
            g_stats_cleared = false;
            reported_hump   = state.hump_sequence;
            reported_impact = state.impact_count;
            last_report_ms  = start_ms - BUDDY4_REPORT_MS;
            tm_printf(
                (UB *) "[imu] run reset: humps, peaks and impacts cleared\n");
        }
        if ((state.hump_sequence != reported_hump) ||
            (state.impact_count != reported_impact) ||
            ((start_ms - last_report_ms) >= BUDDY4_REPORT_MS))
        {
            buddy4_report(&state);
            if (state.hump_sequence != reported_hump)
            {
                tm_printf(
                    (UB *) "[hump] seq=%u peak=%d height=%u mm valid=%u\n",
                    (UINT) state.hump_sequence, (INT) state.hump_peak_deg_x10,
                    (UINT) state.hump_height_mm, (UINT) state.height_valid);
            }
            if (state.impact_count != reported_impact)
            {
                tm_printf((UB *) "[imu] sudden impact, count=%u\n",
                          (UINT) state.impact_count);
            }
            reported_hump   = state.hump_sequence;
            reported_impact = state.impact_count;
            last_report_ms  = start_ms;
        }
        elapsed_ms = app_now_ms() - start_ms;
        (void) tk_dly_tsk((elapsed_ms < BUDDY4_PERIOD_MS)
                              ? (TMO) (BUDDY4_PERIOD_MS - elapsed_ms)
                              : 1);
    }
}

/*** end of file ***/

