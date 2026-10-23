/**
 * @file    buddy4_imu.c
 * @brief   Buddy 4: LSM303DLHC accelerometer + compass (IMU) readings.
 *
 * The compass part is the team's original code (same registers and
 * settings), moved onto robot_hal.c.  The accelerometer part and the tilt
 * filtering and hump processing are implemented in buddy4_processing.c.
 * Physical calibration and known-height track validation are still required.
 */

#include <tk/tkernel.h>

#include "app_main.h"
#include "buddy2_motion.h"
#include "buddy4_imu.h"
#include "comms.h"
#include "pin_config.h"
#include "robot_hal.h"
#include <tm/tmonitor.h>

/* LSM303DLHC compass registers (team's original settings). */
#define LSM303_MAG_CRA_REG (0x00u)
#define LSM303_MAG_CRA_15HZ (0x10u)
#define LSM303_MAG_MR_REG (0x02u)
#define LSM303_MAG_MR_CONTINUOUS (0x00u)
#define LSM303_MAG_OUT_X_H (0x03u)

/* LSM303DLHC accelerometer registers. */
#define LSM303_ACC_CTRL_REG1 (0x20u)
#define LSM303_ACC_100HZ_XYZ (0x57u) /* 100 Hz, X/Y/Z enabled        */
#define LSM303_ACC_CTRL_REG4 (0x23u)
#define LSM303_ACC_HR_2G (0x88u) /* high resolution, +/-2 g      */
#define LSM303_ACC_OUT_X_L (0x28u)
#define LSM303_AUTO_INCREMENT (0x80u) /* multi-byte read              */

#define BUDDY4_RAD_TO_DEG_X10 (572.957795f) /* 180/pi * 10 */

static buddy4_imu_state_t g_state;
static buddy4_processor_t g_processor;
static buddy4_calibration_t g_pending_cal;
static bool g_cal_pending, g_reset_pending;
static bool g_mag_configured, g_accel_configured;
static uint32_t g_retry_ms;

static int16_t buddy4_be16(const uint8_t *p_bytes)
{
    return (int16_t)(((uint16_t)p_bytes[0] << 8) | (uint16_t)p_bytes[1]);
}

static int16_t buddy4_le16(const uint8_t *p_bytes)
{
    return (int16_t)(((uint16_t)p_bytes[1] << 8) | (uint16_t)p_bytes[0]);
}

static bool buddy4_write_reg(uint8_t address, uint8_t reg, uint8_t value)
{
    uint8_t buffer[2];

    buffer[0] = reg;
    buffer[1] = value;
    return (HAL_I2C_OK == hal_i2c_write(IMU_I2C_PORT, address, buffer, 2u, false));
}

static bool buddy4_read_regs(uint8_t address, uint8_t reg, uint8_t *p_buffer, uint32_t length)
{
    if (HAL_I2C_OK != hal_i2c_write(IMU_I2C_PORT, address, &reg, 1u, true))
    {
        return false;
    }
    return (HAL_I2C_OK == hal_i2c_read(IMU_I2C_PORT, address, p_buffer, length, false));
}

bool buddy4_init(void)
{
    bool mag_ok;
    bool accel_ok;

    buddy4_processor_init(&g_processor, NULL);
    g_state = g_processor.state;
    if (!hal_i2c_init(IMU_I2C_PORT, IMU_I2C_BAUDRATE, IMU_SDA_PIN, IMU_SCL_PIN))
        return false;

    mag_ok = buddy4_write_reg(LSM303_MAG_ADDR, LSM303_MAG_CRA_REG, LSM303_MAG_CRA_15HZ) &&
             buddy4_write_reg(LSM303_MAG_ADDR, LSM303_MAG_MR_REG, LSM303_MAG_MR_CONTINUOUS);

    accel_ok = buddy4_write_reg(LSM303_ACCEL_ADDR, LSM303_ACC_CTRL_REG1, LSM303_ACC_100HZ_XYZ) &&
               buddy4_write_reg(LSM303_ACCEL_ADDR, LSM303_ACC_CTRL_REG4, LSM303_ACC_HR_2G);

    g_mag_configured = mag_ok;
    g_accel_configured = accel_ok;
    return mag_ok && accel_ok;
}

void buddy4_update(uint32_t now_ms)
{
    uint8_t buffer[6];
    int16_t mag[3] = {0}, accel[3] = {0};
    bool mok, aok, forward;
    buddy2_motion_state_t motion;
    UINT irq;
    unsigned i;
    bool apply_cal, reset_peak;
    buddy4_calibration_t cal;
    DI(irq);
    apply_cal = g_cal_pending;
    reset_peak = g_reset_pending;
    cal = g_pending_cal;
    g_cal_pending = false;
    g_reset_pending = false;
    EI(irq);
    if (apply_cal)
        buddy4_processor_init(&g_processor, &cal);
    if (reset_peak)
        g_processor.state.max_pitch_deg_x10 = 0;
    /* Retry configuration after a disconnected sensor, without resetting stats. */
    if ((!g_mag_configured || !g_accel_configured) && now_ms - g_retry_ms >= 1000u)
    {
        g_retry_ms = now_ms;
        g_mag_configured =
            buddy4_write_reg(LSM303_MAG_ADDR, LSM303_MAG_CRA_REG, LSM303_MAG_CRA_15HZ) &&
            buddy4_write_reg(LSM303_MAG_ADDR, LSM303_MAG_MR_REG, LSM303_MAG_MR_CONTINUOUS);
        g_accel_configured =
            buddy4_write_reg(LSM303_ACCEL_ADDR, LSM303_ACC_CTRL_REG1, LSM303_ACC_100HZ_XYZ) &&
            buddy4_write_reg(LSM303_ACCEL_ADDR, LSM303_ACC_CTRL_REG4, LSM303_ACC_HR_2G);
    }
    mok = g_mag_configured && buddy4_read_regs(LSM303_MAG_ADDR, LSM303_MAG_OUT_X_H, buffer, 6u);
    if (mok)
    {
        mag[0] = buddy4_be16(buffer);
        mag[2] = buddy4_be16(buffer + 2);
        mag[1] = buddy4_be16(buffer + 4);
    }
    else
        g_mag_configured = false;
    aok =
        g_accel_configured &&
        buddy4_read_regs(LSM303_ACCEL_ADDR, LSM303_ACC_OUT_X_L | LSM303_AUTO_INCREMENT, buffer, 6u);
    if (aok)
        for (i = 0; i < 3; i++)
            accel[i] = (int16_t)(buddy4_le16(buffer + 2 * i) / 16);
    else
        g_accel_configured = false;
    buddy2_get_motion_state(&motion);
    now_ms = app_now_ms();
    /* Buddy 2 signs odometry using commanded direction; slip and externally
     * pushed wheels still require physical validation. */
    forward = now_ms - motion.timestamp_ms <= BUDDY4_STALE_MS && motion.left_speed_mm_s > 0 &&
              motion.right_speed_mm_s > 0;
    buddy4_process(&g_processor, now_ms, mok, mag, aok, accel,
                   (motion.left_distance_mm + motion.right_distance_mm) * 0.5f, forward);
    DI(irq);
    g_state = g_processor.state;
    EI(irq);
}

void buddy4_get_state(buddy4_imu_state_t *p_state)
{
    UINT interrupt_state;

    if (NULL != p_state)
    {
        DI(interrupt_state);
        *p_state = g_state;
        EI(interrupt_state);
        {
            uint32_t now = app_now_ms();
            if (now - p_state->mag_timestamp_ms > BUDDY4_STALE_MS)
                p_state->mag_ok = false;
            if (now - p_state->accel_timestamp_ms > BUDDY4_STALE_MS)
                p_state->accel_ok = false;
            p_state->imu_fault = !p_state->mag_ok || !p_state->accel_ok;
        }
    }
}

void buddy4_reset_max_pitch(void)
{
    UINT irq;
    DI(irq);
    g_reset_pending = true;
    EI(irq);
}

bool buddy4_set_calibration(const buddy4_calibration_t *cal)
{
    UINT irq;
    unsigned i;
    if (!cal || cal->pitch_zero_x10 < -900 || cal->pitch_zero_x10 > 900 ||
        cal->roll_zero_x10 < -1800 || cal->roll_zero_x10 > 1800)
        return false;
    for (i = 0; i < 3; i++)
    {
        /* Comparisons also reject NaN and infinities. */
        if (!(cal->mag_scale[i] >= 0.01f && cal->mag_scale[i] <= 100.0f) ||
            !(cal->accel_bias_mg[i] >= -2000 && cal->accel_bias_mg[i] <= 2000) ||
            !(cal->mag_bias[i] >= -4095 && cal->mag_bias[i] <= 4095))
            return false;
    }
    DI(irq);
    g_pending_cal = *cal;
    g_cal_pending = true;
    EI(irq);
    return true;
}

void buddy4_task(int32_t stacd, void *exinf)
{
    uint32_t reported = 0, last_report = 0;
    char message[384];
    uint8_t packet[BUDDY4_PACKET_SIZE];
    char hex[BUDDY4_PACKET_SIZE * 2 + 1];
    unsigned i;
    (void)stacd;
    (void)exinf;
    while (1)
    {
        uint32_t start = app_now_ms(), elapsed;
        buddy4_imu_state_t s;
        buddy4_update(start);
        buddy4_get_state(&s);
        /* Separate small JSON report preserves existing dashboard protocol and
         * buffer limits. Sequence remains in every report if an event is lost. */
        if (s.hump_sequence != reported || start - last_report >= 1000u)
        {
            static const char digits[] = "0123456789abcdef";
            (void)buddy4_pack(&s, packet);
            for (i = 0; i < BUDDY4_PACKET_SIZE; i++)
            {
                hex[2 * i] = digits[packet[i] >> 4];
                hex[2 * i + 1] = digits[packet[i] & 15];
            }
            hex[BUDDY4_PACKET_SIZE * 2] = 0;
            (void)tm_sprintf(
                (UB *)message,
                (UB *)"{\"imu_fault\":%u,\"imu_calibrated\":%u,\"roll_x10\":%d,\"hump_state\":%u,"
                      "\"hump_count\":%u,\"hump_seq\":%u,\"hump_peak_x10\":%d,\"hump_height_mm\":%"
                      "u,"
                      "\"height_valid\":%u,\"highest_hump_mm\":%u,\"imu_interval_ms\":%u,\"imu_"
                      "packed\":\"%s\"}",
                (UINT)s.imu_fault, (UINT)s.calibrated, (INT)s.roll_deg_x10, (UINT)s.hump_state,
                (UINT)s.hump_count, (UINT)s.hump_sequence, (INT)s.hump_peak_deg_x10,
                (UINT)s.hump_height_mm, (UINT)s.height_valid, (UINT)s.highest_hump_mm,
                (UINT)s.sample_interval_ms, hex);
            (void)comms_publish(COMMS_TOPIC_TELEMETRY, message);
            /* Periodic sequence reporting recovers a dropped event without a
             * high-priority retry loop flooding the four-message outbox. */
            if (s.hump_sequence != reported)
                tm_printf((UB *)"[hump] seq=%u peak=%d height=%u mm valid=%u\n",
                          (UINT)s.hump_sequence, (INT)s.hump_peak_deg_x10, (UINT)s.hump_height_mm,
                          (UINT)s.height_valid);
            reported = s.hump_sequence;
            last_report = start;
        }
        elapsed = app_now_ms() - start;
        (void)tk_dly_tsk(elapsed < BUDDY4_PERIOD_MS ? BUDDY4_PERIOD_MS - elapsed : 1u);
    }
}
