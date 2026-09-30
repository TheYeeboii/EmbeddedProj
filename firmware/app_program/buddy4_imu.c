/**
 * @file    buddy4_imu.c
 * @brief   Buddy 4: LSM303DLHC accelerometer + compass (IMU) readings.
 *
 * The compass part is the team's original code (same registers and
 * settings), moved onto robot_hal.c.  The accelerometer part and the tilt
 * angles are new and are only a starting point for Buddy 4: calibration,
 * filtering, hump detection and height estimation still need to be done.
 */

#include <tk/tkernel.h>

/* From libm.  Declared here instead of including <math.h>, which clashes
 * with the kernel's size_t definition. */
extern float atan2f(float y, float x);
extern float sqrtf(float x);

#include "buddy4_imu.h"
#include "pin_config.h"
#include "robot_hal.h"

/* LSM303DLHC compass registers (team's original settings). */
#define LSM303_MAG_CRA_REG        (0x00u)
#define LSM303_MAG_CRA_15HZ       (0x10u)
#define LSM303_MAG_MR_REG         (0x02u)
#define LSM303_MAG_MR_CONTINUOUS  (0x00u)
#define LSM303_MAG_OUT_X_H        (0x03u)

/* LSM303DLHC accelerometer registers. */
#define LSM303_ACC_CTRL_REG1      (0x20u)
#define LSM303_ACC_100HZ_XYZ      (0x57u)   /* 100 Hz, X/Y/Z enabled        */
#define LSM303_ACC_CTRL_REG4      (0x23u)
#define LSM303_ACC_HR_2G          (0x08u)   /* high resolution, +/-2 g      */
#define LSM303_ACC_OUT_X_L        (0x28u)
#define LSM303_AUTO_INCREMENT     (0x80u)   /* multi-byte read              */

#define BUDDY4_RAD_TO_DEG_X10     (572.957795f)   /* 180/pi * 10 */

static buddy4_imu_state_t g_state;

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

static bool buddy4_read_regs(uint8_t address, uint8_t reg,
                             uint8_t *p_buffer, uint32_t length)
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

    (void)hal_i2c_init(IMU_I2C_PORT, IMU_I2C_BAUDRATE, IMU_SDA_PIN, IMU_SCL_PIN);

    mag_ok = buddy4_write_reg(LSM303_MAG_ADDR, LSM303_MAG_CRA_REG, LSM303_MAG_CRA_15HZ) &&
             buddy4_write_reg(LSM303_MAG_ADDR, LSM303_MAG_MR_REG, LSM303_MAG_MR_CONTINUOUS);

    accel_ok = buddy4_write_reg(LSM303_ACCEL_ADDR, LSM303_ACC_CTRL_REG1, LSM303_ACC_100HZ_XYZ) &&
               buddy4_write_reg(LSM303_ACCEL_ADDR, LSM303_ACC_CTRL_REG4, LSM303_ACC_HR_2G);

    return mag_ok && accel_ok;
}

void buddy4_update(uint32_t now_ms)
{
    buddy4_imu_state_t next;
    uint8_t            buffer[6];
    UINT               interrupt_state;

    DI(interrupt_state);
    next = g_state;
    EI(interrupt_state);

    /* Compass: X, Z, Y order, big-endian (team's original read). */
    next.mag_ok = buddy4_read_regs(LSM303_MAG_ADDR, LSM303_MAG_OUT_X_H, buffer, 6u);
    if (next.mag_ok)
    {
        float heading;

        next.mag_raw[0] = buddy4_be16(&buffer[0]);
        next.mag_raw[2] = buddy4_be16(&buffer[2]);
        next.mag_raw[1] = buddy4_be16(&buffer[4]);

        heading = atan2f((float)next.mag_raw[1], (float)next.mag_raw[0]) * BUDDY4_RAD_TO_DEG_X10;
        if (heading < 0.0f)
        {
            heading += 3600.0f;
        }
        next.heading_deg_x10 = (int16_t)heading;
    }

    /* Accelerometer: X, Y, Z little-endian, 12-bit left-justified -> mg. */
    next.accel_ok = buddy4_read_regs(LSM303_ACCEL_ADDR,
                                     (uint8_t)(LSM303_ACC_OUT_X_L | LSM303_AUTO_INCREMENT),
                                     buffer, 6u);
    if (next.accel_ok)
    {
        float ax;
        float ay;
        float az;

        next.accel_mg[0] = (int16_t)(buddy4_le16(&buffer[0]) / 16);
        next.accel_mg[1] = (int16_t)(buddy4_le16(&buffer[2]) / 16);
        next.accel_mg[2] = (int16_t)(buddy4_le16(&buffer[4]) / 16);

        ax = (float)next.accel_mg[0];
        ay = (float)next.accel_mg[1];
        az = (float)next.accel_mg[2];
        next.pitch_deg_x10 = (int16_t)(atan2f(-ax, sqrtf((ay * ay) + (az * az))) * BUDDY4_RAD_TO_DEG_X10);
        next.roll_deg_x10  = (int16_t)(atan2f(ay, az) * BUDDY4_RAD_TO_DEG_X10);

        if (next.pitch_deg_x10 > next.max_pitch_deg_x10)
        {
            next.max_pitch_deg_x10 = next.pitch_deg_x10;
        }
    }

    next.timestamp_ms = now_ms;

    DI(interrupt_state);
    g_state = next;
    EI(interrupt_state);
}

void buddy4_get_state(buddy4_imu_state_t *p_state)
{
    UINT interrupt_state;

    if (NULL != p_state)
    {
        DI(interrupt_state);
        *p_state = g_state;
        EI(interrupt_state);
    }
}

void buddy4_reset_max_pitch(void)
{
    UINT interrupt_state;

    DI(interrupt_state);
    g_state.max_pitch_deg_x10 = g_state.pitch_deg_x10;
    EI(interrupt_state);
}
