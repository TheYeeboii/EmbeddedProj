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

#include "app.h"
#include "buddy2_motion.h"
#include "buddy3_ir.h"
#include "buddy4_imu.h"
#include "comms.h"
#include "pin_config.h"
#include "robot_fault.h"
#include "robot_hal.h"
#include "robot_wire.h"

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

/* Raw sensor blocks: three 16-bit axes. */
#define BUDDY4_AXES      (3u)
#define BUDDY4_RAW_BYTES (6u)
/* The accelerometer is 12-bit left-justified: divide out four bits. */
#define BUDDY4_ACCEL_SHIFT_DIV (16)

/* Calibration limits accepted by buddy4_set_calibration(). */
#define CAL_PITCH_ZERO_MAX (900)  /* 90.0 degrees */
#define CAL_ROLL_ZERO_MAX  (1800) /* 180.0 degrees */
#define CAL_MAG_SCALE_MIN  (0.01f)
#define CAL_MAG_SCALE_MAX  (100.0f)
#define CAL_ACCEL_BIAS_MAX (2000.0f) /* mg */
#define CAL_MAG_BIAS_MAX   (4095.0f) /* raw counts */

static buddy4_imu_state_t   g_state;
static buddy4_processor_t   g_processor;
static buddy4_calibration_t g_pending_cal;
static bool                 g_b_cal_pending      = false;
static bool                 g_b_reset_pending    = false;
static bool                 g_b_put_down_pending = false;
static bool                 g_b_stats_cleared    = false; /* IMU task only */
static bool                 g_b_mag_configured   = false;
static bool                 g_b_accel_configured = false;
static uint32_t             g_config_ms          = 0u;

static int16_t buddy4_be16 (const uint8_t * p_bytes);
static int16_t buddy4_le16 (const uint8_t * p_bytes);
static void    buddy4_to_rover_axes (int16_t axes[BUDDY4_AXES]);
static bool    buddy4_write_reg (uint8_t address, uint8_t reg, uint8_t value);
static bool buddy4_read_regs (uint8_t address, uint8_t reg, uint8_t * p_buffer,
                              uint32_t length);
static bool buddy4_configure_mag (void);
static bool buddy4_configure_accel (void);
static void buddy4_apply_pending (void);
static bool buddy4_read_mag (int16_t mag[BUDDY4_AXES]);
static bool buddy4_read_accel (int16_t accel[BUDDY4_AXES]);
static void buddy4_report (const buddy4_imu_state_t * p_state);
static void buddy4_report_fault (bool b_fault);

bool buddy4_init (void)
{
    buddy4_processor_init(&g_processor, NULL);
    g_state = g_processor.state;
    if (hal_i2c_init(IMU_I2C_PORT, IMU_I2C_BAUDRATE, IMU_SDA_PIN, IMU_SCL_PIN))
    {
        g_b_mag_configured   = buddy4_configure_mag();
        g_b_accel_configured = buddy4_configure_accel();
    }

    return (g_b_mag_configured && g_b_accel_configured);
}

void buddy4_update (uint32_t now_ms)
{
    int16_t               mag[BUDDY4_AXES]   = {0, 0, 0};
    int16_t               accel[BUDDY4_AXES] = {0, 0, 0};
    bool                  b_mag_read         = false;
    bool                  b_accel_read       = false;
    bool                  b_odometry_fresh   = false;
    bool                  b_forward          = false;
    bool                  b_left_black       = false;
    bool                  b_right_black      = false;
    bool                  b_side_black       = false;
    buddy2_motion_state_t motion             = {0};
    UINT                  interrupt_state    = 0u;

    buddy4_apply_pending();

    /* Re-send the configuration every second, not only after a failed read:
     * the compass was seen stuck in its sleep default (even from boot) while
     * reads still succeeded, which froze the heading.  Stats are kept. */
    if ((now_ms - g_config_ms) >= BUDDY4_CONFIG_MS)
    {
        g_config_ms          = now_ms;
        g_b_mag_configured   = buddy4_configure_mag();
        g_b_accel_configured = buddy4_configure_accel();
    }

    b_mag_read   = buddy4_read_mag(mag);
    b_accel_read = buddy4_read_accel(accel);

    buddy2_get_motion_state(&motion);
    now_ms = app_now_ms();
    /* Buddy 2 signs odometry using commanded direction; slip and externally
     * pushed wheels still require physical validation. */
    b_odometry_fresh = ((now_ms - motion.timestamp_ms) <= BUDDY4_STALE_MS);
    b_forward        = b_odometry_fresh && (motion.left_speed_mm_s > 0.0f) &&
                       (motion.right_speed_mm_s > 0.0f);
    /* Lift latch: lifted off the floor, all three IR sensors read black. */
    buddy3_get_colors(&b_left_black, &b_right_black, &b_side_black);
    buddy4_processor_set_floor(
        &g_processor, !(b_left_black && b_right_black && b_side_black));
    buddy4_process(&g_processor, now_ms, b_mag_read, mag, b_accel_read, accel,
                   (motion.left_distance_mm + motion.right_distance_mm) * 0.5f,
                   (motion.right_distance_mm - motion.left_distance_mm) * 0.5f,
                   b_odometry_fresh, b_forward);

    DI(interrupt_state);
    g_state = g_processor.state;
    EI(interrupt_state);
}

void buddy4_get_state (buddy4_imu_state_t * p_state)
{
    UINT     interrupt_state = 0u;
    uint32_t now_ms          = 0u;

    if (NULL != p_state)
    {
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
}

bool buddy4_set_calibration (const buddy4_calibration_t * p_cal)
{
    UINT     interrupt_state = 0u;
    uint32_t index           = 0u;
    bool     b_valid         = false;

    if ((NULL != p_cal) && (p_cal->pitch_zero_x10 >= -CAL_PITCH_ZERO_MAX) &&
        (p_cal->pitch_zero_x10 <= CAL_PITCH_ZERO_MAX) &&
        (p_cal->roll_zero_x10 >= -CAL_ROLL_ZERO_MAX) &&
        (p_cal->roll_zero_x10 <= CAL_ROLL_ZERO_MAX))
    {
        b_valid = true;
        for (index = 0u; b_valid && (index < BUDDY4_AXES); index++)
        {
            /* Comparisons also reject NaN and infinities. */
            if (!((p_cal->mag_scale[index] >= CAL_MAG_SCALE_MIN) &&
                  (p_cal->mag_scale[index] <= CAL_MAG_SCALE_MAX)) ||
                !((p_cal->accel_bias_mg[index] >= -CAL_ACCEL_BIAS_MAX) &&
                  (p_cal->accel_bias_mg[index] <= CAL_ACCEL_BIAS_MAX)) ||
                !((p_cal->mag_bias[index] >= -CAL_MAG_BIAS_MAX) &&
                  (p_cal->mag_bias[index] <= CAL_MAG_BIAS_MAX)))
            {
                b_valid = false;
            }
        }
    }

    if (b_valid)
    {
        DI(interrupt_state);
        g_pending_cal   = *p_cal;
        g_b_cal_pending = true;
        EI(interrupt_state);
    }

    return b_valid;
}

void buddy4_reset_run (void)
{
    UINT interrupt_state = 0u;

    DI(interrupt_state);
    g_b_reset_pending = true;
    EI(interrupt_state);
}

void buddy4_clear_lift (void)
{
    UINT interrupt_state = 0u;

    /* Clear the shared copy now so the controller's next check does not
     * stop the move it was just told to start; the IMU task clears its
     * own latch at the next update (an overturned rover reads lifted
     * again then). */
    DI(interrupt_state);
    g_b_put_down_pending = true;
    g_state.lifted       = 0u;
    EI(interrupt_state);
}

void buddy4_task (INT stacd, void * p_exinf)
{
    uint32_t reported_hump  = 0u;
    uint32_t reported_lift  = 0u;
    uint32_t last_report_ms = 0u;

    (void) stacd;
    (void) p_exinf;
    for (;;)
    {
        uint32_t           start_ms   = app_now_ms();
        uint32_t           elapsed_ms = 0u;
        buddy4_imu_state_t state      = {0};

        buddy4_update(start_ms);
        buddy4_get_state(&state);
        buddy4_report_fault(0u != state.imu_fault);
        if (g_b_stats_cleared)
        {
            /* Counters went back to 0: not a new hump or lift.  Report
             * at once so the dashboard clears. */
            g_b_stats_cleared = false;
            reported_hump     = state.hump_sequence;
            reported_lift     = state.lift_count;
            last_report_ms    = start_ms - BUDDY4_REPORT_MS;
            tm_printf(
                (UB *) "[imu] run reset: humps, peaks and lifts cleared\n");
        }
        if ((state.hump_sequence != reported_hump) ||
            (state.lift_count != reported_lift) ||
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
            if (state.lift_count != reported_lift)
            {
                tm_printf((UB *) "[imu] lifted or overturned, count=%u\n",
                          (UINT) state.lift_count);
            }
            reported_hump  = state.hump_sequence;
            reported_lift  = state.lift_count;
            last_report_ms = start_ms;
        }
        elapsed_ms = app_now_ms() - start_ms;
        /* The remaining delay is below BUDDY4_PERIOD_MS, so it fits RELTIM;
         * wait at least 1 ms so lower-priority tasks always run. */
        (void) tk_dly_tsk((elapsed_ms < BUDDY4_PERIOD_MS)
                              ? (RELTIM) (BUDDY4_PERIOD_MS - elapsed_ms)
                              : (RELTIM) 1u);
    }
}

static int16_t buddy4_be16 (const uint8_t * p_bytes)
{
    /* Two's-complement reinterpretation of the 16-bit register pair. */
    return (int16_t) (((uint16_t) p_bytes[0] << 8u) | (uint16_t) p_bytes[1]);
}

static int16_t buddy4_le16 (const uint8_t * p_bytes)
{
    /* Two's-complement reinterpretation of the 16-bit register pair. */
    return (int16_t) (((uint16_t) p_bytes[1] << 8u) | (uint16_t) p_bytes[0]);
}

/* The LSM303 is mounted turned 90 degrees: tilting the nose moves the
 * sensor's Y axis and tilting sideways moves X (checked by tilting the
 * rover).  Rotate both sensors into the axes the processing expects, X
 * along the rover (negative = nose up) and Y across it.  A rotation, not a
 * plain swap, so the compass still turns the same way. */
static void buddy4_to_rover_axes (int16_t axes[BUDDY4_AXES])
{
    int16_t sensor_x = axes[0];

    axes[0] = axes[1];
    /* Raw values stay within +/-4096, so the negation cannot overflow. */
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
    /* Write the register address without a stop, then read the data. */
    return (
        (HAL_I2C_OK == hal_i2c_write(IMU_I2C_PORT, address, &reg, 1u, true)) &&
        (HAL_I2C_OK ==
         hal_i2c_read(IMU_I2C_PORT, address, p_buffer, length, false)));
}

static bool buddy4_configure_mag (void)
{
    return (buddy4_write_reg(LSM303_MAG_ADDR, LSM303_MAG_CRA_REG,
                             LSM303_MAG_CRA_15HZ) &&
            buddy4_write_reg(LSM303_MAG_ADDR, LSM303_MAG_MR_REG,
                             LSM303_MAG_MR_CONTINUOUS));
}

static bool buddy4_configure_accel (void)
{
    return (buddy4_write_reg(LSM303_ACCEL_ADDR, LSM303_ACC_CTRL_REG1,
                             LSM303_ACC_100HZ_XYZ) &&
            buddy4_write_reg(LSM303_ACCEL_ADDR, LSM303_ACC_CTRL_REG4,
                             LSM303_ACC_HR_2G));
}

/* Take a queued calibration or run reset.  IMU task only. */
static void buddy4_apply_pending (void)
{
    buddy4_calibration_t cal             = {0};
    bool                 b_apply_cal     = false;
    bool                 b_apply_reset   = false;
    bool                 b_put_down      = false;
    UINT                 interrupt_state = 0u;

    DI(interrupt_state);
    b_apply_cal          = g_b_cal_pending;
    b_apply_reset        = g_b_reset_pending;
    cal                  = g_pending_cal;
    g_b_cal_pending      = false;
    g_b_reset_pending    = false;
    b_put_down           = g_b_put_down_pending;
    g_b_put_down_pending = false;
    EI(interrupt_state);

    if (b_put_down)
    {
        buddy4_processor_put_down(&g_processor);
    }

    if (b_apply_cal)
    {
        buddy4_processor_init(&g_processor, &cal);
        g_b_stats_cleared = true;
    }
    else if (b_apply_reset)
    {
        /* New run: same calibration, everything else from scratch. */
        cal = g_processor.calibration;
        buddy4_processor_init(&g_processor, &cal);
        g_b_stats_cleared = true;
    }
    else
    {
        /* Nothing queued. */
    }
}

/* Read the compass into rover axes.  Returns false (and forces a
 * reconfiguration) when the read fails. */
static bool buddy4_read_mag (int16_t mag[BUDDY4_AXES])
{
    uint8_t buffer[BUDDY4_RAW_BYTES];
    bool b_read = g_b_mag_configured &&
                  buddy4_read_regs(LSM303_MAG_ADDR, LSM303_MAG_OUT_X_H, buffer,
                                   BUDDY4_RAW_BYTES);

    if (b_read)
    {
        /* Register order is X, Z, Y. */
        mag[0] = buddy4_be16(&buffer[0]);
        mag[2] = buddy4_be16(&buffer[2]);
        mag[1] = buddy4_be16(&buffer[4]);
        buddy4_to_rover_axes(mag); /* overflow -4096 stays out of range */
    }
    else
    {
        g_b_mag_configured = false;
    }

    return b_read;
}

/* Read the accelerometer in mg, rover axes.  Returns false (and forces a
 * reconfiguration) when the read fails. */
static bool buddy4_read_accel (int16_t accel[BUDDY4_AXES])
{
    uint8_t  buffer[BUDDY4_RAW_BYTES];
    uint32_t index = 0u;
    bool b_read = g_b_accel_configured &&
                  buddy4_read_regs(LSM303_ACCEL_ADDR,
                                   LSM303_ACC_OUT_X_L | LSM303_AUTO_INCREMENT,
                                   buffer, BUDDY4_RAW_BYTES);

    if (b_read)
    {
        for (index = 0u; index < BUDDY4_AXES; index++)
        {
            /* 12-bit left-justified, 1 mg per count at +/-2 g; the quotient
             * is within +/-2048, so it fits int16_t. */
            accel[index] = (int16_t) (buddy4_le16(&buffer[2u * index]) /
                                      BUDDY4_ACCEL_SHIFT_DIV);
        }
        buddy4_to_rover_axes(accel);
    }
    else
    {
        g_b_accel_configured = false;
    }

    return b_read;
}

/* Mirror imu_fault (a compass or accelerometer read failed, or went stale)
 * into the shared fault byte.  This task is the only writer of the bit, and
 * the byte is only touched when the bit changes. */
static void buddy4_report_fault (bool b_fault)
{
    bool b_set = (0u != (robot_fault_get() & ROBOT_FAULT_IMU));

    if (b_fault && !b_set)
    {
        robot_fault_set(ROBOT_FAULT_IMU);
    }
    else if (!b_fault && b_set)
    {
        robot_fault_clear(ROBOT_FAULT_IMU);
    }
    else
    {
        /* Unchanged. */
    }
}

/* Publish the IMU/hump report: periodically, and at once after an event. */
static void buddy4_report (const buddy4_imu_state_t * p_state)
{
    robot_wire_imu_t values = {0};
    uint8_t          message[ROBOT_WIRE_MESSAGE_MAX];
    uint32_t         length = 0u;

    /* Packed binary report (robot_wire.h, type 6) on rover/telemetry.  The
     * sequence numbers repeat in every report, so a dropped one is
     * recovered by the next. */
    values.b_imu_fault        = (0u != p_state->imu_fault);
    values.b_calibrated       = (0u != p_state->calibrated);
    values.b_height_valid     = (0u != p_state->height_valid);
    values.sample_interval_ms = p_state->sample_interval_ms;
    values.roll_x10           = p_state->roll_deg_x10;
    /* Bit-fields of 2, 3 and 8 bits: the casts cannot lose data. */
    values.hump_state   = (uint8_t) p_state->hump_state;
    values.motion_class = (uint8_t) p_state->motion_class;
    values.lift_count   = (uint8_t) p_state->lift_count;
    /* Only the low 8 bits of the wrapping sequence are sent. */
    values.hump_sequence    = (uint8_t) (p_state->hump_sequence & 0xFFu);
    values.hump_peak_x10    = p_state->hump_peak_deg_x10;
    values.highest_peak_x10 = p_state->highest_peak_deg_x10;
    values.hump_height_mm   = p_state->hump_height_mm;
    values.highest_hump_mm  = p_state->highest_hump_mm;
    values.turn_rate_x10    = p_state->turn_rate_dps_x10;

    length = robot_wire_encode_imu(&values, message, sizeof(message));
    (void) comms_publish(COMMS_TOPIC_TELEMETRY, message, length);
}

/*** end of file ***/

