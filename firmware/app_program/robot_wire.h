/** @file robot_wire.h
 *
 * @brief Buddy 1: binary MQTT message layouts shared with the dashboard.
 *
 * Every rover-to-dashboard message starts with one header byte:
 *   high nibble = message type (ROBOT_WIRE_TYPE_*),
 *   low nibble  = format version (ROBOT_WIRE_VERSION).
 * The fields follow back to back at their bit widths, most significant bit
 * first (robot_packed.h).  Commands from the dashboard start with a 4-bit
 * opcode (ROBOT_WIRE_OP_*) in the high nibble instead, then their values.
 *
 * The layouts are listed in docs/MQTT_BINARY_FORMAT.md; dashboard/
 * rover-wire.js must change in the same commit as this file, and
 * ROBOT_WIRE_VERSION goes up whenever a layout changes.
 *
 * Portable C (no kernel headers), so the host tests can use it.
 */
#ifndef ROBOT_WIRE_H
#define ROBOT_WIRE_H

#include <stdbool.h>
#include <stdint.h>

/** Layout version in the low nibble of every header byte. */
#define ROBOT_WIRE_VERSION (1u)

/* Message types (high nibble of the header byte).  Type 7 is never used:
 * 0x7B is '{', the first byte of JSON text (older firmware), so the
 * dashboard can tell the two apart from byte 0. */
#define ROBOT_WIRE_TYPE_TELEMETRY      (1u) /**< rover/telemetry */
#define ROBOT_WIRE_TYPE_DEBUG          (2u) /**< rover/debug */
#define ROBOT_WIRE_TYPE_HEARTBEAT      (3u) /**< rover/heartbeat */
#define ROBOT_WIRE_TYPE_STATUS_OFFLINE (4u) /**< rover/status (Last Will) */
#define ROBOT_WIRE_TYPE_STATUS_ONLINE  (5u) /**< rover/status (bit 0 set) */
#define ROBOT_WIRE_TYPE_IMU            (6u) /**< rover/telemetry, Buddy 4 */
/* Diagnostic streams (DIAGNOSTICS=1 builds, robot_diagnostics_mqtt.c). */
#define ROBOT_WIRE_TYPE_ACTION   (8u)  /**< rover/actions */
#define ROBOT_WIRE_TYPE_STEERING (9u)  /**< rover/steering */
#define ROBOT_WIRE_TYPE_BARCODE  (10u) /**< rover/barcode */
#define ROBOT_WIRE_TYPE_IR       (11u) /**< rover/ir */
/** Largest diagnostic message (a 4-row barcode sample packet), in bytes. */
#define ROBOT_WIRE_DIAG_MESSAGE_MAX (96u)

/* The complete one-byte rover/status payloads.  Online and offline differ
 * only in bit 4 (the low bit of the type), so the state uses 1 bit. */
#define ROBOT_WIRE_STATUS_OFFLINE_BYTE                                         \
    ((ROBOT_WIRE_TYPE_STATUS_OFFLINE << 4u) | ROBOT_WIRE_VERSION)
#define ROBOT_WIRE_STATUS_ONLINE_BYTE                                          \
    ((ROBOT_WIRE_TYPE_STATUS_ONLINE << 4u) | ROBOT_WIRE_VERSION)

/* Command opcodes (high nibble of byte 0).  0 and 7 are reserved, so an
 * all-zero byte or an old text/JSON command ('{', 's', ...) is rejected. */
#define ROBOT_WIRE_OP_STOP           (1u)
#define ROBOT_WIRE_OP_DRIVE_PRESET   (2u)
#define ROBOT_WIRE_OP_DRIVE          (3u)
#define ROBOT_WIRE_OP_LINE_FOLLOW    (4u)
#define ROBOT_WIRE_OP_LINE_STOP      (5u)
#define ROBOT_WIRE_OP_IMU_RESET      (6u)
#define ROBOT_WIRE_OP_RESET_ALL      (8u)
#define ROBOT_WIRE_OP_MOTOR_TRIM     (9u)
#define ROBOT_WIRE_OP_SPEED_LIMIT    (10u)
#define ROBOT_WIRE_OP_SERVO_ANGLE    (11u)
#define ROBOT_WIRE_OP_BARCODE_MODE   (12u)
#define ROBOT_WIRE_OP_BARCODE_CONFIG (13u)
#define ROBOT_WIRE_OP_LINE_STAGE     (14u)
#define ROBOT_WIRE_OP_LINE_SPEED     (15u)

/* Arguments of ROBOT_WIRE_OP_DRIVE_PRESET (2 bits). */
#define ROBOT_WIRE_PRESET_FORWARD (0u)
#define ROBOT_WIRE_PRESET_REVERSE (1u)
#define ROBOT_WIRE_PRESET_LEFT    (2u)
#define ROBOT_WIRE_PRESET_RIGHT   (3u)

/* Sensor state field of the telemetry message (2 bits). */
#define ROBOT_WIRE_SENSORS_ONLINE        (0u)
#define ROBOT_WIRE_SENSORS_COMPASS_ERROR (1u)
#define ROBOT_WIRE_SENSORS_IMU_ERROR     (2u)

/* Avoidance decision field of the debug message (2 bits). */
#define ROBOT_WIRE_DECISION_NONE  (0u)
#define ROBOT_WIRE_DECISION_LEFT  (1u)
#define ROBOT_WIRE_DECISION_RIGHT (2u)

/** Largest message any layout below produces, in bytes. */
#define ROBOT_WIRE_MESSAGE_MAX (32u)
/** Longest valid command (ROBOT_WIRE_OP_DRIVE), in bytes. */
#define ROBOT_WIRE_COMMAND_MAX (5u)

/** Values of the rover/telemetry message, in their native units. */
typedef struct
{
    uint32_t uptime_s;        /**< 20 bits, saturates after ~12 days. */
    uint8_t  mode;            /**< 3 bits, robot_controller mode index. */
    uint8_t  sensor_state;    /**< 2 bits, ROBOT_WIRE_SENSORS_*. */
    uint8_t  faults;          /**< 8 bits, robot_fault.h mask. */
    uint16_t ir_raw[3];       /**< 12 bits each, ADC 0..4095. */
    uint16_t distance_cm;     /**< 9 bits, 0 = no echo, up to 511. */
    uint8_t  servo_angle_deg; /**< 8 bits, 0..180. */
    uint16_t heading_x10;     /**< 12 bits, 0..3599 tenths of a degree. */
    int16_t  pitch_x10;       /**< 11 bits signed, about +/-90.0 deg. */
    int16_t  max_pitch_x10;   /**< 11 bits signed. */
    uint32_t left_ticks;      /**< Low 16 bits sent; dashboard unwraps. */
    uint32_t right_ticks;     /**< Low 16 bits sent. */
    uint16_t motor_left;      /**< 16 bits, dashboard drive units. */
    uint16_t motor_right;     /**< 16 bits. */
    uint8_t  trim_left_pct;   /**< 6 bits as value - 50, 50..100. */
    uint8_t  trim_right_pct;  /**< 6 bits as value - 50. */
    uint8_t  line_base_pct;   /**< 7 bits, 15..80. */
    uint8_t  line_turn_pct;   /**< 6 bits, 0..60. */
    uint8_t  speed_pct;       /**< 7 bits, 20..100. */
} robot_wire_telemetry_t;

/** Values of the rover/debug message, in their native units. */
typedef struct
{
    uint16_t ir_raw[3];          /**< 12 bits each. */
    uint16_t ir_norm[3];         /**< 10 bits each, 0..1000. */
    int16_t  line_position;      /**< 11 bits signed, -1000..1000. */
    bool     b_line;             /**< 1 bit. */
    bool     b_junction;         /**< 1 bit. */
    bool     b_barcode;          /**< 1 bit. */
    uint8_t  barcode_command;    /**< 7 bits, ASCII letter or 0. */
    uint16_t ultra_left_cm;      /**< 9 bits, 0 = no echo. */
    uint16_t ultra_right_cm;     /**< 9 bits. */
    uint8_t  decision;           /**< 2 bits, ROBOT_WIRE_DECISION_*. */
    uint8_t  avoid_status;       /**< 4 bits, ROBOT_AVOID_STATUS_*. */
    int16_t  heading_change_x10; /**< 12 bits signed, +/-204.7 deg. */
    uint16_t heading_target_x10; /**< 12 bits, 0..4095. */
    int16_t  turn_left_pwm;      /**< 16 bits signed, +/-32767. */
    int16_t  turn_right_pwm;     /**< 16 bits signed. */
    uint16_t forward_mm;         /**< 11 bits, 0..2047 mm. */
    uint16_t forward_target_mm;  /**< 11 bits. */
} robot_wire_debug_t;

/** Values of Buddy 4's IMU / hump report (type 6), in their native units. */
typedef struct
{
    bool     b_imu_fault;        /**< 1 bit. */
    bool     b_calibrated;       /**< 1 bit. */
    bool     b_height_valid;     /**< 1 bit. */
    uint16_t sample_interval_ms; /**< 10 bits, saturates at 1023 ms. */
    int16_t  roll_x10;           /**< 12 bits signed, -1800..1800. */
    uint8_t  hump_state;         /**< 2 bits, BUDDY4_FLAT..DESCENDING. */
    uint8_t  hump_sequence;      /**< 8 bits, completed humps (wraps). */
    int16_t  hump_peak_x10;      /**< 11 bits signed, -900..900. */
    int16_t  highest_peak_x10;   /**< 11 bits signed. */
    uint16_t hump_height_mm;     /**< 8 bits, 0..255 mm. */
    uint16_t highest_hump_mm;    /**< 8 bits. */
    uint8_t  motion_class;       /**< 3 bits, BUDDY4_MOTION_*. */
    int16_t  turn_rate_x10;      /**< 15 bits signed, +/-9000. */
    uint8_t  lift_count;         /**< 8 bits, saturates at 255. */
} robot_wire_imu_t;

/** A decoded dashboard command.  Unused values are 0. */
typedef struct
{
    uint8_t opcode;    /**< ROBOT_WIRE_OP_*. */
    int32_t values[4]; /**< Arguments in their native units. */
} robot_wire_command_t;

/**
 * @brief Build the header byte for a rover-to-dashboard message.
 * @param type ROBOT_WIRE_TYPE_* value.
 * @return (type << 4) | ROBOT_WIRE_VERSION.
 */
uint8_t robot_wire_header (uint32_t type);

/**
 * @brief Pack a telemetry message.
 * @param p_values Values to send.
 * @param p_buffer Output, at least ROBOT_WIRE_MESSAGE_MAX bytes.
 * @param capacity Size of p_buffer.
 * @return Message length in bytes, or 0 when it did not fit.
 */
uint32_t robot_wire_encode_telemetry (const robot_wire_telemetry_t * p_values,
                                      uint8_t * p_buffer, uint32_t capacity);

/**
 * @brief Pack a debug message.
 * @param p_values Values to send.
 * @param p_buffer Output, at least ROBOT_WIRE_MESSAGE_MAX bytes.
 * @param capacity Size of p_buffer.
 * @return Message length in bytes, or 0 when it did not fit.
 */
uint32_t robot_wire_encode_debug (const robot_wire_debug_t * p_values,
                                  uint8_t * p_buffer, uint32_t capacity);

/**
 * @brief Pack Buddy 4's IMU / hump report.
 * @param p_values Values to send.
 * @param p_buffer Output, at least ROBOT_WIRE_MESSAGE_MAX bytes.
 * @param capacity Size of p_buffer.
 * @return Message length in bytes, or 0 when it did not fit.
 */
uint32_t robot_wire_encode_imu (const robot_wire_imu_t * p_values,
                                uint8_t * p_buffer, uint32_t capacity);

/**
 * @brief Decode and check one command from the dashboard.
 *
 * Accepted only when the opcode is known, the length is exactly the one that
 * opcode needs, and the padding bits after the last value are zero.  The
 * values are returned as sent: the controller checks each one's range and
 * reports a rejected value.
 *
 * @param p_data    Received bytes.
 * @param length    Number of received bytes.
 * @param p_command Receives the command when it is accepted.
 * @return True when the command is valid.
 */
bool robot_wire_decode_command (const uint8_t * p_data, uint32_t length,
                                robot_wire_command_t * p_command);

#endif /* ROBOT_WIRE_H */

/*** end of file ***/

