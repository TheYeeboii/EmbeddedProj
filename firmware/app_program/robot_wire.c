/** @file robot_wire.c
 *
 * @brief Buddy 1: binary MQTT message layouts shared with the dashboard.
 *
 * See robot_wire.h and docs/MQTT_BINARY_FORMAT.md.  The field order and
 * widths below are the wire format: keep dashboard/rover-wire.js in step.
 */
#include <stddef.h>

#include "robot_packed.h"
#include "robot_wire.h"

#define WIRE_NIBBLE_BITS (4u)
#define WIRE_NIBBLE_MASK (0x0Fu)
#define WIRE_IR_COUNT    (3u)

/* Field widths in bits (docs/MQTT_BINARY_FORMAT.md). */
#define WIRE_BITS_HEADER    (8u)
#define WIRE_BITS_UPTIME    (20u)
#define WIRE_BITS_MODE      (3u)
#define WIRE_BITS_SENSORS   (2u)
#define WIRE_BITS_FAULTS    (8u)
#define WIRE_BITS_IR_RAW    (12u)
#define WIRE_BITS_IR_NORM   (10u)
#define WIRE_BITS_DISTANCE  (9u)
#define WIRE_BITS_SERVO     (8u)
#define WIRE_BITS_HEADING   (12u)
#define WIRE_BITS_PITCH     (11u)
#define WIRE_BITS_TICKS     (16u)
#define WIRE_BITS_MOTOR     (16u)
#define WIRE_BITS_TRIM      (6u)
#define WIRE_BITS_LINE_BASE (7u)
#define WIRE_BITS_LINE_TURN (6u)
#define WIRE_BITS_SPEED     (7u)
#define WIRE_BITS_POSITION  (11u)
#define WIRE_BITS_FLAG      (1u)
#define WIRE_BITS_BARCODE   (7u)
#define WIRE_BITS_DECISION  (2u)
#define WIRE_BITS_AVOID     (4u)
#define WIRE_BITS_TURN_X10  (12u)
#define WIRE_BITS_PWM       (16u)
#define WIRE_BITS_FORWARD   (11u)
#define WIRE_BITS_OPCODE    (4u)
#define WIRE_BITS_PRESET    (2u)
#define WIRE_BITS_DRIVE     (16u)
#define WIRE_BITS_BC_MODE   (1u)
#define WIRE_BITS_BC_ELEM   (10u)
#define WIRE_BITS_BC_OFFSET (10u)
#define WIRE_BITS_STAGE     (2u)
#define WIRE_BITS_INTERVAL  (10u)
#define WIRE_BITS_ROLL      (12u)
#define WIRE_BITS_HUMP      (2u)
#define WIRE_BITS_COUNT     (8u)
#define WIRE_BITS_HEIGHT    (8u)
#define WIRE_BITS_MOTION    (3u)
#define WIRE_BITS_RATE      (15u)

/* Stored offset of the trim fields: 50..100 % is sent as 0..50. */
#define WIRE_TRIM_OFFSET (50u)
/* Encoder counts: only the low 16 bits are sent. */
#define WIRE_TICKS_MASK (0xFFFFu)

/* Exact length of each command, indexed by opcode; 0 = not a command. */
static const uint8_t g_command_length[] = {
    0u, /* 0  reserved */
    1u, /* 1  stop: opcode only */
    1u, /* 2  drive preset: 4 + 2 bits */
    5u, /* 3  drive: 4 + 16 + 16 bits */
    1u, /* 4  line follow */
    1u, /* 5  line stop */
    1u, /* 6  IMU reset */
    0u, /* 7  reserved ('{' and lower-case text) */
    1u, /* 8  reset all */
    2u, /* 9  motor trim: 4 + 6 + 6 bits */
    2u, /* 10 speed limit: 4 + 7 bits */
    2u, /* 11 servo angle: 4 + 8 bits */
    1u, /* 12 barcode mode: 4 + 1 bits */
    3u, /* 13 barcode config: 4 + 10 + 10 bits */
    1u, /* 14 line stage: 4 + 2 bits */
    3u  /* 15 line speed: 4 + 7 + 6 bits */
};

static void wire_read_arguments (robot_packed_reader_t * p_reader,
                                 robot_wire_command_t *  p_command);

uint8_t robot_wire_header (uint32_t type)
{
    /* Both nibbles are masked, so the result fits in one byte. */
    return (uint8_t) (((type & WIRE_NIBBLE_MASK) << WIRE_NIBBLE_BITS) |
                      (ROBOT_WIRE_VERSION & WIRE_NIBBLE_MASK));
}

uint32_t robot_wire_encode_telemetry (const robot_wire_telemetry_t * p_values,
                                      uint8_t * p_buffer, uint32_t capacity)
{
    robot_packed_writer_t writer = {NULL, 0u, 0u, false};
    uint32_t              index  = 0u;
    uint32_t              length = 0u;

    if (NULL != p_values)
    {
        robot_packed_writer_init(&writer, p_buffer, capacity);
        robot_packed_put(&writer, robot_wire_header(ROBOT_WIRE_TYPE_TELEMETRY),
                         WIRE_BITS_HEADER);
        robot_packed_put(&writer, p_values->uptime_s, WIRE_BITS_UPTIME);
        robot_packed_put(&writer, p_values->mode, WIRE_BITS_MODE);
        robot_packed_put(&writer, p_values->sensor_state, WIRE_BITS_SENSORS);
        robot_packed_put(&writer, p_values->faults, WIRE_BITS_FAULTS);
        for (index = 0u; index < WIRE_IR_COUNT; index++)
        {
            robot_packed_put(&writer, p_values->ir_raw[index],
                             WIRE_BITS_IR_RAW);
        }
        robot_packed_put(&writer, p_values->distance_cm, WIRE_BITS_DISTANCE);
        robot_packed_put(&writer, p_values->servo_angle_deg, WIRE_BITS_SERVO);
        robot_packed_put(&writer, p_values->heading_x10, WIRE_BITS_HEADING);
        robot_packed_put_signed(&writer, p_values->pitch_x10, WIRE_BITS_PITCH);
        robot_packed_put_signed(&writer, p_values->max_pitch_x10,
                                WIRE_BITS_PITCH);
        robot_packed_put(&writer, p_values->left_ticks & WIRE_TICKS_MASK,
                         WIRE_BITS_TICKS);
        robot_packed_put(&writer, p_values->right_ticks & WIRE_TICKS_MASK,
                         WIRE_BITS_TICKS);
        robot_packed_put(&writer, p_values->motor_left, WIRE_BITS_MOTOR);
        robot_packed_put(&writer, p_values->motor_right, WIRE_BITS_MOTOR);
        /* A trim below 50 % cannot be set; sending 0 then is the safe
         * reading of an impossible value. */
        robot_packed_put(&writer,
                         (p_values->trim_left_pct > WIRE_TRIM_OFFSET)
                             ? (p_values->trim_left_pct - WIRE_TRIM_OFFSET)
                             : 0u,
                         WIRE_BITS_TRIM);
        robot_packed_put(&writer,
                         (p_values->trim_right_pct > WIRE_TRIM_OFFSET)
                             ? (p_values->trim_right_pct - WIRE_TRIM_OFFSET)
                             : 0u,
                         WIRE_BITS_TRIM);
        robot_packed_put(&writer, p_values->line_base_pct, WIRE_BITS_LINE_BASE);
        robot_packed_put(&writer, p_values->line_turn_pct, WIRE_BITS_LINE_TURN);
        robot_packed_put(&writer, p_values->speed_pct, WIRE_BITS_SPEED);
        length = robot_packed_length(&writer);
    }

    return length;
}

uint32_t robot_wire_encode_debug (const robot_wire_debug_t * p_values,
                                  uint8_t * p_buffer, uint32_t capacity)
{
    robot_packed_writer_t writer = {NULL, 0u, 0u, false};
    uint32_t              index  = 0u;
    uint32_t              length = 0u;

    if (NULL != p_values)
    {
        robot_packed_writer_init(&writer, p_buffer, capacity);
        robot_packed_put(&writer, robot_wire_header(ROBOT_WIRE_TYPE_DEBUG),
                         WIRE_BITS_HEADER);
        for (index = 0u; index < WIRE_IR_COUNT; index++)
        {
            robot_packed_put(&writer, p_values->ir_raw[index],
                             WIRE_BITS_IR_RAW);
        }
        for (index = 0u; index < WIRE_IR_COUNT; index++)
        {
            robot_packed_put(&writer, p_values->ir_norm[index],
                             WIRE_BITS_IR_NORM);
        }
        robot_packed_put_signed(&writer, p_values->line_position,
                                WIRE_BITS_POSITION);
        robot_packed_put(&writer, p_values->b_line ? 1u : 0u, WIRE_BITS_FLAG);
        robot_packed_put(&writer, p_values->b_junction ? 1u : 0u,
                         WIRE_BITS_FLAG);
        robot_packed_put(&writer, p_values->b_barcode ? 1u : 0u,
                         WIRE_BITS_FLAG);
        robot_packed_put(&writer, p_values->barcode_command, WIRE_BITS_BARCODE);
        robot_packed_put(&writer, p_values->ultra_left_cm, WIRE_BITS_DISTANCE);
        robot_packed_put(&writer, p_values->ultra_right_cm, WIRE_BITS_DISTANCE);
        robot_packed_put(&writer, p_values->decision, WIRE_BITS_DECISION);
        robot_packed_put(&writer, p_values->avoid_status, WIRE_BITS_AVOID);
        robot_packed_put_signed(&writer, p_values->heading_change_x10,
                                WIRE_BITS_TURN_X10);
        robot_packed_put(&writer, p_values->heading_target_x10,
                         WIRE_BITS_HEADING);
        robot_packed_put_signed(&writer, p_values->turn_left_pwm,
                                WIRE_BITS_PWM);
        robot_packed_put_signed(&writer, p_values->turn_right_pwm,
                                WIRE_BITS_PWM);
        robot_packed_put(&writer, p_values->forward_mm, WIRE_BITS_FORWARD);
        robot_packed_put(&writer, p_values->forward_target_mm,
                         WIRE_BITS_FORWARD);
        length = robot_packed_length(&writer);
    }

    return length;
}

uint32_t robot_wire_encode_imu (const robot_wire_imu_t * p_values,
                                uint8_t * p_buffer, uint32_t capacity)
{
    robot_packed_writer_t writer = {NULL, 0u, 0u, false};
    uint32_t              length = 0u;

    if (NULL != p_values)
    {
        robot_packed_writer_init(&writer, p_buffer, capacity);
        robot_packed_put(&writer, robot_wire_header(ROBOT_WIRE_TYPE_IMU),
                         WIRE_BITS_HEADER);
        robot_packed_put(&writer, p_values->b_imu_fault ? 1u : 0u,
                         WIRE_BITS_FLAG);
        robot_packed_put(&writer, p_values->b_calibrated ? 1u : 0u,
                         WIRE_BITS_FLAG);
        robot_packed_put(&writer, p_values->b_height_valid ? 1u : 0u,
                         WIRE_BITS_FLAG);
        robot_packed_put(&writer, p_values->sample_interval_ms,
                         WIRE_BITS_INTERVAL);
        robot_packed_put_signed(&writer, p_values->roll_x10, WIRE_BITS_ROLL);
        robot_packed_put(&writer, p_values->hump_state, WIRE_BITS_HUMP);
        robot_packed_put(&writer, p_values->hump_sequence, WIRE_BITS_COUNT);
        robot_packed_put_signed(&writer, p_values->hump_peak_x10,
                                WIRE_BITS_PITCH);
        robot_packed_put_signed(&writer, p_values->highest_peak_x10,
                                WIRE_BITS_PITCH);
        robot_packed_put(&writer, p_values->hump_height_mm, WIRE_BITS_HEIGHT);
        robot_packed_put(&writer, p_values->highest_hump_mm, WIRE_BITS_HEIGHT);
        robot_packed_put(&writer, p_values->motion_class, WIRE_BITS_MOTION);
        robot_packed_put_signed(&writer, p_values->turn_rate_x10,
                                WIRE_BITS_RATE);
        robot_packed_put(&writer, p_values->lift_count, WIRE_BITS_COUNT);
        length = robot_packed_length(&writer);
    }

    return length;
}

bool robot_wire_decode_command (const uint8_t * p_data, uint32_t length,
                                robot_wire_command_t * p_command)
{
    robot_packed_reader_t reader  = {NULL, 0u, 0u, false};
    robot_wire_command_t  decoded = {0u, {0, 0}};
    uint32_t              opcode  = 0u;
    bool                  b_valid = false;

    if ((NULL != p_data) && (NULL != p_command) && (0u != length))
    {
        robot_packed_reader_init(&reader, p_data, length);
        opcode = robot_packed_get(&reader, WIRE_BITS_OPCODE);
        /* opcode is a 4-bit value, so it indexes the 16-entry table. */
        if ((0u != g_command_length[opcode]) &&
            (length == (uint32_t) g_command_length[opcode]))
        {
            decoded.opcode = (uint8_t) opcode; /* 4 bits. */
            wire_read_arguments(&reader, &decoded);
            b_valid = !reader.b_overrun && robot_packed_rest_is_zero(&reader);
        }
        if (b_valid)
        {
            *p_command = decoded;
        }
    }

    return b_valid;
}

/* Read the arguments of p_command->opcode.  The ranges each field allows are
 * checked again by the controller, which reports a rejected value. */
static void wire_read_arguments (robot_packed_reader_t * p_reader,
                                 robot_wire_command_t *  p_command)
{
    /* Every unsigned field read below is at most 16 bits, so the int32_t
     * conversions are exact. */
    switch (p_command->opcode)
    {
        case ROBOT_WIRE_OP_DRIVE_PRESET:
            p_command->values[0] =
                (int32_t) robot_packed_get(p_reader, WIRE_BITS_PRESET);
        break;

        case ROBOT_WIRE_OP_DRIVE:
            p_command->values[0] =
                (int32_t) robot_packed_get(p_reader, WIRE_BITS_DRIVE);
            p_command->values[1] =
                (int32_t) robot_packed_get(p_reader, WIRE_BITS_DRIVE);
        break;

        case ROBOT_WIRE_OP_MOTOR_TRIM:
            p_command->values[0] =
                (int32_t) (robot_packed_get(p_reader, WIRE_BITS_TRIM) +
                           WIRE_TRIM_OFFSET);
            p_command->values[1] =
                (int32_t) (robot_packed_get(p_reader, WIRE_BITS_TRIM) +
                           WIRE_TRIM_OFFSET);
        break;

        case ROBOT_WIRE_OP_SPEED_LIMIT:
            p_command->values[0] =
                (int32_t) robot_packed_get(p_reader, WIRE_BITS_SPEED);
        break;

        case ROBOT_WIRE_OP_SERVO_ANGLE:
            p_command->values[0] =
                (int32_t) robot_packed_get(p_reader, WIRE_BITS_SERVO);
        break;

        case ROBOT_WIRE_OP_BARCODE_MODE:
            p_command->values[0] =
                (int32_t) robot_packed_get(p_reader, WIRE_BITS_BC_MODE);
        break;

        case ROBOT_WIRE_OP_BARCODE_CONFIG:
            p_command->values[0] =
                (int32_t) robot_packed_get(p_reader, WIRE_BITS_BC_ELEM);
            p_command->values[1] =
                robot_packed_get_signed(p_reader, WIRE_BITS_BC_OFFSET);
        break;

        case ROBOT_WIRE_OP_LINE_STAGE:
            p_command->values[0] =
                (int32_t) robot_packed_get(p_reader, WIRE_BITS_STAGE);
        break;

        case ROBOT_WIRE_OP_LINE_SPEED:
            p_command->values[0] =
                (int32_t) robot_packed_get(p_reader, WIRE_BITS_LINE_BASE);
            p_command->values[1] =
                (int32_t) robot_packed_get(p_reader, WIRE_BITS_LINE_TURN);
        break;

        default:
            /* Commands without arguments. */
        break;
    }
}

/*** end of file ***/

