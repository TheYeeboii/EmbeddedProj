/** @file test_robot_wire.c
 *
 * @brief Host tests for the packed MQTT format (robot_packed.c, robot_wire.c).
 *
 * Round-trips every field at its minimum, maximum and edge values, checks the
 * message lengths, and checks that the command decoder rejects anything that
 * is not exactly a known command.  The two golden messages are also decoded
 * by tests/test_rover_wire.cjs, so the firmware and the dashboard are proved
 * to agree on the bit layout.
 *
 * Build: cc -std=c11 -Wall -Wextra -Werror -Ifirmware/app_program
 *        tests/test_robot_wire.c firmware/app_program/robot_packed.c
 *        firmware/app_program/robot_wire.c
 * Run with "--print" to show the golden messages as hex.
 */
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "robot_packed.h"
#include "robot_wire.h"

#define TEST_TELEMETRY_LENGTH (28u)
#define TEST_DEBUG_LENGTH     (25u)

static void test_print_hex (const char * p_label, const uint8_t * p_data,
                            uint32_t length)
{
    uint32_t index = 0u;

    (void) printf("%s:", p_label);
    for (index = 0u; index < length; index++)
    {
        (void) printf(" %02x", (unsigned int) p_data[index]);
    }
    (void) printf("\n");
}

/* Golden telemetry: every field at an edge of its range. */
static robot_wire_telemetry_t test_golden_telemetry (void)
{
    robot_wire_telemetry_t values = {0};

    values.uptime_s        = 1048575u; /* 20-bit maximum */
    values.mode            = 4u;       /* avoid */
    values.sensor_state    = ROBOT_WIRE_SENSORS_COMPASS_ERROR;
    values.faults          = 0x21u; /* Wi-Fi loss + motor stall */
    values.ir_raw[0]       = 0u;
    values.ir_raw[1]       = 4095u;
    values.ir_raw[2]       = 2048u;
    values.distance_cm     = 400u;
    values.servo_angle_deg = 180u;
    values.heading_x10     = 3599u;
    values.pitch_x10       = -900;
    values.max_pitch_x10   = 900;
    values.left_ticks      = 0x0001FFFFu; /* only 0xFFFF is sent */
    values.right_ticks     = 12345u;
    values.motor_left      = 65535u;
    values.motor_right     = 0u;
    values.trim_left_pct   = 50u;
    values.trim_right_pct  = 100u;
    values.line_base_pct   = 80u;
    values.line_turn_pct   = 60u;
    values.speed_pct       = 20u;

    return values;
}

/* Golden debug message: signed fields negative, the rest at edges. */
static robot_wire_debug_t test_golden_debug (void)
{
    robot_wire_debug_t values = {0};

    values.ir_raw[0]          = 4095u;
    values.ir_raw[1]          = 1u;
    values.ir_raw[2]          = 0u;
    values.ir_norm[0]         = 1000u;
    values.ir_norm[1]         = 0u;
    values.ir_norm[2]         = 500u;
    values.line_position      = -1000;
    values.b_line             = true;
    values.b_junction         = false;
    values.b_barcode          = true;
    values.barcode_command    = 'Z';
    values.ultra_left_cm      = 0u; /* no echo */
    values.ultra_right_cm     = 400u;
    values.decision           = ROBOT_WIRE_DECISION_RIGHT;
    values.avoid_status       = 11u; /* reversing */
    values.heading_change_x10 = -1800;
    values.heading_target_x10 = 3599u;
    values.turn_left_pwm      = -32767;
    values.turn_right_pwm     = 32767;
    values.forward_mm         = 250u;
    values.forward_target_mm  = 2047u;

    return values;
}

static void test_packed_fields (void)
{
    uint8_t               buffer[8];
    robot_packed_writer_t writer;
    robot_packed_reader_t reader;

    robot_packed_writer_init(&writer, buffer, sizeof(buffer));
    robot_packed_put(&writer, 5u, 3u);            /* 1..5 needs 3 bits */
    robot_packed_put(&writer, 999u, 9u);          /* saturates to 511 */
    robot_packed_put_signed(&writer, -1, 11u);    /* all ones */
    robot_packed_put_signed(&writer, 5000, 11u);  /* saturates to 1023 */
    robot_packed_put_signed(&writer, -5000, 11u); /* saturates to -1024 */
    robot_packed_put(&writer, 0xFFFFFFFFu, 32u);
    assert(0u == robot_packed_length(&writer)); /* 77 bits > 64: overflow */

    robot_packed_writer_init(&writer, buffer, sizeof(buffer));
    robot_packed_put(&writer, 5u, 3u);
    robot_packed_put(&writer, 999u, 9u);
    robot_packed_put_signed(&writer, -1, 11u);
    robot_packed_put_signed(&writer, 5000, 11u);
    robot_packed_put_signed(&writer, -5000, 11u);
    assert(6u == robot_packed_length(&writer)); /* 45 bits */

    robot_packed_reader_init(&reader, buffer, 6u);
    assert(5u == robot_packed_get(&reader, 3u));
    assert(511u == robot_packed_get(&reader, 9u));
    assert(-1 == robot_packed_get_signed(&reader, 11u));
    assert(1023 == robot_packed_get_signed(&reader, 11u));
    assert(-1024 == robot_packed_get_signed(&reader, 11u));
    assert(robot_packed_rest_is_zero(&reader));
    assert(!reader.b_overrun);
    (void) robot_packed_get(&reader, 8u); /* only 3 bits left */
    assert(reader.b_overrun);
}

static void test_telemetry_layout (void)
{
    robot_wire_telemetry_t values = test_golden_telemetry();
    uint8_t                buffer[ROBOT_WIRE_MESSAGE_MAX];
    robot_packed_reader_t  reader;
    uint32_t               length = 0u;

    length = robot_wire_encode_telemetry(&values, buffer, sizeof(buffer));
    assert(TEST_TELEMETRY_LENGTH == length);
    assert(0x11u == buffer[0]); /* type 1, version 1 */

    robot_packed_reader_init(&reader, buffer, length);
    assert(0x11u == robot_packed_get(&reader, 8u));
    assert(1048575u == robot_packed_get(&reader, 20u));
    assert(4u == robot_packed_get(&reader, 3u));
    assert(ROBOT_WIRE_SENSORS_COMPASS_ERROR == robot_packed_get(&reader, 2u));
    assert(0x21u == robot_packed_get(&reader, 8u));
    assert(0u == robot_packed_get(&reader, 12u));
    assert(4095u == robot_packed_get(&reader, 12u));
    assert(2048u == robot_packed_get(&reader, 12u));
    assert(400u == robot_packed_get(&reader, 9u));
    assert(180u == robot_packed_get(&reader, 8u));
    assert(3599u == robot_packed_get(&reader, 12u));
    assert(-900 == robot_packed_get_signed(&reader, 11u));
    assert(900 == robot_packed_get_signed(&reader, 11u));
    assert(0xFFFFu == robot_packed_get(&reader, 16u));
    assert(12345u == robot_packed_get(&reader, 16u));
    assert(65535u == robot_packed_get(&reader, 16u));
    assert(0u == robot_packed_get(&reader, 16u));
    assert(0u == robot_packed_get(&reader, 6u));  /* trim 50 */
    assert(50u == robot_packed_get(&reader, 6u)); /* trim 100 */
    assert(80u == robot_packed_get(&reader, 7u));
    assert(60u == robot_packed_get(&reader, 6u));
    assert(20u == robot_packed_get(&reader, 7u));
    assert(robot_packed_rest_is_zero(&reader));
    assert(!reader.b_overrun);
}

static void test_debug_layout (void)
{
    robot_wire_debug_t    values = test_golden_debug();
    uint8_t               buffer[ROBOT_WIRE_MESSAGE_MAX];
    robot_packed_reader_t reader;
    uint32_t              length = 0u;

    length = robot_wire_encode_debug(&values, buffer, sizeof(buffer));
    assert(TEST_DEBUG_LENGTH == length);
    assert(0x21u == buffer[0]);

    robot_packed_reader_init(&reader, buffer, length);
    (void) robot_packed_get(&reader, 8u);
    assert(4095u == robot_packed_get(&reader, 12u));
    assert(1u == robot_packed_get(&reader, 12u));
    assert(0u == robot_packed_get(&reader, 12u));
    assert(1000u == robot_packed_get(&reader, 10u));
    assert(0u == robot_packed_get(&reader, 10u));
    assert(500u == robot_packed_get(&reader, 10u));
    assert(-1000 == robot_packed_get_signed(&reader, 11u));
    assert(5u ==
           robot_packed_get(&reader, 3u)); /* line, no junction, barcode */
    assert((uint32_t) 'Z' == robot_packed_get(&reader, 7u));
    assert(0u == robot_packed_get(&reader, 9u));
    assert(400u == robot_packed_get(&reader, 9u));
    assert(ROBOT_WIRE_DECISION_RIGHT == robot_packed_get(&reader, 2u));
    assert(11u == robot_packed_get(&reader, 4u));
    assert(-1800 == robot_packed_get_signed(&reader, 12u));
    assert(3599u == robot_packed_get(&reader, 12u));
    assert(-32767 == robot_packed_get_signed(&reader, 16u));
    assert(32767 == robot_packed_get_signed(&reader, 16u));
    assert(250u == robot_packed_get(&reader, 11u));
    assert(2047u == robot_packed_get(&reader, 11u));
    assert(robot_packed_rest_is_zero(&reader));
}

static bool test_decode (const uint8_t * p_data, uint32_t length,
                         robot_wire_command_t * p_command)
{
    robot_wire_command_t unused = {0u, {0, 0}};

    return robot_wire_decode_command(p_data, length,
                                     (NULL == p_command) ? &unused : p_command);
}

static void test_valid_commands (void)
{
    static const uint8_t stop[]    = {0x10u};
    static const uint8_t reverse[] = {0x24u}; /* preset 1 = reverse */
    static const uint8_t drive[]   = {0x3Fu, 0xFFu, 0xF0u, 0x00u, 0x00u};
    static const uint8_t trim[]    = {0x90u, 0x00u}; /* 50 / 50: has 0x00 */
    /* 1101 1111101000 1000001100: opcode 13, 1000, -500 (= 1024 - 500). */
    static const uint8_t config[] = {0xDFu, 0xA2u, 0x0Cu};
    /* 1111 1010000 111100 + 7 padding bits: opcode 15, base 80, turn 60. */
    static const uint8_t speed[] = {0xFAu, 0x1Eu, 0x00u};
    robot_wire_command_t command = {0u, {0, 0}};

    assert(test_decode(stop, sizeof(stop), &command));
    assert(ROBOT_WIRE_OP_STOP == command.opcode);
    assert(test_decode(reverse, sizeof(reverse), &command));
    assert((ROBOT_WIRE_OP_DRIVE_PRESET == command.opcode) &&
           ((int32_t) ROBOT_WIRE_PRESET_REVERSE == command.values[0]));
    assert(test_decode(drive, sizeof(drive), &command));
    assert((65535 == command.values[0]) && (0 == command.values[1]));
    assert(test_decode(trim, sizeof(trim), &command));
    assert((50 == command.values[0]) && (50 == command.values[1]));
    assert(test_decode(config, sizeof(config), &command));
    assert((1000 == command.values[0]) && (-500 == command.values[1]));
    assert(test_decode(speed, sizeof(speed), &command));
    assert((ROBOT_WIRE_OP_LINE_SPEED == command.opcode) &&
           (80 == command.values[0]) && (60 == command.values[1]));
}

static void test_rejected_commands (void)
{
    static const uint8_t zero[]      = {0x00u};
    static const uint8_t reserved[]  = {0x70u};
    static const uint8_t json[]      = "{\"cmd\":\"stop\"}";
    static const uint8_t text_stop[] = "stop";
    static const uint8_t padding[]   = {0x11u}; /* stop with a stray bit */
    static const uint8_t short_drv[] = {0x3Fu, 0xFFu, 0xF0u};
    static const uint8_t long_stop[] = {0x10u, 0x00u};

    assert(!test_decode(zero, sizeof(zero), NULL));
    assert(!test_decode(reserved, sizeof(reserved), NULL));
    assert(!test_decode(json, (uint32_t) strlen((const char *) json), NULL));
    assert(!test_decode(text_stop, 1u, NULL)); /* 's' alone */
    assert(!test_decode(text_stop, (uint32_t) strlen("stop"), NULL));
    assert(!test_decode(padding, sizeof(padding), NULL));
    assert(!test_decode(short_drv, sizeof(short_drv), NULL));
    assert(!test_decode(long_stop, sizeof(long_stop), NULL));
    assert(!test_decode(NULL, 1u, NULL));
}

int main (int argc, char * argv[])
{
    robot_wire_telemetry_t telemetry = test_golden_telemetry();
    robot_wire_debug_t     debug     = test_golden_debug();
    uint8_t                buffer[ROBOT_WIRE_MESSAGE_MAX];
    uint32_t               length = 0u;

    test_packed_fields();
    test_telemetry_layout();
    test_debug_layout();
    test_valid_commands();
    test_rejected_commands();

    if ((argc > 1) && (0 == strcmp(argv[1], "--print")))
    {
        length =
            robot_wire_encode_telemetry(&telemetry, buffer, sizeof(buffer));
        test_print_hex("telemetry", buffer, length);
        length = robot_wire_encode_debug(&debug, buffer, sizeof(buffer));
        test_print_hex("debug", buffer, length);
    }

    (void) printf("PASS: packed fields, telemetry (%u bytes), debug (%u bytes) "
                  "and command checks\n",
                  TEST_TELEMETRY_LENGTH, TEST_DEBUG_LENGTH);

    return 0;
}

/*** end of file ***/

