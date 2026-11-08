#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
/* Compile the real command/button/handoff controller against fake hardware. */
#include "../firmware/app_program/robot_controller.c"
#include "../firmware/app_program/robot_packed.h"

static bool     g_b_following;
static bool     g_b_motion;
static bool     g_b_accept_move = true;
static bool     g_b_calibrated  = true;
static bool     g_b_button_high = true;
static bool     g_b_encoder_fault;
static uint8_t  g_stage = 1;
static uint8_t  g_event;
static uint8_t  g_servo_angle = 90;
static char     g_serial_log[512];
static unsigned g_starts;
static unsigned g_stops;
static unsigned g_lefts;
static unsigned g_rights;
static unsigned g_forwards;
int             tm_printf (const unsigned char * fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int written =
        vsnprintf(g_serial_log, sizeof(g_serial_log), (const char *) fmt, args);
    va_end(args);
    return written;
}
void hal_gpio_input_pullup (uint32_t pin)
{
    (void) pin;
}
void buddy4_reset_run (void)
{
}
void buddy2_reset_encoders (void)
{
}
bool hal_gpio_read (uint32_t pin)
{
    (void) pin;
    return g_b_button_high;
}
void buddy2_stop (void)
{
    g_b_motion = false;
    ++g_stops;
}
bool buddy2_motion_active (void)
{
    return g_b_motion;
}
void buddy2_get_motion_state (buddy2_motion_state_t * p_state)
{
    buddy2_motion_state_t empty = {0};
    *p_state                    = empty;
    p_state->left_encoder_fault = g_b_encoder_fault;
}
void buddy2_set_motor_pwm (int16_t left, int16_t right)
{
    (void) left;
    (void) right;
}
bool buddy2_set_trim (uint8_t left_pct, uint8_t right_pct)
{
    (void) left_pct;
    (void) right_pct;
    return true;
}
bool buddy2_set_speed_limit (uint8_t percent)
{
    (void) percent;
    return true;
}
void buddy5_set_servo_angle (uint8_t angle)
{
    g_servo_angle = angle;
}
bool buddy2_turn_left (float32_t angle_deg, int16_t pwm)
{
    assert(90.0f == angle_deg);
    (void) pwm;
    ++g_lefts;
    g_b_motion = g_b_accept_move;
    return g_b_motion;
}
bool buddy2_turn_right (float32_t angle_deg, int16_t pwm)
{
    assert(90.0f == angle_deg);
    (void) pwm;
    ++g_rights;
    g_b_motion = g_b_accept_move;
    return g_b_motion;
}
bool buddy2_move_forward (float32_t distance_mm, int16_t pwm)
{
    assert(300.0f == distance_mm);
    (void) pwm;
    ++g_forwards;
    g_b_motion = g_b_accept_move;
    return g_b_motion;
}
void buddy3_start_line_follow (void)
{
    g_b_following = true;
    ++g_starts;
}
void buddy3_stop_line_follow (void)
{
    g_b_following = false;
    g_event       = 0;
}
bool buddy3_line_following (void)
{
    return g_b_following;
}
bool buddy3_take_barcode (buddy3_barcode_event_t * p_event)
{
    buddy3_barcode_event_t empty  = {0};
    *p_event                      = empty;
    p_event->kind                 = 1 == g_event ? B3BC_FORMAT : B3BC_VALID;
    p_event->decoded.symbol       = (char) g_event;
    p_event->decoded.reversed     = true;
    p_event->decoded.narrow_width = 5;
    p_event->decoded.wide_ratio   = 3;
    bool result                   = g_event != 0;
    g_event                       = 0;
    return result;
}
void buddy3_set_test_stage (uint8_t stage)
{
    g_stage = stage;
}
uint8_t buddy3_test_stage (void)
{
    return g_stage;
}
bool buddy3_barcode_calibrated (void)
{
    return g_b_calibrated;
}
bool buddy3_configure_barcode (uint16_t min_element_mm, int16_t offset_left_mm)
{
    (void) min_element_mm;
    (void) offset_left_mm;
    return g_b_calibrated;
}
void buddy3_get_calibration (buddy3_calibration_t * p_cal)
{
    buddy3_calibration_t calibration = {0};
    *p_cal                           = calibration;
}
void buddy3_set_calibration (const buddy3_calibration_t * p_cal)
{
    (void) p_cal;
}

static void reset (void)
{
    robot_stop_all();
    robot_controller_init();
    g_barcode_mode      = 1;
    g_stage             = 1;
    g_b_calibrated      = true;
    g_b_accept_move     = true;
    g_b_button_high     = true;
    g_line_button_count = 0;
    g_b_line_button_was = false;
    g_starts = g_stops = g_lefts = g_rights = g_forwards = 0;
    g_event                                              = 0;
    g_b_encoder_fault                                    = false;
    g_servo_angle                                        = 90;
    g_serial_log[0]                                      = 0;
}
/* Test-side packer for "name [value [value]]", the same layout the
 * dashboard's rover-wire.js sends.  A value the field cannot carry is never
 * sent (the dashboard refuses it too), so the robot's state is unchanged. */
typedef struct
{
    const char * p_name;
    uint8_t      opcode;
    uint8_t      widths[2];
    int32_t      offset;
} test_command_t;

static const test_command_t g_test_commands[] = {
    {"stop", ROBOT_WIRE_OP_STOP, {0u, 0u}, 0},
    {"line_follow", ROBOT_WIRE_OP_LINE_FOLLOW, {0u, 0u}, 0},
    {"line_stop", ROBOT_WIRE_OP_LINE_STOP, {0u, 0u}, 0},
    {"servo_angle", ROBOT_WIRE_OP_SERVO_ANGLE, {8u, 0u}, 0},
    {"barcode_mode", ROBOT_WIRE_OP_BARCODE_MODE, {1u, 0u}, 0},
    {"line_stage", ROBOT_WIRE_OP_LINE_STAGE, {2u, 0u}, 0},
    {"motor_trim", ROBOT_WIRE_OP_MOTOR_TRIM, {6u, 6u}, 50}};

static void command (const char * c, uint32_t now_ms)
{
    char                  name[24]  = {0};
    int                   values[2] = {0, 0};
    uint8_t               packed[ROBOT_WIRE_COMMAND_MAX];
    robot_packed_writer_t writer;
    size_t                entry = 0u;
    int  count  = sscanf(c, "%23s %d %d", name, &values[0], &values[1]);
    bool b_fits = true;

    while (0 != strcmp(name, g_test_commands[entry].p_name))
    {
        entry++;
        assert(entry < (sizeof(g_test_commands) / sizeof(g_test_commands[0])));
    }
    robot_packed_writer_init(&writer, packed, sizeof(packed));
    robot_packed_put(&writer, g_test_commands[entry].opcode, 4u);
    for (int index = 0;
         (index < 2) && (0u != g_test_commands[entry].widths[index]); index++)
    {
        int32_t  value = values[index] - g_test_commands[entry].offset;
        uint32_t width = g_test_commands[entry].widths[index];

        assert(count > (index + 1));
        b_fits = b_fits && (value >= 0) && ((uint32_t) value < (1u << width));
        robot_packed_put(&writer, (uint32_t) value, width);
    }
    if (b_fits)
    {
        robot_controller_handle_command(packed, robot_packed_length(&writer),
                                        now_ms);
    }
}
int main (void)
{
    reset();
    command("servo_angle 135", 0);
    assert(135 == g_servo_angle);
    command("servo_angle 45", 0);
    assert(45 == g_servo_angle);
    command("servo_angle 181", 0);
    assert(45 == g_servo_angle);
    command("line_follow", 0);
    assert(90 == g_servo_angle);
    command("servo_angle 135", 0);
    assert(90 == g_servo_angle);
    for (char letter = 'A'; letter <= 'Z'; ++letter)
    {
        reset();
        command("barcode_mode 2", 0);
        assert(1 == g_barcode_mode);
        /* GP21 must still start without a network or serial connection. */
        g_b_button_high = false;
        robot_controller_step(10);
        robot_controller_step(30);
        robot_controller_step(50);
        g_b_button_high = true;
        robot_controller_step(70);
        assert(g_b_following);
        g_event = (uint8_t) letter;
        robot_controller_step(90);
        assert(g_b_following && 1 == g_starts &&
               g_lefts + g_rights + g_forwards == 0);
        assert(strstr(g_serial_log, "Decoded:") &&
               strstr(g_serial_log, "reverse"));
        robot_controller_step(110);
        assert(!g_event && 1 == g_starts);
        g_b_button_high = false;
        robot_controller_step(130);
        robot_controller_step(150);
        robot_controller_step(170);
        assert(!g_b_following && !g_b_motion); /* GP21 still stops. */
    }
    reset();
    command("line_follow", 0);
    g_event = 1;
    robot_controller_step(10);
    assert(g_b_following && g_lefts + g_rights + g_forwards == 0 &&
           strstr(g_serial_log, "rejected"));
    reset();
    command("barcode_mode 0", 0);
    command("line_follow", 0);
    g_serial_log[0] = 0;
    g_event         = 'F';
    robot_controller_step(10);
    assert(!g_event && !strstr(g_serial_log, "Decoded:"));
    assert(g_b_following && g_lefts + g_rights + g_forwards == 0);
    command("stop", 20);
    assert(!g_b_following);
    reset();
    command("line_follow", 0);
    assert(robot_controller_begin_obstacle_avoidance());
    assert(!g_b_following && robot_controller_obstacle_avoidance_active());
    puts("PASS: read-only letters, rejection logs, mode gate, servo commands, "
         "GP21 start/stop, avoidance handoff");
    return 0;
}

/*** end of file ***/

