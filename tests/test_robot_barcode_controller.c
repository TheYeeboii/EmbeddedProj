#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
/* Compile the real command/button/handoff controller against fake hardware. */
#include "../firmware/app_program/robot_controller.c"

static bool     following, motion, accept_move = true, calibrated = true,
                                   button_high = true;
static bool     encoder_fault;
static uint8_t  stage = 1, event, servo_angle = 90;
static char     serial_log[512];
static unsigned starts, stops, lefts, rights, forwards;
int             tm_printf (const unsigned char * fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(serial_log, sizeof(serial_log), (const char *) fmt, args);
    va_end(args);
    return n;
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
    return button_high;
}
void buddy2_stop (void)
{
    motion = false;
    ++stops;
}
bool buddy2_motion_active (void)
{
    return motion;
}
void buddy2_get_motion_state (buddy2_motion_state_t * s)
{
    buddy2_motion_state_t empty = {0};
    *s                          = empty;
    s->left_encoder_fault       = encoder_fault;
}
void buddy2_set_motor_pwm (int16_t l, int16_t r)
{
    (void) l;
    (void) r;
}
bool buddy2_set_trim (uint8_t l, uint8_t r)
{
    (void) l;
    (void) r;
    return true;
}
bool buddy2_set_speed_limit (uint8_t p)
{
    (void) p;
    return true;
}
void buddy5_set_servo_angle (uint8_t angle)
{
    servo_angle = angle;
}
bool buddy2_turn_left (float a, int16_t p)
{
    assert(90.0f == a);
    (void) p;
    ++lefts;
    motion = accept_move;
    return motion;
}
bool buddy2_turn_right (float a, int16_t p)
{
    assert(90.0f == a);
    (void) p;
    ++rights;
    motion = accept_move;
    return motion;
}
bool buddy2_move_forward (float d, int16_t p)
{
    assert(300.0f == d);
    (void) p;
    ++forwards;
    motion = accept_move;
    return motion;
}
void buddy3_start_line_follow (void)
{
    following = true;
    ++starts;
}
void buddy3_stop_line_follow (void)
{
    following = false;
    event     = 0;
}
bool buddy3_line_following (void)
{
    return following;
}
bool buddy3_take_barcode (buddy3_barcode_event_t * v)
{
    buddy3_barcode_event_t empty = {0};
    *v                           = empty;
    v->kind                      = 1 == event ? B3BC_FORMAT : B3BC_VALID;
    v->decoded.symbol            = (char) event;
    v->decoded.reversed          = true;
    v->decoded.narrow_width      = 5;
    v->decoded.wide_ratio        = 3;
    bool result                  = event != 0;
    event                        = 0;
    return result;
}
void buddy3_set_test_stage (uint8_t s)
{
    stage = s;
}
uint8_t buddy3_test_stage (void)
{
    return stage;
}
bool buddy3_barcode_calibrated (void)
{
    return calibrated;
}
bool buddy3_configure_barcode (uint16_t c, int16_t o)
{
    (void) c;
    (void) o;
    return calibrated;
}
void buddy3_get_calibration (buddy3_calibration_t * c)
{
    buddy3_calibration_t z = {0};
    *c                     = z;
}
void buddy3_set_calibration (const buddy3_calibration_t * c)
{
    (void) c;
}

static void reset (void)
{
    robot_stop_all();
    robot_controller_init();
    g_barcode_mode      = 1;
    stage               = 1;
    calibrated          = true;
    accept_move         = true;
    button_high         = true;
    g_line_button_count = 0;
    g_b_line_button_was   = false;
    starts = stops = lefts = rights = forwards = 0;
    event                                      = 0;
    encoder_fault                              = false;
    servo_angle                                = 90;
    serial_log[0]                              = 0;
}
static void command (const char * c, uint32_t n)
{
    robot_controller_handle_command(c, n);
}
int main (void)
{
    reset();
    command("{\"cmd\":\"servo_angle\",\"angle\":135}", 0);
    assert(135 == servo_angle);
    command("servo_angle 45", 0);
    assert(45 == servo_angle);
    command("servo_angle 181", 0);
    assert(45 == servo_angle);
    command("line_follow", 0);
    assert(90 == servo_angle);
    command("servo_angle 135", 0);
    assert(90 == servo_angle);
    for (char letter = 'A'; letter <= 'Z'; ++letter)
    {
        reset();
        command("barcode_mode 2", 0);
        assert(1 == g_barcode_mode);
        /* GP21 must still start without a network or serial connection. */
        button_high = false;
        robot_controller_step(10);
        robot_controller_step(30);
        robot_controller_step(50);
        button_high = true;
        robot_controller_step(70);
        assert(following);
        event = (uint8_t) letter;
        robot_controller_step(90);
        assert(following && 1 == starts && lefts + rights + forwards == 0);
        assert(strstr(serial_log, "Decoded:") && strstr(serial_log, "reverse"));
        robot_controller_step(110);
        assert(!event && 1 == starts);
        button_high = false;
        robot_controller_step(130);
        robot_controller_step(150);
        robot_controller_step(170);
        assert(!following && !motion); /* GP21 still stops. */
    }
    reset();
    command("line_follow", 0);
    event = 1;
    robot_controller_step(10);
    assert(following && lefts + rights + forwards == 0 &&
           strstr(serial_log, "rejected"));
    reset();
    command("barcode_mode 0", 0);
    command("line_follow", 0);
    serial_log[0] = 0;
    event         = 'F';
    robot_controller_step(10);
    assert(!event && !strstr(serial_log, "Decoded:"));
    assert(following && lefts + rights + forwards == 0);
    command("stop", 20);
    assert(!following);
    reset();
    command("line_follow", 0);
    assert(robot_controller_begin_obstacle_avoidance());
    assert(!following && robot_controller_obstacle_avoidance_active());
    puts("PASS: read-only letters, rejection logs, mode gate, servo commands, "
         "GP21 start/stop, avoidance handoff");
    return 0;
}

/*** end of file ***/

