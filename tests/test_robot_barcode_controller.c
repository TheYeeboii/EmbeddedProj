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
static bool     g_b_manual_scan;
static uint8_t  g_stage = 1;
static uint8_t  g_event;
static uint8_t  g_servo_angle = 90;
static char     g_serial_log[4096];
static unsigned g_starts;
static unsigned g_stops;
static unsigned g_lefts;
static unsigned g_rights;
static unsigned g_forwards;
static uint8_t  g_turn_request;
static uint8_t  g_turn_reached;
#if BARCODE_DIGITAL_INPUT
void barcode_reading_set_enabled (bool enabled)
{
    assert(enabled == (g_barcode_mode != 0u));
}
#endif
int             tm_printf (const unsigned char * fmt, ...)
{
    /* Keep every line since the last clear: one step can print twice. */
    va_list args;
    size_t  used = strlen(g_serial_log);

    if (used > (sizeof(g_serial_log) - 256u))
    {
        used            = 0u;
        g_serial_log[0] = 0;
    }
    va_start(args, fmt);
    int written = vsnprintf(&g_serial_log[used], sizeof(g_serial_log) - used,
                            (const char *) fmt, args);
    va_end(args);
    return written;
}
void hal_gpio_input_pullup (uint32_t pin)
{
    (void) pin;
}
static bool g_b_lifted;
void        buddy4_reset_run (void)
{
}
/* A new start asks Buddy 4 to end a lift latch.  The stub leaves
 * g_b_lifted alone: an overturned rover reads lifted again at once. */
static unsigned g_lift_clears;
void            buddy4_clear_lift (void)
{
    ++g_lift_clears;
}
void buddy4_get_state (buddy4_imu_state_t * p_state)
{
    buddy4_imu_state_t empty = {0};
    *p_state                 = empty;
    p_state->lifted          = g_b_lifted ? 1u : 0u;
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
    p_state->left_speed_mm_s    = g_b_motion ? 100.0f : 0.0f;
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
    assert(ROBOT_JUNCTION_AXLE_MM == distance_mm);
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
void buddy3_set_manual_barcode (bool b_enabled)
{
    g_b_manual_scan = b_enabled;
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
    bool result = g_event != 0 && (!BARCODE_DIGITAL_INPUT || g_event != 1u);
    g_event                       = 0;
    return result;
}
static unsigned g_zeros;
static uint8_t  g_level_sensor;
static uint16_t g_white_level;
static uint16_t g_black_level;
bool buddy3_set_ir_levels (uint8_t sensor, uint16_t white, uint16_t black)
{
    bool b_ok = (sensor <= 2u) && (white < black) &&
                (((uint32_t) black - white) >= 200u);

    if (b_ok)
    {
        g_level_sensor = sensor;
        g_white_level  = white;
        g_black_level  = black;
    }
    return b_ok;
}
void buddy3_zero_barcode_white (void)
{
    ++g_zeros;
}
void buddy3_set_junction_turn (uint8_t turn)
{
    g_turn_request = turn;
}
bool buddy3_take_junction_turn (uint8_t * p_turn)
{
    bool b_taken = (0u != g_turn_reached);

    *p_turn        = g_turn_reached;
    g_turn_reached = 0u;
    return b_taken;
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
static int16_t   g_line_base_pwm;
static float32_t g_line_gain;
void             buddy3_set_line_speed (int16_t base_pwm, float32_t gain)
{
    g_line_base_pwm = base_pwm;
    g_line_gain     = gain;
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
    g_b_link_seen       = false;
    g_b_link_lost       = false;
    g_b_lifted          = false;
    robot_fault_update(ROBOT_FAULT_ALL, 0u);
    g_starts = g_stops = g_lefts = g_rights = g_forwards = 0;
    g_event                                              = 0;
    g_turn_request                                       = 0;
    g_turn_reached                                       = 0;
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
    uint8_t      widths[4];
    int32_t      offset;
} test_command_t;

static const test_command_t g_test_commands[] = {
    {"stop", ROBOT_WIRE_OP_STOP, {0u}, 0},
    {"reset_all", ROBOT_WIRE_OP_RESET_ALL, {0u, 0u}, 0},
    {"preset", ROBOT_WIRE_OP_DRIVE_PRESET, {2u, 0u}, 0},
    {"line_follow", ROBOT_WIRE_OP_LINE_FOLLOW, {0u, 0u}, 0},
    {"line_stop", ROBOT_WIRE_OP_LINE_STOP, {0u, 0u}, 0},
    {"servo_angle", ROBOT_WIRE_OP_SERVO_ANGLE, {8u, 0u}, 0},
    {"barcode_mode", ROBOT_WIRE_OP_BARCODE_MODE, {1u, 2u, 12u, 12u}, 0},
    {"line_stage", ROBOT_WIRE_OP_LINE_STAGE, {2u, 0u}, 0},
    {"line_speed", ROBOT_WIRE_OP_LINE_SPEED, {7u, 6u}, 0},
    {"motor_trim", ROBOT_WIRE_OP_MOTOR_TRIM, {6u, 6u}, 50}};

static void command (const char * c, uint32_t now_ms)
{
    char                  name[24]  = {0};
    int                   values[4] = {0, 0, 0, 0};
    uint8_t               packed[ROBOT_WIRE_COMMAND_MAX];
    robot_packed_writer_t writer;
    size_t                entry = 0u;
    int  count  = sscanf(c, "%23s %d %d %d %d", name, &values[0], &values[1],
                         &values[2], &values[3]);
    bool b_fits = true;

    while (0 != strcmp(name, g_test_commands[entry].p_name))
    {
        entry++;
        assert(entry < (sizeof(g_test_commands) / sizeof(g_test_commands[0])));
    }
    robot_packed_writer_init(&writer, packed, sizeof(packed));
    robot_packed_put(&writer, g_test_commands[entry].opcode, 4u);
    for (int index = 0;
         (index < 4) && (0u != g_test_commands[entry].widths[index]); index++)
    {
        int32_t  value = values[index] - g_test_commands[entry].offset;
        uint32_t width = g_test_commands[entry].widths[index];

        /* Missing trailing values are sent as 0. */
        assert((count > (index + 1)) || (index > 0));
        b_fits = b_fits && (value >= 0) && ((uint32_t) value < (1u << width));
        robot_packed_put(&writer, (uint32_t) value, width);
    }
    if (b_fits)
    {
        robot_controller_handle_command(packed, robot_packed_length(&writer),
                                        now_ms);
    }
}
/* Line speed in percent of full power sets only Buddy 3's steering
 * settings; an out-of-range value changes nothing; reset_all restores the
 * boot speed (60 % straight, 27 % extra on curves). */
static void test_line_speed (void)
{
    reset();
    command("line_speed 40 10", 0);
    assert(13106 == g_line_base_pwm); /* 40 % of 32767 */
    assert((g_line_gain > 3.27f) && (g_line_gain < 3.28f));
    command("line_speed 90 10", 0); /* Above the 80 % limit. */
    assert(13106 == g_line_base_pwm);
    command("line_speed 40 61", 0); /* Above the 60 % curve limit. */
    assert((g_line_gain > 3.27f) && (g_line_gain < 3.28f));
    command("reset_all", 0);
    assert(19660 == g_line_base_pwm); /* 60 % */
    assert((g_line_gain > 8.84f) && (g_line_gain < 8.85f));
}

/* Change Requirement 1: mode and run flags share one locked byte. */
static void test_packed_state (void)
{
    reset();
    command("preset 0", 0); /* forward */
    assert((uint8_t) (ROBOT_MODE_DRIVE | ROBOT_CTL_DRIVE_FORWARD) ==
           (g_controller_state & (ROBOT_CTL_MODE_MASK | ROBOT_CTL_RUN_FLAGS)));
    assert(robot_controller_obstacle_watch_enabled());
    assert(1u == robot_controller_get_mode()); /* Telemetry index: drive. */
    command("preset 1", 10);                   /* reverse */
    assert(!robot_ctl_flag(ROBOT_CTL_DRIVE_FORWARD));
    assert(!robot_controller_obstacle_watch_enabled());
    command("stop", 20);
    assert((uint8_t) ROBOT_MODE_IDLE == (g_controller_state & 0x7Fu));
}

/* Manual barcode reading: on only while both wheels drive forward, and a
 * held key (the same drive command again) keeps it on without restarting. */
static void test_manual_barcode (void)
{
    reset();
    command("preset 0", 0); /* forward */
    assert(g_b_manual_scan && !g_b_following);
    command("preset 0", 300); /* key held: same drive again */
    assert(g_b_manual_scan);
    command("preset 2", 600); /* left: one wheel stopped, not straight */
    assert(!g_b_manual_scan);
    command("preset 0", 900);
    command("preset 1", 1200); /* reverse */
    assert(!g_b_manual_scan);
    command("preset 0", 1500);
    command("stop", 1600); /* releasing W only pauses the scan */
    assert(g_b_manual_scan);
    g_zeros = 0u;
    command("reset_all", 1650);
    assert(!g_b_manual_scan);
    assert((BARCODE_DIGITAL_INPUT ? 0u : 1u) == g_zeros);
    command("preset 0", 1700);
    command("line_follow", 1800); /* line following reads barcodes itself */
    assert(!g_b_manual_scan && g_b_following);
    command("preset 0", 1900); /* driving by hand stops line following */
    assert(g_b_manual_scan && !g_b_following);
}

/* Team decision: Wi-Fi loss stops line following after a 2 s grace, but
 * only a link that was up counts. */
static void test_link_loss (void)
{
    reset();
    command("line_follow", 0);
    robot_controller_step(1000); /* Link up: it can now be lost. */
    robot_fault_set(ROBOT_FAULT_WIFI_LOSS);
    robot_controller_step(1020); /* Loss seen; grace starts. */
    robot_controller_step(3010);
    assert(g_b_following); /* 1990 ms: still inside the grace period. */
    robot_controller_step(3020);
    assert(!g_b_following && (ROBOT_MODE_IDLE == robot_ctl_mode()));
    assert(strstr(g_serial_log, "Wi-Fi lost"));
    /* A deliberate GP21 start afterwards is not stopped again. */
    command("line_follow", 3040);
    robot_controller_step(9000);
    assert(g_b_following);
    /* A robot that never had a network is never stopped by this rule. */
    reset();
    robot_fault_set(ROBOT_FAULT_WIFI_LOSS);
    command("line_follow", 0);
    robot_controller_step(10);
    robot_controller_step(60000);
    assert(g_b_following);
}

/* Lifting or overturning the rover ends the run: every motor stops and
 * stays stopped after it is put down again. */
static void test_lifted_stop (void)
{
    reset();
    command("line_follow", 0);
    robot_controller_step(10);
    assert(g_b_following);
    g_b_lifted = true;
    robot_controller_step(30);
    assert(!g_b_following && !g_b_motion &&
           (ROBOT_MODE_IDLE == robot_ctl_mode()));
    assert(strstr(g_serial_log, "lifted or overturned"));
    g_b_lifted = false;
    robot_controller_step(50);
    assert(!g_b_following && (ROBOT_MODE_IDLE == robot_ctl_mode()));
    /* Still overturned: a new drive is stopped at the next step. */
    g_b_lifted    = true;
    g_lift_clears = 0u;
    command("preset 0", 60);
    assert(1u == g_lift_clears); /* a new drive ends a lift latch */
    assert(ROBOT_MODE_DRIVE == robot_ctl_mode());
    robot_controller_step(70);
    assert(ROBOT_MODE_IDLE == robot_ctl_mode());
    /* Idle and lifted: nothing to stop, no repeated log line. */
    g_serial_log[0] = 0;
    robot_controller_step(90);
    assert(0 == g_serial_log[0]);
    /* Idle and lifted, but a late PWM write left a wheel turning. */
    g_b_motion = true;
    robot_controller_step(110);
    assert(!g_b_motion);
}

/* Barcode sensor levels from the dashboard: applied at any time (even
 * while line following); both 0 leaves them alone; a bad pair is refused. */
static void test_barcode_levels (void)
{
    reset();
    g_white_level = g_black_level = 0u;
    command("line_follow", 0);
    command("barcode_mode 1 0 1300 3900", 10);
    assert(0u == g_level_sensor && 1300u == g_white_level &&
           3900u == g_black_level);
    assert(g_b_following && 1 == g_barcode_mode); /* Run not disturbed. */
    command("barcode_mode 1", 20);
    assert(1300u == g_white_level && 3900u == g_black_level);
    command("barcode_mode 1 2 250 3000", 25); /* Right line sensor. */
    assert(2u == g_level_sensor && 250u == g_white_level);
    g_white_level = 1300u;
    command("barcode_mode 1 0 3000 3100", 30);
    assert(1300u == g_white_level && strstr(g_serial_log, "rejected"));
    /* A mode change still needs the rover stopped. */
    command("barcode_mode 0", 40);
    assert(1 == g_barcode_mode);
    command("stop", 50);
    command("barcode_mode 0", 60);
    assert(0 == g_barcode_mode);
}

/* Barcode letters: A turns left and C right at the next junction; any
 * other letter means straight on, and the latest letter wins. */
static void test_barcode_turns (void)
{
    reset();
    command("line_follow", 0);
    g_event = 'A';
    robot_controller_step(10);
    if (BARCODE_DIGITAL_INPUT)
    {
        assert(strstr(g_serial_log, "Decoded: A"));
        g_event = 'C';
        robot_controller_step(20);
        assert(strstr(g_serial_log, "Decoded: C"));
        assert(g_b_following && BUDDY3_TURN_NONE == g_turn_request);
        assert(0u == (g_lefts + g_rights + g_forwards));
        return;
    }
    assert(BUDDY3_TURN_LEFT == g_turn_request &&
           strstr(g_serial_log, "A: turn left"));
    g_event = 'F';
    robot_controller_step(20);
    assert(BUDDY3_TURN_NONE == g_turn_request);
    g_event = 'C';
    robot_controller_step(30);
    assert(BUDDY3_TURN_RIGHT == g_turn_request);
    assert(0u == (g_lefts + g_rights + g_forwards)); /* Nothing until then. */

    /* Buddy 3 stops on the junction and hands over the turn. */
    g_b_following  = false;
    g_turn_reached = BUDDY3_TURN_RIGHT;
    robot_controller_step(40);
    assert(1u == g_forwards && (ROBOT_MODE_MOTION == robot_ctl_mode()));
    robot_controller_step(50); /* Still moving: nothing new. */
    assert(1u == g_forwards && 0u == g_rights);
    g_b_motion = false; /* Wheels now over the junction. */
    robot_controller_step(60);
    assert(1u == g_rights && 0u == g_lefts && 1u == g_starts);
    g_b_motion = false; /* Turn done: line following again. */
    robot_controller_step(70);
    assert(2u == g_starts && g_b_following &&
           (ROBOT_MODE_LINE == robot_ctl_mode()));
    robot_controller_step(80);
    assert(2u == g_starts && 1u == g_rights);

    /* Left turn; an encoder fault after the forward move stops it. */
    reset();
    command("line_follow", 0);
    g_b_following  = false;
    g_turn_reached = BUDDY3_TURN_LEFT;
    robot_controller_step(10);
    g_b_motion        = false;
    g_b_encoder_fault = true;
    robot_controller_step(20);
    assert(0u == g_lefts && (ROBOT_MODE_IDLE == robot_ctl_mode()));
    assert(strstr(g_serial_log, "encoder fault"));

    /* Letters read outside line following never set a turn. */
    reset();
    command("preset 0", 0);
    g_event = 'A';
    robot_controller_step(10);
    assert(BUDDY3_TURN_NONE == g_turn_request);

    /* GP21 stop during the turn ends it. */
    reset();
    command("line_follow", 0);
    g_b_following  = false;
    g_turn_reached = BUDDY3_TURN_LEFT;
    robot_controller_step(10);
    command("stop", 20);
    g_b_motion = false;
    robot_controller_step(30);
    assert(0u == g_lefts && 1u == g_starts &&
           (ROBOT_MODE_IDLE == robot_ctl_mode()));
}

int main (void)
{
    test_line_speed();
    test_packed_state();
    test_manual_barcode();
    test_link_loss();
    test_lifted_stop();
    test_barcode_turns();
    test_barcode_levels();
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
               (0u == (g_lefts + g_rights + g_forwards)));
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
    assert(g_b_following && (0u == (g_lefts + g_rights + g_forwards)));
    assert((strstr(g_serial_log, "rejected") != NULL) == !BARCODE_DIGITAL_INPUT);
    reset();
    command("barcode_mode 0", 0);
    command("line_follow", 0);
    g_serial_log[0] = 0;
    g_event         = 'F';
    robot_controller_step(10);
    assert(!g_event && !strstr(g_serial_log, "Decoded:"));
    assert(g_b_following && (0u == (g_lefts + g_rights + g_forwards)));
    command("stop", 20);
    assert(!g_b_following);
    reset();
    command("line_follow", 0);
    assert(robot_controller_begin_obstacle_avoidance());
    assert(!g_b_following && robot_controller_obstacle_avoidance_active());
    puts(BARCODE_DIGITAL_INPUT ? "PASS: digital A/C scans are logged without turns; button, Wi-Fi and lift stops still work" :
         "PASS: letters move nothing until a junction, rejection logs, mode "
         "gate, servo commands, GP21 start/stop, avoidance handoff, "
         "packed state, manual barcode, Wi-Fi-loss stop, A/C junction "
         "turns, sensor levels, line speed");
    return 0;
}

/*** end of file ***/

