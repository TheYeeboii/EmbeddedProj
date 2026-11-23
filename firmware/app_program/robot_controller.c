/**
 * @file    robot_controller.c
 * @brief   Vehicle controller: decides what the robot does next.
 *
 * Ported from handle_command_payload() and the barcode / safety parts of
 * the old main.c.  Behaviour kept the same, except:
 *   - "stop" now also stops line following and any distance/turn move.
 *   - Raw drive commands stop line following (otherwise the line follower
 *     would overwrite them 10 ms later).
 *   - Barcode letters are printed (mode 1, the default; mode 0 discards
 *     them).  While line following, A turns left and C right at the next
 *     junction; any other letter goes straight on.
 *   - Lifting or overturning the rover (Buddy 4) stops every motor; this
 *     ends a demo run.
 *
 * PWM UNITS: the dashboard still sends the old 0..65535 scale.  Since the
 * PWM fix, the motion library uses 0..32767 = 0..100 %, so dashboard values
 * are halved (ROBOT_LEGACY_TO_PWM) to give exactly the same duty cycle as
 * before.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy2_motion.h"
#include "buddy3_ir.h"
#include "buddy4_imu.h"
#include "buddy5_ultrasonic.h"
#include "pin_config.h"
#include "subsystem3/barcode_reading.h"
#include "robot_controller.h"
#include "robot_fault.h"
#include "robot_steering_trace.h"
#include "robot_hal.h"
#include "barcode_tuning.h"
#include "robot_wire.h"

#define ROBOT_LEGACY_MAX (65535)

/* Limits for the dashboard "line_speed" command, percent of full power. */
#define ROBOT_LINE_BASE_MIN_PCT (15)
#define ROBOT_LINE_BASE_MAX_PCT (80)
#define ROBOT_LINE_TURN_MAX_PCT (60)
/* Boot line speed; buddy3_ir.c starts with the same base PWM. */
#define ROBOT_LINE_BASE_DEFAULT_PCT (60)
#define ROBOT_LINE_TURN_DEFAULT_PCT (27)
/* Curve power in percent to Buddy 3's steering gain: percent of full PWM,
 * and the gain is stored per 1000 line-position units, so 100 x 1000. */
#define ROBOT_LINE_GAIN_DIVISOR (100000.0f)

#define ROBOT_FORWARD_SPEED       (30000u) /* dashboard units */
#define ROBOT_REVERSE_SPEED       (20000u)
#define ROBOT_DRIVE_TIMEOUT_MS    (1000u) /* raw drive safety stop */
#define ROBOT_SERVO_MIN_ANGLE     (0)
#define ROBOT_SERVO_MAX_ANGLE     (180)
#define ROBOT_SERVO_FORWARD_ANGLE (90u)
#define ROBOT_PERCENT_MAX         (100)

/* Accepted ranges of the barcode commands. */
#define ROBOT_LINE_STAGE_MAX       (3)
#define ROBOT_BARCODE_MODE_MAX     (1)
#define ROBOT_BARCODE_MODE_OBSERVE (1u)

#define ROBOT_BUTTON_DEBOUNCE_STEPS (3u) /* 3 x 20 ms = 60 ms */
/* Team decision: line following stops once Wi-Fi has been lost this long. */
#define ROBOT_LINK_LOSS_GRACE_MS (2000u)
/* Lifted and idle: a wheel faster than this is still driven, so stop it. */
#define ROBOT_LIFT_WHEEL_MM_S (5.0f)

typedef enum
{
    ROBOT_MODE_IDLE = 0,
    ROBOT_MODE_DRIVE,
    ROBOT_MODE_MOTION,
    ROBOT_MODE_LINE,
    ROBOT_MODE_AVOID
} robot_mode_t;

/* Packed controller state (Change Requirement 1): one byte.
 *   bits 0-2  mode (robot_mode_t, 5 values)
 *   bit  4    resume line following when obstacle avoidance ends
 *   bit  5    the manual drive is forwards (watched for obstacles)
 *   bit  7    the GP21 button was pressed at the last step
 * Written by the controller task AND the Buddy 5 avoidance task (the
 * robot_controller_*_obstacle_avoidance() calls), so every change is a
 * read-modify-write inside DI()/EI() (robot_ctl_update); a read is one byte
 * load and needs no lock. */
#define ROBOT_CTL_MODE_MASK      (0x07u)
#define ROBOT_CTL_AVOID_RESUME   (0x10u)
#define ROBOT_CTL_DRIVE_FORWARD  (0x20u)
#define ROBOT_CTL_BUTTON_WAS     (0x80u)
/* Flags that describe the current run; every mode change rewrites them. */
#define ROBOT_CTL_RUN_FLAGS (ROBOT_CTL_AVOID_RESUME | ROBOT_CTL_DRIVE_FORWARD)
#define ROBOT_CTL_ALL (0xFFu)

static volatile uint8_t g_controller_state = (uint8_t) ROBOT_MODE_IDLE;
static uint32_t         g_drive_left       = 0u;
static uint32_t         g_drive_right      = 0u;
static uint32_t         g_last_drive_ms    = 0u;
/* Controller task only: GP21 debounce, 0..3 (2 bits used). */
static uint8_t g_line_button_count = 0u;
/* 0 = off, 1 = read: log letters and turn on A/C (1 bit used). */
static uint8_t g_barcode_mode = ROBOT_BARCODE_MODE_OBSERVE;
/* Controller task only: Wi-Fi-loss safety stop (robot_check_link_loss). */
static bool     g_b_link_seen  = false;
static bool     g_b_link_lost  = false;
static uint32_t g_link_lost_ms = 0u;

static robot_mode_t robot_ctl_mode (void);
static void         robot_ctl_enter (robot_mode_t mode, uint8_t run_flags);
static bool         robot_ctl_flag (uint8_t flag);
static void         robot_ctl_set_flag (uint8_t flag, bool b_value);
static void         robot_ctl_update (uint8_t clear_mask, uint8_t set_mask);

static int16_t  robot_legacy_to_pwm (uint32_t legacy);
static uint32_t robot_clamp_legacy (int32_t value);
static void     robot_stop_all (void);
static void     robot_drive (uint32_t left, uint32_t right, int32_t sign,
                             uint32_t now_ms);
static void     robot_set_trim (int32_t left, int32_t right);
static void     robot_set_speed_limit (int32_t percent);
static void     robot_set_servo_angle (int32_t angle);
static void     robot_set_line_speed (int32_t base_pct, int32_t turn_pct);
static void     robot_start_line (void);
static void     robot_stop_line (void);
static void     robot_set_line_stage (int32_t stage);
static void     robot_set_barcode_mode (int32_t mode);
static void     robot_barcode_settings (const int32_t * p_values);
static void     robot_reset_all (void);
static bool     robot_run_simple (uint8_t opcode);
static void     robot_drive_preset (int32_t preset, uint32_t now_ms);
static void     robot_run_valued (const robot_wire_command_t * p_command,
                                  uint32_t                     now_ms);
static void     robot_check_link_loss (uint32_t now_ms);
static void     robot_check_lifted (void);

/* ------------------------------------------------------------------ *
 *  Public API
 * ------------------------------------------------------------------ */

void robot_controller_init (void)
{
    robot_ctl_update(ROBOT_CTL_ALL, (uint8_t) ROBOT_MODE_IDLE);
    hal_gpio_input_pullup(BUTTON_LINE_PIN);
}

void robot_controller_handle_command (const uint8_t * p_data, uint32_t length,
                                      uint32_t now_ms)
{
    robot_wire_command_t command = {0u, {0, 0, 0, 0}};

    /* Wrong length, unknown opcode or stray padding bits: ignore it. */
    if (!robot_wire_decode_command(p_data, length, &command))
    {
        tm_printf((UB *) "[cmd] ignored: %u-byte message is not a valid "
                         "command\n",
                  (UINT) length);
    }
    else if (robot_run_simple(command.opcode))
    {
        /* Handled: a command without values. */
    }
    else
    {
        robot_run_valued(&command, now_ms);
    }
}

void robot_controller_step (uint32_t now_ms)
{
    static uint32_t last_barcode_sequence;
    barcode_reading_result_t barcode;
    bool b_button_now = false;

    if (barcode_reading_latest(&barcode) && barcode.sequence != last_barcode_sequence)
    {
        last_barcode_sequence = barcode.sequence;
        if (ROBOT_BARCODE_MODE_OBSERVE == g_barcode_mode)
        {
            tm_printf((UB *) "[barcode] Decoded: %c (%s, GPIO timing)\n",
                      barcode.symbol, barcode.reversed ? "reverse" : "forward");
        }
    }
    /* GP21 button: toggle line following (on press, not while held).
     * Debounced: the pin must read "pressed" for 3 steps in a row, so one
     * press can never toggle twice because of contact bounce. */
    if (!hal_gpio_read(BUTTON_LINE_PIN))
    {
        if (g_line_button_count < ROBOT_BUTTON_DEBOUNCE_STEPS)
        {
            g_line_button_count++;
        }
    }
    else
    {
        g_line_button_count = 0u;
    }
    b_button_now = (g_line_button_count >= ROBOT_BUTTON_DEBOUNCE_STEPS);
    if (b_button_now && !robot_ctl_flag(ROBOT_CTL_BUTTON_WAS))
    {
        if (buddy3_line_following() ||
            (ROBOT_MODE_MOTION == robot_ctl_mode()) ||
            (ROBOT_MODE_AVOID == robot_ctl_mode()))
        {
            robot_stop_line();
        }
        else
        {
            robot_start_line();
        }
    }
    robot_ctl_set_flag(ROBOT_CTL_BUTTON_WAS, b_button_now);

    /* Safety: a raw drive command must be refreshed within 1 s. */
    if ((ROBOT_MODE_DRIVE == robot_ctl_mode()) &&
        ((now_ms - g_last_drive_ms) > ROBOT_DRIVE_TIMEOUT_MS))
    {
        robot_stop_all();
        tm_printf((UB *) "[safety] drive command timed out - stopped\n");
    }
    robot_check_link_loss(now_ms);
    robot_check_lifted();
}

void robot_controller_get_drive (uint32_t * p_left, uint32_t * p_right)
{
    if ((NULL != p_left) && (NULL != p_right))
    {
        *p_left  = g_drive_left;
        *p_right = g_drive_right;
    }
}

const char * robot_controller_mode_name (void)
{
    /* Indexed by robot_mode_t. */
    static const char * const names[] = {"idle", "drive", "motion", "line",
                                         "avoid"};

    return names[robot_ctl_mode()];
}

uint8_t robot_controller_get_mode (void)
{
    /* robot_mode_t has 5 values, so the index fits the 3-bit wire field. */
    return (uint8_t) robot_ctl_mode();
}

bool robot_controller_obstacle_watch_enabled (void)
{
    /* The forward-facing sensor can take over while driving forward or
     * following a line. */
    return (
        ((ROBOT_MODE_DRIVE == robot_ctl_mode()) &&
         robot_ctl_flag(ROBOT_CTL_DRIVE_FORWARD)) ||
        ((ROBOT_MODE_LINE == robot_ctl_mode()) && buddy3_line_following()));
}

bool robot_controller_begin_obstacle_avoidance (void)
{
    bool b_begun     = robot_controller_obstacle_watch_enabled();
    bool b_from_line = (ROBOT_MODE_LINE == robot_ctl_mode());

    if (b_begun)
    {
        /* Release line steering before giving the wheels to avoidance. */
        buddy3_stop_line_follow();
        buddy2_stop();
        g_drive_left  = 0u;
        g_drive_right = 0u;
        robot_steering_owner(TRACE_AVOIDANCE);
        /* Remember the source (read before stopping line following) so
         * completion can hand a line-follow run back to Buddy 3. */
        robot_ctl_enter(ROBOT_MODE_AVOID,
                        b_from_line ? ROBOT_CTL_AVOID_RESUME : 0u);
    }

    return b_begun;
}

bool robot_controller_obstacle_avoidance_active (void)
{
    return (ROBOT_MODE_AVOID == robot_ctl_mode());
}

void robot_controller_finish_obstacle_avoidance (void)
{
    if (ROBOT_MODE_AVOID == robot_ctl_mode())
    {
        buddy2_stop();
        g_drive_left  = 0u;
        g_drive_right = 0u;
        if (robot_ctl_flag(ROBOT_CTL_AVOID_RESUME))
        {
            tm_printf(
                (UB *) "[avoid] handing control back to line following\n");
            robot_start_line(); /* Clears the flags, mode becomes line. */
        }
        else
        {
            robot_ctl_enter(ROBOT_MODE_IDLE, 0u);
        }
    }
}

void robot_controller_abort_obstacle_avoidance (void)
{
    if (ROBOT_MODE_AVOID == robot_ctl_mode())
    {
        robot_stop_all();
    }
}

/* ------------------------------------------------------------------ *
 *  Actions
 * ------------------------------------------------------------------ */

/* Dashboard units 0..65535 are half the motor API range. */
static int16_t robot_legacy_to_pwm (uint32_t legacy)
{
    /* legacy <= 65535, so legacy / 2 <= 32767 fits int16_t. */
    return (int16_t) (legacy / 2u);
}

static uint32_t robot_clamp_legacy (int32_t value)
{
    uint32_t result = 0u;

    if (value < 0)
    {
        result = 0u;
    }
    else if (value > ROBOT_LEGACY_MAX)
    {
        result = (uint32_t) ROBOT_LEGACY_MAX;
    }
    else
    {
        result = (uint32_t) value; /* 0..65535 here. */
    }

    return result;
}

static void robot_stop_all (void)
{
    /* Stop line following only if it owns the wheels. The digital barcode
     * reader tracks its own capture timeout independently of the motors. */
    if (buddy3_line_following())
    {
        buddy3_stop_line_follow();
    }
    buddy2_stop();
    g_drive_left  = 0u;
    g_drive_right = 0u;
    robot_ctl_enter(ROBOT_MODE_IDLE, 0u);
}

/* Raw motor command.  Signs: +1 forward, -1 reverse. */
static void robot_drive (uint32_t left, uint32_t right, int32_t sign,
                         uint32_t now_ms)
{
    bool b_forward = (sign > 0) && (left > 0u) && (right > 0u);

    if ((left > 0u) || (right > 0u))
    {
        buddy4_clear_lift(); /* Driving again: the rover is on the ground. */
    }

    /* A held key re-sends this command every 300 ms. */
    if (buddy3_line_following())
    {
        buddy3_stop_line_follow();
    }
    buddy2_stop();
    /* A manual sonar test may have left the scanner looking sideways. Any
     * drive command must restore the forward view before motion begins. */
    buddy5_set_servo_angle(ROBOT_SERVO_FORWARD_ANGLE);
    robot_steering_owner(TRACE_MANUAL);
    g_drive_left    = left;
    g_drive_right   = right;
    g_last_drive_ms = now_ms;
    /* sign is +/-1 and the PWM is at most 32767, so the products fit. */
    buddy2_set_motor_pwm((int16_t) (sign * robot_legacy_to_pwm(left)),
                         (int16_t) (sign * robot_legacy_to_pwm(right)));
    robot_ctl_enter(((0u == left) && (0u == right)) ? ROBOT_MODE_IDLE
                                                    : ROBOT_MODE_DRIVE,
                    b_forward ? ROBOT_CTL_DRIVE_FORWARD : 0u);
}

static void robot_set_trim (int32_t left, int32_t right)
{
    /* Both values are checked to be 0..100 before the casts. */
    if ((left >= 0) && (left <= ROBOT_PERCENT_MAX) && (right >= 0) &&
        (right <= ROBOT_PERCENT_MAX) &&
        buddy2_set_trim((uint8_t) left, (uint8_t) right))
    {
        tm_printf((UB *) "[cmd] motor trim L %d R %d (percent)\n", (INT) left,
                  (INT) right);
    }
    else
    {
        tm_printf((UB *) "[cmd] motor trim rejected (each %d..100): %d %d\n",
                  (INT) BUDDY2_TRIM_MIN, (INT) left, (INT) right);
    }
}

static void robot_set_speed_limit (int32_t percent)
{
    /* The value is checked to be in range before the cast. */
    if ((percent >= (int32_t) BUDDY2_SPEED_MIN) &&
        (percent <= ROBOT_PERCENT_MAX) &&
        buddy2_set_speed_limit((uint8_t) percent))
    {
        tm_printf((UB *) "[cmd] speed limit %d%%\n", (INT) percent);
    }
    else
    {
        tm_printf((UB *) "[cmd] speed limit rejected (%d..100): %d\n",
                  (INT) BUDDY2_SPEED_MIN, (INT) percent);
    }
}

/** Set the ultrasonic scanner angle for a stationary mechanical check. */
static void robot_set_servo_angle (int32_t angle)
{
    if (ROBOT_MODE_IDLE != robot_ctl_mode())
    {
        tm_printf((UB *) "[cmd] servo rejected: stop the rover before moving "
                         "the scanner\n");
    }
    else if ((angle < ROBOT_SERVO_MIN_ANGLE) || (angle > ROBOT_SERVO_MAX_ANGLE))
    {
        tm_printf((UB *) "[cmd] servo angle rejected (0..180): %d\n",
                  (INT) angle);
    }
    else
    {
        buddy5_set_servo_angle((uint8_t) angle); /* 0..180 checked above. */
        tm_printf((UB *) "[cmd] servo angle %d deg\n", (INT) angle);
    }
}

/* Line-follow speed in percent of full power: base = straight speed,
 * turn = extra/less power on the outer/inner wheel on curves. */
static void robot_set_line_speed (int32_t base_pct, int32_t turn_pct)
{
    if ((base_pct < ROBOT_LINE_BASE_MIN_PCT) ||
        (base_pct > ROBOT_LINE_BASE_MAX_PCT) || (turn_pct < 0) ||
        (turn_pct > ROBOT_LINE_TURN_MAX_PCT))
    {
        tm_printf((UB *) "[cmd] line speed rejected (base %d..%d, turn 0..%d): "
                         "%d %d\n",
                  (INT) ROBOT_LINE_BASE_MIN_PCT, (INT) ROBOT_LINE_BASE_MAX_PCT,
                  (INT) ROBOT_LINE_TURN_MAX_PCT, (INT) base_pct,
                  (INT) turn_pct);
    }
    else
    {
        /* base_pct <= 80, so the scaled PWM fits int16_t. */
        buddy3_set_line_speed(
            (int16_t) ((base_pct * BUDDY2_PWM_MAX) / ROBOT_PERCENT_MAX),
            ((float32_t) turn_pct * (float32_t) BUDDY2_PWM_MAX) /
                ROBOT_LINE_GAIN_DIVISOR);
        tm_printf((UB *) "[cmd] line speed base %d turn %d (percent)\n",
                  (INT) base_pct, (INT) turn_pct);
    }
}

static void robot_start_line (void)
{
    buddy4_clear_lift(); /* Starting again: the rover is on the ground. */
    buddy2_stop();
    g_drive_left  = 0u;
    g_drive_right = 0u;
    buddy5_set_servo_angle(ROBOT_SERVO_FORWARD_ANGLE);
    buddy3_start_line_follow();
    robot_steering_owner(TRACE_TRACKING);
    robot_ctl_enter(ROBOT_MODE_LINE, 0u);
    tm_printf((UB *) "[cmd] line following ON\n");
}

static void robot_stop_line (void)
{
    robot_stop_all();
    tm_printf((UB *) "[cmd] line following OFF\n");
}

static void robot_set_line_stage (int32_t stage)
{
    if ((stage < 0) || (stage > ROBOT_LINE_STAGE_MAX))
    {
        tm_printf((UB *) "[cmd] line_stage must be 0..3\n");
    }
    else
    {
        robot_stop_all();
        buddy3_set_test_stage((uint8_t) stage); /* 0..3 checked above. */
        tm_printf(
            (UB *) "[cmd] line_stage %d selected; send line_follow to start\n",
            (INT) stage);
    }
}

static void robot_set_barcode_mode (int32_t mode)
{
    if ((mode < 0) || (mode > ROBOT_BARCODE_MODE_MAX) ||
        (ROBOT_MODE_IDLE != robot_ctl_mode()))
    {
        tm_printf((UB *) "[barcode] stop first; use 0=off or 1=read\n");
    }
    else
    {
        g_barcode_mode = (uint8_t) mode; /* 0 or 1 checked above. */
        barcode_reading_set_enabled(0 != mode);
        tm_printf((UB *) "[barcode] mode=%d (0 off, 1 read)\n", (INT) mode);
    }
}

/* Barcode command: mode, then one IR sensor (0 barcode, 1 left, 2 right)
 * and its white and black levels (both 0 = leave them).  Levels apply at
 * any time; the mode only changes when it differs, and only while
 * stopped. */
static void robot_barcode_settings (const int32_t * p_values)
{
    /* Reserved sensor 3 configures raw barcode hysteresis; mode is polarity.
     * This command never changes barcode navigation or motor ownership. */
    if (3 == p_values[1])
    {
        (void) barcode_tuning_request((uint16_t) p_values[2],
                                      (uint16_t) p_values[3],
                                      0 != p_values[0]);
        return;
    }
    if ((0 != p_values[2]) || (0 != p_values[3]))
    {
        /* 2- and 12-bit wire fields, so they fit their types. */
        if (!buddy3_set_ir_levels((uint8_t) p_values[1], (uint16_t) p_values[2],
                                  (uint16_t) p_values[3]))
        {
            tm_printf((UB *) "[ir] levels rejected: sensor %d white %d black "
                             "%d (black must be at least %u above white)\n",
                      (INT) p_values[1], (INT) p_values[2], (INT) p_values[3],
                      (UINT) BUDDY3_LEVEL_MIN_SPAN);
        }
    }
    if ((int32_t) g_barcode_mode != p_values[0])
    {
        robot_set_barcode_mode(p_values[0]);
    }
}

/* Dashboard "Reset everything": stop, then return every runtime setting and
 * counter to its boot value without rebooting the Pico. */
static void robot_reset_all (void)
{
    (void) ir_tuning_request(0u, 0u, 0u, false);
    (void) ir_tuning_request(1u, 0u, 0u, false);
    (void) ir_tuning_request(2u, 0u, 0u, false);
    robot_stop_all();
    buddy2_reset_encoders(); /* odometry + latched encoder faults */
    (void) buddy2_set_trim((uint8_t) BUDDY2_TRIM_DEFAULT_LEFT,
                           (uint8_t) BUDDY2_TRIM_DEFAULT_RIGHT);
    (void) buddy2_set_speed_limit((uint8_t) BUDDY2_SPEED_DEFAULT);
    buddy5_set_servo_angle(ROBOT_SERVO_FORWARD_ANGLE);
    robot_set_line_speed(ROBOT_LINE_BASE_DEFAULT_PCT,
                         ROBOT_LINE_TURN_DEFAULT_PCT);
    buddy4_reset_run();
    g_barcode_mode = ROBOT_BARCODE_MODE_OBSERVE;
    barcode_reading_reset();
    tm_printf((UB *) "[cmd] reset_all: stopped; faults, odometry and IMU "
                     "cleared; defaults restored\n");
}

/* Commands without values.  Returns false when the opcode is not one. */
static bool robot_run_simple (uint8_t opcode)
{
    bool b_handled = true;

    switch (opcode)
    {
        case ROBOT_WIRE_OP_STOP:
            robot_stop_all();
            tm_printf((UB *) "[cmd] stop\n");
        break;

        case ROBOT_WIRE_OP_LINE_FOLLOW:
            robot_start_line();
        break;

        case ROBOT_WIRE_OP_LINE_STOP:
            robot_stop_line();
        break;

        case ROBOT_WIRE_OP_IMU_RESET:
            buddy4_reset_run();
            tm_printf((UB *) "[cmd] imu_reset\n");
        break;

        case ROBOT_WIRE_OP_RESET_ALL:
            robot_reset_all();
        break;

        default:
            b_handled = false;
        break;
    }

    return b_handled;
}

/* The four keyboard/button moves, at the fixed speeds the old "forward",
 * "reverse", "left" and "right" text commands used. */
static void robot_drive_preset (int32_t preset, uint32_t now_ms)
{
    switch (preset)
    {
        case (int32_t) ROBOT_WIRE_PRESET_FORWARD:
            robot_drive(ROBOT_FORWARD_SPEED, ROBOT_FORWARD_SPEED, 1, now_ms);
        break;

        case (int32_t) ROBOT_WIRE_PRESET_REVERSE:
            robot_drive(ROBOT_REVERSE_SPEED, ROBOT_REVERSE_SPEED, -1, now_ms);
        break;

        case (int32_t) ROBOT_WIRE_PRESET_LEFT:
            robot_drive(0u, ROBOT_FORWARD_SPEED, 1, now_ms);
        break;

        default: /* ROBOT_WIRE_PRESET_RIGHT: the field has only 2 bits. */
            robot_drive(ROBOT_FORWARD_SPEED, 0u, 1, now_ms);
        break;
    }
}

/* Carry out a command that carries values.  Each setter checks the range of
 * its values and prints why it rejected one. */
static void robot_run_valued (const robot_wire_command_t * p_command,
                              uint32_t                     now_ms)
{
    const int32_t * p_values = p_command->values;

    switch (p_command->opcode)
    {
        case ROBOT_WIRE_OP_DRIVE_PRESET:
            robot_drive_preset(p_values[0], now_ms);
        break;

        case ROBOT_WIRE_OP_DRIVE:
            robot_drive(robot_clamp_legacy(p_values[0]),
                        robot_clamp_legacy(p_values[1]), 1, now_ms);
        break;

        case ROBOT_WIRE_OP_MOTOR_TRIM:
            robot_set_trim(p_values[0], p_values[1]); /* percent */
        break;

        case ROBOT_WIRE_OP_SPEED_LIMIT:
            robot_set_speed_limit(p_values[0]);
        break;

        case ROBOT_WIRE_OP_SERVO_ANGLE:
            robot_set_servo_angle(p_values[0]); /* accepted only when idle */
        break;

        case ROBOT_WIRE_OP_IR_THRESHOLDS:
            (void) ir_tuning_request((uint8_t) p_command->values[0],
                                      (uint16_t) p_command->values[2],
                                      (uint16_t) p_command->values[3],
                                      0 != p_command->values[1]);
        break;

        case ROBOT_WIRE_OP_BARCODE_MODE:
            robot_barcode_settings(p_values);
        break;

        case ROBOT_WIRE_OP_BARCODE_CONFIG:
            tm_printf((UB *) "[barcode] IRQ reader uses timing; distance settings unavailable\n");
        break;

        case ROBOT_WIRE_OP_LINE_STAGE:
            robot_set_line_stage(p_values[0]);
        break;

        case ROBOT_WIRE_OP_LINE_SPEED:
            robot_set_line_speed(p_values[0], p_values[1]); /* percent */
        break;

        default:
            /* robot_wire_decode_command() accepts no other opcode. */
        break;
    }
}

/* Wi-Fi-loss safety stop (Change Requirement 1 fault mask, team decision):
 * line following stops once the Wi-Fi-loss bit has been set for
 * ROBOT_LINK_LOSS_GRACE_MS.  Only a link that was up and then dropped
 * counts, so GP21 still starts line following on a robot that never had a
 * network (or a build without Wi-Fi).  After such a stop the robot is
 * treated as offline, so a deliberate GP21 press works again. */
static void robot_check_link_loss (uint32_t now_ms)
{
    bool b_lost = (0u != (robot_fault_get() & ROBOT_FAULT_WIFI_LOSS));

    if (!b_lost)
    {
        g_b_link_seen = true;
        g_b_link_lost = false;
    }
    else if (!g_b_link_seen)
    {
        /* No link to lose. */
    }
    else if (!g_b_link_lost)
    {
        g_b_link_lost  = true;
        g_link_lost_ms = now_ms;
    }
    else if (ROBOT_MODE_LINE != robot_ctl_mode())
    {
        /* Not line following: the grace period starts with the next run. */
        g_link_lost_ms = now_ms;
    }
    else if ((now_ms - g_link_lost_ms) >= ROBOT_LINK_LOSS_GRACE_MS)
    {
        robot_stop_all();
        g_b_link_seen = false;
        g_b_link_lost = false;
        tm_printf((UB *) "[safety] Wi-Fi lost for 2 s - line following "
                         "stopped\n");
    }
    else
    {
        /* Line following, still within the grace period. */
    }
}

/* Lifted or overturned (Buddy 4): stop every motor and stay idle.  Picking
 * the rover up ends a demo run, so nothing restarts until a new command;
 * while it stays overturned, any new move is stopped at the next step. */
static void robot_check_lifted (void)
{
    buddy4_imu_state_t    imu    = {0};
    buddy2_motion_state_t motion = {0};

    buddy4_get_state(&imu);
    if (0u == imu.lifted)
    {
        /* Not lifted: nothing to do. */
    }
    else if (ROBOT_MODE_IDLE != robot_ctl_mode())
    {
        robot_stop_all();
        tm_printf((UB *) "[safety] lifted or overturned - motors stopped\n");
    }
    else
    {
        /* Idle, yet a wheel still turns: the avoidance task (lower
         * priority) can write its PWM just after robot_stop_all(). */
        buddy2_get_motion_state(&motion);
        if ((motion.left_speed_mm_s > ROBOT_LIFT_WHEEL_MM_S) ||
            (motion.left_speed_mm_s < -ROBOT_LIFT_WHEEL_MM_S) ||
            (motion.right_speed_mm_s > ROBOT_LIFT_WHEEL_MM_S) ||
            (motion.right_speed_mm_s < -ROBOT_LIFT_WHEEL_MM_S))
        {
            buddy2_stop();
        }
    }
}

/* ------------------------------------------------------------------ *
 *  Packed controller state (see g_controller_state)
 * ------------------------------------------------------------------ */

static robot_mode_t robot_ctl_mode (void)
{
    /* One byte load.  The field only ever holds a robot_mode_t value
     * written by robot_ctl_enter(), so the conversion is exact. */
    return (robot_mode_t) (g_controller_state & ROBOT_CTL_MODE_MASK);
}

/* Switch mode and replace all run flags with run_flags in one locked step,
 * so the avoidance task never sees a new mode with stale flags. */
static void robot_ctl_enter (robot_mode_t mode, uint8_t run_flags)
{
    robot_ctl_update((uint8_t) (ROBOT_CTL_MODE_MASK | ROBOT_CTL_RUN_FLAGS),
                     (uint8_t) (((uint32_t) mode & ROBOT_CTL_MODE_MASK) |
                                ((uint32_t) run_flags & ROBOT_CTL_RUN_FLAGS)));
}

static bool robot_ctl_flag (uint8_t flag)
{
    return (0u != (g_controller_state & flag));
}

static void robot_ctl_set_flag (uint8_t flag, bool b_value)
{
    robot_ctl_update(flag, b_value ? flag : 0u);
}

/* The only place g_controller_state is written: clear, then set, in one
 * locked step so a write from the other task cannot be lost. */
static void robot_ctl_update (uint8_t clear_mask, uint8_t set_mask)
{
    UINT     interrupt_state = 0u;
    uint32_t state           = 0u;

    DI(interrupt_state);
    state = ((uint32_t) g_controller_state & ~(uint32_t) clear_mask) |
            (uint32_t) set_mask;
    g_controller_state = (uint8_t) (state & 0xFFu); /* Masked to one byte. */
    EI(interrupt_state);
}

/*** end of file ***/

