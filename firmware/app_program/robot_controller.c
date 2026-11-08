/**
 * @file    robot_controller.c
 * @brief   Vehicle controller: decides what the robot does next.
 *
 * Ported from handle_command_payload() and the barcode / safety parts of
 * the old main.c.  Behaviour kept the same, except:
 *   - "stop" now also stops line following and any distance/turn move.
 *   - Raw drive commands stop line following (otherwise the line follower
 *     would overwrite them 10 ms later).
 *   - Barcodes default to observe-only; execute mode acts immediately,
 *     resumes tracking on completion, and stops on rejection/timeout.
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
#include "robot_controller.h"
#include "robot_steering_trace.h"
#include "robot_hal.h"

#define ROBOT_LEGACY_MAX (65535)

/* Limits for the dashboard "line_speed" command, percent of full power. */
#define ROBOT_LINE_BASE_MIN_PCT (15)
#define ROBOT_LINE_BASE_MAX_PCT (80)
#define ROBOT_LINE_TURN_MAX_PCT (60)
#define ROBOT_LINE_BASE_DEFAULT_PCT                                            \
    (35) /* boot line speed, see buddy3_ir.c                                   \
          */
#define ROBOT_LINE_TURN_DEFAULT_PCT (27)

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
#define ROBOT_BARCODE_ELEMENT_MAX  (1000)
#define ROBOT_BARCODE_OFFSET_MAX   (500)

#define ROBOT_NAME_MAX              (24u)
#define ROBOT_BUTTON_DEBOUNCE_STEPS (3u) /* 3 x 20 ms = 60 ms */

typedef enum
{
    ROBOT_MODE_IDLE = 0,
    ROBOT_MODE_DRIVE,
    ROBOT_MODE_MOTION,
    ROBOT_MODE_LINE,
    ROBOT_MODE_AVOID
} robot_mode_t;

/* Commands that take one or two numbers. */
typedef enum
{
    ROBOT_CMD_DRIVE,
    ROBOT_CMD_TRIM,
    ROBOT_CMD_SPEED_LIMIT,
    ROBOT_CMD_SERVO_ANGLE,
    ROBOT_CMD_BARCODE_MODE,
    ROBOT_CMD_BARCODE_CONFIG,
    ROBOT_CMD_LINE_STAGE,
    ROBOT_CMD_LINE_SPEED
} robot_command_id_t;

/* A command name and the JSON keys of its numbers; in the plain-text form
 * the numbers follow the name in the same order.  p_second_key is NULL for
 * single-number commands. */
typedef struct
{
    const char *       p_name;
    const char *       p_first_key;
    const char *       p_second_key;
    robot_command_id_t id;
} robot_command_t;

static const robot_command_t g_commands[] = {
    {"drive", "left", "right", ROBOT_CMD_DRIVE},
    {"motor_trim", "left", "right", ROBOT_CMD_TRIM},
    {"speed_limit", "percent", NULL, ROBOT_CMD_SPEED_LIMIT},
    {"servo_angle", "angle", NULL, ROBOT_CMD_SERVO_ANGLE},
    {"barcode_mode", "mode", NULL, ROBOT_CMD_BARCODE_MODE},
    {"barcode_config", "min_element_mm", "offset_left_mm",
     ROBOT_CMD_BARCODE_CONFIG},
    {"line_stage", "stage", NULL, ROBOT_CMD_LINE_STAGE},
    {"line_speed", "base", "turn", ROBOT_CMD_LINE_SPEED}};

#define ROBOT_COMMAND_COUNT (sizeof(g_commands) / sizeof(g_commands[0]))

static robot_mode_t g_mode          = ROBOT_MODE_IDLE;
static uint32_t     g_drive_left    = 0u;
static uint32_t     g_drive_right   = 0u;
static uint32_t     g_last_drive_ms = 0u;
static bool         g_b_resume_line = false;
/* Remember whether a Buddy 5 avoidance run interrupted line following. */
static bool     g_b_avoid_resume_line = false;
static bool     g_b_drive_is_forward  = false;
static bool     g_b_motion_is_forward = false;
static bool     g_b_line_button_was   = false;
static uint32_t g_line_button_count   = 0u;
/* Read-only until the demo mapping is known. */
static uint8_t g_barcode_mode = ROBOT_BARCODE_MODE_OBSERVE;

static bool robot_is_space (char character);
static bool robot_str_equal (const char * p_first, const char * p_second);
static const char * robot_find (const char * p_text, const char * p_needle);
static bool         robot_parse_int (const char * p_text, int32_t * p_value,
                                     const char ** pp_end);
static bool         robot_json_int (const char * p_json, const char * p_key,
                                    int32_t * p_value);
static bool         robot_json_command (const char * p_json, char * p_name);
static const char * robot_first_word (const char * p_text, char * p_name);
static int16_t      robot_legacy_to_pwm (uint32_t legacy);
static uint32_t     robot_clamp_legacy (int32_t value);
static void         robot_stop_all (void);
static void         robot_drive (uint32_t left, uint32_t right, int32_t sign,
                                 uint32_t now_ms);
static void         robot_set_trim (int32_t left, int32_t right);
static void         robot_set_speed_limit (int32_t percent);
static void         robot_set_servo_angle (int32_t angle);
static void         robot_set_line_speed (int32_t base_pct, int32_t turn_pct);
static void         robot_start_line (void);
static void         robot_stop_line (void);
static void         robot_set_line_stage (int32_t stage);
static void         robot_set_barcode_mode (int32_t mode);
static void         robot_reset_all (void);
static bool         robot_simple_command (const char * p_name, uint32_t now_ms);
static bool robot_command_args (const robot_command_t * p_entry, bool b_json,
                                const char * p_command, const char * p_rest,
                                int32_t values[2]);
static void robot_run_command (robot_command_id_t id, const int32_t values[2],
                               uint32_t now_ms);
static void robot_log_barcode (const buddy3_barcode_event_t * p_event);

/* ------------------------------------------------------------------ *
 *  Public API
 * ------------------------------------------------------------------ */

void robot_controller_init (void)
{
    g_mode                = ROBOT_MODE_IDLE;
    g_b_drive_is_forward  = false;
    g_b_motion_is_forward = false;
    g_b_line_button_was   = false;
    hal_gpio_input_pullup(BUTTON_LINE_PIN);
}

void robot_controller_handle_command (const char * p_command, uint32_t now_ms)
{
    char         name[ROBOT_NAME_MAX];
    int32_t      values[2] = {-1, -1};
    const char * p_rest    = NULL;
    uint32_t     index     = 0u;
    bool         b_json    = false;
    bool         b_named   = false;

    if ((NULL != p_command) && ('\0' != p_command[0]))
    {
        /* JSON form, e.g. {"cmd":"drive","left":30000,"right":30000}, or
         * plain text, e.g. "drive 30000 30000" or "speed_limit 60". */
        b_json = ('{' == p_command[0]);
        if (b_json)
        {
            b_named = robot_json_command(p_command, name);
        }
        else
        {
            p_rest  = robot_first_word(p_command, name);
            b_named = true;
        }

        if (!b_named)
        {
            tm_printf((UB *) "[cmd] ignored (no \"cmd\"): %s\n", p_command);
        }
        else if (robot_simple_command(name, now_ms))
        {
            /* Handled: a command without numbers. */
        }
        else
        {
            while ((index < ROBOT_COMMAND_COUNT) &&
                   !robot_str_equal(name, g_commands[index].p_name))
            {
                index++;
            }
            if (ROBOT_COMMAND_COUNT == index)
            {
                tm_printf((UB *) "[cmd] ignored: %s\n", p_command);
            }
            else if (robot_command_args(&g_commands[index], b_json, p_command,
                                        p_rest, values))
            {
                robot_run_command(g_commands[index].id, values, now_ms);
            }
            else
            {
                /* Missing numbers: ignored without a message, as before. */
            }
        }
    }
}

void robot_controller_step (uint32_t now_ms)
{
    buddy3_barcode_event_t barcode      = {0};
    bool                   b_button_now = false;
    bool                   b_moving     = buddy2_motion_active();

    /* Consume outside the sensor task. Letters never imply numeric motions. */
    if (buddy3_take_barcode(&barcode) &&
        (ROBOT_BARCODE_MODE_OBSERVE == g_barcode_mode))
    {
        robot_log_barcode(&barcode);
    }
    if ((ROBOT_MODE_MOTION == g_mode) && !b_moving && !g_b_resume_line)
    {
        g_b_motion_is_forward = false;
        g_mode                = ROBOT_MODE_IDLE;
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
    if (b_button_now && !g_b_line_button_was)
    {
        if (buddy3_line_following() || (ROBOT_MODE_MOTION == g_mode) ||
            (ROBOT_MODE_AVOID == g_mode))
        {
            robot_stop_line();
        }
        else
        {
            robot_start_line();
        }
    }
    g_b_line_button_was = b_button_now;

    /* Safety: a raw drive command must be refreshed within 1 s. */
    if ((ROBOT_MODE_DRIVE == g_mode) &&
        ((now_ms - g_last_drive_ms) > ROBOT_DRIVE_TIMEOUT_MS))
    {
        robot_stop_all();
        tm_printf((UB *) "[safety] drive command timed out - stopped\n");
    }
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

    return names[g_mode];
}

bool robot_controller_obstacle_watch_enabled (void)
{
    /* Watched: forward manual drive, line following, and the forward move
     * of a future mapped barcode action.  The monitor only takes over when
     * the forward-facing ultrasonic sensor reports an actual close echo, so
     * an idle rover never starts an avoidance move. */
    return (((ROBOT_MODE_DRIVE == g_mode) && g_b_drive_is_forward) ||
            ((ROBOT_MODE_LINE == g_mode) && buddy3_line_following()) ||
            ((ROBOT_MODE_MOTION == g_mode) && g_b_motion_is_forward));
}

bool robot_controller_begin_obstacle_avoidance (void)
{
    bool b_begun = robot_controller_obstacle_watch_enabled();

    if (b_begun)
    {
        /* Remember the source before stopping line following so completion
         * can hand a line-follow run back to Buddy 3. */
        g_b_avoid_resume_line = (ROBOT_MODE_LINE == g_mode);

        /* This also stops line following so it cannot overwrite a turn. */
        buddy3_stop_line_follow();
        buddy2_stop();
        g_drive_left          = 0u;
        g_drive_right         = 0u;
        g_b_resume_line       = false;
        g_b_drive_is_forward  = false;
        g_b_motion_is_forward = false;
        robot_steering_owner(TRACE_AVOIDANCE);
        g_mode = ROBOT_MODE_AVOID;
    }

    return b_begun;
}

bool robot_controller_obstacle_avoidance_active (void)
{
    return (ROBOT_MODE_AVOID == g_mode);
}

void robot_controller_finish_obstacle_avoidance (void)
{
    if (ROBOT_MODE_AVOID == g_mode)
    {
        buddy2_stop();
        g_drive_left          = 0u;
        g_drive_right         = 0u;
        g_b_drive_is_forward  = false;
        g_b_motion_is_forward = false;
        if (g_b_avoid_resume_line)
        {
            g_b_avoid_resume_line = false;
            tm_printf(
                (UB *) "[avoid] handing control back to line following\n");
            robot_start_line();
        }
        else
        {
            g_b_avoid_resume_line = false;
            g_mode                = ROBOT_MODE_IDLE;
        }
    }
}

void robot_controller_abort_obstacle_avoidance (void)
{
    if (ROBOT_MODE_AVOID == g_mode)
    {
        robot_stop_all();
    }
}

/* ------------------------------------------------------------------ *
 *  Tiny text helpers (no libc)
 * ------------------------------------------------------------------ */

static bool robot_is_space (char character)
{
    return ((' ' == character) || ('\t' == character) || ('\r' == character) ||
            ('\n' == character));
}

static bool robot_str_equal (const char * p_first, const char * p_second)
{
    while (('\0' != *p_first) && (*p_first == *p_second))
    {
        p_first++;
        p_second++;
    }

    return (*p_first == *p_second);
}

/* Find p_needle in p_text; returns a pointer into p_text or NULL. */
static const char * robot_find (const char * p_text, const char * p_needle)
{
    const char * p_start = p_text;
    const char * p_match = NULL;

    while ((NULL == p_match) && ('\0' != *p_start))
    {
        const char * p_text_char   = p_start;
        const char * p_needle_char = p_needle;

        while (('\0' != *p_needle_char) && (*p_text_char == *p_needle_char))
        {
            p_text_char++;
            p_needle_char++;
        }
        if ('\0' == *p_needle_char)
        {
            p_match = p_start;
        }
        p_start++;
    }

    return p_match;
}

/* Parse a (possibly negative) decimal integer; pp_end gets the stop point. */
static bool robot_parse_int (const char * p_text, int32_t * p_value,
                             const char ** pp_end)
{
    int32_t sign     = 1;
    int32_t value    = 0;
    bool    b_digits = false;

    while (robot_is_space(*p_text))
    {
        p_text++;
    }
    if ('-' == *p_text)
    {
        sign = -1;
        p_text++;
    }
    while ((*p_text >= '0') && (*p_text <= '9'))
    {
        /* A digit character minus '0' is 0..9. */
        value    = (value * 10) + (int32_t) (*p_text - '0');
        b_digits = true;
        p_text++;
    }
    if (NULL != pp_end)
    {
        *pp_end = p_text;
    }
    if (b_digits)
    {
        *p_value = sign * value;
    }

    return b_digits;
}

/* JSON: read the number after "key": */
static bool robot_json_int (const char * p_json, const char * p_key,
                            int32_t * p_value)
{
    char         pattern[ROBOT_NAME_MAX + 3u];
    uint32_t     index   = 0u;
    const char * p_found = NULL;
    bool         b_found = false;

    pattern[index] = '"';
    index++;
    while (('\0' != *p_key) && (index < (ROBOT_NAME_MAX + 1u)))
    {
        pattern[index] = *p_key;
        index++;
        p_key++;
    }
    pattern[index] = '"';
    index++;
    pattern[index] = '\0';

    p_found = robot_find(p_json, pattern);
    if (NULL != p_found)
    {
        p_found = robot_find(p_found, ":");
    }
    if (NULL != p_found)
    {
        b_found = robot_parse_int(p_found + 1, p_value, NULL);
    }

    return b_found;
}

/* JSON: copy the string value of "cmd" (or "command") into p_name. */
static bool robot_json_command (const char * p_json, char * p_name)
{
    const char * p_found = robot_find(p_json, "\"cmd\"");
    uint32_t     length  = 0u;

    if (NULL == p_found)
    {
        p_found = robot_find(p_json, "\"command\"");
    }
    if (NULL != p_found)
    {
        p_found = robot_find(p_found, ":");
    }
    if (NULL != p_found)
    {
        p_found++;
        while (robot_is_space(*p_found))
        {
            p_found++;
        }
        if ('"' == *p_found)
        {
            p_found++;
            while (('"' != *p_found) && ('\0' != *p_found) &&
                   (length < (ROBOT_NAME_MAX - 1u)))
            {
                p_name[length] = *p_found;
                length++;
                p_found++;
            }
        }
    }
    p_name[length] = '\0';

    return (length > 0u);
}

/* Plain text: first word into p_name, rest returned. */
static const char * robot_first_word (const char * p_text, char * p_name)
{
    uint32_t length = 0u;

    while (robot_is_space(*p_text))
    {
        p_text++;
    }
    while (('\0' != *p_text) && !robot_is_space(*p_text) &&
           (length < (ROBOT_NAME_MAX - 1u)))
    {
        p_name[length] = *p_text;
        length++;
        p_text++;
    }
    p_name[length] = '\0';

    return p_text;
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
    buddy3_stop_line_follow();
    buddy2_stop();
    g_drive_left          = 0u;
    g_drive_right         = 0u;
    g_b_resume_line       = false;
    g_b_avoid_resume_line = false;
    g_b_drive_is_forward  = false;
    g_b_motion_is_forward = false;
    g_mode                = ROBOT_MODE_IDLE;
}

/* Raw motor command.  Signs: +1 forward, -1 reverse. */
static void robot_drive (uint32_t left, uint32_t right, int32_t sign,
                         uint32_t now_ms)
{
    buddy3_stop_line_follow();
    buddy2_stop();
    /* A manual sonar test may have left the scanner looking sideways. Any
     * drive command must restore the forward view before motion begins. */
    buddy5_set_servo_angle(ROBOT_SERVO_FORWARD_ANGLE);
    robot_steering_owner(TRACE_MANUAL);
    g_drive_left          = left;
    g_drive_right         = right;
    g_last_drive_ms       = now_ms;
    g_b_resume_line       = false;
    g_b_avoid_resume_line = false;
    g_b_drive_is_forward  = (sign > 0) && (left > 0u) && (right > 0u);
    g_b_motion_is_forward = false;
    /* sign is +/-1 and the PWM is at most 32767, so the products fit. */
    buddy2_set_motor_pwm((int16_t) (sign * robot_legacy_to_pwm(left)),
                         (int16_t) (sign * robot_legacy_to_pwm(right)));
    g_mode =
        ((0u == left) && (0u == right)) ? ROBOT_MODE_IDLE : ROBOT_MODE_DRIVE;
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
    if (ROBOT_MODE_IDLE != g_mode)
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
    buddy3_calibration_t calibration = {0};

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
        buddy3_get_calibration(&calibration);
        /* base_pct <= 80, so the scaled PWM fits int16_t. */
        calibration.base_pwm =
            (int16_t) ((base_pct * BUDDY2_PWM_MAX) / ROBOT_PERCENT_MAX);
        calibration.proportional_gain =
            ((float32_t) turn_pct * (float32_t) BUDDY2_PWM_MAX) / 100000.0f;
        buddy3_set_calibration(&calibration);
        tm_printf((UB *) "[cmd] line speed base %d turn %d (percent)\n",
                  (INT) base_pct, (INT) turn_pct);
    }
}

static void robot_start_line (void)
{
    g_b_resume_line = false;
    buddy2_stop();
    g_drive_left          = 0u;
    g_drive_right         = 0u;
    g_b_drive_is_forward  = false;
    g_b_motion_is_forward = false;
    g_b_avoid_resume_line = false;
    buddy5_set_servo_angle(ROBOT_SERVO_FORWARD_ANGLE);
    buddy3_start_line_follow();
    robot_steering_owner(TRACE_TRACKING);
    g_mode = ROBOT_MODE_LINE;
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
        (ROBOT_MODE_IDLE != g_mode))
    {
        tm_printf((UB *) "[barcode] stop first; use 0=off or 1=observe; letter "
                         "actions are not configured\n");
    }
    else
    {
        g_barcode_mode = (uint8_t) mode; /* 0 or 1 checked above. */
        tm_printf((UB *) "[barcode] mode=%d (0 off, 1 observe)\n", (INT) mode);
    }
}

/* Dashboard "Reset everything": stop, then return every runtime setting and
 * counter to its boot value without rebooting the Pico. */
static void robot_reset_all (void)
{
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
    /* The defaults are small whole millimetres, so they fit the types. */
    (void) buddy3_configure_barcode((uint16_t) BUDDY3_BARCODE_MIN_ELEMENT_MM,
                                    (int16_t) BUDDY3_BARCODE_OFFSET_LEFT_MM);
    tm_printf((UB *) "[cmd] reset_all: stopped; faults, odometry and IMU "
                     "cleared; defaults restored\n");
}

/* Commands without numbers.  Returns false when p_name is not one. */
static bool robot_simple_command (const char * p_name, uint32_t now_ms)
{
    bool b_handled = true;

    if (robot_str_equal(p_name, "stop"))
    {
        robot_stop_all();
        tm_printf((UB *) "[cmd] stop\n");
    }
    else if (robot_str_equal(p_name, "forward"))
    {
        robot_drive(ROBOT_FORWARD_SPEED, ROBOT_FORWARD_SPEED, 1, now_ms);
    }
    else if (robot_str_equal(p_name, "reverse"))
    {
        robot_drive(ROBOT_REVERSE_SPEED, ROBOT_REVERSE_SPEED, -1, now_ms);
    }
    else if (robot_str_equal(p_name, "left"))
    {
        robot_drive(0u, ROBOT_FORWARD_SPEED, 1, now_ms);
    }
    else if (robot_str_equal(p_name, "right"))
    {
        robot_drive(ROBOT_FORWARD_SPEED, 0u, 1, now_ms);
    }
    else if (robot_str_equal(p_name, "line_follow"))
    {
        robot_start_line();
    }
    else if (robot_str_equal(p_name, "line_stop"))
    {
        robot_stop_line();
    }
    else if (robot_str_equal(p_name, "imu_reset"))
    {
        buddy4_reset_run();
        tm_printf((UB *) "[cmd] imu_reset\n");
    }
    else if (robot_str_equal(p_name, "reset_all"))
    {
        robot_reset_all();
    }
    else
    {
        b_handled = false;
    }

    return b_handled;
}

/* Read the one or two numbers of a command from its JSON keys or, in the
 * plain-text form, from the words after the name.  Returns false when a
 * number is missing. */
static bool robot_command_args (const robot_command_t * p_entry, bool b_json,
                                const char * p_command, const char * p_rest,
                                int32_t values[2])
{
    bool b_ok = false;

    if (b_json)
    {
        b_ok = robot_json_int(p_command, p_entry->p_first_key, &values[0]) &&
               ((NULL == p_entry->p_second_key) ||
                robot_json_int(p_command, p_entry->p_second_key, &values[1]));
    }
    else
    {
        b_ok = robot_parse_int(p_rest, &values[0], &p_rest) &&
               ((NULL == p_entry->p_second_key) ||
                robot_parse_int(p_rest, &values[1], &p_rest));
    }

    return b_ok;
}

/* Carry out a command whose numbers were read successfully. */
static void robot_run_command (robot_command_id_t id, const int32_t values[2],
                               uint32_t now_ms)
{
    switch (id)
    {
        case ROBOT_CMD_DRIVE:
            robot_drive(robot_clamp_legacy(values[0]),
                        robot_clamp_legacy(values[1]), 1, now_ms);
        break;

        case ROBOT_CMD_TRIM:
            robot_set_trim(values[0], values[1]); /* percent */
        break;

        case ROBOT_CMD_SPEED_LIMIT:
            robot_set_speed_limit(values[0]);
        break;

        case ROBOT_CMD_SERVO_ANGLE:
            robot_set_servo_angle(values[0]); /* accepted only when idle */
        break;

        case ROBOT_CMD_BARCODE_MODE:
            robot_set_barcode_mode(values[0]);
        break;

        case ROBOT_CMD_BARCODE_CONFIG:
            /* Ranges are checked before the casts. */
            if ((values[0] >= 0) && (values[0] <= ROBOT_BARCODE_ELEMENT_MAX) &&
                (values[1] >= -ROBOT_BARCODE_OFFSET_MAX) &&
                (values[1] <= ROBOT_BARCODE_OFFSET_MAX))
            {
                (void) buddy3_configure_barcode((uint16_t) values[0],
                                                (int16_t) values[1]);
            }
        break;

        case ROBOT_CMD_LINE_STAGE:
            robot_set_line_stage(values[0]);
        break;

        case ROBOT_CMD_LINE_SPEED:
            robot_set_line_speed(values[0], values[1]); /* percent */
        break;

        default:
            /* Every table entry is handled above. */
        break;
    }
}

/* Print a decoded or rejected barcode (observe mode only). */
static void robot_log_barcode (const buddy3_barcode_event_t * p_event)
{
    if (B3BC_VALID == p_event->kind)
    {
        /* Widths are small positive millimetre values, so x100 fits UINT. */
        tm_printf((UB *) "[barcode] Decoded: %c (%s), narrow=%u/100 mm "
                         "ratio=%u/100 ts=%u dropped=%u; observe only\n",
                  p_event->decoded.symbol,
                  p_event->decoded.reversed ? "reverse" : "forward",
                  (UINT) (p_event->decoded.narrow_width * 100.0f),
                  (UINT) (p_event->decoded.wide_ratio * 100.0f),
                  (UINT) p_event->ts_ms, (UINT) p_event->dropped);
    }
    else
    {
        tm_printf((UB *) "[barcode] rejected: %s (%s) ts=%u dropped=%u\n",
                  buddy3_barcode_event_name(p_event->kind),
                  (B3BC_FORMAT == p_event->kind)
                      ? code39_status_name(p_event->format_status)
                      : "capture",
                  (UINT) p_event->ts_ms, (UINT) p_event->dropped);
    }
}

/*** end of file ***/

