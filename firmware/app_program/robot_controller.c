/**
 * @file    robot_controller.c
 * @brief   Vehicle controller: decides what the robot does next.
 *
 * Ported from handle_command_payload() and the barcode / safety parts of
 * the old main.c.  Behaviour kept the same, except:
 *   - "stop" now also stops line following and any distance/turn move.
 *   - Raw drive commands stop line following (otherwise the line follower
 *     would overwrite them 10 ms later).
 *   - After a barcode action the robot resumes line following.
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
#include "pin_config.h"
#include "robot_controller.h"
#include "robot_hal.h"

#define ROBOT_LEGACY_TO_PWM(v)        ((int16_t)((uint32_t)(v) / 2u))
#define ROBOT_LEGACY_MAX              (65535)

#define ROBOT_FORWARD_SPEED           (30000u)   /* dashboard units */
#define ROBOT_REVERSE_SPEED           (20000u)
#define ROBOT_DEFAULT_MOTION_PWM      (20000)    /* dashboard units */
#define ROBOT_DRIVE_TIMEOUT_MS        (1000u)    /* raw drive safety stop */

#define ROBOT_BARCODE_TURN_DEG        (90.0f)
#define ROBOT_BARCODE_STRAIGHT_MM     (300.0f)
#define ROBOT_BARCODE_PWM             (20000)    /* dashboard units */

#define ROBOT_NAME_MAX                (24u)
#define ROBOT_BUTTON_DEBOUNCE_STEPS   (3u)       /* 3 x 20 ms = 60 ms */

typedef enum
{
    ROBOT_MODE_IDLE = 0,
    ROBOT_MODE_DRIVE,
    ROBOT_MODE_MOTION,
    ROBOT_MODE_LINE
} robot_mode_t;

static robot_mode_t g_mode              = ROBOT_MODE_IDLE;
static uint32_t     g_drive_left        = 0u;
static uint32_t     g_drive_right       = 0u;
static uint32_t     g_last_drive_ms     = 0u;
static bool         g_resume_line       = false;
static bool         g_line_button_was   = false;
static uint32_t     g_line_button_count = 0u;

/* ------------------------------------------------------------------ *
 *  Tiny text helpers (no libc)
 * ------------------------------------------------------------------ */

static bool robot_is_space(char c)
{
    return (' ' == c) || ('\t' == c) || ('\r' == c) || ('\n' == c);
}

static bool robot_str_equal(const char *p_a, const char *p_b)
{
    while (('\0' != *p_a) && (*p_a == *p_b))
    {
        p_a++;
        p_b++;
    }
    return (*p_a == *p_b);
}

/* Find p_needle in p_text; returns a pointer into p_text or NULL. */
static const char *robot_find(const char *p_text, const char *p_needle)
{
    const char *p_start;

    for (p_start = p_text; '\0' != *p_start; p_start++)
    {
        const char *p_t = p_start;
        const char *p_n = p_needle;

        while (('\0' != *p_n) && (*p_t == *p_n))
        {
            p_t++;
            p_n++;
        }
        if ('\0' == *p_n)
        {
            return p_start;
        }
    }
    return NULL;
}

/* Parse a (possibly negative) decimal integer; p_end gets the stop point. */
static bool robot_parse_int(const char *p_text, int32_t *p_value, const char **p_end)
{
    int32_t sign   = 1;
    int32_t value  = 0;
    bool    digits = false;

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
        value  = (value * 10) + (int32_t)(*p_text - '0');
        digits = true;
        p_text++;
    }
    if (NULL != p_end)
    {
        *p_end = p_text;
    }
    if (digits)
    {
        *p_value = sign * value;
    }
    return digits;
}

/* JSON: read the number after "key": */
static bool robot_json_int(const char *p_json, const char *p_key, int32_t *p_value)
{
    char        pattern[ROBOT_NAME_MAX + 3u];
    uint32_t    index = 0u;
    const char *p_found;

    pattern[index++] = '"';
    while (('\0' != *p_key) && (index < (ROBOT_NAME_MAX + 1u)))
    {
        pattern[index++] = *p_key++;
    }
    pattern[index++] = '"';
    pattern[index]   = '\0';

    p_found = robot_find(p_json, pattern);
    if (NULL == p_found)
    {
        return false;
    }
    p_found = robot_find(p_found, ":");
    if (NULL == p_found)
    {
        return false;
    }
    return robot_parse_int(p_found + 1, p_value, NULL);
}

/* JSON: copy the string value of "cmd" (or "command") into p_name. */
static bool robot_json_command(const char *p_json, char *p_name)
{
    const char *p_found = robot_find(p_json, "\"cmd\"");
    uint32_t    length  = 0u;

    if (NULL == p_found)
    {
        p_found = robot_find(p_json, "\"command\"");
    }
    if (NULL == p_found)
    {
        return false;
    }
    p_found = robot_find(p_found, ":");
    if (NULL == p_found)
    {
        return false;
    }
    p_found++;
    while (robot_is_space(*p_found))
    {
        p_found++;
    }
    if ('"' != *p_found)
    {
        return false;
    }
    p_found++;
    while (('"' != *p_found) && ('\0' != *p_found) && (length < (ROBOT_NAME_MAX - 1u)))
    {
        p_name[length++] = *p_found++;
    }
    p_name[length] = '\0';
    return (length > 0u);
}

/* Plain text: first word into p_name, rest returned. */
static const char *robot_first_word(const char *p_text, char *p_name)
{
    uint32_t length = 0u;

    while (robot_is_space(*p_text))
    {
        p_text++;
    }
    while (('\0' != *p_text) && !robot_is_space(*p_text) && (length < (ROBOT_NAME_MAX - 1u)))
    {
        p_name[length++] = *p_text++;
    }
    p_name[length] = '\0';
    return p_text;
}

/* ------------------------------------------------------------------ *
 *  Actions
 * ------------------------------------------------------------------ */

static uint32_t robot_clamp_legacy(int32_t value)
{
    if (value < 0)
    {
        return 0u;
    }
    if (value > ROBOT_LEGACY_MAX)
    {
        return (uint32_t)ROBOT_LEGACY_MAX;
    }
    return (uint32_t)value;
}

static void robot_stop_all(void)
{
    buddy3_stop_line_follow();
    buddy2_stop();
    g_drive_left  = 0u;
    g_drive_right = 0u;
    g_resume_line = false;
    g_mode        = ROBOT_MODE_IDLE;
}

/* Raw motor command.  Signs: +1 forward, -1 reverse. */
static void robot_drive(uint32_t left, uint32_t right, int32_t sign, uint32_t now_ms)
{
    buddy3_stop_line_follow();
    buddy2_stop();
    g_drive_left    = left;
    g_drive_right   = right;
    g_last_drive_ms = now_ms;
    g_resume_line   = false;
    buddy2_set_motor_pwm((int16_t)(sign * ROBOT_LEGACY_TO_PWM(left)),
                         (int16_t)(sign * ROBOT_LEGACY_TO_PWM(right)));
    g_mode = ((0u == left) && (0u == right)) ? ROBOT_MODE_IDLE : ROBOT_MODE_DRIVE;
}

static bool robot_valid_pwm(int32_t pwm)
{
    return (pwm > 0) && (pwm <= ROBOT_LEGACY_MAX);
}

/* move_forward / move_backward / turn_left / turn_right */
static bool robot_motion(const char *p_name, int32_t amount, int32_t pwm)
{
    int16_t api_pwm = ROBOT_LEGACY_TO_PWM(pwm);
    bool    started = false;

    if ((amount <= 0) || !robot_valid_pwm(pwm))
    {
        return false;
    }
    buddy3_stop_line_follow();
    g_resume_line = false;

    if (robot_str_equal(p_name, "move_forward"))
    {
        started = buddy2_move_forward((float)amount, api_pwm);
    }
    else if (robot_str_equal(p_name, "move_backward"))
    {
        started = buddy2_move_backward((float)amount, api_pwm);
    }
    else if (robot_str_equal(p_name, "turn_left"))
    {
        started = buddy2_turn_left((float)amount, api_pwm);
    }
    else if (robot_str_equal(p_name, "turn_right"))
    {
        started = buddy2_turn_right((float)amount, api_pwm);
    }
    else
    {
        /* not a motion command */
    }

    if (started)
    {
        g_mode = ROBOT_MODE_MOTION;
        tm_printf((UB *)"[cmd] %s %d (pwm %d)\n", p_name, (INT)amount, (INT)pwm);
    }
    return started;
}

static void robot_start_line(void)
{
    buddy2_stop();
    g_drive_left  = 0u;
    g_drive_right = 0u;
    buddy3_start_line_follow();
    g_mode = ROBOT_MODE_LINE;
    tm_printf((UB *)"[cmd] line following ON\n");
}

static void robot_stop_line(void)
{
    robot_stop_all();
    tm_printf((UB *)"[cmd] line following OFF\n");
}

/* Commands shared by the JSON and plain-text forms. */
static bool robot_simple_command(const char *p_name, uint32_t now_ms)
{
    if (robot_str_equal(p_name, "stop"))
    {
        robot_stop_all();
        tm_printf((UB *)"[cmd] stop\n");
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
    else
    {
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------ *
 *  Public API
 * ------------------------------------------------------------------ */

void robot_controller_init(void)
{
    g_mode            = ROBOT_MODE_IDLE;
    g_line_button_was = false;
    hal_gpio_input_pullup(BUTTON_LINE_PIN);
}

void robot_controller_handle_command(const char *p_command, uint32_t now_ms)
{
    char    name[ROBOT_NAME_MAX];
    int32_t left     = -1;
    int32_t right    = -1;
    int32_t amount   = 0;
    int32_t pwm      = ROBOT_DEFAULT_MOTION_PWM;

    if ((NULL == p_command) || ('\0' == p_command[0]))
    {
        return;
    }

    if ('{' == p_command[0])
    {
        /* JSON form, e.g. {"cmd":"drive","left":30000,"right":30000} */
        if (!robot_json_command(p_command, name))
        {
            tm_printf((UB *)"[cmd] ignored (no \"cmd\"): %s\n", p_command);
            return;
        }
        if (robot_simple_command(name, now_ms))
        {
            return;
        }
        if (robot_str_equal(name, "drive"))
        {
            if (robot_json_int(p_command, "left", &left) &&
                robot_json_int(p_command, "right", &right))
            {
                robot_drive(robot_clamp_legacy(left), robot_clamp_legacy(right), 1, now_ms);
            }
            return;
        }
        (void)robot_json_int(p_command, "max_pwm", &pwm);
        if (!robot_json_int(p_command, "distance_mm", &amount))
        {
            (void)robot_json_int(p_command, "angle_deg", &amount);
        }
        if (!robot_motion(name, amount, pwm))
        {
            tm_printf((UB *)"[cmd] ignored: %s\n", p_command);
        }
        return;
    }

    /* Plain-text form, e.g. "drive 30000 30000" or "move_forward 500 20000" */
    {
        const char *p_rest = robot_first_word(p_command, name);

        if (robot_simple_command(name, now_ms))
        {
            return;
        }
        if (robot_str_equal(name, "drive"))
        {
            if (robot_parse_int(p_rest, &left, &p_rest) &&
                robot_parse_int(p_rest, &right, &p_rest))
            {
                robot_drive(robot_clamp_legacy(left), robot_clamp_legacy(right), 1, now_ms);
            }
            return;
        }
        if (robot_parse_int(p_rest, &amount, &p_rest))
        {
            (void)robot_parse_int(p_rest, &pwm, &p_rest);
        }
        if (!robot_motion(name, amount, pwm))
        {
            tm_printf((UB *)"[cmd] ignored: %s\n", p_command);
        }
    }
}

void robot_controller_step(uint32_t now_ms)
{
    uint8_t barcode = 0u;
    bool    button_now;
    bool    moving = buddy2_motion_active();

    /* 1. Barcode seen while following the line: do what it says. */
    if (buddy3_take_barcode(&barcode) && buddy3_line_following())
    {
        buddy3_stop_line_follow();
        if (BUDDY3_BARCODE_LEFT == barcode)
        {
            tm_printf((UB *)"[barcode] turn left\n");
            (void)buddy2_turn_left(ROBOT_BARCODE_TURN_DEG, ROBOT_LEGACY_TO_PWM(ROBOT_BARCODE_PWM));
        }
        else if (BUDDY3_BARCODE_RIGHT == barcode)
        {
            tm_printf((UB *)"[barcode] turn right\n");
            (void)buddy2_turn_right(ROBOT_BARCODE_TURN_DEG, ROBOT_LEGACY_TO_PWM(ROBOT_BARCODE_PWM));
        }
        else
        {
            tm_printf((UB *)"[barcode] go straight\n");
            (void)buddy2_move_forward(ROBOT_BARCODE_STRAIGHT_MM, ROBOT_LEGACY_TO_PWM(ROBOT_BARCODE_PWM));
        }
        g_resume_line = true;
        g_mode        = ROBOT_MODE_MOTION;
        moving        = true;
    }

    /* 2. Barcode action finished: back to following the line. */
    if (g_resume_line && !moving)
    {
        g_resume_line = false;
        robot_start_line();
    }
    else if ((ROBOT_MODE_MOTION == g_mode) && !moving && !g_resume_line)
    {
        g_mode = ROBOT_MODE_IDLE;
    }
    else
    {
        /* nothing */
    }

    /* 3. GP21 button: toggle line following (on press, not while held). */
    /* Debounced: the pin must read "pressed" for 3 steps in a row, so one
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
    button_now = (g_line_button_count >= ROBOT_BUTTON_DEBOUNCE_STEPS);
    if (button_now && !g_line_button_was)
    {
        if (buddy3_line_following())
        {
            robot_stop_line();
        }
        else
        {
            robot_start_line();
        }
    }
    g_line_button_was = button_now;

    /* 4. Safety: a raw drive command must be refreshed within 1 s. */
    if ((ROBOT_MODE_DRIVE == g_mode) &&
        ((now_ms - g_last_drive_ms) > ROBOT_DRIVE_TIMEOUT_MS))
    {
        robot_stop_all();
        tm_printf((UB *)"[safety] drive command timed out - stopped\n");
    }
}

void robot_controller_get_drive(uint32_t *p_left, uint32_t *p_right)
{
    if ((NULL != p_left) && (NULL != p_right))
    {
        *p_left  = g_drive_left;
        *p_right = g_drive_right;
    }
}

const char *robot_controller_mode_name(void)
{
    static const char *const names[] = { "idle", "drive", "motion", "line" };

    return names[g_mode];
}
