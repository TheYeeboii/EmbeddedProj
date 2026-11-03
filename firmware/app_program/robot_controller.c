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
#include "pin_config.h"
#include "robot_controller.h"
#include "robot_hal.h"
#include "robot_barcode_action.h"

#define ROBOT_LEGACY_TO_PWM(v)        ((int16_t)((uint32_t)(v) / 2u))
#define ROBOT_LEGACY_MAX              (65535)

/* Limits for the dashboard "line_speed" command, percent of full power. */
#define ROBOT_LINE_BASE_MIN_PCT       (15)
#define ROBOT_LINE_BASE_MAX_PCT       (80)
#define ROBOT_LINE_TURN_MAX_PCT       (60)

#define ROBOT_FORWARD_SPEED           (30000u)   /* dashboard units */
#define ROBOT_REVERSE_SPEED           (20000u)
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
    ROBOT_MODE_LINE,
    ROBOT_MODE_AVOID
} robot_mode_t;

static robot_mode_t g_mode              = ROBOT_MODE_IDLE;
static uint32_t     g_drive_left        = 0u;
static uint32_t     g_drive_right       = 0u;
static uint32_t     g_last_drive_ms     = 0u;
static bool         g_resume_line       = false;
/* Remember whether a Buddy 5 avoidance run interrupted line following. */
static bool         g_avoid_resume_line = false;
static bool         g_drive_is_forward  = false;
static bool         g_motion_is_forward = false;
static bool         g_line_button_was   = false;
static uint32_t     g_line_button_count = 0u;
static uint8_t g_barcode_mode = ROBOT_BARCODE_DEFAULT_MODE;
static robot_barcode_action_t g_barcode_action;

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
    robot_barcode_action_cancel(&g_barcode_action);
    buddy3_stop_line_follow();
    buddy2_stop();
    g_drive_left  = 0u;
    g_drive_right = 0u;
    g_resume_line = false;
    g_avoid_resume_line = false;
    g_drive_is_forward  = false;
    g_motion_is_forward = false;
    g_mode        = ROBOT_MODE_IDLE;
}

/* Raw motor command.  Signs: +1 forward, -1 reverse. */
static void robot_drive(uint32_t left, uint32_t right, int32_t sign, uint32_t now_ms)
{
    robot_barcode_action_cancel(&g_barcode_action);
    buddy3_stop_line_follow();
    buddy2_stop();
    g_drive_left    = left;
    g_drive_right   = right;
    g_last_drive_ms = now_ms;
    g_resume_line   = false;
    g_avoid_resume_line = false;
    g_drive_is_forward = (sign > 0) && (left > 0u) && (right > 0u);
    g_motion_is_forward = false;
    buddy2_set_motor_pwm((int16_t)(sign * ROBOT_LEGACY_TO_PWM(left)),
                         (int16_t)(sign * ROBOT_LEGACY_TO_PWM(right)));
    g_mode = ((0u == left) && (0u == right)) ? ROBOT_MODE_IDLE : ROBOT_MODE_DRIVE;
}

static void robot_set_trim(int32_t left, int32_t right)
{
    if ((left >= 0) && (left <= 100) && (right >= 0) && (right <= 100) &&
        buddy2_set_trim((uint8_t)left, (uint8_t)right))
    {
        tm_printf((UB *)"[cmd] motor trim L %d R %d (percent)\n", (INT)left, (INT)right);
    }
    else
    {
        tm_printf((UB *)"[cmd] motor trim rejected (each %d..100): %d %d\n",
                  (INT)BUDDY2_TRIM_MIN, (INT)left, (INT)right);
    }
}

static void robot_set_speed_limit(int32_t percent)
{
    if ((percent >= (int32_t)BUDDY2_SPEED_MIN) && (percent <= 100) &&
        buddy2_set_speed_limit((uint8_t)percent))
    {
        tm_printf((UB *)"[cmd] speed limit %d%%\n", (INT)percent);
    }
    else
    {
        tm_printf((UB *)"[cmd] speed limit rejected (%d..100): %d\n",
                  (INT)BUDDY2_SPEED_MIN, (INT)percent);
    }
}

/* Line-follow speed in percent of full power: base = straight speed,
 * turn = extra/less power on the outer/inner wheel on curves. */
static void robot_set_line_speed(int32_t base_pct, int32_t turn_pct)
{
    buddy3_calibration_t calibration;

    if ((base_pct < ROBOT_LINE_BASE_MIN_PCT) || (base_pct > ROBOT_LINE_BASE_MAX_PCT) ||
        (turn_pct < 0) || (turn_pct > ROBOT_LINE_TURN_MAX_PCT))
    {
        tm_printf((UB *)"[cmd] line speed rejected (base %d..%d, turn 0..%d): %d %d\n",
                  (INT)ROBOT_LINE_BASE_MIN_PCT, (INT)ROBOT_LINE_BASE_MAX_PCT,
                  (INT)ROBOT_LINE_TURN_MAX_PCT, (INT)base_pct, (INT)turn_pct);
        return;
    }
    buddy3_get_calibration(&calibration);
    calibration.base_pwm          = (int16_t)((base_pct * BUDDY2_PWM_MAX) / 100);
    calibration.proportional_gain = ((float)turn_pct * (float)BUDDY2_PWM_MAX) / 100000.0f;
    buddy3_set_calibration(&calibration);
    tm_printf((UB *)"[cmd] line speed base %d turn %d (percent)\n", (INT)base_pct, (INT)turn_pct);
}

static void robot_start_line(void)
{
    robot_barcode_action_cancel(&g_barcode_action);
    g_resume_line = false;
    buddy2_stop();
    g_drive_left  = 0u;
    g_drive_right = 0u;
    g_drive_is_forward  = false;
    g_motion_is_forward = false;
    g_avoid_resume_line = false;
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
static void robot_set_line_stage(int32_t stage)
{
    if (stage < 0 || stage > 3)
    {
        tm_printf((UB *)"[cmd] line_stage must be 0..3\n");
        return;
    }
    robot_stop_all();
    buddy3_set_test_stage((uint8_t)stage);
    tm_printf((UB *)"[cmd] line_stage %d selected; send line_follow to start\n", (INT)stage);
}

static void robot_set_barcode_mode(int32_t mode)
{
    if (mode < 0 || mode > 2 || g_mode != ROBOT_MODE_IDLE) {
        tm_printf((UB *)"[barcode] stop first; mode must be 0=off, 1=observe, 2=execute\n");
        return;
    }
    if (mode == 2 && (!buddy3_barcode_calibrated() || buddy3_test_stage() == 0u)) {
        tm_printf((UB *)"[barcode] execute rejected: configure distance reader and select stage 1..3\n");
        return;
    }
    g_barcode_mode = (uint8_t)mode;
    tm_printf((UB *)"[barcode] mode=%d (0 off, 1 observe, 2 immediate actions)\n", (INT)mode);
}

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
    g_drive_is_forward  = false;
    g_motion_is_forward = false;
    g_line_button_was = false;
    hal_gpio_input_pullup(BUTTON_LINE_PIN);
}

void robot_controller_handle_command(const char *p_command, uint32_t now_ms)
{
    char    name[ROBOT_NAME_MAX];
    int32_t left     = -1;
    int32_t right    = -1;
    int32_t amount   = 0;

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
        if (robot_str_equal(name, "motor_trim"))
        {
            /* {"cmd":"motor_trim","left":95,"right":100}  (percent) */
            if (robot_json_int(p_command, "left", &left) &&
                robot_json_int(p_command, "right", &right))
            {
                robot_set_trim(left, right);
            }
            return;
        }
        if (robot_str_equal(name, "speed_limit"))
        {
            /* {"cmd":"speed_limit","percent":60} */
            if (robot_json_int(p_command, "percent", &amount))
            {
                robot_set_speed_limit(amount);
            }
            return;
        }
        if (robot_str_equal(name, "barcode_mode"))
        {
            if (robot_json_int(p_command, "mode", &amount)) robot_set_barcode_mode(amount);
            return;
        }
        if (robot_str_equal(name, "barcode_config"))
        {
            if (robot_json_int(p_command, "cell_mm", &left) &&
                robot_json_int(p_command, "offset_left_mm", &right) &&
                left >= 0 && left <= 1000 && right >= -500 && right <= 500)
                (void)buddy3_configure_barcode((uint16_t)left, (int16_t)right);
            return;
        }
        if (robot_str_equal(name, "line_stage"))
        {
            if (robot_json_int(p_command, "stage", &amount))
            {
                robot_set_line_stage(amount);
            }
            return;
        }
        if (robot_str_equal(name, "line_speed"))
        {
            /* {"cmd":"line_speed","base":37,"turn":27}  (percent) */
            if (robot_json_int(p_command, "base", &left) &&
                robot_json_int(p_command, "turn", &right))
            {
                robot_set_line_speed(left, right);
            }
            return;
        }
        tm_printf((UB *)"[cmd] ignored: %s\n", p_command);
        return;
    }

    /* Plain-text form, e.g. "drive 30000 30000" or "speed_limit 60" */
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
        if (robot_str_equal(name, "motor_trim"))
        {
            if (robot_parse_int(p_rest, &left, &p_rest) &&
                robot_parse_int(p_rest, &right, &p_rest))
            {
                robot_set_trim(left, right);
            }
            return;
        }
        if (robot_str_equal(name, "speed_limit"))
        {
            if (robot_parse_int(p_rest, &amount, &p_rest))
            {
                robot_set_speed_limit(amount);
            }
            return;
        }
        if (robot_str_equal(name, "barcode_mode"))
        {
            if (robot_parse_int(p_rest, &amount, &p_rest)) robot_set_barcode_mode(amount);
            return;
        }
        if (robot_str_equal(name, "barcode_config"))
        {
            if (robot_parse_int(p_rest, &left, &p_rest) &&
                robot_parse_int(p_rest, &right, &p_rest) &&
                left >= 0 && left <= 1000 && right >= -500 && right <= 500)
                (void)buddy3_configure_barcode((uint16_t)left, (int16_t)right);
            return;
        }
        if (robot_str_equal(name, "line_stage"))
        {
            if (robot_parse_int(p_rest, &amount, &p_rest))
            {
                robot_set_line_stage(amount);
            }
            return;
        }
        if (robot_str_equal(name, "line_speed"))
        {
            if (robot_parse_int(p_rest, &left, &p_rest) &&
                robot_parse_int(p_rest, &right, &p_rest))
            {
                robot_set_line_speed(left, right);
            }
            return;
        }
        tm_printf((UB *)"[cmd] ignored: %s\n", p_command);
    }
}

void robot_controller_step(uint32_t now_ms)
{
    uint8_t barcode = 0u;
    uint8_t barcode_action;
    buddy2_motion_state_t motion_state = {0};
    bool    button_now;
    bool    moving = buddy2_motion_active();
    bool encoder_fault;
    buddy2_get_motion_state(&motion_state);
    encoder_fault = motion_state.left_encoder_fault || motion_state.right_encoder_fault;
    if (g_barcode_action.active && encoder_fault) {
        robot_stop_all();
        moving = false;
        tm_printf((UB *)"[barcode] encoder fault interrupted motion; stopped, no resume\n");
    }

    /* Decode does not directly own motors. Observe first; mode 2 hands off
     * immediately, as required by the track, not at a later junction. */
    if (buddy3_take_barcode(&barcode)) {
        bool allowed = !encoder_fault && g_mode == ROBOT_MODE_LINE && buddy3_line_following() &&
                       buddy3_barcode_calibrated() && buddy3_test_stage() != 0u;
        barcode_action = robot_barcode_action_request(&g_barcode_action, barcode,
                           g_barcode_mode, allowed, now_ms);
        if (barcode_action == RB_OBSERVED) {
            tm_printf((UB *)"[barcode] observed %u; movement disabled\n", (UINT)barcode);
        } else if (barcode_action >= RB_LEFT && barcode_action <= RB_STRAIGHT) {
            bool accepted;
            buddy3_stop_line_follow();
            buddy2_stop();
            g_motion_is_forward = barcode_action == RB_STRAIGHT;
            if (barcode_action == RB_LEFT)
                accepted = buddy2_turn_left(ROBOT_BARCODE_TURN_DEG, ROBOT_LEGACY_TO_PWM(ROBOT_BARCODE_PWM));
            else if (barcode_action == RB_RIGHT)
                accepted = buddy2_turn_right(ROBOT_BARCODE_TURN_DEG, ROBOT_LEGACY_TO_PWM(ROBOT_BARCODE_PWM));
            else
                accepted = buddy2_move_forward(ROBOT_BARCODE_STRAIGHT_MM, ROBOT_LEGACY_TO_PWM(ROBOT_BARCODE_PWM));
            if (!accepted) {
                robot_stop_all();
                tm_printf((UB *)"[barcode] motion rejected; stopped\n");
            } else {
                g_resume_line = true;
                g_mode = ROBOT_MODE_MOTION;
                moving = true;
                tm_printf((UB *)"[barcode] executing %u immediately\n", (UINT)barcode);
            }
        }
    }

    barcode_action = robot_barcode_action_poll(&g_barcode_action, moving, now_ms);
    if (barcode_action == RB_STOP) {
        robot_stop_all();
        moving = false;
        tm_printf((UB *)"[barcode] motion exceeded 8 seconds; stopped\n");
    } else if (barcode_action == RB_RESUME && g_resume_line) {
        tm_printf((UB *)"[barcode] motion complete; resume tracking\n");
        robot_start_line();
    }
    else if ((ROBOT_MODE_MOTION == g_mode) && !moving && !g_resume_line)
    {
        g_motion_is_forward = false;
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
        if (buddy3_line_following() || g_mode == ROBOT_MODE_MOTION || g_mode == ROBOT_MODE_AVOID)
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
    static const char *const names[] = { "idle", "drive", "motion", "line", "avoid" };

    return names[g_mode];
}

bool robot_controller_obstacle_watch_enabled(void)
{
    /* Watched: forward manual drive, line following, and the forward move
     * of a "go straight" barcode action.  The monitor only takes over when
     * the forward-facing ultrasonic sensor reports an actual close echo, so
     * an idle rover never starts an avoidance move. */
    return ((ROBOT_MODE_DRIVE == g_mode) && g_drive_is_forward) ||
           ((ROBOT_MODE_LINE == g_mode) && buddy3_line_following()) ||
           ((ROBOT_MODE_MOTION == g_mode) && g_motion_is_forward);
}

bool robot_controller_begin_obstacle_avoidance(void)
{
    if (!robot_controller_obstacle_watch_enabled())
    {
        return false;
    }

    /* Remember the source before stopping line following so completion can
     * hand a line-follow run back to Buddy 3. */
    g_avoid_resume_line = (ROBOT_MODE_LINE == g_mode);

    /* This also stops line following so it cannot overwrite a turn command. */
    robot_barcode_action_cancel(&g_barcode_action);
    buddy3_stop_line_follow();
    buddy2_stop();
    g_drive_left  = 0u;
    g_drive_right = 0u;
    g_resume_line = false;
    g_drive_is_forward  = false;
    g_motion_is_forward = false;
    g_mode        = ROBOT_MODE_AVOID;
    return true;
}

bool robot_controller_obstacle_avoidance_active(void)
{
    return (ROBOT_MODE_AVOID == g_mode);
}

void robot_controller_finish_obstacle_avoidance(void)
{
    if (ROBOT_MODE_AVOID == g_mode)
    {
        buddy2_stop();
        g_drive_left  = 0u;
        g_drive_right = 0u;
        g_drive_is_forward  = false;
        g_motion_is_forward = false;
        if (g_avoid_resume_line)
        {
            g_avoid_resume_line = false;
            tm_printf((UB *)"[avoid] handing control back to line following\n");
            robot_start_line();
        }
        else
        {
            g_avoid_resume_line = false;
            g_mode = ROBOT_MODE_IDLE;
        }
    }
}
