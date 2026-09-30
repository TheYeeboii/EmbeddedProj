#include "buddy2_motion.h"

#include <math.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"

#include "pin_config.h"

#define BUDDY2_CONTROL_PERIOD_MS 20u
#define BUDDY2_PI 3.14159265358979323846f
#define BUDDY2_DEFAULT_WHEEL_DIAMETER_MM 65.0f
#define BUDDY2_DEFAULT_TRACK_WIDTH_MM 150.0f
#define BUDDY2_DEFAULT_TICKS_PER_REV 20.0f
#define BUDDY2_DEFAULT_PWM_LIMIT 32767u
#define BUDDY2_DEFAULT_PWM_DEADBAND 0u
#define BUDDY2_MAX_SPEED_MM_S 320.0f
#define BUDDY2_POSITION_TOLERANCE_MM 3.0f
#define BUDDY2_PID_INTEGRAL_LIMIT 1000.0f

static volatile int32_t g_left_ticks = 0;
static volatile int32_t g_right_ticks = 0;
static buddy2_calibration_t g_calibration = {
    BUDDY2_DEFAULT_WHEEL_DIAMETER_MM,
    BUDDY2_DEFAULT_TRACK_WIDTH_MM,
    BUDDY2_DEFAULT_TICKS_PER_REV,
    BUDDY2_DEFAULT_TICKS_PER_REV,
    BUDDY2_DEFAULT_PWM_LIMIT,
    BUDDY2_DEFAULT_PWM_DEADBAND
};
static buddy2_motion_state_t g_state;
static uint32_t g_last_update_ms = 0u;
static int16_t g_left_command = 0;
static int16_t g_right_command = 0;
static int8_t g_left_direction = 1;
static int8_t g_right_direction = 1;
static bool g_initialized = false;

static struct
{
    bool active;
    float target_distance_mm;
    float start_left_distance_mm;
    float start_right_distance_mm;
    int16_t max_pwm;
    int8_t left_direction;
    int8_t right_direction;
    float left_integral;
    float right_integral;
    float left_previous_error;
    float right_previous_error;
} g_motion;

static void buddy2_encoder_callback(uint gpio, uint32_t events)
{
    if ((events & GPIO_IRQ_EDGE_RISE) == 0u)
    {
        return;
    }
    if (gpio == ENCODER_LEFT_A)
    {
        g_left_ticks++;
    }
    else if (gpio == ENCODER_RIGHT_A)
    {
        g_right_ticks++;
    }
}

static float buddy2_mm_per_tick(float ticks_per_rev)
{
    if (ticks_per_rev <= 0.0f)
    {
        return 0.0f;
    }
    return (BUDDY2_PI * g_calibration.wheel_diameter_mm) / ticks_per_rev;
}

static int16_t buddy2_clamp_pwm(int16_t value)
{
    if (value > (int16_t)g_calibration.pwm_limit)
    {
        return (int16_t)g_calibration.pwm_limit;
    }
    if (value < -(int16_t)g_calibration.pwm_limit)
    {
        return -(int16_t)g_calibration.pwm_limit;
    }
    return value;
}

static void buddy2_apply_one(uint pin_a, uint pin_b, int16_t pwm)
{
    uint16_t magnitude = (uint16_t)(pwm < 0 ? -pwm : pwm);
    if (magnitude > 0u && magnitude < g_calibration.pwm_deadband)
    {
        magnitude = g_calibration.pwm_deadband;
    }
    if (pwm >= 0)
    {
        pwm_set_gpio_level(pin_a, magnitude);
        pwm_set_gpio_level(pin_b, 0u);
    }
    else
    {
        pwm_set_gpio_level(pin_a, 0u);
        pwm_set_gpio_level(pin_b, magnitude);
    }
}

void buddy2_motion_init(void)
{
    gpio_set_function(MOTOR_LEFT_PWM_A, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_LEFT_PWM_B, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_RIGHT_PWM_A, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_RIGHT_PWM_B, GPIO_FUNC_PWM);
    pwm_set_enabled(pwm_gpio_to_slice_num(MOTOR_LEFT_PWM_A), true);
    pwm_set_enabled(pwm_gpio_to_slice_num(MOTOR_RIGHT_PWM_A), true);

    gpio_init(ENCODER_LEFT_A);
    gpio_set_dir(ENCODER_LEFT_A, GPIO_IN);
    gpio_pull_up(ENCODER_LEFT_A);
    gpio_init(ENCODER_RIGHT_A);
    gpio_set_dir(ENCODER_RIGHT_A, GPIO_IN);
    gpio_pull_up(ENCODER_RIGHT_A);
    gpio_set_irq_enabled_with_callback(ENCODER_LEFT_A, GPIO_IRQ_EDGE_RISE, true, &buddy2_encoder_callback);
    gpio_set_irq_enabled(ENCODER_RIGHT_A, GPIO_IRQ_EDGE_RISE, true);

    g_initialized = true;
    buddy2_stop();
    buddy2_reset_encoders();
    memset(&g_state, 0, sizeof(g_state));
    memset(&g_motion, 0, sizeof(g_motion));
}

void buddy2_set_motor_pwm(int16_t left_pwm, int16_t right_pwm)
{
    if (!g_initialized)
    {
        return;
    }
    g_left_command = buddy2_clamp_pwm(left_pwm);
    g_right_command = buddy2_clamp_pwm(right_pwm);
    g_left_direction = g_left_command < 0 ? -1 : 1;
    g_right_direction = g_right_command < 0 ? -1 : 1;
    buddy2_apply_one(MOTOR_LEFT_PWM_A, MOTOR_LEFT_PWM_B, g_left_command);
    buddy2_apply_one(MOTOR_RIGHT_PWM_A, MOTOR_RIGHT_PWM_B, g_right_command);
}

void buddy2_stop(void)
{
    g_motion.active = false;
    g_left_command = 0;
    g_right_command = 0;
    if (g_initialized)
    {
        buddy2_apply_one(MOTOR_LEFT_PWM_A, MOTOR_LEFT_PWM_B, 0);
        buddy2_apply_one(MOTOR_RIGHT_PWM_A, MOTOR_RIGHT_PWM_B, 0);
    }
}

void buddy2_reset_encoders(void)
{
    uint32_t interrupt_state = save_and_disable_interrupts();
    g_left_ticks = 0;
    g_right_ticks = 0;
    restore_interrupts(interrupt_state);
}

void buddy2_get_motion_state(buddy2_motion_state_t *state)
{
    if (state == NULL)
    {
        return;
    }
    uint32_t interrupt_state = save_and_disable_interrupts();
    state->left_ticks = g_state.left_ticks;
    state->right_ticks = g_state.right_ticks;
    restore_interrupts(interrupt_state);
    state->left_speed_mm_s = g_state.left_speed_mm_s;
    state->right_speed_mm_s = g_state.right_speed_mm_s;
    state->left_distance_mm = g_state.left_distance_mm;
    state->right_distance_mm = g_state.right_distance_mm;
    state->timestamp_ms = g_state.timestamp_ms;
}

static bool buddy2_start_motion(float left_distance_mm, float right_distance_mm, int16_t max_pwm)
{
    if (!g_initialized || max_pwm <= 0 || (left_distance_mm == 0.0f && right_distance_mm == 0.0f))
    {
        return false;
    }
    g_motion.active = true;
    g_motion.target_distance_mm = fmaxf(fabsf(left_distance_mm), fabsf(right_distance_mm));
    g_motion.start_left_distance_mm = g_state.left_distance_mm;
    g_motion.start_right_distance_mm = g_state.right_distance_mm;
    g_motion.max_pwm = buddy2_clamp_pwm(max_pwm);
    g_motion.left_direction = left_distance_mm < 0.0f ? -1 : 1;
    g_motion.right_direction = right_distance_mm < 0.0f ? -1 : 1;
    g_motion.left_integral = 0.0f;
    g_motion.right_integral = 0.0f;
    g_motion.left_previous_error = 0.0f;
    g_motion.right_previous_error = 0.0f;
    buddy2_set_motor_pwm((int16_t)(g_motion.left_direction * g_motion.max_pwm),
                         (int16_t)(g_motion.right_direction * g_motion.max_pwm));
    return true;
}

bool buddy2_move_forward(float distance_mm, int16_t max_pwm)
{
    return buddy2_start_motion(fabsf(distance_mm), fabsf(distance_mm), max_pwm);
}

bool buddy2_move_backward(float distance_mm, int16_t max_pwm)
{
    return buddy2_start_motion(-fabsf(distance_mm), -fabsf(distance_mm), max_pwm);
}

bool buddy2_turn_left(float angle_deg, int16_t max_pwm)
{
    float distance = BUDDY2_PI * g_calibration.track_width_mm * fabsf(angle_deg) / 360.0f;
    return buddy2_start_motion(-distance, distance, max_pwm);
}

bool buddy2_turn_right(float angle_deg, int16_t max_pwm)
{
    float distance = BUDDY2_PI * g_calibration.track_width_mm * fabsf(angle_deg) / 360.0f;
    return buddy2_start_motion(distance, -distance, max_pwm);
}

bool buddy2_motion_active(void)
{
    return g_motion.active;
}

void buddy2_update(uint32_t now_ms)
{
    if (!g_initialized || (now_ms - g_last_update_ms) < BUDDY2_CONTROL_PERIOD_MS)
    {
        return;
    }
    float dt = (float)(now_ms - g_last_update_ms) / 1000.0f;
    g_last_update_ms = now_ms;

    int32_t left_ticks;
    int32_t right_ticks;
    uint32_t interrupt_state = save_and_disable_interrupts();
    left_ticks = g_left_ticks;
    right_ticks = g_right_ticks;
    restore_interrupts(interrupt_state);

    int32_t left_delta = left_ticks - g_state.left_ticks;
    int32_t right_delta = right_ticks - g_state.right_ticks;
    float left_mm_tick = buddy2_mm_per_tick(g_calibration.left_ticks_per_wheel_rev);
    float right_mm_tick = buddy2_mm_per_tick(g_calibration.right_ticks_per_wheel_rev);
    float left_distance = (float)left_delta * left_mm_tick * (float)g_left_direction;
    float right_distance = (float)right_delta * right_mm_tick * (float)g_right_direction;
    g_state.left_speed_mm_s = left_distance / dt;
    g_state.right_speed_mm_s = right_distance / dt;
    g_state.left_distance_mm += left_distance;
    g_state.right_distance_mm += right_distance;
    g_state.left_ticks = left_ticks;
    g_state.right_ticks = right_ticks;
    g_state.timestamp_ms = now_ms;

    if (!g_motion.active)
    {
        return;
    }

    float left_progress = fabsf(g_state.left_distance_mm - g_motion.start_left_distance_mm);
    float right_progress = fabsf(g_state.right_distance_mm - g_motion.start_right_distance_mm);
    float progress = (left_progress + right_progress) * 0.5f;
    if (progress >= (g_motion.target_distance_mm - BUDDY2_POSITION_TOLERANCE_MM))
    {
        buddy2_stop();
        return;
    }

    float target_speed = BUDDY2_MAX_SPEED_MM_S * ((float)g_motion.max_pwm / (float)g_calibration.pwm_limit);
    float left_target = target_speed * (float)g_motion.left_direction;
    float right_target = target_speed * (float)g_motion.right_direction;
    float left_error = left_target - g_state.left_speed_mm_s;
    float right_error = right_target - g_state.right_speed_mm_s;
    float left_derivative = (left_error - g_motion.left_previous_error) / dt;
    float right_derivative = (right_error - g_motion.right_previous_error) / dt;
    g_motion.left_previous_error = left_error;
    g_motion.right_previous_error = right_error;
    g_motion.left_integral = fminf(BUDDY2_PID_INTEGRAL_LIMIT, fmaxf(-BUDDY2_PID_INTEGRAL_LIMIT, g_motion.left_integral + left_error * dt));
    g_motion.right_integral = fminf(BUDDY2_PID_INTEGRAL_LIMIT, fmaxf(-BUDDY2_PID_INTEGRAL_LIMIT, g_motion.right_integral + right_error * dt));
    float left_output = (float)g_motion.left_direction * (float)g_motion.max_pwm * 0.65f + 0.15f * left_error + 0.02f * g_motion.left_integral + 0.005f * left_derivative;
    float right_output = (float)g_motion.right_direction * (float)g_motion.max_pwm * 0.65f + 0.15f * right_error + 0.02f * g_motion.right_integral + 0.005f * right_derivative;
    buddy2_set_motor_pwm((int16_t)left_output, (int16_t)right_output);
}

void buddy2_set_calibration(const buddy2_calibration_t *calibration)
{
    if (calibration != NULL && calibration->wheel_diameter_mm > 0.0f &&
        calibration->track_width_mm > 0.0f && calibration->left_ticks_per_wheel_rev > 0.0f &&
        calibration->right_ticks_per_wheel_rev > 0.0f && calibration->pwm_limit > 0u)
    {
        g_calibration = *calibration;
    }
}

void buddy2_get_calibration(buddy2_calibration_t *calibration)
{
    if (calibration != NULL)
    {
        *calibration = g_calibration;
    }
}

bool moveForward(float distance_mm, int16_t max_pwm)
{
    return buddy2_move_forward(distance_mm, max_pwm);
}

bool moveBackward(float distance_mm, int16_t max_pwm)
{
    return buddy2_move_backward(distance_mm, max_pwm);
}

bool turnLeft(float angle_deg, int16_t max_pwm)
{
    return buddy2_turn_left(angle_deg, max_pwm);
}

bool turnRight(float angle_deg, int16_t max_pwm)
{
    return buddy2_turn_right(angle_deg, max_pwm);
}

void stop(void)
{
    buddy2_stop();
}
