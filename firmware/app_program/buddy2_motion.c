/**
 * @file    buddy2_motion.c
 * @brief   Buddy 2 motion-control library - micro T-Kernel port.
 *
 * Ported from the team's Pico SDK (CMake) version.  What changed:
 *
 *  - All Pico SDK calls go through robot_hal.c (see robot_hal.h for why).
 *  - Encoder interrupts go through gpio_irq.c (kernel tk_def_int) instead
 *    of gpio_set_irq_enabled_with_callback(), which does not work under
 *    micro T-Kernel because the kernel owns the interrupt vector table.
 *  - The PWM block is taken out of reset here; micro T-Kernel does not do
 *    that for us the way the Pico SDK runtime does.
 *  - PWM wrap is set explicitly (BUDDY2_PWM_WRAP) and API values are
 *    scaled to it, so BUDDY2_PWM_MAX really means 100 % duty.  (Before,
 *    the wrap was left at 65535 while the API stopped at 32767, so the
 *    motors never got more than ~50 %.)
 *  - A kernel mutex protects all shared state, because the motion task
 *    and other tasks can now run "at the same time".
 *  - Floats are clamped BEFORE being cast to int16_t (the old cast could
 *    overflow).
 *
 * The control law itself (feed-forward + PID gains) is unchanged on
 * purpose: tune it on the real car as the next step.
 */

#include <tk/tkernel.h>

#include "buddy2_motion.h"
#include "gpio_irq.h"
#include "pin_config.h"
#include "robot_hal.h"
#include "robot_action_log.h"

/* ------------------------------------------------------------------ *
 *  Tunables
 * ------------------------------------------------------------------ */

#define BUDDY2_PI                        (3.14159265358979f)

/* PWM counter top.  clk_sys is 125 MHz on this port, so
 * 125 MHz / (12499 + 1) = 10 kHz.  If the motors feel weak at low speed,
 * try a lower frequency (a bigger wrap, e.g. 62499 = 2 kHz). */
#define BUDDY2_PWM_WRAP                  (12499u)

/* Default geometry - REPLACE with measured values (BUDDY2_CALIBRATION.md). */
#define BUDDY2_DEFAULT_WHEEL_DIAMETER_MM (65.0f)
#define BUDDY2_DEFAULT_TRACK_WIDTH_MM    (150.0f)
/* 387 rising edges per wheel turn: measured on the left wheel (Oct).
 * Same encoder model on both sides - re-measure the right wheel. */
#define BUDDY2_DEFAULT_TICKS_PER_REV     (387.0f)
#define BUDDY2_DEFAULT_PWM_DEADBAND      (0u)

#define BUDDY2_MAX_SPEED_MM_S            (320.0f)
#define BUDDY2_POSITION_TOLERANCE_MM     (3.0f)

/* Control law (unchanged from the CMake version - tune these next). */
#define BUDDY2_FEEDFORWARD_GAIN          (0.65f)
#define BUDDY2_KP                        (0.15f)
#define BUDDY2_KI                        (0.02f)
#define BUDDY2_KD                        (0.005f)
#define BUDDY2_PID_INTEGRAL_LIMIT        (1000.0f)

/* Encoder check: a wheel driven at >= BUDDY2_ENCODER_CHECK_MIN_PWM (API
 * units, after trim and speed limit) that gives no tick for
 * BUDDY2_ENCODER_FAULT_MS is flagged as faulty.  Below that PWM the wheel
 * may legitimately not turn, so it is not checked. */
#define BUDDY2_ENCODER_CHECK_MIN_PWM     (3000)
#define BUDDY2_ENCODER_FAULT_MS          (800u)

/* ------------------------------------------------------------------ *
 *  State
 * ------------------------------------------------------------------ */

/* Written by the encoder interrupt, read by tasks. */
static volatile int32_t g_left_ticks  = 0;
static volatile int32_t g_right_ticks = 0;

static buddy2_calibration_t g_calibration =
{
    BUDDY2_DEFAULT_WHEEL_DIAMETER_MM,
    BUDDY2_DEFAULT_TRACK_WIDTH_MM,
    BUDDY2_DEFAULT_TICKS_PER_REV,
    BUDDY2_DEFAULT_TICKS_PER_REV,
    (uint16_t)BUDDY2_PWM_MAX,
    BUDDY2_DEFAULT_PWM_DEADBAND
};

static buddy2_motion_state_t g_state;
/* Open-loop trim in percent (BUDDY2_TRIM_MIN..100).  Only scales
 * buddy2_set_motor_pwm(); distance/turn moves use the speed PID instead. */
static uint8_t  g_left_trim_pct   = BUDDY2_TRIM_DEFAULT_LEFT;
static uint8_t  g_right_trim_pct  = BUDDY2_TRIM_DEFAULT_RIGHT;
/* Global speed limit in percent: scales open-loop PWM and the max_pwm of
 * distance/turn moves. */
static uint8_t  g_speed_pct       = BUDDY2_SPEED_DEFAULT;
static uint32_t g_last_update_ms = 0u;
static int8_t   g_left_direction  = 1;
static int8_t   g_right_direction = 1;
/* PWM actually sent to each motor (API units), for the encoder check. */
static int16_t  g_left_applied_pwm  = 0;
static int16_t  g_right_applied_pwm = 0;
/* How long each wheel has been driven without a tick. */
static uint32_t g_left_no_tick_ms   = 0u;
static uint32_t g_right_no_tick_ms  = 0u;
static bool     g_initialized     = false;
static ID       g_mutex_id        = 0;

typedef struct
{
    bool    active;
    float   target_distance_mm;
    float   start_left_distance_mm;
    float   start_right_distance_mm;
    int16_t max_pwm;
    int8_t  left_direction;
    int8_t  right_direction;
    float   left_integral;
    float   right_integral;
    float   left_previous_error;
    float   right_previous_error;
} buddy2_move_t;

static buddy2_move_t g_motion;

/* ------------------------------------------------------------------ *
 *  Small helpers (no libm needed)
 * ------------------------------------------------------------------ */

static float buddy2_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float buddy2_clampf(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static void buddy2_lock(void)
{
    (void)tk_loc_mtx(g_mutex_id, TMO_FEVR);
}

static void buddy2_unlock(void)
{
    (void)tk_unl_mtx(g_mutex_id);
}

/* ------------------------------------------------------------------ *
 *  Encoder interrupt (runs in interrupt context - keep it tiny)
 * ------------------------------------------------------------------ */

static void buddy2_encoder_callback(uint32_t gpio, uint32_t events)
{
    if (0u == (events & HAL_GPIO_EDGE_RISE))
    {
        return;
    }
    if (ENCODER_LEFT_A == gpio)
    {
        g_left_ticks++;
    }
    else if (ENCODER_RIGHT_A == gpio)
    {
        g_right_ticks++;
    }
    else
    {
        /* Not ours - ignore. */
    }
}

/* ------------------------------------------------------------------ *
 *  Motor output
 * ------------------------------------------------------------------ */

static float buddy2_mm_per_tick(float ticks_per_rev)
{
    if (ticks_per_rev <= 0.0f)
    {
        return 0.0f;
    }
    return (BUDDY2_PI * g_calibration.wheel_diameter_mm) / ticks_per_rev;
}

/* Clamp a float command to the calibrated limit, then convert. */
static int16_t buddy2_clamp_pwm(float value)
{
    return (int16_t)buddy2_clampf(value, (float)g_calibration.pwm_limit);
}

/* Drive one H-bridge.  pwm is in API units (-32767..32767). */
static void buddy2_apply_one(uint32_t pin_a, uint32_t pin_b, int16_t pwm)
{
    uint32_t magnitude = (uint32_t)((pwm < 0) ? -(int32_t)pwm : (int32_t)pwm);
    uint32_t level;

    if ((magnitude > 0u) && (magnitude < g_calibration.pwm_deadband))
    {
        magnitude = g_calibration.pwm_deadband;
    }

    /* Scale API units to PWM counter ticks. */
    level = (magnitude * (BUDDY2_PWM_WRAP + 1u)) / (uint32_t)BUDDY2_PWM_MAX;

    if (pwm >= 0)
    {
        hal_pwm_set_level(pin_a, level);
        hal_pwm_set_level(pin_b, 0u);
    }
    else
    {
        hal_pwm_set_level(pin_a, 0u);
        hal_pwm_set_level(pin_b, level);
    }
}

/* Caller must hold the mutex. */
static void buddy2_set_motor_pwm_locked(int16_t left_pwm, int16_t right_pwm)
{
    int16_t left_command  = buddy2_clamp_pwm((float)left_pwm);
    int16_t right_command = buddy2_clamp_pwm((float)right_pwm);

    /* Keep the last direction while stopped, so coasting ticks are still
     * counted the right way. */
    if (0 != left_command)
    {
        g_left_direction = (left_command < 0) ? -1 : 1;
    }
    if (0 != right_command)
    {
        g_right_direction = (right_command < 0) ? -1 : 1;
    }

    g_left_applied_pwm  = left_command;
    g_right_applied_pwm = right_command;
    buddy2_apply_one(MOTOR_LEFT_PWM_A,  MOTOR_LEFT_PWM_B,  left_command);
    buddy2_apply_one(MOTOR_RIGHT_PWM_A, MOTOR_RIGHT_PWM_B, right_command);
}

/* Caller must hold the mutex. */
static void buddy2_stop_locked(void)
{
    g_motion.active     = false;
    g_left_applied_pwm  = 0;
    g_right_applied_pwm = 0;
    buddy2_apply_one(MOTOR_LEFT_PWM_A,  MOTOR_LEFT_PWM_B,  0);
    buddy2_apply_one(MOTOR_RIGHT_PWM_A, MOTOR_RIGHT_PWM_B, 0);
    robot_action_record(ROBOT_ACTION_NONE, ROBOT_ACTION_STOP, 0, 0,
                        (uint32_t)(hal_time_us() / 1000u));
}

static void buddy2_encoder_pin_init(uint32_t pin)
{
    /* The encoders pull their output low, so the pin needs a pull-up. */
    hal_gpio_input_pullup(pin);
    (void)gpio_irq_register(pin, HAL_GPIO_EDGE_RISE, buddy2_encoder_callback);
}

/* ------------------------------------------------------------------ *
 *  Public API
 * ------------------------------------------------------------------ */

int32_t buddy2_motion_init(void)
{
    T_CMTX cmtx;
    ID     mutex_id;

    cmtx.exinf  = NULL;
    /* Priority inheritance.  Note: TA_TFIFO / TA_TPRI / TA_INHERIT /
     * TA_CEILING are one 2-bit field, NOT flags - OR-ing two of them
     * (e.g. TA_TPRI | TA_INHERIT == TA_CEILING) gives E_PAR. */
    cmtx.mtxatr = TA_INHERIT;
    cmtx.ceilpri = 0;
    mutex_id = tk_cre_mtx(&cmtx);
    if (mutex_id < E_OK)
    {
        return (int32_t)mutex_id;
    }
    g_mutex_id = mutex_id;

    /* micro T-Kernel leaves the PWM block in reset - release it. */
    hal_pwm_release_reset();

    /* Motor pins.  Pins A and B of a motor share one PWM slice. */
    hal_pwm_pin_init(MOTOR_LEFT_PWM_A,  BUDDY2_PWM_WRAP);
    hal_pwm_pin_init(MOTOR_LEFT_PWM_B,  BUDDY2_PWM_WRAP);
    hal_pwm_pin_init(MOTOR_RIGHT_PWM_A, BUDDY2_PWM_WRAP);
    hal_pwm_pin_init(MOTOR_RIGHT_PWM_B, BUDDY2_PWM_WRAP);

    buddy2_encoder_pin_init(ENCODER_LEFT_A);
    buddy2_encoder_pin_init(ENCODER_RIGHT_A);

    buddy2_lock();
    g_initialized = true;
    buddy2_stop_locked();
    buddy2_unlock();

    buddy2_reset_encoders();

    return 0;
}

void buddy2_set_motor_pwm(int16_t left_pwm, int16_t right_pwm)
{
    if (!g_initialized)
    {
        return;
    }
    buddy2_lock();
    buddy2_set_motor_pwm_locked(
        (int16_t)(((int32_t)left_pwm  * (int32_t)g_left_trim_pct  * (int32_t)g_speed_pct) / 10000),
        (int16_t)(((int32_t)right_pwm * (int32_t)g_right_trim_pct * (int32_t)g_speed_pct) / 10000));
    /* Observe requested steering before trim; trim alone is not a turn.
     * Logging only queues transitions and never publishes while locked. */
    robot_action_record(robot_action_direction_for_pwm(left_pwm, right_pwm),
                        ROBOT_ACTION_PWM, left_pwm, right_pwm,
                        (uint32_t)(hal_time_us() / 1000u));
    buddy2_unlock();
}

bool buddy2_set_speed_limit(uint8_t percent)
{
    if ((percent < BUDDY2_SPEED_MIN) || (percent > 100u))
    {
        return false;
    }
    buddy2_lock();
    g_speed_pct = percent;
    buddy2_unlock();
    return true;
}

uint8_t buddy2_get_speed_limit(void)
{
    uint8_t percent;

    buddy2_lock();
    percent = g_speed_pct;
    buddy2_unlock();
    return percent;
}

bool buddy2_set_trim(uint8_t left_pct, uint8_t right_pct)
{
    if ((left_pct < BUDDY2_TRIM_MIN) || (left_pct > 100u) ||
        (right_pct < BUDDY2_TRIM_MIN) || (right_pct > 100u))
    {
        return false;
    }
    buddy2_lock();
    g_left_trim_pct  = left_pct;
    g_right_trim_pct = right_pct;
    buddy2_unlock();
    return true;
}

void buddy2_get_trim(uint8_t *p_left_pct, uint8_t *p_right_pct)
{
    if ((NULL != p_left_pct) && (NULL != p_right_pct))
    {
        buddy2_lock();
        *p_left_pct  = g_left_trim_pct;
        *p_right_pct = g_right_trim_pct;
        buddy2_unlock();
    }
}

void buddy2_stop(void)
{
    if (!g_initialized)
    {
        return;
    }
    buddy2_lock();
    buddy2_stop_locked();
    buddy2_unlock();
}

void buddy2_reset_encoders(void)
{
    UINT interrupt_state;

    buddy2_lock();
    DI(interrupt_state);
    g_left_ticks  = 0;
    g_right_ticks = 0;
    EI(interrupt_state);
    g_state.left_ticks        = 0;
    g_state.right_ticks       = 0;
    g_state.left_distance_mm  = 0.0f;
    g_state.right_distance_mm = 0.0f;
    /* Keep an active move consistent with the zeroed odometry. */
    g_motion.start_left_distance_mm  = 0.0f;
    g_motion.start_right_distance_mm = 0.0f;
    buddy2_unlock();
}

void buddy2_get_motion_state(buddy2_motion_state_t *p_state)
{
    if ((NULL == p_state) || !g_initialized)
    {
        return;
    }
    buddy2_lock();
    *p_state = g_state;
    buddy2_unlock();
}

static bool buddy2_start_motion(float left_distance_mm,
                                float right_distance_mm,
                                int16_t max_pwm)
{
    float left_abs;
    float right_abs;

    if (!g_initialized || (max_pwm <= 0) ||
        ((0.0f == left_distance_mm) && (0.0f == right_distance_mm)))
    {
        return false;
    }

    left_abs  = buddy2_absf(left_distance_mm);
    right_abs = buddy2_absf(right_distance_mm);

    buddy2_lock();
    g_motion.active                  = true;
    g_motion.target_distance_mm      = (left_abs > right_abs) ? left_abs : right_abs;
    g_motion.start_left_distance_mm  = g_state.left_distance_mm;
    g_motion.start_right_distance_mm = g_state.right_distance_mm;
    g_motion.max_pwm                 = buddy2_clamp_pwm(((float)max_pwm * (float)g_speed_pct) / 100.0f);
    g_motion.left_direction          = (left_distance_mm  < 0.0f) ? -1 : 1;
    g_motion.right_direction         = (right_distance_mm < 0.0f) ? -1 : 1;
    g_motion.left_integral           = 0.0f;
    g_motion.right_integral          = 0.0f;
    g_motion.left_previous_error     = 0.0f;
    g_motion.right_previous_error    = 0.0f;
    buddy2_set_motor_pwm_locked(
        (int16_t)(g_motion.left_direction  * g_motion.max_pwm),
        (int16_t)(g_motion.right_direction * g_motion.max_pwm));
    /* Record the planned direction, not straight-move PID adjustments. */
    robot_action_record(left_distance_mm < right_distance_mm ? ROBOT_ACTION_LEFT :
                        (left_distance_mm > right_distance_mm ? ROBOT_ACTION_RIGHT : ROBOT_ACTION_NONE),
                        ROBOT_ACTION_MOTION,
                        (int16_t)(g_motion.left_direction * max_pwm),
                        (int16_t)(g_motion.right_direction * max_pwm),
                        (uint32_t)(hal_time_us() / 1000u));
    buddy2_unlock();

    return true;
}

bool buddy2_move_forward(float distance_mm, int16_t max_pwm)
{
    float distance = buddy2_absf(distance_mm);
    return buddy2_start_motion(distance, distance, max_pwm);
}

bool buddy2_move_backward(float distance_mm, int16_t max_pwm)
{
    float distance = buddy2_absf(distance_mm);
    return buddy2_start_motion(-distance, -distance, max_pwm);
}

bool buddy2_turn_left(float angle_deg, int16_t max_pwm)
{
    /* In-place turn: each wheel travels an arc of the track circle. */
    float distance = BUDDY2_PI * g_calibration.track_width_mm *
                     buddy2_absf(angle_deg) / 360.0f;
    return buddy2_start_motion(-distance, distance, max_pwm);
}

bool buddy2_turn_right(float angle_deg, int16_t max_pwm)
{
    float distance = BUDDY2_PI * g_calibration.track_width_mm *
                     buddy2_absf(angle_deg) / 360.0f;
    return buddy2_start_motion(distance, -distance, max_pwm);
}

bool buddy2_motion_active(void)
{
    bool active;

    buddy2_lock();
    active = g_motion.active;
    buddy2_unlock();

    return active;
}

/*
 * Encoder check for one wheel.  Caller must hold the mutex.
 * Returns true if the wheel has JUST become faulty (this call).
 */
static bool buddy2_check_encoder(int16_t applied_pwm, int32_t tick_delta,
                                 uint32_t elapsed_ms, uint32_t *p_no_tick_ms,
                                 bool *p_fault)
{
    int32_t magnitude = (applied_pwm < 0) ? -(int32_t)applied_pwm : (int32_t)applied_pwm;

    if (0 != tick_delta)
    {
        *p_no_tick_ms = 0u;
        *p_fault      = false;
        return false;
    }
    if (magnitude < BUDDY2_ENCODER_CHECK_MIN_PWM)
    {
        /* Too little power to expect movement: restart the timer, keep the
         * last verdict. */
        *p_no_tick_ms = 0u;
        return false;
    }
    if (*p_no_tick_ms < BUDDY2_ENCODER_FAULT_MS)
    {
        *p_no_tick_ms += elapsed_ms;
        if ((*p_no_tick_ms >= BUDDY2_ENCODER_FAULT_MS) && !*p_fault)
        {
            *p_fault = true;
            return true;
        }
    }
    return false;
}

void buddy2_update(uint32_t now_ms)
{
    uint32_t elapsed_ms;
    float    dt;
    int32_t  left_ticks;
    int32_t  right_ticks;

    if (!g_initialized)
    {
        return;
    }

    buddy2_lock();

    elapsed_ms = now_ms - g_last_update_ms;
    if ((elapsed_ms < BUDDY2_CONTROL_PERIOD_MS) && (0u != g_last_update_ms))
    {
        buddy2_unlock();
        return;
    }
    dt = (float)elapsed_ms / 1000.0f;
    g_last_update_ms = now_ms;

    /* 32-bit reads are atomic on the Cortex-M0+, so no interrupt lock.
     * Each wheel uses its OWN encoder; a dead one is reported below. */
    left_ticks  = g_left_ticks;
    right_ticks = g_right_ticks;

    {
        int32_t left_delta   = left_ticks  - g_state.left_ticks;
        int32_t right_delta  = right_ticks - g_state.right_ticks;
        float   left_mm_tick  = buddy2_mm_per_tick(g_calibration.left_ticks_per_wheel_rev);
        float   right_mm_tick = buddy2_mm_per_tick(g_calibration.right_ticks_per_wheel_rev);
        float   left_distance  = (float)left_delta  * left_mm_tick  * (float)g_left_direction;
        float   right_distance = (float)right_delta * right_mm_tick * (float)g_right_direction;

        g_state.left_speed_mm_s    = left_distance  / dt;
        g_state.right_speed_mm_s   = right_distance / dt;
        g_state.left_distance_mm  += left_distance;
        g_state.right_distance_mm += right_distance;
        g_state.left_ticks         = left_ticks;
        g_state.right_ticks        = right_ticks;
        g_state.timestamp_ms       = now_ms;

        {
            bool left_new  = buddy2_check_encoder(g_left_applied_pwm, left_delta, elapsed_ms,
                                                  &g_left_no_tick_ms, &g_state.left_encoder_fault);
            bool right_new = buddy2_check_encoder(g_right_applied_pwm, right_delta, elapsed_ms,
                                                  &g_right_no_tick_ms, &g_state.right_encoder_fault);

            /* A distance/turn move cannot finish correctly without both
             * encoders (its PID would drive the silent wheel flat out), so
             * stop it.  Raw drive and line following only report it. */
            if ((left_new || right_new) && g_motion.active)
            {
                buddy2_stop_locked();
            }
        }
    }

    if (g_motion.active)
    {
        float left_progress  = buddy2_absf(g_state.left_distance_mm  - g_motion.start_left_distance_mm);
        float right_progress = buddy2_absf(g_state.right_distance_mm - g_motion.start_right_distance_mm);
        float progress       = (left_progress + right_progress) * 0.5f;

        if (progress >= (g_motion.target_distance_mm - BUDDY2_POSITION_TOLERANCE_MM))
        {
            buddy2_stop_locked();
        }
        else
        {
            float target_speed = BUDDY2_MAX_SPEED_MM_S *
                                 ((float)g_motion.max_pwm / (float)g_calibration.pwm_limit);
            float left_target  = target_speed * (float)g_motion.left_direction;
            float right_target = target_speed * (float)g_motion.right_direction;
            float left_error   = left_target  - g_state.left_speed_mm_s;
            float right_error  = right_target - g_state.right_speed_mm_s;
            float left_derivative  = (left_error  - g_motion.left_previous_error)  / dt;
            float right_derivative = (right_error - g_motion.right_previous_error) / dt;
            float left_output;
            float right_output;

            g_motion.left_previous_error  = left_error;
            g_motion.right_previous_error = right_error;
            g_motion.left_integral  = buddy2_clampf(g_motion.left_integral  + (left_error  * dt),
                                                    BUDDY2_PID_INTEGRAL_LIMIT);
            g_motion.right_integral = buddy2_clampf(g_motion.right_integral + (right_error * dt),
                                                    BUDDY2_PID_INTEGRAL_LIMIT);

            left_output  = ((float)g_motion.left_direction  * (float)g_motion.max_pwm * BUDDY2_FEEDFORWARD_GAIN)
                         + (BUDDY2_KP * left_error)
                         + (BUDDY2_KI * g_motion.left_integral)
                         + (BUDDY2_KD * left_derivative);
            right_output = ((float)g_motion.right_direction * (float)g_motion.max_pwm * BUDDY2_FEEDFORWARD_GAIN)
                         + (BUDDY2_KP * right_error)
                         + (BUDDY2_KI * g_motion.right_integral)
                         + (BUDDY2_KD * right_derivative);

            buddy2_set_motor_pwm_locked(buddy2_clamp_pwm(left_output),
                                        buddy2_clamp_pwm(right_output));
        }
    }

    buddy2_unlock();
}

void buddy2_set_calibration(const buddy2_calibration_t *p_calibration)
{
    if ((NULL != p_calibration) &&
        (p_calibration->wheel_diameter_mm > 0.0f) &&
        (p_calibration->track_width_mm > 0.0f) &&
        (p_calibration->left_ticks_per_wheel_rev > 0.0f) &&
        (p_calibration->right_ticks_per_wheel_rev > 0.0f) &&
        (p_calibration->pwm_limit > 0u) &&
        (p_calibration->pwm_limit <= (uint16_t)BUDDY2_PWM_MAX))
    {
        buddy2_lock();
        g_calibration = *p_calibration;
        buddy2_unlock();
    }
}

void buddy2_get_calibration(buddy2_calibration_t *p_calibration)
{
    if (NULL != p_calibration)
    {
        buddy2_lock();
        *p_calibration = g_calibration;
        buddy2_unlock();
    }
}
