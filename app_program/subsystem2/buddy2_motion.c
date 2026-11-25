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
#include "robot_fault.h"
#include "robot_steering_trace.h"
#include "app.h"

/* ------------------------------------------------------------------ *
 *  Tunables
 * ------------------------------------------------------------------ */

#define BUDDY2_PI (3.14159265358979f)

/* PWM counter top.  clk_sys is 125 MHz on this port, so
 * 125 MHz / (12499 + 1) = 10 kHz.  If the motors feel weak at low speed,
 * try a lower frequency (a bigger wrap, e.g. 62499 = 2 kHz). */
#define BUDDY2_PWM_WRAP (12499u)

/* Default geometry - REPLACE with measured values (BUDDY2_CALIBRATION.md). */
#define BUDDY2_DEFAULT_WHEEL_DIAMETER_MM (65.0f)
#define BUDDY2_DEFAULT_TRACK_WIDTH_MM    (150.0f)
/* 387 rising edges per wheel turn: measured on the left wheel (Oct).
 * Same encoder model on both sides - re-measure the right wheel. */
#define BUDDY2_DEFAULT_TICKS_PER_REV (387.0f)
#define BUDDY2_DEFAULT_PWM_DEADBAND  (0u)

#define BUDDY2_MAX_SPEED_MM_S        (320.0f)
#define BUDDY2_POSITION_TOLERANCE_MM (3.0f)

/* Control law (unchanged from the CMake version - tune these next). */
#define BUDDY2_FEEDFORWARD_GAIN   (0.65f)
#define BUDDY2_KP                 (0.15f)
#define BUDDY2_KI                 (0.02f)
#define BUDDY2_KD                 (0.005f)
#define BUDDY2_PID_INTEGRAL_LIMIT (1000.0f)

/* Encoder check: a wheel driven at >= the check PWM (API units, after trim
 * and speed limit) that gives no tick for BUDDY2_ENCODER_FAULT_MS is flagged
 * as faulty.  Below that PWM the wheel may legitimately not turn, so it is
 * not checked.  A distance/turn move's PID raises the power on a wheel that
 * does not turn, so there 3000 is enough.  Open-loop drive (line following,
 * raw PWM, avoidance) keeps its power: the motors need about 5400 to keep
 * turning (the right one more), and at 3000 a wheel that simply stalled at
 * low power, e.g. line following at the 40 % limit (~4600), was reported as
 * an encoder fault (25 Nov). */
#define BUDDY2_ENCODER_CHECK_MIN_PWM      (3000)
#define BUDDY2_ENCODER_CHECK_OPEN_MIN_PWM (6500)
#define BUDDY2_ENCODER_FAULT_MS           (800u)

/* Trace values are tenths and saturate just inside the integer ranges. */
#define BUDDY2_TRACE_LIMIT     (2147480000.0f)
#define BUDDY2_TRACE_SPEED_MAX (32767)
/* Trim and speed limit are percentages; both scale an open-loop command. */
#define BUDDY2_PERCENT         (100u)
#define BUDDY2_PERCENT_SQUARED (10000)
/* speed_pct value meaning "use the global speed limit (g_speed_pct)". */
#define BUDDY2_SPEED_USE_LIMIT (0u)
/* Use the global speed limit but do not apply temporary barcode slowing. */
#define BUDDY2_SPEED_USE_LIMIT_IGNORE_BARCODE (255u)
#define BUDDY2_DEG_PER_TURN                   (360.0f)
#define BUDDY2_MS_PER_S                       (1000.0f)
/* Shortest time between two control steps.  The cyclic handler wakes the
 * motion task every BUDDY2_CONTROL_PERIOD_MS, but a higher-priority task
 * (USB, Wi-Fi) can hold one wake-up back by a millisecond or two, so the
 * next, on-time one arrives after only 18-19 ms.  Asking for a full period
 * skipped that step and left the motors uncorrected for 40 ms.  Half a
 * period accepts it, and still skips a queued wake-up that runs straight
 * after a late one (its tiny time step would only add noise). */
#define BUDDY2_MIN_STEP_MS (BUDDY2_CONTROL_PERIOD_MS / 2u)

/* ------------------------------------------------------------------ *
 *  State
 * ------------------------------------------------------------------ */

typedef struct
{
    bool      b_active;
    float32_t target_distance_mm;
    float32_t start_left_distance_mm;
    float32_t start_right_distance_mm;
    int16_t   max_pwm;
    int16_t   requested_max_pwm;
    uint8_t   requested_speed_pct;
    int8_t    left_direction;
    int8_t    right_direction;
    float32_t left_integral;
    float32_t right_integral;
    float32_t left_previous_error;
    float32_t right_previous_error;
} buddy2_move_t;

/* Written by the encoder interrupt, read by tasks. */
static volatile int32_t g_left_ticks  = 0;
static volatile int32_t g_right_ticks = 0;

static buddy2_calibration_t g_calibration = {
    BUDDY2_DEFAULT_WHEEL_DIAMETER_MM, BUDDY2_DEFAULT_TRACK_WIDTH_MM,
    BUDDY2_DEFAULT_TICKS_PER_REV,     BUDDY2_DEFAULT_TICKS_PER_REV,
    (uint16_t) BUDDY2_PWM_MAX,        BUDDY2_DEFAULT_PWM_DEADBAND};

static buddy2_motion_state_t g_state;
/* Open-loop trim in percent (BUDDY2_TRIM_MIN..100).  Only scales
 * buddy2_set_motor_pwm(); distance/turn moves use the speed PID instead. */
static uint8_t g_left_trim_pct  = BUDDY2_TRIM_DEFAULT_LEFT;
static uint8_t g_right_trim_pct = BUDDY2_TRIM_DEFAULT_RIGHT;
/* Global speed limit in percent: scales open-loop PWM and the max_pwm of
 * distance/turn moves. */
static uint8_t  g_speed_pct = BUDDY2_SPEED_DEFAULT;
static bool     g_barcode_slowdown;
static int16_t  g_open_left_request;
static int16_t  g_open_right_request;
static uint8_t  g_open_speed_request = BUDDY2_SPEED_USE_LIMIT;
static uint32_t g_last_update_ms     = 0u;
static int8_t   g_left_direction     = 1;
static int8_t   g_right_direction    = 1;
/* PWM actually sent to each motor (API units), for the encoder check. */
static int16_t g_left_applied_pwm  = 0;
static int16_t g_right_applied_pwm = 0;
/* How long each wheel has been driven without a tick. */
static uint32_t      g_left_no_tick_ms  = 0u;
static uint32_t      g_right_no_tick_ms = 0u;
static bool          g_b_initialized    = false;
static ID            g_mutex_id         = 0;
static buddy2_move_t g_motion;

static float32_t buddy2_absf (float32_t value);
static float32_t buddy2_clampf (float32_t value, float32_t limit);
static void      buddy2_lock (void);
static void      buddy2_unlock (void);
static void      buddy2_encoder_isr (uint32_t gpio, uint32_t events);
static float32_t buddy2_mm_per_tick (float32_t ticks_per_rev);
static int16_t   buddy2_clamp_pwm (float32_t value);
static void      buddy2_apply_one (uint32_t pin_a, uint32_t pin_b, int16_t pwm);
static void buddy2_set_motor_pwm_locked (int16_t left_pwm, int16_t right_pwm);
static void buddy2_drive_locked (int16_t left_pwm, int16_t right_pwm,
                                 const robot_steering_input_t * p_input,
                                 uint8_t                        speed_pct);
static bool buddy2_speed_valid (uint8_t percent);
static uint8_t buddy2_effective_speed (uint8_t requested);
static int32_t buddy2_trace_distance (float32_t distance_mm);
static int16_t buddy2_trace_speed (float32_t speed);
static void    buddy2_trace (int16_t left, int16_t right,
                             const robot_steering_input_t * p_input);
static void    buddy2_stop_locked (const robot_steering_input_t * p_input);
static void    buddy2_encoder_pin_init (uint32_t pin);
static bool buddy2_start_motion (float32_t left_distance_mm,
                                 float32_t right_distance_mm, int16_t max_pwm,
                                 uint8_t speed_pct);
static bool buddy2_check_encoder (int16_t applied_pwm, int32_t min_pwm,
                                  int32_t tick_delta, uint32_t elapsed_ms,
                                  uint32_t * p_no_tick_ms, bool * p_fault);
static void buddy2_report_faults (bool b_left_fault, bool b_right_fault);
static void buddy2_update_odometry (uint32_t now_ms, uint32_t elapsed_ms,
                                    float32_t dt_s);
static void buddy2_run_pid (float32_t dt_s);

/* ------------------------------------------------------------------ *
 *  Public API
 * ------------------------------------------------------------------ */

int32_t buddy2_motion_init (void)
{
    T_CMTX  cmtx     = {0};
    ID      mutex_id = 0;
    int32_t result   = 0;

    cmtx.exinf = NULL;
    /* Priority inheritance.  Note: TA_TFIFO / TA_TPRI / TA_INHERIT /
     * TA_CEILING are one 2-bit field, NOT flags - OR-ing two of them
     * (e.g. TA_TPRI | TA_INHERIT == TA_CEILING) gives E_PAR. */
    cmtx.mtxatr  = TA_INHERIT;
    cmtx.ceilpri = 0;
    mutex_id     = tk_cre_mtx(&cmtx);
    if (mutex_id < E_OK)
    {
        result = (int32_t) mutex_id; /* Negative kernel error code. */
    }
    else
    {
        g_mutex_id = mutex_id;

        /* micro T-Kernel leaves the PWM block in reset - release it. */
        hal_pwm_release_reset();

        /* Motor pins.  Pins A and B of a motor share one PWM slice. */
        hal_pwm_pin_init(MOTOR_LEFT_PWM_A, BUDDY2_PWM_WRAP);
        hal_pwm_pin_init(MOTOR_LEFT_PWM_B, BUDDY2_PWM_WRAP);
        hal_pwm_pin_init(MOTOR_RIGHT_PWM_A, BUDDY2_PWM_WRAP);
        hal_pwm_pin_init(MOTOR_RIGHT_PWM_B, BUDDY2_PWM_WRAP);

        buddy2_encoder_pin_init(ENCODER_LEFT_A);
        buddy2_encoder_pin_init(ENCODER_RIGHT_A);

        buddy2_lock();
        g_b_initialized = true;
        buddy2_stop_locked(NULL);
        buddy2_unlock();

        buddy2_reset_encoders();
    }

    return result;
}

void buddy2_set_motor_pwm (int16_t left_pwm, int16_t right_pwm)
{
    buddy2_set_motor_pwm_observed(left_pwm, right_pwm, NULL);
}

void buddy2_set_motor_pwm_observed (int16_t left_pwm, int16_t right_pwm,
                                    const robot_steering_input_t * p_input)
{
    if (g_b_initialized)
    {
        buddy2_lock();
        buddy2_drive_locked(left_pwm, right_pwm, p_input,
                            BUDDY2_SPEED_USE_LIMIT);
        buddy2_unlock();
    }
}

bool buddy2_set_motor_pwm_at_speed (int16_t left_pwm, int16_t right_pwm,
                                    uint8_t speed_pct)
{
    bool b_driven = false;

    if (g_b_initialized && buddy2_speed_valid(speed_pct))
    {
        buddy2_lock();
        buddy2_drive_locked(left_pwm, right_pwm, NULL, speed_pct);
        buddy2_unlock();
        b_driven = true;
    }

    return b_driven;
}

bool buddy2_set_speed_limit (uint8_t percent)
{
    bool b_valid = buddy2_speed_valid(percent);

    if (b_valid)
    {
        buddy2_lock();
        g_speed_pct = percent;
        buddy2_unlock();
    }

    return b_valid;
}

uint8_t buddy2_get_speed_limit (void)
{
    uint8_t percent = 0u;

    buddy2_lock();
    percent = g_speed_pct;
    buddy2_unlock();

    return percent;
}

/* Caller holds the motion mutex. Preserve the configured speed while the
 * barcode override also limits explicit-speed commands and PID moves. */
static uint8_t buddy2_effective_speed (uint8_t requested)
{
    bool b_ignore_barcode =
        (BUDDY2_SPEED_USE_LIMIT_IGNORE_BARCODE == requested);
    uint8_t percent =
        ((BUDDY2_SPEED_USE_LIMIT == requested) || b_ignore_barcode)
            ? g_speed_pct
            : requested;
    if (!b_ignore_barcode && g_barcode_slowdown &&
        (percent > BUDDY2_BARCODE_SPEED_PCT))
    {
        percent = BUDDY2_BARCODE_SPEED_PCT;
    }
    return percent;
}

void buddy2_get_barcode_speed (bool * active, uint8_t * effective_pct)
{
    buddy2_lock();
    *active        = g_barcode_slowdown;
    *effective_pct = buddy2_effective_speed(BUDDY2_SPEED_USE_LIMIT);
    buddy2_unlock();
}

void buddy2_set_barcode_slowdown (bool active)
{
    if (!g_b_initialized)
    {
        return;
    }
    buddy2_lock();
    if (g_barcode_slowdown != active)
    {
        g_barcode_slowdown = active;
        if (g_motion.b_active)
        {
            int16_t previous_max = g_motion.max_pwm;
            uint8_t percent =
                buddy2_effective_speed(g_motion.requested_speed_pct);
            float32_t target_speed;
            g_motion.max_pwm =
                buddy2_clamp_pwm((float32_t) g_motion.requested_max_pwm *
                                 percent / BUDDY2_PERCENT);
            target_speed =
                BUDDY2_MAX_SPEED_MM_S *
                ((float32_t) g_motion.max_pwm / g_calibration.pwm_limit);
            /* Avoid derivative kick and integral carry-over on speed changes.
             */
            g_motion.left_integral  = 0.0f;
            g_motion.right_integral = 0.0f;
            g_motion.left_previous_error =
                target_speed * g_motion.left_direction -
                g_state.left_speed_mm_s;
            g_motion.right_previous_error =
                target_speed * g_motion.right_direction -
                g_state.right_speed_mm_s;
            if (previous_max > 0)
            {
                buddy2_set_motor_pwm_locked(
                    buddy2_clamp_pwm((float32_t) g_left_applied_pwm *
                                     g_motion.max_pwm / previous_max),
                    buddy2_clamp_pwm((float32_t) g_right_applied_pwm *
                                     g_motion.max_pwm / previous_max));
            }
            else
            {
                buddy2_set_motor_pwm_locked(
                    (int16_t) (g_motion.left_direction * g_motion.max_pwm),
                    (int16_t) (g_motion.right_direction * g_motion.max_pwm));
            }
            buddy2_trace(g_left_applied_pwm, g_right_applied_pwm, NULL);
        }
        else if ((g_open_left_request != 0) || (g_open_right_request != 0))
        {
            buddy2_drive_locked(g_open_left_request, g_open_right_request, NULL,
                                g_open_speed_request);
        }
    }
    buddy2_unlock();
}

bool buddy2_set_trim (uint8_t left_pct, uint8_t right_pct)
{
    bool b_valid =
        (left_pct >= BUDDY2_TRIM_MIN) && (left_pct <= BUDDY2_PERCENT) &&
        (right_pct >= BUDDY2_TRIM_MIN) && (right_pct <= BUDDY2_PERCENT);

    if (b_valid)
    {
        buddy2_lock();
        g_left_trim_pct  = left_pct;
        g_right_trim_pct = right_pct;
        buddy2_unlock();
    }

    return b_valid;
}

void buddy2_get_trim (uint8_t * p_left_pct, uint8_t * p_right_pct)
{
    if ((NULL != p_left_pct) && (NULL != p_right_pct))
    {
        buddy2_lock();
        *p_left_pct  = g_left_trim_pct;
        *p_right_pct = g_right_trim_pct;
        buddy2_unlock();
    }
}

void buddy2_stop (void)
{
    buddy2_stop_observed(NULL);
}

void buddy2_stop_observed (const robot_steering_input_t * p_input)
{
    if (g_b_initialized)
    {
        buddy2_lock();
        buddy2_stop_locked(p_input);
        buddy2_unlock();
    }
}

void buddy2_reset_encoders (void)
{
    UINT interrupt_state = 0u;

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
    /* A reset also forgets latched encoder faults; a genuinely silent
     * wheel is flagged again by the next powered check. */
    g_state.left_encoder_fault  = false;
    g_state.right_encoder_fault = false;
    g_left_no_tick_ms           = 0u;
    g_right_no_tick_ms          = 0u;
    buddy2_unlock();
}

void buddy2_get_encoder_position (buddy2_encoder_position_t * p_position)
{
    UINT    interrupt_state = 0u;
    int32_t left_ticks      = 0;
    int32_t right_ticks     = 0;
    int32_t left_delta      = 0;
    int32_t right_delta     = 0;

    if ((NULL != p_position) && g_b_initialized)
    {
        buddy2_lock();
        DI(interrupt_state);
        left_ticks  = g_left_ticks;
        right_ticks = g_right_ticks;
        EI(interrupt_state);
        /* Wrap-safe tick differences: unsigned subtraction, then back to a
         * signed delta that is small between two updates. */
        left_delta =
            (int32_t) ((uint32_t) left_ticks - (uint32_t) g_state.left_ticks);
        right_delta =
            (int32_t) ((uint32_t) right_ticks - (uint32_t) g_state.right_ticks);
        p_position->left_distance_mm =
            g_state.left_distance_mm +
            ((float32_t) left_delta *
             buddy2_mm_per_tick(g_calibration.left_ticks_per_wheel_rev) *
             (float32_t) g_left_direction);
        p_position->right_distance_mm =
            g_state.right_distance_mm +
            ((float32_t) right_delta *
             buddy2_mm_per_tick(g_calibration.right_ticks_per_wheel_rev) *
             (float32_t) g_right_direction);
        p_position->left_encoder_fault  = g_state.left_encoder_fault;
        p_position->right_encoder_fault = g_state.right_encoder_fault;
        buddy2_unlock();
    }
}

void buddy2_get_motion_state (buddy2_motion_state_t * p_state)
{
    if ((NULL != p_state) && g_b_initialized)
    {
        buddy2_lock();
        *p_state = g_state;
        buddy2_unlock();
    }
}

bool buddy2_move_forward (float32_t distance_mm, int16_t max_pwm)
{
    float32_t distance = buddy2_absf(distance_mm);

    return buddy2_start_motion(distance, distance, max_pwm,
                               BUDDY2_SPEED_USE_LIMIT);
}

bool buddy2_move_backward (float32_t distance_mm, int16_t max_pwm)
{
    float32_t distance = buddy2_absf(distance_mm);

    return buddy2_start_motion(-distance, -distance, max_pwm,
                               BUDDY2_SPEED_USE_LIMIT);
}

bool buddy2_move_backward_at_speed (float32_t distance_mm, int16_t max_pwm,
                                    uint8_t speed_pct)
{
    float32_t distance  = buddy2_absf(distance_mm);
    bool      b_started = false;

    if (buddy2_speed_valid(speed_pct))
    {
        b_started =
            buddy2_start_motion(-distance, -distance, max_pwm, speed_pct);
    }

    return b_started;
}

bool buddy2_turn_left (float32_t angle_deg, int16_t max_pwm)
{
    /* In-place turn: each wheel travels an arc of the track circle. */
    float32_t distance =
        (BUDDY2_PI * g_calibration.track_width_mm * buddy2_absf(angle_deg)) /
        BUDDY2_DEG_PER_TURN;

    return buddy2_start_motion(-distance, distance, max_pwm,
                               BUDDY2_SPEED_USE_LIMIT);
}

bool buddy2_turn_right (float32_t angle_deg, int16_t max_pwm)
{
    float32_t distance =
        (BUDDY2_PI * g_calibration.track_width_mm * buddy2_absf(angle_deg)) /
        BUDDY2_DEG_PER_TURN;

    return buddy2_start_motion(distance, -distance, max_pwm,
                               BUDDY2_SPEED_USE_LIMIT);
}

bool buddy2_turn_left_at_speed_limit (float32_t angle_deg, int16_t max_pwm)
{
    float32_t distance =
        (BUDDY2_PI * g_calibration.track_width_mm * buddy2_absf(angle_deg)) /
        BUDDY2_DEG_PER_TURN;

    return buddy2_start_motion(-distance, distance, max_pwm,
                               BUDDY2_SPEED_USE_LIMIT_IGNORE_BARCODE);
}

bool buddy2_turn_right_at_speed_limit (float32_t angle_deg, int16_t max_pwm)
{
    float32_t distance =
        (BUDDY2_PI * g_calibration.track_width_mm * buddy2_absf(angle_deg)) /
        BUDDY2_DEG_PER_TURN;

    return buddy2_start_motion(distance, -distance, max_pwm,
                               BUDDY2_SPEED_USE_LIMIT_IGNORE_BARCODE);
}

bool buddy2_motion_active (void)
{
    bool b_active = false;

    buddy2_lock();
    b_active = g_motion.b_active;
    buddy2_unlock();

    return b_active;
}

void buddy2_update (uint32_t now_ms)
{
    uint32_t  elapsed_ms = 0u;
    float32_t dt_s       = 0.0f;

    if (g_b_initialized)
    {
        buddy2_lock();
        /* Wrap-safe difference; on the first call it is the time since
         * boot.  A step only runs with a non-zero elapsed time, so dt_s
         * below is never 0. */
        elapsed_ms = now_ms - g_last_update_ms;
        if (elapsed_ms >= BUDDY2_MIN_STEP_MS)
        {
            dt_s             = (float32_t) elapsed_ms / BUDDY2_MS_PER_S;
            g_last_update_ms = now_ms;
            buddy2_update_odometry(now_ms, elapsed_ms, dt_s);
            if (g_motion.b_active)
            {
                buddy2_run_pid(dt_s);
            }
        }
        buddy2_unlock();
    }
}

void buddy2_set_calibration (const buddy2_calibration_t * p_calibration)
{
    if ((NULL != p_calibration) && (p_calibration->wheel_diameter_mm > 0.0f) &&
        (p_calibration->track_width_mm > 0.0f) &&
        (p_calibration->left_ticks_per_wheel_rev > 0.0f) &&
        (p_calibration->right_ticks_per_wheel_rev > 0.0f) &&
        (p_calibration->pwm_limit > 0u) &&
        (p_calibration->pwm_limit <= (uint16_t) BUDDY2_PWM_MAX))
    {
        buddy2_lock();
        g_calibration = *p_calibration;
        buddy2_unlock();
    }
}

void buddy2_get_calibration (buddy2_calibration_t * p_calibration)
{
    if (NULL != p_calibration)
    {
        buddy2_lock();
        *p_calibration = g_calibration;
        buddy2_unlock();
    }
}

/* ------------------------------------------------------------------ *
 *  Small helpers (no libm needed)
 * ------------------------------------------------------------------ */

static float32_t buddy2_absf (float32_t value)
{
    return (value < 0.0f) ? -value : value;
}

static float32_t buddy2_clampf (float32_t value, float32_t limit)
{
    float32_t result = value;

    if (value > limit)
    {
        result = limit;
    }
    else if (value < -limit)
    {
        result = -limit;
    }
    else
    {
        result = value;
    }

    return result;
}

static void buddy2_lock (void)
{
    (void) tk_loc_mtx(g_mutex_id, TMO_FEVR);
}

static void buddy2_unlock (void)
{
    (void) tk_unl_mtx(g_mutex_id);
}

/* ------------------------------------------------------------------ *
 *  Encoder interrupt (runs in interrupt context - keep it tiny)
 * ------------------------------------------------------------------ */

static void buddy2_encoder_isr (uint32_t gpio, uint32_t events)
{
    if (0u != (events & HAL_GPIO_EDGE_RISE))
    {
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
}

/* ------------------------------------------------------------------ *
 *  Motor output
 * ------------------------------------------------------------------ */

static float32_t buddy2_mm_per_tick (float32_t ticks_per_rev)
{
    float32_t mm_per_tick = 0.0f;

    if (ticks_per_rev > 0.0f)
    {
        mm_per_tick =
            (BUDDY2_PI * g_calibration.wheel_diameter_mm) / ticks_per_rev;
    }

    return mm_per_tick;
}

/* Clamp a float command to the calibrated limit, then convert. */
static int16_t buddy2_clamp_pwm (float32_t value)
{
    /* pwm_limit <= BUDDY2_PWM_MAX, so the clamped value fits int16_t. */
    return (int16_t) buddy2_clampf(value, (float32_t) g_calibration.pwm_limit);
}

/* Drive one H-bridge.  pwm is in API units (-32767..32767). */
static void buddy2_apply_one (uint32_t pin_a, uint32_t pin_b, int16_t pwm)
{
    /* |pwm| <= 32767, so the magnitude is exact as uint32_t. */
    uint32_t magnitude =
        (uint32_t) ((pwm < 0) ? -(int32_t) pwm : (int32_t) pwm);
    uint32_t level = 0u;

    if ((magnitude > 0u) && (magnitude < g_calibration.pwm_deadband))
    {
        magnitude = g_calibration.pwm_deadband;
    }

    /* Scale API units to PWM counter ticks. */
    level = (magnitude * (BUDDY2_PWM_WRAP + 1u)) / (uint32_t) BUDDY2_PWM_MAX;

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
static void buddy2_set_motor_pwm_locked (int16_t left_pwm, int16_t right_pwm)
{
    int16_t left_command  = buddy2_clamp_pwm((float32_t) left_pwm);
    int16_t right_command = buddy2_clamp_pwm((float32_t) right_pwm);

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
    buddy2_apply_one(MOTOR_LEFT_PWM_A, MOTOR_LEFT_PWM_B, left_command);
    buddy2_apply_one(MOTOR_RIGHT_PWM_A, MOTOR_RIGHT_PWM_B, right_command);
}

/*
 * Open-loop drive with trim and a speed percentage applied.  Caller must hold
 * the mutex.  speed_pct is BUDDY2_SPEED_USE_LIMIT (global limit) or an
 * already validated BUDDY2_SPEED_MIN..100 value.
 */
static void buddy2_drive_locked (int16_t left_pwm, int16_t right_pwm,
                                 const robot_steering_input_t * p_input,
                                 uint8_t                        speed_pct)
{
    uint8_t percent = buddy2_effective_speed(speed_pct);

    g_open_left_request  = left_pwm;
    g_open_right_request = right_pwm;
    g_open_speed_request = speed_pct;

    /* trim and speed are at most 100 %, so each product divided by
     * 100 * 100 stays within the int16_t command range. */
    buddy2_set_motor_pwm_locked(
        (int16_t) (((int32_t) left_pwm * (int32_t) g_left_trim_pct *
                    (int32_t) percent) /
                   BUDDY2_PERCENT_SQUARED),
        (int16_t) (((int32_t) right_pwm * (int32_t) g_right_trim_pct *
                    (int32_t) percent) /
                   BUDDY2_PERCENT_SQUARED));
    /* Observe requested steering before trim; trim alone is not a turn.
     * Logging only queues transitions and never publishes while locked.
     * Same kernel clock as the steering trace; it wraps, so consumers use
     * differences. */
    robot_action_record(robot_action_direction_for_pwm(left_pwm, right_pwm),
                        ROBOT_ACTION_PWM, left_pwm, right_pwm, app_now_ms());
    buddy2_trace(left_pwm, right_pwm, p_input);
}

/* True when percent is a usable speed (BUDDY2_SPEED_MIN..100). */
static bool buddy2_speed_valid (uint8_t percent)
{
    return ((percent >= BUDDY2_SPEED_MIN) && (percent <= BUDDY2_PERCENT));
}

/* Distance in tenths of a millimetre, saturated; NaN gives 0. */
static int32_t buddy2_trace_distance (float32_t distance_mm)
{
    float32_t value  = distance_mm * 10.0f;
    int32_t   result = 0;

    if (value > BUDDY2_TRACE_LIMIT)
    {
        result = INT32_MAX;
    }
    else if (value < -BUDDY2_TRACE_LIMIT)
    {
        result = -INT32_MAX;
    }
    else if (value >= -BUDDY2_TRACE_LIMIT)
    {
        /* In range, so the conversion is defined. */
        result = (int32_t) value;
    }
    else
    {
        result = 0; /* NaN fails every comparison above. */
    }

    return result;
}

/* Speed in tenths of mm/s, saturated to int16_t. */
static int16_t buddy2_trace_speed (float32_t speed)
{
    int32_t value = buddy2_trace_distance(speed);

    if (value > BUDDY2_TRACE_SPEED_MAX)
    {
        value = BUDDY2_TRACE_SPEED_MAX;
    }
    else if (value < -BUDDY2_TRACE_SPEED_MAX)
    {
        value = -BUDDY2_TRACE_SPEED_MAX;
    }
    else
    {
        /* Already in range. */
    }

    /* Clamped to +/-32767 above. */
    return (int16_t) value;
}

/* Snapshot while the same mutex protects accepted output and odometry. */
static void buddy2_trace (int16_t left, int16_t right,
                          const robot_steering_input_t * p_input)
{
    robot_steering_event_t event = {0};

    event.ts_ms = app_now_ms(); /* Same kernel clock as IR and odometry. */
    event.requested[0]    = left;
    event.requested[1]    = right;
    event.applied[0]      = g_left_applied_pwm;
    event.applied[1]      = g_right_applied_pwm;
    event.distance_x10[0] = buddy2_trace_distance(g_state.left_distance_mm);
    event.distance_x10[1] = buddy2_trace_distance(g_state.right_distance_mm);
    event.speed_x10[0]    = buddy2_trace_speed(g_state.left_speed_mm_s);
    event.speed_x10[1]    = buddy2_trace_speed(g_state.right_speed_mm_s);
    event.encoder_ms      = g_state.timestamp_ms;
    /* Bit 0 left fault, bit 1 right fault. */
    event.faults    = (uint8_t) ((g_state.left_encoder_fault ? 1u : 0u) |
                                 (g_state.right_encoder_fault ? 2u : 0u));
    event.has_input = (NULL != p_input);
    if (NULL != p_input)
    {
        event.input = *p_input;
    }
    robot_steering_record(&event);
}

/* Caller must hold the mutex. */
static void buddy2_stop_locked (const robot_steering_input_t * p_input)
{
    g_motion.b_active    = false;
    g_open_left_request  = 0;
    g_open_right_request = 0;
    g_left_applied_pwm   = 0;
    g_right_applied_pwm  = 0;
    buddy2_apply_one(MOTOR_LEFT_PWM_A, MOTOR_LEFT_PWM_B, 0);
    buddy2_apply_one(MOTOR_RIGHT_PWM_A, MOTOR_RIGHT_PWM_B, 0);
    /* The millisecond clock wraps; consumers use differences. */
    robot_action_record(ROBOT_ACTION_NONE, ROBOT_ACTION_STOP, 0, 0,
                        app_now_ms());
    buddy2_trace(0, 0, p_input);
}

static void buddy2_encoder_pin_init (uint32_t pin)
{
    /* The encoders pull their output low, so the pin needs a pull-up. */
    hal_gpio_input_pullup(pin);
    (void) gpio_irq_register(pin, HAL_GPIO_EDGE_RISE, buddy2_encoder_isr);
}

/* speed_pct is BUDDY2_SPEED_USE_LIMIT (global limit) or an already
 * validated BUDDY2_SPEED_MIN..100 value. */
static bool buddy2_start_motion (float32_t left_distance_mm,
                                 float32_t right_distance_mm, int16_t max_pwm,
                                 uint8_t speed_pct)
{
    float32_t left_abs  = buddy2_absf(left_distance_mm);
    float32_t right_abs = buddy2_absf(right_distance_mm);
    uint8_t   direction = ROBOT_ACTION_NONE;
    uint8_t   percent   = speed_pct;
    bool      b_started = false;

    /* A zero-length move (both distances 0) is rejected. */
    if (g_b_initialized && (max_pwm > 0) &&
        ((left_abs > 0.0f) || (right_abs > 0.0f)))
    {
        if (left_distance_mm < right_distance_mm)
        {
            direction = ROBOT_ACTION_LEFT;
        }
        else if (left_distance_mm > right_distance_mm)
        {
            direction = ROBOT_ACTION_RIGHT;
        }
        else
        {
            direction = ROBOT_ACTION_NONE;
        }

        buddy2_lock();
        g_motion.requested_max_pwm   = max_pwm;
        g_motion.requested_speed_pct = speed_pct;
        percent                      = buddy2_effective_speed(speed_pct);
        g_motion.b_active            = true;
        g_motion.target_distance_mm =
            (left_abs > right_abs) ? left_abs : right_abs;
        g_motion.start_left_distance_mm  = g_state.left_distance_mm;
        g_motion.start_right_distance_mm = g_state.right_distance_mm;
        g_motion.max_pwm =
            buddy2_clamp_pwm(((float32_t) max_pwm * (float32_t) percent) /
                             (float32_t) BUDDY2_PERCENT);
        g_motion.left_direction       = (left_distance_mm < 0.0f) ? -1 : 1;
        g_motion.right_direction      = (right_distance_mm < 0.0f) ? -1 : 1;
        g_motion.left_integral        = 0.0f;
        g_motion.right_integral       = 0.0f;
        g_motion.left_previous_error  = 0.0f;
        g_motion.right_previous_error = 0.0f;
        /* Direction is +/-1 and max_pwm is clamped, so the products fit. */
        buddy2_set_motor_pwm_locked(
            (int16_t) (g_motion.left_direction * g_motion.max_pwm),
            (int16_t) (g_motion.right_direction * g_motion.max_pwm));
        /* Record the planned direction, not straight-move PID adjustments. */
        robot_action_record(direction, ROBOT_ACTION_MOTION,
                            (int16_t) (g_motion.left_direction * max_pwm),
                            (int16_t) (g_motion.right_direction * max_pwm),
                            app_now_ms());
        buddy2_trace((int16_t) (g_motion.left_direction * max_pwm),
                     (int16_t) (g_motion.right_direction * max_pwm), NULL);
        buddy2_unlock();
        b_started = true;
    }

    return b_started;
}

/*
 * Encoder check for one wheel.  Caller must hold the mutex.
 * Returns true if the wheel has JUST become faulty (this call).
 */
static bool buddy2_check_encoder (int16_t applied_pwm, int32_t min_pwm,
                                  int32_t tick_delta, uint32_t elapsed_ms,
                                  uint32_t * p_no_tick_ms, bool * p_fault)
{
    int32_t magnitude =
        (applied_pwm < 0) ? -(int32_t) applied_pwm : (int32_t) applied_pwm;
    bool b_new_fault = false;

    if (0 != tick_delta)
    {
        *p_no_tick_ms = 0u;
        *p_fault      = false;
    }
    else if (magnitude < min_pwm)
    {
        /* Too little power to expect movement: restart the timer, keep the
         * last verdict. */
        *p_no_tick_ms = 0u;
    }
    else if (*p_no_tick_ms < BUDDY2_ENCODER_FAULT_MS)
    {
        *p_no_tick_ms += elapsed_ms;
        if ((*p_no_tick_ms >= BUDDY2_ENCODER_FAULT_MS) && !*p_fault)
        {
            *p_fault    = true;
            b_new_fault = true;
        }
    }
    else
    {
        /* Already timed out: the fault stays latched. */
    }

    return b_new_fault;
}

/* Mirror the per-wheel verdicts into the shared fault byte (robot_fault.h).
 * Both wheels powered with no ticks = motor stall; only one = encoder fault.
 * Buddy 2 is the only writer of these two bits, and the byte is only touched
 * when they change. */
static void buddy2_report_faults (bool b_left_fault, bool b_right_fault)
{
    uint8_t wanted  = 0u;
    uint8_t current = (uint8_t) (robot_fault_get() & (ROBOT_FAULT_ENCODER |
                                                      ROBOT_FAULT_MOTOR_STALL));

    if (b_left_fault && b_right_fault)
    {
        wanted = ROBOT_FAULT_MOTOR_STALL;
    }
    else if (b_left_fault || b_right_fault)
    {
        wanted = ROBOT_FAULT_ENCODER;
    }
    else
    {
        wanted = 0u;
    }
    if (wanted != current)
    {
        robot_fault_update(ROBOT_FAULT_ENCODER | ROBOT_FAULT_MOTOR_STALL,
                           wanted);
    }
}

/* Update distances, speeds and the encoder checks.  Caller holds the
 * mutex. */
static void buddy2_update_odometry (uint32_t now_ms, uint32_t elapsed_ms,
                                    float32_t dt_s)
{
    /* 32-bit reads are atomic on the Cortex-M0+, so no interrupt lock.
     * Each wheel uses its OWN encoder; a dead one is reported below. */
    int32_t   left_ticks  = g_left_ticks;
    int32_t   right_ticks = g_right_ticks;
    int32_t   left_delta  = left_ticks - g_state.left_ticks;
    int32_t   right_delta = right_ticks - g_state.right_ticks;
    float32_t left_mm_tick =
        buddy2_mm_per_tick(g_calibration.left_ticks_per_wheel_rev);
    float32_t right_mm_tick =
        buddy2_mm_per_tick(g_calibration.right_ticks_per_wheel_rev);
    float32_t left_distance =
        (float32_t) left_delta * left_mm_tick * (float32_t) g_left_direction;
    float32_t right_distance =
        (float32_t) right_delta * right_mm_tick * (float32_t) g_right_direction;
    bool b_left_new  = false;
    bool b_right_new = false;
    /* A PID move raises the power on a silent wheel; open-loop drive does
     * not, so it is only checked where the wheel must turn. */
    int32_t min_pwm = g_motion.b_active ? BUDDY2_ENCODER_CHECK_MIN_PWM
                                        : BUDDY2_ENCODER_CHECK_OPEN_MIN_PWM;

    g_state.left_speed_mm_s  = left_distance / dt_s;
    g_state.right_speed_mm_s = right_distance / dt_s;
    g_state.left_distance_mm += left_distance;
    g_state.right_distance_mm += right_distance;
    g_state.left_ticks   = left_ticks;
    g_state.right_ticks  = right_ticks;
    g_state.timestamp_ms = now_ms;

    b_left_new  = buddy2_check_encoder(g_left_applied_pwm, min_pwm, left_delta,
                                       elapsed_ms, &g_left_no_tick_ms,
                                       &g_state.left_encoder_fault);
    b_right_new = buddy2_check_encoder(
        g_right_applied_pwm, min_pwm, right_delta, elapsed_ms,
        &g_right_no_tick_ms, &g_state.right_encoder_fault);
    buddy2_report_faults(g_state.left_encoder_fault,
                         g_state.right_encoder_fault);

    /* A distance/turn move cannot finish correctly without both encoders
     * (its PID would drive the silent wheel flat out), so stop it.  Raw
     * drive and line following only report it. */
    if ((b_left_new || b_right_new) && g_motion.b_active)
    {
        buddy2_stop_locked(NULL);
    }
}

/* Speed PID for an active distance/turn move.  Caller holds the mutex. */
static void buddy2_run_pid (float32_t dt_s)
{
    float32_t left_progress =
        buddy2_absf(g_state.left_distance_mm - g_motion.start_left_distance_mm);
    float32_t right_progress = buddy2_absf(g_state.right_distance_mm -
                                           g_motion.start_right_distance_mm);
    float32_t progress       = (left_progress + right_progress) * 0.5f;

    if (progress >=
        (g_motion.target_distance_mm - BUDDY2_POSITION_TOLERANCE_MM))
    {
        buddy2_stop_locked(NULL);
    }
    else
    {
        float32_t target_speed =
            BUDDY2_MAX_SPEED_MM_S * ((float32_t) g_motion.max_pwm /
                                     (float32_t) g_calibration.pwm_limit);
        float32_t left_target =
            target_speed * (float32_t) g_motion.left_direction;
        float32_t right_target =
            target_speed * (float32_t) g_motion.right_direction;
        float32_t left_error  = left_target - g_state.left_speed_mm_s;
        float32_t right_error = right_target - g_state.right_speed_mm_s;
        float32_t left_derivative =
            (left_error - g_motion.left_previous_error) / dt_s;
        float32_t right_derivative =
            (right_error - g_motion.right_previous_error) / dt_s;
        float32_t left_output  = 0.0f;
        float32_t right_output = 0.0f;

        g_motion.left_previous_error  = left_error;
        g_motion.right_previous_error = right_error;
        g_motion.left_integral =
            buddy2_clampf(g_motion.left_integral + (left_error * dt_s),
                          BUDDY2_PID_INTEGRAL_LIMIT);
        g_motion.right_integral =
            buddy2_clampf(g_motion.right_integral + (right_error * dt_s),
                          BUDDY2_PID_INTEGRAL_LIMIT);

        left_output = ((float32_t) g_motion.left_direction *
                       (float32_t) g_motion.max_pwm * BUDDY2_FEEDFORWARD_GAIN) +
                      (BUDDY2_KP * left_error) +
                      (BUDDY2_KI * g_motion.left_integral) +
                      (BUDDY2_KD * left_derivative);
        right_output =
            ((float32_t) g_motion.right_direction *
             (float32_t) g_motion.max_pwm * BUDDY2_FEEDFORWARD_GAIN) +
            (BUDDY2_KP * right_error) + (BUDDY2_KI * g_motion.right_integral) +
            (BUDDY2_KD * right_derivative);

        buddy2_set_motor_pwm_locked(buddy2_clamp_pwm(left_output),
                                    buddy2_clamp_pwm(right_output));
        buddy2_trace(buddy2_clamp_pwm(left_output),
                     buddy2_clamp_pwm(right_output), NULL);
    }
}

/*** end of file ***/

