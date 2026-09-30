/**
 * @file    buddy3_ir.c
 * @brief   Buddy 3: IR line following and barcode decoding.
 *
 * Ported from the team's Pico SDK (CMake) version.  The algorithm is
 * unchanged; what changed:
 *   - ADC access goes through robot_hal.c.
 *   - State shared with other tasks is copied under a short interrupt
 *     lock (DI/EI) so readers never see a half-updated struct.
 *   - No libc/libm: small local helpers instead of memset/fmaxf/fminf.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy2_motion.h"
#include "buddy3_ir.h"
#include "pin_config.h"
#include "robot_hal.h"

#define BUDDY3_LINE_THRESHOLD       (350u)
#define BUDDY3_JUNCTION_THRESHOLD   (800u)
#define BUDDY3_NORMALIZED_MAX       (1000)
#define BUDDY3_LINE_LOST_STOP_MS    (250u)
#define BUDDY3_BARCODE_START        (0xA5u)
#define BUDDY3_BARCODE_BITS         (8u)
#define BUDDY3_SENSOR_COUNT         (3u)

/* ---- Sensor layout ---------------------------------------------------
 * The two FRONT sensors (ADC0, ADC1) sit on the black bracket; the third
 * (ADC2) is on the white arm beside the car and reads the barcodes.
 * Steering compares the two front sensors (position = right - left).  The
 * same rule works whether the line is wider than the sensor gap (ours:
 * both on black = centred) or narrower (both on white = centred).
 *   BUDDY3_LINE_BETWEEN_SENSORS 1 : this "compare the two" algorithm
 *   BUDDY3_LINE_BETWEEN_SENSORS 0 : the old CMake algorithm
 * If the car steers AWAY from the line instead of back onto it, swap the
 * two numbers below (sensor wires or motor wires are the other way round).
 */
#define BUDDY3_LINE_BETWEEN_SENSORS (1)
/* Confirmed 30 Sep with a hand test: ADC0 = left sensor, ADC1 = center. */
#define BUDDY3_FRONT_LEFT           (0u)   /* ADC index of the left-front sensor  */
#define BUDDY3_FRONT_RIGHT          (1u)   /* ADC index of the right-front sensor */
/* All three sensors on black for this long = the car was lifted or put on
 * a dark surface (the side sensor is normally on white): stop. */
#define BUDDY3_ALL_BLACK_STOP_MS    (3000u)   /* > time to drive off the black start box */
/* Our line is 18 mm, narrower than the gap between the front sensors, so
 * "both on floor" is NORMAL (line between them) - it only means "lost" if
 * it lasts long.  If neither front sensor touches the line for this long,
 * stop. */
#define BUDDY3_NO_TOUCH_STOP_MS     (6000u)
#define BUDDY3_EDGE_THRESHOLD       (500u) /* normalized: sensor is over black */
/* Steering tested on the car (30 Sep): black under the CENTER sensor ("R")
 * -> left wheel faster (turn right); under the LEFT sensor -> right wheel
 * faster.  Both correct, so no swap is needed.
 * If one sensor stays on black this long, the turn is too weak (curve):
 * run the inner wheel slightly backwards for a tighter turn. */
#define BUDDY3_HARD_TURN_MS         (600u)
#define BUDDY3_HARD_TURN_INNER_PWM  (-5000)
/* Both front sensors white for this long after a touch = the line has
 * probably slipped outside: spin on the spot towards the side that saw
 * it last until a sensor finds it again (the 6 s stop still applies). */
#define BUDDY3_SEARCH_AFTER_MS      (700u)

/* Units: 32767 = 100 % motor duty.  Base 12000 is about 37 % duty - on the
 * car, much less than ~35 % does not overcome friction (the old CMake value
 * was only ~18 %).  Steering gains: a full-scale line offset (1000) gives a
 * 9000 difference between the wheels. */
static buddy3_calibration_t g_calibration =
{
    { 0u, 0u, 0u },
    { 4095u, 4095u, 4095u },
    false,
    9.0f,
    2.5f,
    12000
};

static buddy3_ir_state_t g_state;
static volatile bool     g_line_following     = false;
static uint32_t          g_last_update_ms     = 0u;
static int16_t           g_previous_position  = 0;
static uint32_t          g_line_lost_since_ms = 0u;
static uint8_t           g_barcode_shift      = 0u;
static uint8_t           g_barcode_bits       = 0u;
static bool              g_barcode_frame      = false;
static uint8_t           g_pending_barcode    = 0u;
static bool              g_barcode_ready      = false;
static bool              g_reported_lost      = false;
static bool              g_seen_since_start   = false;
static bool              g_start_pending      = false;
static uint32_t          g_last_touch_ms      = 0u;
static uint8_t           g_last_touch_side    = 0u;   /* 0 none, 1 L, 2 R, 3 both */
static uint32_t          g_all_black_since_ms = 0u;
static bool              g_reported_black     = false;
static uint32_t          g_side_since_ms      = 0u;   /* when the current L/R pattern began */
static uint8_t           g_last_single_side   = 0u;   /* 1 L, 2 R: last side seen alone */
static bool              g_reported_search    = false;

static int16_t buddy3_clamp_pwm(float value)
{
    if (value > (float)BUDDY2_PWM_MAX)
    {
        return (int16_t)BUDDY2_PWM_MAX;
    }
    if (value < -(float)BUDDY2_PWM_MAX)
    {
        return (int16_t)(-BUDDY2_PWM_MAX);
    }
    return (int16_t)value;
}

static uint16_t buddy3_normalize(uint16_t raw, uint16_t minimum,
                                 uint16_t maximum, bool line_is_dark)
{
    int32_t value;

    if (maximum <= minimum)
    {
        return 0u;
    }
    value = (((int32_t)raw - (int32_t)minimum) * BUDDY3_NORMALIZED_MAX) /
            ((int32_t)maximum - (int32_t)minimum);
    if (value < 0)
    {
        value = 0;
    }
    if (value > BUDDY3_NORMALIZED_MAX)
    {
        value = BUDDY3_NORMALIZED_MAX;
    }
    if (line_is_dark)
    {
        value = BUDDY3_NORMALIZED_MAX - value;
    }
    return (uint16_t)value;
}

/* Called every sample with the barcode sensor value.  Looks for the start
 * byte 0xA5, then collects the next 8 bits as the command. */
static void buddy3_decode_barcode(uint16_t sensor_value)
{
    uint8_t bit = (sensor_value >= BUDDY3_LINE_THRESHOLD) ? 1u : 0u;

    g_barcode_shift = (uint8_t)((uint8_t)(g_barcode_shift << 1u) | bit);
    if (!g_barcode_frame)
    {
        if (BUDDY3_BARCODE_START == g_barcode_shift)
        {
            g_barcode_frame = true;
            g_barcode_bits  = 0u;
            g_barcode_shift = 0u;
        }
        return;
    }

    g_barcode_bits++;
    if (BUDDY3_BARCODE_BITS == g_barcode_bits)
    {
        if ((BUDDY3_BARCODE_LEFT == g_barcode_shift) ||
            (BUDDY3_BARCODE_RIGHT == g_barcode_shift) ||
            (BUDDY3_BARCODE_STRAIGHT == g_barcode_shift))
        {
            g_pending_barcode = g_barcode_shift;
            g_barcode_ready   = true;
        }
        g_barcode_frame = false;
        g_barcode_bits  = 0u;
        g_barcode_shift = 0u;
    }
}

void buddy3_init(void)
{
    buddy3_ir_state_t empty = { { 0u, 0u, 0u }, { 0u, 0u, 0u }, 0, false, false, false, 0u };

    hal_adc_init();
    hal_adc_pin_init(IR_SENSOR_LEFT_PIN);
    hal_adc_pin_init(IR_SENSOR_CENTER_PIN);
    hal_adc_pin_init(IR_SENSOR_RIGHT_PIN);

    g_state            = empty;
    g_last_update_ms   = 0u;
    g_line_following   = false;
}

void buddy3_set_calibration(const buddy3_calibration_t *p_calibration)
{
    UINT interrupt_state;

    if (NULL == p_calibration)
    {
        return;
    }
    DI(interrupt_state);
    g_calibration = *p_calibration;
    EI(interrupt_state);
}

void buddy3_get_calibration(buddy3_calibration_t *p_calibration)
{
    UINT interrupt_state;

    if (NULL != p_calibration)
    {
        DI(interrupt_state);
        *p_calibration = g_calibration;
        EI(interrupt_state);
    }
}

void buddy3_update(uint32_t now_ms)
{
    buddy3_ir_state_t next;
    uint16_t          raw[BUDDY3_SENSOR_COUNT];
    uint32_t          index;
#if !BUDDY3_LINE_BETWEEN_SENSORS
    uint32_t          weighted_total;
#endif
    int16_t           previous_position;
    UINT              interrupt_state;

    if ((now_ms - g_last_update_ms) < BUDDY3_SAMPLE_PERIOD_MS)
    {
        return;
    }
    g_last_update_ms = now_ms;

    /* Read the sensors first (slow part, no lock held). */
    raw[0] = hal_adc_read(IR_ADC_CHANNEL_LEFT);
    raw[1] = hal_adc_read(IR_ADC_CHANNEL_CENTER);
    raw[2] = hal_adc_read(IR_ADC_CHANNEL_RIGHT);

    next = g_state;
    for (index = 0u; index < BUDDY3_SENSOR_COUNT; index++)
    {
        next.raw[index]        = raw[index];
        next.normalized[index] = buddy3_normalize(raw[index],
                                                  g_calibration.minimum[index],
                                                  g_calibration.maximum[index],
                                                  g_calibration.line_is_dark);
    }

    previous_position = g_previous_position;

#if BUDDY3_LINE_BETWEEN_SENSORS
    /* Front sensors STRADDLE the line (see the photo in docs/): normally both
     * see the floor.  When one of them touches the line, steer towards that
     * side so the line goes back between them.  position = right - left:
     * > 0 -> line under the right-front sensor -> turn right. */
    {
        int32_t left  = (int32_t)next.normalized[BUDDY3_FRONT_LEFT];
        int32_t right = (int32_t)next.normalized[BUDDY3_FRONT_RIGHT];

        next.line_seen     = (left >= BUDDY3_EDGE_THRESHOLD) || (right >= BUDDY3_EDGE_THRESHOLD);
        next.junction_seen = (left >= BUDDY3_JUNCTION_THRESHOLD) &&
                             (right >= BUDDY3_JUNCTION_THRESHOLD);
        /* Both black = a crossing line (or the car was lifted): go straight. */
        next.line_position = next.junction_seen ? 0 : (int16_t)(right - left);
        g_previous_position = next.line_position;
    }
#else
    /* Original algorithm: the line runs UNDER the two front sensors. */
    weighted_total     = (uint32_t)next.normalized[0] + (uint32_t)next.normalized[1];
    next.line_seen     = (weighted_total >= BUDDY3_LINE_THRESHOLD);
    next.junction_seen = (next.normalized[0] >= BUDDY3_JUNCTION_THRESHOLD) &&
                         (next.normalized[1] >= BUDDY3_JUNCTION_THRESHOLD);
    if (next.line_seen)
    {
        int32_t weighted_position = ((int32_t)next.normalized[1] -
                                     (int32_t)next.normalized[0]) * 1000;

        next.line_position   = (int16_t)(weighted_position / (int32_t)weighted_total);
        g_previous_position  = next.line_position;
        g_line_lost_since_ms = 0u;
    }
    else
    {
        next.line_position = 0;
        if (0u == g_line_lost_since_ms)
        {
            g_line_lost_since_ms = now_ms;
        }
    }
#endif

    /* The outrigger sensor (ADC2) reads the barcodes beside the track. */
    buddy3_decode_barcode(next.normalized[2]);
    next.barcode_ready   = g_barcode_ready;
    next.barcode_command = g_pending_barcode;

    DI(interrupt_state);
    g_state = next;
    EI(interrupt_state);

#if BUDDY3_LINE_BETWEEN_SENSORS
    {
        /* Which front sensor(s) are on black right now: 1 left, 2 right. */
        uint8_t side = (uint8_t)
            (((next.normalized[BUDDY3_FRONT_LEFT]  >= BUDDY3_EDGE_THRESHOLD) ? 1u : 0u) |
             ((next.normalized[BUDDY3_FRONT_RIGHT] >= BUDDY3_EDGE_THRESHOLD) ? 2u : 0u));

        /* Sensor check while idle: say which side sees the line, so the
         * wiring can be verified by hand (see docs). */
        if (side != g_last_touch_side)
        {
            static const char *const color[] = { "white", "BLACK" };
            static const char *const steer[] = { "straight", "steer LEFT", "steer RIGHT", "straight (crossing / start box)" };
            const char *p_left  = color[side & 1u];
            const char *p_right = color[(side >> 1) & 1u];

            if (g_line_following)
            {
                /* Run log: every change, with what the car is told to do.
                 * Compare with what it really did to check the wiring. */
                tm_printf((UB *)"[line] %u.%02u s   front L=%s R=%s   -> %s\n",
                          (UINT)(now_ms / 1000u), (UINT)((now_ms % 1000u) / 10u),
                          p_left, p_right, steer[side]);
            }
            else
            {
                tm_printf((UB *)"[sensor] front L=%s R=%s\n", p_left, p_right);
            }
        }
        if (side != g_last_touch_side)
        {
            g_side_since_ms = now_ms;
        }
        g_last_touch_side = side;
        if ((1u == side) || (2u == side))
        {
            g_last_single_side = side;
        }

        if (g_start_pending)
        {
            g_last_touch_ms = now_ms;   /* start the "no line" timer now */
            g_start_pending = false;
        }
        if (0u != side)
        {
            g_last_touch_ms = now_ms;
            if (g_reported_lost && g_line_following)
            {
                tm_printf((UB *)"[line] line found - driving\n");
            }
            g_reported_lost = false;
        }
    }
#endif

    if (!g_line_following)
    {
        return;
    }

#if BUDDY3_LINE_BETWEEN_SENSORS
    /* Lifted / dark surface: every sensor reads "black". */
    if ((next.normalized[0] >= BUDDY3_EDGE_THRESHOLD) &&
        (next.normalized[1] >= BUDDY3_EDGE_THRESHOLD) &&
        (next.normalized[2] >= BUDDY3_EDGE_THRESHOLD))
    {
        if (0u == g_all_black_since_ms)
        {
            g_all_black_since_ms = now_ms;
        }
        if ((now_ms - g_all_black_since_ms) > BUDDY3_ALL_BLACK_STOP_MS)
        {
            buddy2_stop();
            if (!g_reported_black)
            {
                tm_printf((UB *)"[line] all sensors black - lifted or dark surface? stopped\n");
                g_reported_black = true;
            }
            return;
        }
    }
    else
    {
        g_all_black_since_ms = 0u;
        g_reported_black     = false;
    }

    if ((now_ms - g_last_touch_ms) > BUDDY3_NO_TOUCH_STOP_MS)
    {
        buddy2_stop();
        if (!g_reported_lost)
        {
            tm_printf((UB *)"[line] no line seen for %u s - stopped\n",
                      (UINT)(BUDDY3_NO_TOUCH_STOP_MS / 1000u));
            g_reported_lost = true;
        }
        return;
    }
#else
    /* Line must be seen at least once before driving, then may be lost for
     * up to BUDDY3_LINE_LOST_STOP_MS. */
    if (!next.line_seen && (!g_seen_since_start ||
        ((now_ms - g_line_lost_since_ms) > BUDDY3_LINE_LOST_STOP_MS)))
    {
        buddy2_stop();
        if (!g_reported_lost)
        {
            tm_printf((UB *)"[line] no line under the sensors (L=%u C=%u raw) - waiting\n",
                      (UINT)next.raw[0], (UINT)next.raw[1]);
            g_reported_lost = true;
        }
        return;
    }
    g_seen_since_start = true;
    if (g_reported_lost)
    {
        tm_printf((UB *)"[line] line found - driving\n");
        g_reported_lost = false;
    }
#endif

#if BUDDY3_LINE_BETWEEN_SENSORS
    /* Digital sensors -> simple three-way steering (the derivative term is
     * not used here: the position only jumps between -1000, 0 and +1000).
     *   one sensor black  : turn towards it (harder if it stays black)
     *   both white, recent: straight (line is between the sensors)
     *   both white, long  : spin towards the side that saw it last
     *   both black        : straight (crossing / start box) */
    (void)previous_position;
    {
        int32_t  base       = (int32_t)g_calibration.base_pwm;
        int32_t  turn       = (int32_t)(g_calibration.proportional_gain * 1000.0f);
        int32_t  left_pwm   = base;
        int32_t  right_pwm  = base;
        uint8_t  side       = g_last_touch_side;
        uint32_t held_ms    = now_ms - g_side_since_ms;

        if ((1u == side) || (2u == side))
        {
            int32_t outer = base + turn;
            int32_t inner = (held_ms > BUDDY3_HARD_TURN_MS) ? BUDDY3_HARD_TURN_INNER_PWM
                                                            : (base - turn);

            if (2u == side)            /* line under the center sensor: turn right */
            {
                left_pwm  = outer;
                right_pwm = inner;
            }
            else                       /* line under the left sensor: turn left */
            {
                left_pwm  = inner;
                right_pwm = outer;
            }
            g_reported_search = false;
        }
        else if ((0u == side) && (0u != g_last_single_side) &&
                 ((now_ms - g_last_touch_ms) > BUDDY3_SEARCH_AFTER_MS))
        {
            if (2u == g_last_single_side)
            {
                left_pwm  = base;
                right_pwm = -base;
            }
            else
            {
                left_pwm  = -base;
                right_pwm = base;
            }
            if (!g_reported_search)
            {
                tm_printf((UB *)"[line] line lost - searching %s\n",
                          (2u == g_last_single_side) ? "RIGHT" : "LEFT");
                g_reported_search = true;
            }
        }
        else
        {
            /* straight */
        }
        buddy2_set_motor_pwm(buddy3_clamp_pwm((float)left_pwm),
                             buddy3_clamp_pwm((float)right_pwm));
    }
#else
    {
        float derivative = (float)next.line_position - (float)previous_position;
        float correction = (g_calibration.proportional_gain * (float)next.line_position) +
                           (g_calibration.derivative_gain * derivative);

        buddy2_set_motor_pwm(buddy3_clamp_pwm((float)g_calibration.base_pwm + correction),
                             buddy3_clamp_pwm((float)g_calibration.base_pwm - correction));
    }
#endif
}

void buddy3_start_line_follow(void)
{
    g_reported_lost      = false;
    g_seen_since_start   = false;
    g_start_pending      = true;
    g_line_lost_since_ms = 0u;
    g_last_single_side   = 0u;
    g_reported_search    = false;
    g_side_since_ms      = g_last_update_ms;
    g_line_following     = true;
}

void buddy3_stop_line_follow(void)
{
    g_line_following = false;
    buddy2_stop();
}

bool buddy3_line_following(void)
{
    return g_line_following;
}

void buddy3_get_state(buddy3_ir_state_t *p_state)
{
    UINT interrupt_state;

    if (NULL != p_state)
    {
        DI(interrupt_state);
        *p_state = g_state;
        EI(interrupt_state);
    }
}

void buddy3_get_colors(bool *p_left_black, bool *p_right_black, bool *p_side_black)
{
    buddy3_ir_state_t state;

    buddy3_get_state(&state);
    if ((NULL != p_left_black) && (NULL != p_right_black) && (NULL != p_side_black))
    {
        *p_left_black  = (state.normalized[BUDDY3_FRONT_LEFT]  >= BUDDY3_EDGE_THRESHOLD);
        *p_right_black = (state.normalized[BUDDY3_FRONT_RIGHT] >= BUDDY3_EDGE_THRESHOLD);
        *p_side_black  = (state.normalized[2] >= BUDDY3_EDGE_THRESHOLD);
    }
}

bool buddy3_take_barcode(uint8_t *p_command)
{
    UINT interrupt_state;
    bool taken = false;

    if (NULL == p_command)
    {
        return false;
    }
    DI(interrupt_state);
    if (g_barcode_ready)
    {
        *p_command            = g_pending_barcode;
        g_barcode_ready       = false;
        g_state.barcode_ready = false;
        taken                 = true;
    }
    EI(interrupt_state);

    return taken;
}
