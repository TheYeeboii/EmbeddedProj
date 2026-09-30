#include "buddy3_ir.h"

#include <math.h>
#include <string.h>

#include "hardware/adc.h"

#include "buddy2_motion.h"
#include "pin_config.h"

#define BUDDY3_SAMPLE_PERIOD_MS 10u
#define BUDDY3_LINE_THRESHOLD 350u
#define BUDDY3_JUNCTION_THRESHOLD 800u
#define BUDDY3_LINE_POSITION_LIMIT 1000
#define BUDDY3_BARCODE_START 0xA5u
#define BUDDY3_BARCODE_LEFT 1u
#define BUDDY3_BARCODE_RIGHT 2u
#define BUDDY3_BARCODE_STRAIGHT 3u

static buddy3_calibration_t g_calibration = {
    {0u, 0u, 0u},
    {4095u, 4095u, 4095u},
    false,
    18.0f,
    5.0f,
    12000
};
static buddy3_ir_state_t g_state;
static bool g_line_following = false;
static uint32_t g_last_update_ms = 0u;
static int16_t g_previous_position = 0;
static uint32_t g_line_lost_since_ms = 0u;
static uint8_t g_barcode_shift = 0u;
static uint8_t g_barcode_bits = 0u;
static bool g_barcode_frame = false;
static uint8_t g_pending_barcode = 0u;
static bool g_barcode_ready = false;

static uint16_t buddy3_read_channel(uint channel)
{
    adc_select_input(channel);
    (void)adc_read();
    return adc_read();
}

static uint16_t buddy3_normalize(uint16_t raw, uint16_t minimum, uint16_t maximum, bool line_is_dark)
{
    if (maximum <= minimum)
    {
        return 0u;
    }
    int32_t value = ((int32_t)raw - minimum) * 1000 / (maximum - minimum);
    if (value < 0) value = 0;
    if (value > 1000) value = 1000;
    if (line_is_dark) value = 1000 - value;
    return (uint16_t)value;
}

static void buddy3_decode_barcode(uint16_t center_value)
{
    uint8_t bit = center_value >= BUDDY3_LINE_THRESHOLD ? 1u : 0u;
    g_barcode_shift = (uint8_t)((g_barcode_shift << 1u) | bit);
    if (!g_barcode_frame)
    {
        if (g_barcode_shift == BUDDY3_BARCODE_START)
        {
            g_barcode_frame = true;
            g_barcode_bits = 0u;
            g_barcode_shift = 0u;
        }
        return;
    }

    g_barcode_bits++;
    if (g_barcode_bits == 8u)
    {
        if (g_barcode_shift == BUDDY3_BARCODE_LEFT ||
            g_barcode_shift == BUDDY3_BARCODE_RIGHT ||
            g_barcode_shift == BUDDY3_BARCODE_STRAIGHT)
        {
            g_pending_barcode = g_barcode_shift;
            g_barcode_ready = true;
        }
        g_barcode_frame = false;
        g_barcode_bits = 0u;
        g_barcode_shift = 0u;
    }
}

void buddy3_init(void)
{
    adc_init();
    adc_gpio_init(IR_SENSOR_LEFT_PIN);
    adc_gpio_init(IR_SENSOR_CENTER_PIN);
    adc_gpio_init(IR_SENSOR_RIGHT_PIN);
    memset(&g_state, 0, sizeof(g_state));
    g_last_update_ms = 0u;
    g_line_following = false;
}

void buddy3_set_calibration(const buddy3_calibration_t *calibration)
{
    if (calibration == NULL)
    {
        return;
    }
    g_calibration = *calibration;
}

void buddy3_get_calibration(buddy3_calibration_t *calibration)
{
    if (calibration != NULL)
    {
        *calibration = g_calibration;
    }
}

void buddy3_update(uint32_t now_ms)
{
    if ((now_ms - g_last_update_ms) < BUDDY3_SAMPLE_PERIOD_MS)
    {
        return;
    }
    g_last_update_ms = now_ms;

    g_state.raw[0] = buddy3_read_channel(IR_ADC_CHANNEL_LEFT);
    g_state.raw[1] = buddy3_read_channel(IR_ADC_CHANNEL_CENTER);
    g_state.raw[2] = buddy3_read_channel(IR_ADC_CHANNEL_RIGHT);
    for (size_t index = 0u; index < 3u; index++)
    {
        g_state.normalized[index] = buddy3_normalize(g_state.raw[index],
                                                     g_calibration.minimum[index],
                                                     g_calibration.maximum[index],
                                                     g_calibration.line_is_dark);
    }

    int16_t previous_position = g_previous_position;
    uint32_t weighted_total = (uint32_t)g_state.normalized[0] +
                              (uint32_t)g_state.normalized[1];
    g_state.line_seen = weighted_total >= BUDDY3_LINE_THRESHOLD;
    g_state.junction_seen = g_state.normalized[0] >= BUDDY3_JUNCTION_THRESHOLD &&
                            g_state.normalized[1] >= BUDDY3_JUNCTION_THRESHOLD;
    if (g_state.line_seen)
    {
        int32_t weighted_position = ((int32_t)g_state.normalized[1] - (int32_t)g_state.normalized[0]) * 1000;
        g_state.line_position = (int16_t)(weighted_position / (int32_t)weighted_total);
        g_previous_position = g_state.line_position;
        g_line_lost_since_ms = 0u;
    }
    else
    {
        g_state.line_position = 0;
        if (g_line_lost_since_ms == 0u)
        {
            g_line_lost_since_ms = now_ms;
        }
    }

    buddy3_decode_barcode(g_state.normalized[2]);
    g_state.barcode_ready = g_barcode_ready;
    g_state.barcode_command = g_pending_barcode;

    if (!g_line_following)
    {
        return;
    }
    if (!g_state.line_seen && (now_ms - g_line_lost_since_ms) > 250u)
    {
        buddy2_stop();
        return;
    }

    float derivative = (float)g_state.line_position - (float)previous_position;
    float correction = g_calibration.proportional_gain * (float)g_state.line_position +
                       g_calibration.derivative_gain * derivative;
    int16_t left_pwm = (int16_t)fmaxf(-32767.0f, fminf(32767.0f, (float)g_calibration.base_pwm + correction));
    int16_t right_pwm = (int16_t)fmaxf(-32767.0f, fminf(32767.0f, (float)g_calibration.base_pwm - correction));
    buddy2_set_motor_pwm(left_pwm, right_pwm);
}

void buddy3_start_line_follow(void)
{
    g_line_following = true;
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

void buddy3_get_state(buddy3_ir_state_t *state)
{
    if (state != NULL)
    {
        *state = g_state;
    }
}

bool buddy3_take_barcode(uint8_t *command)
{
    if (!g_barcode_ready || command == NULL)
    {
        return false;
    }
    *command = g_pending_barcode;
    g_barcode_ready = false;
    g_state.barcode_ready = false;
    return true;
}
