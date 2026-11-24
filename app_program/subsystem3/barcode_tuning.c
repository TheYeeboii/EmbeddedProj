#include <tk/tkernel.h>
#include "barcode_tuning.h"
#include "pin_config.h"

static barcode_tuning_state_t g_applied[IR_TUNING_SENSOR_COUNT];
static barcode_tuning_state_t g_requested[IR_TUNING_SENSOR_COUNT];
static bool g_pending[IR_TUNING_SENSOR_COUNT];

bool ir_tuning_request (uint8_t sensor, uint16_t low, uint16_t high, bool inverted)
{
    UINT lock;
    if ((sensor >= IR_TUNING_SENSOR_COUNT) || (high > 4095u) ||
        ((0u != high) && (low >= high)) ||
        ((0u == high) && (0u != low)))
    {
        return false;
    }
    /* The barcode sensor (0) is a digital input: it has no thresholds. */
    if ((sensor == 0u) && (high != 0u))
    {
        return false;
    }
    DI(lock);
    g_requested[sensor].low = low;
    g_requested[sensor].high = high;
    g_requested[sensor].inverted = inverted;
    g_requested[sensor].enabled = (0u != high);
    g_pending[sensor] = true;
    EI(lock);
    return true;
}

bool ir_tuning_sample (uint8_t sensor, uint16_t raw, bool legacy_black)
{
    UINT lock;
    if (sensor >= IR_TUNING_SENSOR_COUNT)
    {
        return legacy_black;
    }
    DI(lock);
    if (g_pending[sensor])
    {
        g_applied[sensor] = g_requested[sensor];
        /* Initialize within the grey band as white after reconfiguration. */
        g_applied[sensor].black = false;
        g_pending[sensor] = false;
    }
    g_applied[sensor].raw = raw;
    if (sensor == 0u)
    {
        g_applied[sensor].enabled = false;
        g_applied[sensor].black = (raw == BARCODE_DIGITAL_BLACK_LEVEL);
    }
    else if (!g_applied[sensor].enabled)
    {
        g_applied[sensor].black = legacy_black;
    }
    else if (raw <= g_applied[sensor].low)
    {
        g_applied[sensor].black = g_applied[sensor].inverted;
    }
    else if (raw >= g_applied[sensor].high)
    {
        g_applied[sensor].black = !g_applied[sensor].inverted;
    }
    /* Inside the band the previous classified colour is retained. */
    EI(lock);
    return g_applied[sensor].black;
}

void ir_tuning_get (uint8_t sensor, barcode_tuning_state_t * state)
{
    UINT lock;
    if (sensor >= IR_TUNING_SENSOR_COUNT)
    {
        return;
    }
    DI(lock);
    *state = g_applied[sensor];
    EI(lock);
}

/* Compatibility entry points for the existing barcode-only command. */
bool barcode_tuning_request (uint16_t low, uint16_t high, bool inverted)
{
    return ir_tuning_request(0u, low, high, inverted);
}

bool barcode_tuning_sample (uint16_t raw, bool legacy_black)
{
    return ir_tuning_sample(0u, raw, legacy_black);
}

void barcode_tuning_get (barcode_tuning_state_t * state)
{
    ir_tuning_get(0u, state);
}
