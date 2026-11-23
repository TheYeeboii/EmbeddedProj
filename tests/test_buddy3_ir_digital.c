/* Reuse the real IR adapter's existing hardware/motion fixtures. */
#define main analog_fixture_main
#include "test_buddy3_ir_junction.c"
#undef main

static bool digital_pin_initialized;
static barcode_reading_result_t latest;

bool barcode_reading_latest (barcode_reading_result_t *result)
{
    *result = latest;
    return latest.sequence != 0u;
}

void hal_gpio_input_pullup (uint32_t pin)
{
    assert(pin == 28u);
    digital_pin_initialized = true;
}

bool hal_gpio_read (uint32_t pin)
{
    assert(pin == 28u);
    return g_adc[2] != 0u;
}

int main (void)
{
    barcode_tuning_state_t tuning;
    buddy3_ir_state_t state;
    buddy3_barcode_event_t event;
    assert(BARCODE_DIGITAL_INPUT == 1);
    assert(BARCODE_DIGITAL_BLACK_LEVEL == 1u);
    buddy3_init();
    assert(digital_pin_initialized);
    g_adc[0] = 200u;
    g_adc[1] = 200u;
    g_adc[2] = 0u;
    buddy3_update(BUDDY3_SAMPLE_PERIOD_MS);
    buddy3_get_state(&state);
    assert(state.raw[2] == 0u && state.normalized[2] == 0u);
    assert(!state.barcode_ready && !buddy3_take_barcode(&event));
    g_adc[2] = 1u;
    buddy3_update(BUDDY3_SAMPLE_PERIOD_MS + 1u);
    barcode_tuning_get(&tuning);
    assert(tuning.raw == 0u && !tuning.black && !tuning.enabled);
    assert(!barcode_tuning_request(200u, 400u, false));
    assert(!buddy3_set_ir_levels(BUDDY3_LEVELS_BARCODE, 200u, 4000u));
    buddy3_zero_barcode_white();
    assert(ir_tuning_request(1u, 220u, 260u, false));
    g_adc[1] = 270u;
    buddy3_update(2u * BUDDY3_SAMPLE_PERIOD_MS);
    buddy3_get_state(&state);
    assert(state.raw[1] == 270u && state.normalized[1] < 500u);
    assert(state.raw[2] == 1u && state.normalized[2] == 1000u);
    barcode_tuning_get(&tuning);
    assert(tuning.raw == 1u && tuning.black && !tuning.enabled);
    assert(g_state.black[BUDDY3_FRONT_LEFT]);
    ir_tuning_get(1u, &tuning);
    assert(tuning.enabled);
    latest = (barcode_reading_result_t) { .sequence = 1u, .symbol = 'A' };
    assert(buddy3_take_barcode(&event));
    assert(event.kind == B3BC_VALID && event.decoded.symbol == 'A');
    assert(!buddy3_take_barcode(&event));
    buddy3_get_state(&state);
    assert(state.barcode_ready && state.barcode_command == 'A');

    buddy3_set_test_stage(3);
    buddy3_start_line_follow();
    latest = (barcode_reading_result_t) { .sequence = 2u, .symbol = 'C', .reversed = true };
    assert(buddy3_take_barcode(&event) && event.decoded.reversed);

    g_junction.phase = B3J_CROSS;
    g_junction.started_ms = 1000u;
    g_adc[0] = 3000u;
    g_adc[1] = 1300u;
    g_adc[2] = 1u;
    buddy3_update(1000u);
    assert(!g_test_barcode_slowdown);
    assert(g_left_pwm > 0 && g_right_pwm > 0);
    latest = (barcode_reading_result_t) { .sequence = 3u, .symbol = 'F' };
    assert(buddy3_take_barcode(&event) && event.decoded.symbol == 'F');
    buddy3_get_state(&state);
    assert(state.barcode_ready && state.barcode_command == 'F');

    /* A change on GP28 cannot start or clear the all-black stop timer. */
    assert(!buddy3_dark_surface_stop(&g_state, 1000u));
    g_state.black[BUDDY3_SIDE_SENSOR] = false;
    assert(buddy3_dark_surface_stop(&g_state, 4001u));
    g_state.black[BUDDY3_FRONT_LEFT] = false;
    g_state.black[BUDDY3_SIDE_SENSOR] = true;
    assert(!buddy3_dark_surface_stop(&g_state, 4011u));
    puts("PASS: GP28 does not control motion; confirmed scans are delivered even during junctions");
    return 0;
}
