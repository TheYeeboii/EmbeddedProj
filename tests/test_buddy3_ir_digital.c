/* Reuse the real IR adapter's existing hardware/motion fixtures. */
#define main analog_fixture_main
#include "test_buddy3_ir_junction.c"
#undef main

static bool digital_pin_initialized;

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
    assert(BARCODE_DIGITAL_INPUT == 1);
    assert(BARCODE_DIGITAL_BLACK_LEVEL == 1u);
    buddy3_init();
    assert(digital_pin_initialized);
    assert(!g_b_zero_request);
    g_adc[0] = 200u;
    g_adc[1] = 200u;
    g_adc[2] = 0u;
    buddy3_update(BUDDY3_SAMPLE_PERIOD_MS);
    assert(!g_b_barcode_black);
    buddy3_get_state(&state);
    assert(state.raw[2] == 0u && state.normalized[2] == 0u);
    g_adc[2] = 1u;
    buddy3_update(BUDDY3_SAMPLE_PERIOD_MS + 1u); /* Side-only path. */
    assert(g_b_barcode_black);
    barcode_tuning_get(&tuning);
    assert(tuning.raw == 1u && tuning.black && !tuning.enabled);
    assert(!barcode_tuning_request(200u, 400u, false));
    assert(!buddy3_set_ir_levels(BUDDY3_LEVELS_BARCODE, 200u, 4000u));
    buddy3_zero_barcode_white();
    assert(!g_b_zero_request);
    assert(ir_tuning_request(1u, 220u, 260u, false));
    g_adc[1] = 270u;
    buddy3_update(2u * BUDDY3_SAMPLE_PERIOD_MS);
    buddy3_get_state(&state);
    assert(state.raw[1] == 270u && state.normalized[1] < 500u);
    assert(state.raw[2] == 1u && state.normalized[2] == 1000u);
    assert(g_state.black[BUDDY3_FRONT_LEFT]);
    ir_tuning_get(1u, &tuning);
    assert(tuning.enabled);
    assert(scan_barcode(SCAN_MANUAL, BUDDY3_BARCODE_PERIOD_MS, 0.4f) == 1u);
    buddy3_init();
    buddy3_set_test_stage(3);
    buddy3_start_line_follow();
    g_junction.phase = B3J_CROSS;
    g_junction.started_ms = 1000u;
    g_adc[0] = 3000u;
    g_adc[1] = 1300u;
    g_adc[2] = 1u;
    buddy3_update(1000u);
    assert(g_test_barcode_slowdown);
    g_adc[2] = 0u;
    buddy3_update(1001u);
    buddy3_update(4001u);
    assert(g_test_barcode_slowdown); /* Exactly 3 seconds remains slow. */
    buddy3_update(4002u);
    assert(!g_test_barcode_slowdown);
    puts("PASS: barcode slowdown triggers during junctions and releases only after over 3 seconds of white");
    puts("PASS: GP28 HIGH=black, no ADC2 reads, both sample paths, analog front tuning, rejected digital thresholds and full digital Code39 pass");
    return 0;
}
