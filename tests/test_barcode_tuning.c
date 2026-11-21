#include <assert.h>
#include "barcode_tuning.h"
int main(void) {
    barcode_tuning_state_t state;
    assert(!barcode_tuning_sample(100,false));
    assert(barcode_tuning_request(220,260,false));
    barcode_tuning_get(&state);
    assert(!state.enabled);
    assert(!barcode_tuning_sample(240,true));
    assert(barcode_tuning_sample(260,false));
    assert(barcode_tuning_sample(240,false));
    assert(!barcode_tuning_sample(220,true));
    assert(!barcode_tuning_request(300,200,false));
    assert(!barcode_tuning_request(0,4096,false));
    assert(barcode_tuning_request(220,260,true));
    assert(barcode_tuning_sample(220,false));
    assert(barcode_tuning_sample(240,false));
    assert(!barcode_tuning_sample(260,true));
    assert(barcode_tuning_request(0,0,false));
    assert(barcode_tuning_sample(100,true));
    barcode_tuning_get(&state);
    assert(!state.enabled);
    /* Independent front-sensor thresholds, opposite polarities. */
    assert(ir_tuning_request(1, 220, 260, false));
    assert(ir_tuning_request(2, 900, 1000, true));
    assert(ir_tuning_sample(1, 270, false));
    assert(ir_tuning_sample(2, 850, false));
    assert(ir_tuning_sample(1, 240, false));
    assert(!ir_tuning_sample(2, 1050, true));
    ir_tuning_get(1, &state);
    assert(state.low == 220 && state.high == 260 && state.black);
    assert(!ir_tuning_request(3, 200, 300, false));
    assert(ir_tuning_request(1, 0, 0, false));
    assert(!ir_tuning_sample(1, 1000, false));
    ir_tuning_get(2, &state);
    assert(state.enabled && state.inverted && !state.black);
    return 0;
}
