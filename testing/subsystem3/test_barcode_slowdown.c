#include <assert.h>
#include "barcode_slowdown.h"

int main (void)
{
    barcode_slowdown_t state = {0};
    assert(!barcode_slowdown_update(&state, false, 0));
    assert(barcode_slowdown_update(&state, true, 1));
    assert(barcode_slowdown_update(&state, false, 2));
    assert(barcode_slowdown_update(&state, false, 3002));
    assert(!barcode_slowdown_update(&state, false, 3003));
    assert(barcode_slowdown_update(&state, true, 4000));
    assert(barcode_slowdown_update(&state, false, 4001));
    assert(barcode_slowdown_update(&state, true, 7000));
    assert(barcode_slowdown_update(&state, false, 7001));
    assert(barcode_slowdown_update(&state, false, 10001));
    assert(!barcode_slowdown_update(&state, false, 10002));
    assert(barcode_slowdown_update(&state, true, UINT32_MAX - 50));
    assert(barcode_slowdown_update(&state, false, UINT32_MAX - 40));
    assert(barcode_slowdown_update(&state, false, 2959));
    assert(!barcode_slowdown_update(&state, false, 2960));
    return 0;
}
