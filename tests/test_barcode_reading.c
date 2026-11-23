#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

#include "barcode_reading.h"
#include "code39.h"
#include "gpio_irq.h"
#include "pin_config.h"
#include <tm/tmonitor.h>

static gpio_irq_cb_t g_callback;
static uint64_t g_now_us = UINT32_MAX - 3000u;
static UINT g_pending;
static unsigned int g_batch;
static unsigned int g_lost;
static unsigned int g_widths;
static char g_decoded[5];
static size_t g_decoded_count;
static uint32_t g_last_sequence;
static jmp_buf g_finished;

ID tk_cre_flg(const T_CFLG *settings)
{
    assert(settings->iflgptn == 0u);
    return 1;
}

ER tk_del_flg(ID id)
{
    assert(id == 1);
    return E_OK;
}

int32_t gpio_irq_register(uint32_t pin, uint32_t events, gpio_irq_cb_t callback)
{
    assert(pin == BARCODE_DIGITAL_PIN);
    assert(events == (HAL_GPIO_EDGE_RISE | HAL_GPIO_EDGE_FALL));
    g_callback = callback;
    return E_OK;
}

uint64_t hal_time_us(void)
{
    return g_now_us;
}

ER tk_set_flg(ID id, UINT pattern)
{
    assert(id == 1);
    g_pending |= pattern;
    return E_OK;
}

static void edge(bool black, uint32_t duration_us)
{
    g_now_us += duration_us;
    const bool high = black == (BARCODE_DIGITAL_BLACK_LEVEL != 0u);
    g_callback(BARCODE_DIGITAL_PIN, high ? HAL_GPIO_EDGE_RISE : HAL_GPIO_EDGE_FALL);
}

static void character(uint16_t pattern, bool reversed)
{
    edge(true, 1000u);
    for (uint32_t i = 0u; i < CODE39_CHARACTER_ELEMENTS; ++i) {
        const uint32_t bit = reversed ? i : CODE39_CHARACTER_ELEMENTS - 1u - i;
        const uint32_t width_us = (pattern & (1u << bit)) ? 2200u : 1000u;
        edge((i & 1u) != 0u, width_us + 20u * i);
    }
}

static void frame(uint16_t pattern, bool reversed)
{
    character(0x094u, reversed);
    character(pattern, reversed);
    character(0x094u, reversed);
}

ER tk_wai_flg(ID id, UINT requested, UINT mode, UINT *received, int timeout)
{
    assert(id == 1 && mode == (TWF_ORW | TWF_BITCLR));
    assert(timeout == TMO_FEVR || timeout == 1500);
    barcode_reading_result_t result;
    if (barcode_reading_latest(&result) && result.sequence != g_last_sequence) {
        assert(g_decoded_count < sizeof(g_decoded) - 1u);
        assert(result.sequence == g_last_sequence + 1u);
        assert(result.reversed == (result.symbol == 'P'));
        g_last_sequence = result.sequence;
        g_decoded[g_decoded_count++] = result.symbol;
    }
    if (g_pending == 0u) {
        switch (g_batch++) {
        case 0:
            // A stray bar before the first valid frame must not shift alignment.
            edge(true, 1000u);
            edge(false, 1000u);
            break;
        case 1:
            frame(0x109u, false); // A
            break;
        case 2:
            frame(0x052u, true); // P and reversed '*' have overlapping patterns.
            break;
        case 3:
            character(0x094u, false);
            character(0x109u, false); // No stop marker, so no decoded result.
            break;
        case 4:
            assert(timeout == 1500 && g_decoded_count == 2u);
            return E_TMOUT;
        case 5:
            assert(timeout == TMO_FEVR);
            // Overflow the 31 usable slots before the task can drain them.
            for (uint32_t i = 0u; i < 32u; ++i) {
                edge((i & 1u) == 0u, 1000u);
            }
            break;
        case 6:
            g_callback(BARCODE_DIGITAL_PIN, HAL_GPIO_EDGE_RISE | HAL_GPIO_EDGE_FALL);
            break;
        case 7:
            frame(0x058u, false); // F after loss
            break;
        case 8:
            barcode_reading_set_enabled(false);
            frame(0x109u, false);
            break;
        case 9:
            barcode_reading_set_enabled(true);
            break;
        case 10:
            assert(barcode_reading_latest(&result) && result.symbol == 'F');
            character(0x094u, false);
            character(0x058u, false);
            break;
        case 11:
            assert(timeout == 1500);
            barcode_reading_reset();
            break;
        case 12:
            assert(timeout == TMO_FEVR && !barcode_reading_latest(&result));
            frame(0x058u, false); // The same letter can be read again after reset.
            break;
        default:
            longjmp(g_finished, 1);
        }
    }
    *received = g_pending & requested;
    assert(*received != 0u);
    g_pending &= ~*received;
    return E_OK;
}

int tm_printf(const UB *format, ...)
{
    if (strcmp((const char *) format,
               "[barcode] Edge lost. Capture discarded.\n") == 0) {
        ++g_lost;
    } else if (strcmp((const char *) format,
                      "[barcode] width[%u] %s=%u us\n") == 0) {
        ++g_widths;
    }
    return 0;
}

int main(void)
{
    assert(barcode_reading_init() == E_OK);
    if (setjmp(g_finished) == 0) {
        barcode_reading_task(0, NULL);
    }
    assert(strcmp(g_decoded, "APFF") == 0);
    assert(g_lost == 2u);
#if BUDDY3_DEBUG
    assert(g_widths == 16u * CODE39_CHARACTER_ELEMENTS);
#else
    assert(g_widths == 0u);
#endif
    puts("PASS: GPIO barcode frames in both directions, timeout, edge loss, disable and rescan after reset");
    return 0;
}
