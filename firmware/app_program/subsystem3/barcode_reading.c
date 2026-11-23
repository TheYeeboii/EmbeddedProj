#include "../pin_config.h"
#include "barcode_reading.h"
#include "../gpio_irq.h"
#include "../robot_debug.h"
#include "../robot_hal.h"
#include <tm/tmonitor.h>

#define CHARACTER_WIDTHS 9u
#define LOST_QUEUE_FULL 1u
#define LOST_PAIRED_EDGE 2u
#define EDGE_QUEUE_SIZE 32u
#define EDGE_PENDING    1u
#define RESET_CAPTURE   2u
#define RESET_RESULT    4u
#define EDGE_TIMEOUT_MS 1500u
#define SYMBOL_MASK     0x7fu
#define RESULT_REVERSED 0x80u
#define RESULT_SHIFT    8u

typedef struct
{
    uint32_t timestamp_us;
    bool black;
} barcode_edge_t;

typedef enum
{
    FIND_START,
    READ_LETTER,
    READ_STOP
} barcode_stage_t;

typedef struct
{
    uint32_t widths[CHARACTER_WIDTHS];
#if BUDDY3_DEBUG
    uint32_t frame_widths[3][CHARACTER_WIDTHS];
    bool frame_ready;
#endif
    uint32_t previous_edge_us;
    uint8_t width_count;
    barcode_stage_t stage;
    bool capturing;
    bool reversed;
    char letter;
} barcode_capture_t;

static ID g_edge_flag;
static barcode_edge_t g_edges[EDGE_QUEUE_SIZE];
static uint8_t g_read_index;
static uint8_t g_write_index;
static uint8_t g_edge_loss;
static uint32_t g_enabled = 1u;
/* The reader writes this word. Controller and telemetry may read either core. */
static uint32_t g_latest;

static void barcode_pin_changed (uint32_t pin, uint32_t events)
{
    const uint32_t timestamp_us = hal_time_us32();
    const uint8_t next = (g_write_index + 1u) % EDGE_QUEUE_SIZE;
    (void) pin;

    if (__atomic_load_n(&g_enabled, __ATOMIC_RELAXED) == 0u)
    {
        return;
    }
    if (events != HAL_GPIO_EDGE_RISE && events != HAL_GPIO_EDGE_FALL)
    {
        g_edge_loss |= LOST_PAIRED_EDGE;
    }
    else if (next == g_read_index)
    {
        g_edge_loss |= LOST_QUEUE_FULL;
    }
    else
    {
        const bool level_high = events == HAL_GPIO_EDGE_RISE;
        g_edges[g_write_index] = (barcode_edge_t) {
            timestamp_us, level_high == (BARCODE_DIGITAL_BLACK_LEVEL != 0u)
        };
        g_write_index = next;
    }
    (void) tk_set_flg(g_edge_flag, EDGE_PENDING);
}

ER barcode_reading_init (void)
{
    const T_CFLG settings = {0};
    g_edge_flag = tk_cre_flg(&settings);
    if (g_edge_flag < E_OK)
    {
        return g_edge_flag;
    }

    const ER result = gpio_irq_register(BARCODE_DIGITAL_PIN,
        HAL_GPIO_EDGE_RISE | HAL_GPIO_EDGE_FALL, barcode_pin_changed);
    if (result != E_OK)
    {
        (void) tk_del_flg(g_edge_flag);
        g_edge_flag = 0;
    }
    return result;
}

void barcode_reading_set_enabled (bool enabled)
{
    __atomic_store_n(&g_enabled, enabled ? 1u : 0u, __ATOMIC_RELAXED);
    if (g_edge_flag > 0)
    {
        (void) tk_set_flg(g_edge_flag, RESET_CAPTURE);
    }
}

void barcode_reading_reset (void)
{
    __atomic_store_n(&g_enabled, 1u, __ATOMIC_RELAXED);
    if (g_edge_flag > 0)
    {
        (void) tk_set_flg(g_edge_flag, RESET_RESULT);
    }
}

bool barcode_reading_latest (barcode_reading_result_t * p_result)
{
    const uint32_t latest = __atomic_load_n(&g_latest, __ATOMIC_ACQUIRE);
    if (p_result == NULL || (latest & SYMBOL_MASK) == 0u)
    {
        return false;
    }
    *p_result = (barcode_reading_result_t) {
        .sequence = latest >> RESULT_SHIFT,
        .symbol = (char) (latest & SYMBOL_MASK),
        .reversed = (latest & RESULT_REVERSED) != 0u,
    };
    return true;
}

static bool take_edge (barcode_edge_t * p_edge, uint8_t * p_lost)
{
    UINT interrupt_state;
    DI(interrupt_state);
    /* The GPIO handler and consumer both run on core 0. */
    *p_lost = g_edge_loss;
    if (*p_lost)
    {
        g_read_index = g_write_index;
        g_edge_loss = 0u;
    }
    const bool available = g_read_index != g_write_index;
    if (available)
    {
        *p_edge = g_edges[g_read_index];
        g_read_index = (g_read_index + 1u) % EDGE_QUEUE_SIZE;
    }
    EI(interrupt_state);
    return available;
}

static bool classify_widths(const uint32_t widths_us[9], char pattern[10])
{
  bool widths_valid = true;

  for (unsigned int i = 0; i < 9u; ++i) {
    // Mark zero widths invalid and keep building the diagnostic pattern.
    if (widths_us[i] == 0u) {
      widths_valid = false;
    }
    pattern[i] = '0';
  }
  pattern[9] = '\0';

  for (unsigned int colour = 0; colour < 2u; ++colour) {
    const unsigned int wide_count = colour == 0u ? 2u : 1u;
    uint32_t smallest_wide = 0;

    // Mark the widest elements of this colour in their original positions.
    for (unsigned int pick = 0; pick < wide_count; ++pick) {
      unsigned int widest = 9u;
      for (unsigned int i = colour; i < 9u; i += 2u) {
        if (pattern[i] == '0' &&
            (widest == 9u || widths_us[i] > widths_us[widest])) {
          widest = i;
        }
      }
      pattern[widest] = '1';
      smallest_wide = widths_us[widest];
    }

    uint64_t narrow_sum = 0;
    for (unsigned int i = colour; i < 9u; i += 2u) {
      if (pattern[i] == '0') {
        narrow_sum += widths_us[i];
      }
    }

    // Require the smallest wide element to be at least 1.4 times the
    // average of the three narrow elements of this colour.
    if (15u * (uint64_t)smallest_wide < 7u * narrow_sum) {
      widths_valid = false;
    }
  }

  return widths_valid;
}

static char decode_character(const char pattern[10], bool reverse_order)
{
  static const uint16_t letter_patterns[] = {
    0x109u, 0x049u, 0x148u, 0x019u, 0x118u, 0x058u, 0x00du,
    0x10cu, 0x04cu, 0x01cu, 0x103u, 0x043u, 0x142u, 0x013u,
    0x112u, 0x052u, 0x007u, 0x106u, 0x046u, 0x016u, 0x181u,
    0x0c1u, 0x1c0u, 0x091u, 0x190u, 0x0d0u,
  };

  uint16_t bits = 0;
  for (unsigned int i = 0; i < 9u; ++i) {
    const unsigned int index = reverse_order ? 8u - i : i;
    bits = (uint16_t)((bits << 1u) | (pattern[index] == '1'));
  }

  // The opening and closing markers use the same pattern.
  if (bits == 0x094u) {
    return '*';
  }

  for (unsigned int i = 0; i < sizeof(letter_patterns) / sizeof(letter_patterns[0]); ++i) {
    if (bits == letter_patterns[i]) {
      return (char)('A' + i);
    }
  }

  return '\0';
}

static void finish_character (barcode_capture_t * p_capture)
{
    char pattern[10];
    const bool valid = classify_widths(p_capture->widths, pattern);
    const bool finding_start = p_capture->stage == FIND_START;
    char symbol = valid ? decode_character(
        pattern, !finding_start && p_capture->reversed) : '\0';

    if (finding_start)
    {
        const bool reversed = valid && symbol != '*' &&
                              decode_character(pattern, true) == '*';
        if (symbol != '*' && !reversed)
        {
            /* Try the next black bar without throwing away a partial marker. */
            for (uint32_t i = 2u; i < CHARACTER_WIDTHS; ++i)
            {
                p_capture->widths[i - 2u] = p_capture->widths[i];
            }
            p_capture->width_count = CHARACTER_WIDTHS - 2u;
            return;
        }
        symbol = '*';
        p_capture->reversed = reversed;
    }

#if BUDDY3_DEBUG
    if (finding_start ||
        (p_capture->stage == READ_LETTER && symbol >= 'A' && symbol <= 'Z') ||
        (p_capture->stage == READ_STOP && symbol == '*'))
    {
        for (uint8_t i = 0u; i < CHARACTER_WIDTHS; ++i)
        {
            p_capture->frame_widths[p_capture->stage][i] = p_capture->widths[i];
        }
        p_capture->frame_ready = p_capture->stage == READ_STOP;
    }
#endif
    p_capture->capturing = false;
    if (finding_start)
    {
        p_capture->stage = READ_LETTER;
    }
    else if (p_capture->stage == READ_LETTER && symbol >= 'A' && symbol <= 'Z')
    {
        p_capture->letter = symbol;
        p_capture->stage = READ_STOP;
    }
    else if (p_capture->stage == READ_STOP && symbol == '*')
    {
        p_capture->stage = FIND_START;
        // Publish only a letter enclosed by two markers of the same orientation.
        if (__atomic_load_n(&g_enabled, __ATOMIC_RELAXED) != 0u)
        {
            const uint32_t previous = __atomic_load_n(&g_latest, __ATOMIC_RELAXED);
            const uint32_t next = (previous & ~0xffu) + (1u << RESULT_SHIFT);
            __atomic_store_n(&g_latest, next | (uint8_t) p_capture->letter |
                (p_capture->reversed ? RESULT_REVERSED : 0u), __ATOMIC_RELEASE);
        }
    }
    else
    {
        p_capture->stage = FIND_START;
    }
}

static void capture_edge (barcode_capture_t * p_capture, const barcode_edge_t * p_edge)
{
    if (!p_capture->capturing)
    {
        /* Wait through the white gap before each character. */
        if (p_edge->black)
        {
            p_capture->capturing = true;
            p_capture->width_count = 0u;
            p_capture->previous_edge_us = p_edge->timestamp_us;
        }
        return;
    }

    const bool expected_black = (p_capture->width_count % 2u) != 0u;
    if (p_edge->black != expected_black)
    {
        p_capture->capturing = false;
        p_capture->stage = FIND_START;
        return;
    }

    p_capture->widths[p_capture->width_count++] =
        p_edge->timestamp_us - p_capture->previous_edge_us;
    p_capture->previous_edge_us = p_edge->timestamp_us;
    if (p_capture->width_count == CHARACTER_WIDTHS)
    {
        finish_character(p_capture);
    }
}

void barcode_reading_task (INT start_code, void * p_context)
{
    barcode_capture_t capture = {0};
    (void) start_code;
    (void) p_context;

    for (;;)
    {
        UINT flags;
        const TMO timeout = capture.capturing || capture.stage != FIND_START
                            ? (TMO) EDGE_TIMEOUT_MS : TMO_FEVR;
        const ER result = tk_wai_flg(g_edge_flag,
                                    EDGE_PENDING | RESET_CAPTURE | RESET_RESULT,
                                    TWF_ORW | TWF_BITCLR, &flags, timeout);
        if (result == E_TMOUT)
        {
            capture = (barcode_capture_t) {0};
        }
        if (result != E_OK)
        {
            continue;
        }

        // A new operating mode invalidates edges captured under the old mode.
        if ((flags & (RESET_CAPTURE | RESET_RESULT)) != 0u)
        {
            UINT interrupt_state;
            DI(interrupt_state);
            g_read_index = g_write_index;
            g_edge_loss = 0u;
            EI(interrupt_state);
            capture = (barcode_capture_t) {0};
        }
        if ((flags & RESET_RESULT) != 0u)
        {
            /* Keep the sequence so a repeat of the same letter is a new read. */
            const uint32_t latest = __atomic_load_n(&g_latest, __ATOMIC_RELAXED);
            __atomic_store_n(&g_latest, latest & ~0xffu, __ATOMIC_RELEASE);
        }

        barcode_edge_t edge;
        uint8_t lost;
        while (take_edge(&edge, &lost))
        {
            capture_edge(&capture, &edge);
        }
        if (lost != 0u)
        {
            capture = (barcode_capture_t) {0};
            tm_printf((const UB *) "[barcode] Edge lost. Capture discarded (%s).\n",
                lost == (LOST_PAIRED_EDGE | LOST_QUEUE_FULL) ? "paired GPIO edges and queue full" :
                (lost & LOST_PAIRED_EDGE) != 0u ? "rise + fall pending" : "queue full");
        }
#if BUDDY3_DEBUG
        if (capture.frame_ready && !capture.capturing && capture.stage == FIND_START)
        {
            // tm_printf masks interrupts. Print only after the complete frame is captured.
            capture.frame_ready = false;
            for (uint8_t character = 0u; character < 3u; ++character)
            {
                const char symbol = character == 1u ? capture.letter : '*';
                tm_printf((const UB *) "[barcode] character=%c\n", symbol);
                for (uint8_t i = 0u; i < CHARACTER_WIDTHS; ++i)
                {
                    tm_printf((const UB *) "[barcode] width[%u] %s=%u us\n",
                              (UINT) i, (i % 2u == 0u) ? "black" : "white",
                              (UINT) capture.frame_widths[character][i]);
                }
            }
        }
#endif
    }
}
