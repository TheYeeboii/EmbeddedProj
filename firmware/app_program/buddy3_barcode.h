#ifndef BUDDY3_BARCODE_H
#define BUDDY3_BARCODE_H
#include <stdbool.h>
#include <stdint.h>
#include "code39.h"
#ifndef BUDDY3_BARCODE_MIN_ELEMENT_MM
#define BUDDY3_BARCODE_MIN_ELEMENT_MM 3.0f /* Decode by default; 0 disables. */
#endif
#ifndef BUDDY3_BARCODE_OFFSET_LEFT_MM
#define BUDDY3_BARCODE_OFFSET_LEFT_MM 0.0f /* Left positive; right negative. */
#endif
typedef struct {
    float min_element_mm; /* Resolution bound, NOT the actual printed width. */
} buddy3_barcode_config_t;
typedef enum {
    B3BC_NONE, B3BC_VALID, B3BC_FORMAT, B3BC_INTERRUPTED,
    B3BC_REVERSE, B3BC_SAMPLE_GAP, B3BC_STALL, B3BC_TIMEOUT
} buddy3_barcode_event_kind_t;
typedef struct {
    buddy3_barcode_event_kind_t kind;
    code39_status_t format_status;
    uint32_t ts_ms;
    uint32_t dropped; /* Adapter's cumulative count of unread events dropped. */
    code39_result_t decoded;
} buddy3_barcode_event_t;
typedef struct {
    float position, edge_position, white_distance, leading_quiet, max_sample_gap;
    float widths[CODE39_FRAME_ELEMENTS];
    uint32_t edge_ms[CODE39_FRAME_ELEMENTS];
    uint32_t last_progress_ms, started_ms;
    uint8_t count;
    bool active, previous_black, have_sample;
} buddy3_barcode_decoder_t;
void buddy3_barcode_reset(buddy3_barcode_decoder_t *s);
/* Forward vehicle travel only; reversed PRINT orientation is supported.
 * No allocation, I/O, or motor control. Emits at most one event per call.
 * Widths and edge timestamps remain available until reset/next frame. */
bool buddy3_barcode_step(buddy3_barcode_decoder_t *s,
    const buddy3_barcode_config_t *cfg, bool black, float travel_mm,
    uint32_t now, bool permitted, buddy3_barcode_event_t *event);
float buddy3_barcode_travel(float left_mm, float right_mm,
                           float track_mm, float offset_left_mm);
const char *buddy3_barcode_event_name(buddy3_barcode_event_kind_t kind);
#endif
