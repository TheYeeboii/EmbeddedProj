#ifndef BUDDY3_BARCODE_H
#define BUDDY3_BARCODE_H
#include <stdbool.h>
#include <stdint.h>

#ifndef BUDDY3_BARCODE_CELL_MM
#define BUDDY3_BARCODE_CELL_MM 0.0f /* Measure a cell before enabling. */
#endif
#ifndef BUDDY3_BARCODE_OFFSET_LEFT_MM
#define BUDDY3_BARCODE_OFFSET_LEFT_MM 0.0f /* Left positive; right negative. */
#endif

typedef struct {
    float cell_mm;
} buddy3_barcode_config_t;
typedef struct {
    float position, next_sample, white_distance;
    uint32_t last_progress_ms;
    uint16_t frame;
    uint8_t bits;
    bool active, armed, previous_black, have_sample;
} buddy3_barcode_decoder_t;
void buddy3_barcode_reset(buddy3_barcode_decoder_t *s);
/* One observation, with signed sensor travel since the previous observation.
 * Returns 1/2/3 once per valid frame; zero otherwise. Requires a two-cell
 * white lead-in and a fixed-width 0xA5 + command frame, MSB first.
 * Aborts on reverse, excessive sample gaps, disabled input or stalled frame. */
uint8_t buddy3_barcode_step(buddy3_barcode_decoder_t *s,
    const buddy3_barcode_config_t *cfg, bool black, float travel_mm,
    uint32_t now, bool permitted);
/* Differential-drive local forward travel at a lateral sensor offset. */
float buddy3_barcode_travel(float left_mm, float right_mm,
                           float track_mm, float offset_left_mm);
#endif
