#ifndef BARCODE_TUNING_H
#define BARCODE_TUNING_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t low;
    uint16_t high;
    uint16_t raw;
    bool enabled;
    bool inverted;
    bool black;
} barcode_tuning_state_t;

/* Sensor IDs match dashboard numbering: barcode 0, front left 1, right 2. */
#define IR_TUNING_SENSOR_COUNT (3u)
bool ir_tuning_request (uint8_t sensor, uint16_t low, uint16_t high,
                         bool inverted);
bool ir_tuning_sample (uint8_t sensor, uint16_t raw, bool legacy_black);
void ir_tuning_get (uint8_t sensor, barcode_tuning_state_t * state);

/* Requests are applied by the IR task on its next sample. */
bool barcode_tuning_request (uint16_t low, uint16_t high, bool inverted);
bool barcode_tuning_sample (uint16_t raw, bool legacy_black);
void barcode_tuning_get (barcode_tuning_state_t * state);
void barcode_tuning_publish (uint32_t now);

#endif
