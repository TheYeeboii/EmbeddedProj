#ifndef BUDDY3_IR_H
#define BUDDY3_IR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t minimum[3];
    uint16_t maximum[3];
    bool line_is_dark;
    float proportional_gain;
    float derivative_gain;
    int16_t base_pwm;
} buddy3_calibration_t;

typedef struct
{
    uint16_t raw[3];
    uint16_t normalized[3];
    int16_t line_position;
    bool line_seen;
    bool junction_seen;
    bool barcode_ready;
    uint8_t barcode_command;
} buddy3_ir_state_t;

void buddy3_init(void);
void buddy3_set_calibration(const buddy3_calibration_t *calibration);
void buddy3_get_calibration(buddy3_calibration_t *calibration);
void buddy3_update(uint32_t now_ms);
void buddy3_start_line_follow(void);
void buddy3_stop_line_follow(void);
bool buddy3_line_following(void);
void buddy3_get_state(buddy3_ir_state_t *state);
bool buddy3_take_barcode(uint8_t *command);

#endif
