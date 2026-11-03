#ifndef BUDDY3_STEERING_H
#define BUDDY3_STEERING_H
#include <stdint.h>
#include <stdbool.h>

/* Experimental settings: validate on the car, then tune one at a time. */
#define B3_CORRECTION_MS 300u
#define B3_RECOVERY_MAX_MS 1200u
#define B3_ALIGN_WHITE_MS 100u
#define B3_BOTH_BLACK_MS 150u
enum { B3_TRACK, B3_MICRO, B3_CORNER, B3_SEEK, B3_ALIGN, B3_AMBIGUOUS, B3_STOP };
typedef struct {
    uint8_t phase, side, direction;
    uint32_t side_since, recovery_since, white_since;
    uint32_t black_since;
    bool corner_active, recovering, touched, white_pending, black_pending, stopped;
} buddy3_steering_t;
typedef struct { int16_t left, right; uint8_t phase; } buddy3_steering_output_t;
void buddy3_steering_reset(buddy3_steering_t *s);
buddy3_steering_output_t buddy3_steering_step(buddy3_steering_t *s,
    uint8_t stage, uint8_t side, uint32_t now, int16_t base, int16_t correction);
#endif
