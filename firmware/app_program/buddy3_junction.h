#ifndef BUDDY3_JUNCTION_H
#define BUDDY3_JUNCTION_H
#include <stdbool.h>
#include <stdint.h>

/* Initial track settings: qualify with the physical junction width. */
#define B3_JUNCTION_CONFIRM_MS 30u
#define B3_JUNCTION_MIN_MM 25.0f
#define B3_JUNCTION_CLEAR_MM 5.0f
#define B3_JUNCTION_MAX_MM 100.0f
#define B3_JUNCTION_MAX_MS 2500u
#define B3_JUNCTION_MAX_HEADING_ERROR_MM 20.0f
#define B3_JUNCTION_HEADING_PWM_PER_MM 120.0f
enum { B3J_IDLE, B3J_CONFIRM, B3J_CROSS, B3J_CLEAR, B3J_STOP };
typedef struct {
    uint8_t phase;
    uint32_t started_ms;
    float start_left, start_right, clear_start;
    bool clear_pending;
} buddy3_junction_t;
typedef struct {
    int16_t left, right;
    uint8_t phase;
    bool handled, stopped;
} buddy3_junction_output_t;
void buddy3_junction_reset(buddy3_junction_t *s);
bool buddy3_junction_active(const buddy3_junction_t *s);
/* Candidate both-black => straight now, commit after debounce.
 * While committed, ignore branch edges, hold entry heading using encoders,
 * then release only after minimum travel and a distance-qualified white exit.
 * No claim is made that two sensors distinguish a cross from a T-junction. */
buddy3_junction_output_t buddy3_junction_step(buddy3_junction_t *s,
    uint8_t side, uint32_t now, float left_mm, float right_mm,
    bool encoders_healthy, bool entry_allowed, int16_t base);
#endif
