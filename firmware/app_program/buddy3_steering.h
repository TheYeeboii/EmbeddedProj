/** @file buddy3_steering.h
 *
 * @brief Buddy 3 line-steering state machine: micro correction, corners,
 *        recovery and the both-black stop.
 */
#ifndef BUDDY3_STEERING_H
#define BUDDY3_STEERING_H

#include <stdint.h>
#include <stdbool.h>

/* Experimental settings: validate on the car, then tune one at a time. */
#define B3_CORRECTION_MS   300u
#define B3_RECOVERY_MAX_MS 1200u
#define B3_ALIGN_WHITE_MS  100u
#define B3_BOTH_BLACK_MS   150u

/** Steering phase reported in buddy3_steering_output_t.phase. */
typedef enum
{
    B3_TRACK,
    B3_MICRO,
    B3_CORNER,
    B3_SEEK,
    B3_ALIGN,
    B3_AMBIGUOUS,
    B3_STOP
} buddy3_steering_phase_t;

/** Steering memory kept between calls to buddy3_steering_step(). */
typedef struct
{
    uint8_t  phase;
    uint8_t  side;
    uint8_t  direction;
    uint32_t side_since;
    uint32_t recovery_since;
    uint32_t white_since;
    uint32_t black_since;
    bool     corner_active;
    bool     recovering;
    bool     touched;
    bool     white_pending;
    bool     black_pending;
    bool     stopped;
} buddy3_steering_t;

/** Wheel commands and phase produced by one steering step. */
typedef struct
{
    int16_t left;
    int16_t right;
    uint8_t phase;
} buddy3_steering_output_t;

/**
 * @brief Clear all steering memory, including a latched stop.
 * @param p_state Steering state to clear.
 */
void buddy3_steering_reset (buddy3_steering_t * p_state);

/**
 * @brief Run one steering decision.
 * @param p_state    Steering memory, updated in place.
 * @param stage      Enabled behaviour: 1 micro only, 2 adds corners,
 *                   3 adds recovery after a lost corner.
 * @param side       Sensor pattern: 0 both white, 1 left black,
 *                   2 right black, 3 both black.
 * @param now        Current time in milliseconds.
 * @param base       Base wheel command.
 * @param correction Wheel differential, clamped to 0..base.
 * @return Wheel commands and the phase that produced them.
 */
buddy3_steering_output_t buddy3_steering_step (buddy3_steering_t * p_state,
                                               uint8_t stage, uint8_t side,
                                               uint32_t now, int16_t base,
                                               int16_t correction);

#endif /* BUDDY3_STEERING_H */

/*** end of file ***/

