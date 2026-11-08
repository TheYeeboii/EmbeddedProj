/** @file buddy3_junction.h
 *
 * @brief Buddy 3 cross-junction handling: go straight across a both-black
 *        junction while holding the entry heading with the encoders.
 */
#ifndef BUDDY3_JUNCTION_H
#define BUDDY3_JUNCTION_H

#include <stdbool.h>
#include <stdint.h>
#include "robot_types.h"

/* Initial track settings: qualify with the physical junction width. */
#define B3_JUNCTION_CONFIRM_MS           30u
#define B3_JUNCTION_MIN_MM               25.0f
#define B3_JUNCTION_CLEAR_MM             5.0f
#define B3_JUNCTION_MAX_MM               100.0f
#define B3_JUNCTION_MAX_MS               2500u
#define B3_JUNCTION_MAX_HEADING_ERROR_MM 20.0f
#define B3_JUNCTION_HEADING_PWM_PER_MM   120.0f

/** Junction phase reported in buddy3_junction_output_t.phase. */
typedef enum
{
    B3J_IDLE,
    B3J_CONFIRM,
    B3J_CROSS,
    B3J_CLEAR,
    B3J_STOP
} buddy3_junction_phase_t;

/** Junction memory kept between calls to buddy3_junction_step(). */
typedef struct
{
    uint8_t   phase;
    uint32_t  started_ms;
    float32_t start_left;
    float32_t start_right;
    float32_t clear_start;
    bool      clear_pending;
} buddy3_junction_t;

/** Wheel commands and status produced by one junction step. */
typedef struct
{
    int16_t left;
    int16_t right;
    uint8_t phase;
    bool    handled;
    bool    stopped;
} buddy3_junction_output_t;

/**
 * @brief Clear the junction memory, including a latched stop.
 * @param p_state Junction state to clear.
 */
void buddy3_junction_reset (buddy3_junction_t * p_state);

/**
 * @brief Tell whether a junction is being confirmed or crossed.
 * @param p_state Junction state.
 * @return True while the junction owns the wheels.
 */
bool buddy3_junction_active (const buddy3_junction_t * p_state);

/**
 * @brief Run one junction decision.  A candidate both-black goes straight
 *        now and commits after a debounce.  While committed, branch edges are
 *        ignored, the entry heading is held using the encoders, and the
 *        junction is released only after minimum travel and a
 *        distance-qualified white exit.  No claim is made that two sensors
 *        distinguish a cross from a T-junction.
 * @param p_state          Junction memory, updated in place.
 * @param side             Sensor pattern: 0 both white ... 3 both black.
 * @param now              Current time in milliseconds.
 * @param left_mm          Left wheel odometry in millimetres.
 * @param right_mm         Right wheel odometry in millimetres.
 * @param encoders_healthy False when an encoder fault is latched.
 * @param entry_allowed    False to ignore new junctions.
 * @param base             Base wheel command.
 * @return Wheel commands; handled is false when the junction is idle.
 */
buddy3_junction_output_t
buddy3_junction_step (buddy3_junction_t * p_state, uint8_t side, uint32_t now,
                      float32_t left_mm, float32_t right_mm,
                      bool encoders_healthy, bool entry_allowed, int16_t base);

#endif /* BUDDY3_JUNCTION_H */

/*** end of file ***/

