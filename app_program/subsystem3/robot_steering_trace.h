/** @file robot_steering_trace.h
 *
 * @brief Queue of steering decisions (wheel commands plus the IR evidence
 *        behind them) for the dashboard steering timeline.
 */
#ifndef ROBOT_STEERING_TRACE_H
#define ROBOT_STEERING_TRACE_H

#include <stdbool.h>
#include <stdint.h>

#define ROBOT_STEERING_QUEUE_SIZE 64u

/** Who owns the wheels (robot_steering_event_t.owner). */
typedef enum
{
    TRACE_STOPPED,
    TRACE_MANUAL,
    TRACE_TRACKING,
    TRACE_JUNCTION,
    TRACE_SEARCH,
    TRACE_AVOIDANCE,
    TRACE_MOTION
} robot_steering_owner_t;

/** Line-following phase (robot_steering_input_t.phase). */
typedef enum
{
    TRACE_STRAIGHT,
    TRACE_MICRO,
    TRACE_CORNER,
    TRACE_SEEK,
    TRACE_ALIGN,
    TRACE_AMBIGUOUS,
    TRACE_STOP,
    TRACE_CROSS,
    TRACE_LEGACY,
    TRACE_DARK
} robot_steering_phase_t;

/** IR evidence behind one line-following decision. */
typedef struct robot_steering_input
{
    uint32_t ts_ms;
    uint16_t raw[2];
    uint16_t norm[2];
    uint16_t threshold;
    uint16_t calibration[4];
    uint8_t  dark;
    uint8_t  phase;
} robot_steering_input_t;

/** One accepted wheel command with its context. */
typedef struct
{
    uint32_t               seq;
    uint32_t               ts_ms;
    uint32_t               dropped;
    uint32_t               encoder_ms;
    int32_t                distance_x10[2];
    int16_t                requested[2];
    int16_t                applied[2];
    int16_t                speed_x10[2];
    robot_steering_input_t input;
    uint8_t                owner;
    uint8_t                direction;
    uint8_t                faults;
    bool                   has_input;
} robot_steering_event_t;

/**
 * @brief Seed the diagnostics session once, before tasks start.  The
 *        session is a diagnostic identifier, not a key.
 * @param session Boot-unique identifier sent with every diagnostic packet.
 */
void robot_diagnostics_init (uint32_t session);

/**
 * @brief Read the diagnostics session identifier.
 * @return Value given to robot_diagnostics_init().
 */
uint32_t robot_diagnostics_session (void);

/**
 * @brief Set the owner reported for commands without IR input.
 * @param owner One of robot_steering_owner_t; larger values are ignored.
 */
void robot_steering_owner (uint8_t owner);

/**
 * @brief Record an accepted wheel command.  The motor mutex is held by the
 *        caller.  Fixed-size copy and queue only, no I/O.
 * @param p_event Command and evidence; seq, dropped, owner and direction are
 *                filled in here.
 */
void robot_steering_record (const robot_steering_event_t * p_event);

/**
 * @brief Copy the oldest queued event without removing it.
 * @param p_event Receives the event.
 * @return True when an event was copied.
 */
bool robot_steering_peek (robot_steering_event_t * p_event);

/**
 * @brief Remove the oldest event if it is still the one that was published.
 * @param seq Sequence number of the published event.
 */
void robot_steering_ack (uint32_t seq);

/**
 * @brief Copy the latest event when the queue is empty.
 * @param p_event Receives the event.
 * @return True when the queue was empty and an event exists.
 */
bool robot_steering_snapshot (robot_steering_event_t * p_event);

#endif /* ROBOT_STEERING_TRACE_H */

/*** end of file ***/

