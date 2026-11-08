/** @file robot_action_log.h
 *
 * @brief Queue of steering-direction changes (turn left/right and their
 *        stops) taken at the accepted motor-command boundary.
 */
#ifndef ROBOT_ACTION_LOG_H
#define ROBOT_ACTION_LOG_H

#include <stdbool.h>
#include <stdint.h>

#define ROBOT_ACTION_QUEUE_SIZE    32u
#define ROBOT_ACTION_PWM_THRESHOLD 256

/** Steering intent derived from the wheel commands. */
typedef enum
{
    ROBOT_ACTION_NONE,
    ROBOT_ACTION_LEFT,
    ROBOT_ACTION_RIGHT
} robot_action_direction_t;

/** Event kind stored in robot_action_event_t.action. */
typedef enum
{
    ROBOT_ACTION_TURN_LEFT,
    ROBOT_ACTION_TURN_RIGHT,
    ROBOT_ACTION_STOP_LEFT,
    ROBOT_ACTION_STOP_RIGHT
} robot_action_kind_t;

/** Which motor path produced the command. */
typedef enum
{
    ROBOT_ACTION_PWM,
    ROBOT_ACTION_MOTION,
    ROBOT_ACTION_STOP
} robot_action_source_t;

/** One queued event, or the current state for a snapshot. */
typedef struct
{
    uint32_t seq;
    uint32_t ts_ms;
    uint32_t dropped;
    uint8_t  action;
    uint8_t  direction;
    uint8_t  source;
    int16_t  left_pwm;
    int16_t  right_pwm;
} robot_action_event_t;

/**
 * @brief Classify a wheel command as straight, left or right.
 * @param left  Left wheel command.
 * @param right Right wheel command.
 * @return ROBOT_ACTION_LEFT/RIGHT when the difference exceeds
 *         ROBOT_ACTION_PWM_THRESHOLD, otherwise ROBOT_ACTION_NONE.
 */
uint8_t robot_action_direction_for_pwm (int16_t left, int16_t right);

/**
 * @brief Record an accepted motor command.  No formatting or network calls.
 *        Direction describes intent, not measured motion.  Repeated states do
 *        not enqueue events.  Switching L->R emits stop-left, then
 *        start-right.
 * @param direction ROBOT_ACTION_NONE, LEFT or RIGHT.
 * @param source    ROBOT_ACTION_PWM, MOTION or STOP.
 * @param left      Left wheel command.
 * @param right     Right wheel command.
 * @param ts_ms     Time of the command in milliseconds.
 */
void robot_action_record (uint8_t direction, uint8_t source, int16_t left,
                          int16_t right, uint32_t ts_ms);

/**
 * @brief Copy the oldest queued event without removing it.
 * @param p_event Receives the event.
 * @return True when an event was copied.
 */
bool robot_action_peek (robot_action_event_t * p_event);

/**
 * @brief Remove the oldest event if it is still the one that was published.
 * @param seq Sequence number of the published event.
 */
void robot_action_ack (uint32_t seq);

/**
 * @brief Copy the current steering state.
 * @param p_state Receives the state; seq is the latest sequence number.
 */
void robot_action_snapshot (robot_action_event_t * p_state);

/**
 * @brief Copy the current state only when no events are queued.
 * @param p_state Receives the state.
 * @return True when the queue was empty and p_state was filled.
 */
bool robot_action_snapshot_if_empty (robot_action_event_t * p_state);

#endif /* ROBOT_ACTION_LOG_H */

/*** end of file ***/

