/** @file robot_diagnostics_off.c
 *
 * @brief Buddy 1: empty stand-ins for the diagnostic streams.
 *
 * Built only with DIAGNOSTICS=0 (the default; see pico_rp2040.mk), in place
 * of the diagnostic modules behind the dashboard's tuning views:
 *   robot_action_log.c      (rover/actions)
 *   robot_steering_trace.c  (rover/steering)
 *   robot_ir_stream.c       (rover/ir)
 *   robot_diagnostics_mqtt.c (publishes the streams)
 *
 * Those modules only record what the robot did and send it as packed
 * binary; no control code reads anything back from them.  So the motor, line
 * code keeps calling the same functions, and here they do nothing.
 * The demo build then sends only the packed binary messages (Change
 * Requirement 1) and saves the trace buffers' RAM.  Build with
 * DIAGNOSTICS=1 to get the streams back for tuning.
 *
 * This file implements the diagnostic modules' headers instead of one of its own
 * (a documented deviation from BARR-C 4.2.a, docs/CODING_STANDARD.md).
 */
#include <stdbool.h>
#include <stdint.h>

#include "robot_action_log.h"
#include "robot_diagnostics_mqtt.h"
#include "robot_ir_stream.h"
#include "robot_steering_trace.h"

/* ---- rover/actions ---- */

uint8_t robot_action_direction_for_pwm (int16_t left, int16_t right)
{
    (void) left;
    (void) right;

    return ROBOT_ACTION_NONE;
}

void robot_action_record (uint8_t direction, uint8_t source, int16_t left,
                          int16_t right, uint32_t ts_ms)
{
    (void) direction;
    (void) source;
    (void) left;
    (void) right;
    (void) ts_ms;
}

bool robot_action_peek (robot_action_event_t * p_event)
{
    (void) p_event;

    return false;
}

void robot_action_ack (uint32_t seq)
{
    (void) seq;
}

void robot_action_snapshot (robot_action_event_t * p_state)
{
    (void) p_state;
}

bool robot_action_snapshot_if_empty (robot_action_event_t * p_state)
{
    (void) p_state;

    return false;
}

void robot_action_publish (uint32_t now_ms)
{
    (void) now_ms;
}

/* ---- rover/steering ---- */

void robot_diagnostics_init (uint32_t session)
{
    (void) session;
}

uint32_t robot_diagnostics_session (void)
{
    return 0u;
}

void robot_steering_owner (uint8_t owner)
{
    (void) owner;
}

void robot_steering_record (const robot_steering_event_t * p_event)
{
    (void) p_event;
}

bool robot_steering_peek (robot_steering_event_t * p_event)
{
    (void) p_event;

    return false;
}

void robot_steering_ack (uint32_t seq)
{
    (void) seq;
}

bool robot_steering_snapshot (robot_steering_event_t * p_event)
{
    (void) p_event;

    return false;
}

void robot_steering_publish (uint32_t now_ms)
{
    (void) now_ms;
}

/* ---- rover/ir ---- */

void robot_ir_stream_record (const robot_ir_stream_sample_t * p_observation)
{
    (void) p_observation;
}

bool robot_ir_stream_peek (robot_ir_stream_batch_t * p_batch)
{
    (void) p_batch;

    return false;
}

void robot_ir_stream_ack (const robot_ir_stream_batch_t * p_batch)
{
    (void) p_batch;
}

void robot_ir_stream_publish (uint32_t now_ms)
{
    (void) now_ms;
}

/*** end of file ***/

