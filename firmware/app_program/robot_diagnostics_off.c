/** @file robot_diagnostics_off.c
 *
 * @brief Buddy 1: empty stand-ins for the diagnostic streams.
 *
 * Built only with DIAGNOSTICS=0 (the default; see pico_rp2040.mk), in place
 * of the eight modules behind the dashboard's tuning views:
 *   robot_action_log.c   / robot_action_mqtt.c    (rover/actions)
 *   robot_steering_trace.c / robot_steering_mqtt.c (rover/steering)
 *   robot_barcode_trace.c / robot_barcode_mqtt.c   (rover/barcode)
 *   robot_ir_stream.c    / robot_ir_mqtt.c         (rover/ir)
 *
 * Those modules only record what the robot did and send it as JSON; no
 * control code reads anything back from them.  So the motor, line and
 * barcode code keeps calling the same functions, and here they do nothing.
 * The demo build then sends only the packed binary messages (Change
 * Requirement 1) and saves the trace buffers' RAM.  Build with
 * DIAGNOSTICS=1 to get the streams back for tuning.
 *
 * This file implements the eight modules' headers instead of one of its own
 * (a documented deviation from BARR-C 4.2.a, docs/CODING_STANDARD.md).
 */
#include <stdbool.h>
#include <stdint.h>

#include "robot_action_log.h"
#include "robot_action_mqtt.h"
#include "robot_barcode_mqtt.h"
#include "robot_barcode_trace.h"
#include "robot_ir_mqtt.h"
#include "robot_ir_stream.h"
#include "robot_steering_mqtt.h"
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

/* ---- rover/barcode ---- */

int32_t robot_barcode_scaled (float32_t value, float32_t scale)
{
    /* Only used to fill trace records, which are discarded here. */
    (void) value;
    (void) scale;

    return 0;
}

void robot_barcode_trace_configure (const int32_t config[BARCODE_CONFIG_COUNT])
{
    (void) config;
}

void robot_barcode_trace_enable (bool enabled)
{
    (void) enabled;
}

void robot_barcode_trace_cancel (uint32_t now)
{
    (void) now;
}

void robot_barcode_trace_observe (
    const buddy3_barcode_decoder_t *    p_decoder,
    const robot_barcode_observation_t * p_observation, bool was_active,
    float32_t unfinished_before, const buddy3_barcode_event_t * p_result)
{
    (void) p_decoder;
    (void) p_observation;
    (void) was_active;
    (void) unfinished_before;
    (void) p_result;
}

bool robot_barcode_packet_peek (robot_barcode_packet_t * p_packet, uint32_t now)
{
    (void) p_packet;
    (void) now;

    return false;
}

void robot_barcode_packet_ack (const robot_barcode_packet_t * p_packet)
{
    (void) p_packet;
}

void robot_barcode_publish (uint32_t now_ms)
{
    (void) now_ms;
}

/* ---- rover/ir ---- */

void robot_ir_stream_record (const robot_barcode_observation_t * p_observation,
                             uint16_t minimum, uint16_t maximum, bool inverted)
{
    (void) p_observation;
    (void) minimum;
    (void) maximum;
    (void) inverted;
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

