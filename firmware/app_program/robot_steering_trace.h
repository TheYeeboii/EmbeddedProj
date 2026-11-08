#ifndef ROBOT_STEERING_TRACE_H
#define ROBOT_STEERING_TRACE_H
#include <stdbool.h>
#include <stdint.h>

#define ROBOT_STEERING_QUEUE_SIZE 64u
enum
{
    TRACE_STOPPED,
    TRACE_MANUAL,
    TRACE_TRACKING,
    TRACE_JUNCTION,
    TRACE_SEARCH,
    TRACE_AVOIDANCE,
    TRACE_MOTION
};
enum
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
};
typedef struct robot_steering_input
{
    uint32_t ts_ms;
    uint16_t raw[2], norm[2], threshold, calibration[4];
    uint8_t  dark, phase;
} robot_steering_input_t;
typedef struct
{
    uint32_t               seq, ts_ms, dropped, encoder_ms;
    int32_t                distance_x10[2];
    int16_t                requested[2], applied[2], speed_x10[2];
    robot_steering_input_t input;
    uint8_t                owner, direction, faults;
    bool                   has_input;
} robot_steering_event_t;

/* Seed once before tasks start; session is a diagnostic identifier, not a key.
 */
void     robot_diagnostics_init (uint32_t session);
uint32_t robot_diagnostics_session (void);
void     robot_steering_owner (uint8_t owner);
/* Motor mutex held by caller. Fixed-size copy/queue only, no I/O. */
void robot_steering_record (const robot_steering_event_t * event);
bool robot_steering_peek (robot_steering_event_t * event);
void robot_steering_ack (uint32_t seq);
bool robot_steering_snapshot (robot_steering_event_t * event);
void robot_steering_publish (uint32_t now_ms);
#endif

/*** end of file ***/

