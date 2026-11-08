#include <tk/tkernel.h>
#include "robot_steering_trace.h"

static robot_steering_event_t queue[ROBOT_STEERING_QUEUE_SIZE], current;
static uint32_t               head, count, sequence, dropped, session_id;
static uint8_t                owner = TRACE_MANUAL;
static bool                   have_current;

void robot_diagnostics_init (uint32_t session)
{
    session_id = session;
}
uint32_t robot_diagnostics_session (void)
{
    return session_id;
}
void robot_steering_owner (uint8_t next)
{
    UINT state;
    if (next > TRACE_MOTION)
    {
        return;
    }
    DI(state);
    owner = next;
    EI(state);
}
void robot_steering_record (const robot_steering_event_t * input)
{
    UINT                   state;
    robot_steering_event_t event = *input;
    uint8_t                side, previous_side;
    int32_t difference = (int32_t) event.requested[1] - event.requested[0];
    event.direction    = difference > 256 ? 1u : difference < -256 ? 2u : 0u;
    DI(state);
    event.owner = event.has_input
                      ? (TRACE_CROSS == event.input.phase
                             ? TRACE_JUNCTION
                             : (TRACE_SEEK == event.input.phase ||
                                        TRACE_ALIGN == event.input.phase
                                    ? TRACE_SEARCH
                                    : TRACE_TRACKING))
                      : owner;
    if (0 == event.requested[0] && 0 == event.requested[1])
    {
        event.owner = TRACE_STOPPED;
    }
    side =
        event.has_input
            ? (uint8_t) ((event.input.norm[0] >= event.input.threshold) |
                         ((event.input.norm[1] >= event.input.threshold) << 1))
            : 0;
    previous_side =
        current.has_input
            ? (uint8_t) ((current.input.norm[0] >= current.input.threshold) |
                         ((current.input.norm[1] >= current.input.threshold)
                          << 1))
            : 0;
    /* Save transitions at the command boundary, plus 10 Hz steady evidence.
     * Never coalesce a pulse or sensor/phase/owner transition. */
    if (have_current && event.direction == current.direction &&
        event.owner == current.owner && event.has_input == current.has_input &&
        event.input.phase == current.input.phase && side == previous_side &&
        event.faults == current.faults && (event.ts_ms - current.ts_ms) < 100u)
    {
        EI(state);
        return;
    }
    if (ROBOT_STEERING_QUEUE_SIZE == count)
    {
        head = (head + 1u) % ROBOT_STEERING_QUEUE_SIZE;
        --count;
        ++dropped;
    }
    event.seq                                         = ++sequence;
    event.dropped                                     = dropped;
    current                                           = event;
    have_current                                      = true;
    queue[(head + count) % ROBOT_STEERING_QUEUE_SIZE] = event;
    ++count;
    EI(state);
}
bool robot_steering_peek (robot_steering_event_t * event)
{
    UINT state;
    bool available;
    DI(state);
    available = count != 0;
    if (available)
    {
        *event         = queue[head];
        event->dropped = dropped;
    }
    EI(state);
    return available;
}
void robot_steering_ack (uint32_t seq)
{
    UINT state;
    DI(state);
    if (count && queue[head].seq == seq)
    {
        head = (head + 1u) % ROBOT_STEERING_QUEUE_SIZE;
        --count;
    }
    EI(state);
}
bool robot_steering_snapshot (robot_steering_event_t * event)
{
    UINT state;
    bool ready;
    DI(state);
    ready = !count && have_current;
    if (ready)
    {
        *event         = current;
        event->dropped = dropped;
    }
    EI(state);
    return ready;
}

/*** end of file ***/

