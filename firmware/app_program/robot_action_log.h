#ifndef ROBOT_ACTION_LOG_H
#define ROBOT_ACTION_LOG_H
#include <stdbool.h>
#include <stdint.h>

#define ROBOT_ACTION_QUEUE_SIZE 32u
#define ROBOT_ACTION_PWM_THRESHOLD 256
enum { ROBOT_ACTION_NONE, ROBOT_ACTION_LEFT, ROBOT_ACTION_RIGHT };
enum { ROBOT_ACTION_TURN_LEFT, ROBOT_ACTION_TURN_RIGHT,
       ROBOT_ACTION_STOP_LEFT, ROBOT_ACTION_STOP_RIGHT };
enum { ROBOT_ACTION_PWM, ROBOT_ACTION_MOTION, ROBOT_ACTION_STOP };
typedef struct {
    uint32_t seq, ts_ms, dropped;
    uint8_t action, direction, source;
    int16_t left_pwm, right_pwm;
} robot_action_event_t;

/* Called at the accepted motor-command boundary; no formatting/network calls.
 * Direction describes intent, not measured motion. Repeated states do not
 * enqueue events. Switching L->R emits stop-left, then start-right. */
uint8_t robot_action_direction_for_pwm(int16_t left, int16_t right);
void robot_action_record(uint8_t direction, uint8_t source,
                         int16_t left, int16_t right, uint32_t ts_ms);
bool robot_action_peek(robot_action_event_t *event);
void robot_action_ack(uint32_t seq);
void robot_action_snapshot(robot_action_event_t *state);
bool robot_action_snapshot_if_empty(robot_action_event_t *state);

/* Called by the controller task, never from the wheel-control critical path.
 * Drains at most two events per call; a current-state snapshot is sent each
 * second when caught up. Network publication remains in the radio task. */
void robot_action_publish(uint32_t now_ms);
#endif
