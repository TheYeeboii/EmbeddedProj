#include "robot_barcode_action.h"

void robot_barcode_action_cancel(robot_barcode_action_t *s)
{
    s->active = false;
}
uint8_t robot_barcode_action_request(robot_barcode_action_t *s, uint8_t command,
    uint8_t mode, bool allowed, uint32_t now_ms)
{
    if (s->active || !allowed || command < 1u || command > 3u || mode == 0u)
        return RB_NONE;
    if (mode != 2u) return RB_OBSERVED;
    s->active = true;
    s->started_ms = now_ms;
    return command; /* Shared values: left=1, right=2, straight=3. */
}
uint8_t robot_barcode_action_poll(robot_barcode_action_t *s,
    bool motion_active, uint32_t now_ms)
{
    if (!s->active) return RB_NONE;
    if ((now_ms - s->started_ms) >= ROBOT_BARCODE_ACTION_TIMEOUT_MS) {
        s->active = false;
        return RB_STOP;
    }
    if (!motion_active) {
        s->active = false;
        return RB_RESUME;
    }
    return RB_NONE;
}
