#ifndef ROBOT_BARCODE_ACTION_H
#define ROBOT_BARCODE_ACTION_H
#include <stdbool.h>
#include <stdint.h>

#ifndef ROBOT_BARCODE_DEFAULT_MODE
#define ROBOT_BARCODE_DEFAULT_MODE 1u /* 0 off, 1 observe, 2 execute */
#endif
#define ROBOT_BARCODE_ACTION_TIMEOUT_MS 8000u
enum { RB_NONE, RB_LEFT, RB_RIGHT, RB_STRAIGHT, RB_OBSERVED, RB_RESUME, RB_STOP };
typedef struct {
    bool active;
    uint32_t started_ms;
} robot_barcode_action_t;
void robot_barcode_action_cancel(robot_barcode_action_t *s);
uint8_t robot_barcode_action_request(robot_barcode_action_t *s, uint8_t command,
    uint8_t mode, bool allowed, uint32_t now_ms);
uint8_t robot_barcode_action_poll(robot_barcode_action_t *s,
    bool motion_active, uint32_t now_ms);
#endif
