#include <assert.h>
#include <string.h>
#include "barcode_tuning.h"
#include "comms.h"
#include "robot_packed.h"

static uint8_t captured[14];
static unsigned sent;

void buddy2_get_barcode_speed(bool * active, uint8_t * effective)
{
    *active = true;
    *effective = 10u;
}
uint8_t comms_get_link_state(void) { return COMMS_LINK_MQTT_UP; }
bool comms_publish(uint32_t topic, const uint8_t *data, uint32_t length)
{
    assert(topic == COMMS_TOPIC_DEBUG);
    assert(length == sizeof captured);
    memcpy(captured, data, length);
    ++sent;
    return true;
}

int main(void)
{
    robot_packed_reader_t reader;
    assert(barcode_tuning_request(220, 260, true));
    assert(barcode_tuning_sample(200, false));
    barcode_tuning_publish(200);
    assert(sent == 1);
    robot_packed_reader_init(&reader, captured, sizeof captured);
    assert(robot_packed_get(&reader, 8) == 0xc1);
    assert(robot_packed_get(&reader, 32) == 200);
    assert(robot_packed_get(&reader, 12) == 220);
    assert(robot_packed_get(&reader, 12) == 260);
    assert(robot_packed_get(&reader, 12) == 200);
    assert(robot_packed_get(&reader, 1) == 1);
    assert(robot_packed_get(&reader, 1) == 1);
    assert(robot_packed_get(&reader, 1) == 1);
    assert(robot_packed_get(&reader, 2) == 0);
    assert(robot_packed_get(&reader, 1) == 0); /* analogue fixture */
    (void) robot_packed_get(&reader, 5);
    (void) robot_packed_get(&reader, 1);
    assert(robot_packed_get(&reader, 16) == 0); /* age before MQTT queue */
    assert(robot_packed_get(&reader, 1) == 1);
    assert(robot_packed_get(&reader, 7) == 10);
    assert(!reader.b_overrun);
    barcode_tuning_publish(210);
    assert(sent == 1);
    assert(ir_tuning_request(1, 220, 260, false));
    assert(ir_tuning_sample(1, 300, false));
    barcode_tuning_publish(400);
    robot_packed_reader_init(&reader, captured, sizeof captured);
    (void) robot_packed_get(&reader, 8);
    (void) robot_packed_get(&reader, 32);
    assert(robot_packed_get(&reader, 12) == 220);
    assert(robot_packed_get(&reader, 12) == 260);
    assert(robot_packed_get(&reader, 12) == 300);
    assert(robot_packed_get(&reader, 1) == 1);
    assert(robot_packed_get(&reader, 1) == 0);
    assert(robot_packed_get(&reader, 1) == 1);
    assert(robot_packed_get(&reader, 2) == 1);
    assert(robot_packed_get(&reader, 1) == 0); /* analogue fixture */
    (void) robot_packed_get(&reader, 5);
    (void) robot_packed_get(&reader, 1);
    assert(robot_packed_get(&reader, 16) == 0); /* age before MQTT queue */
    assert(robot_packed_get(&reader, 1) == 1);
    assert(robot_packed_get(&reader, 7) == 10);
    assert(!reader.b_overrun);
    return 0;
}
