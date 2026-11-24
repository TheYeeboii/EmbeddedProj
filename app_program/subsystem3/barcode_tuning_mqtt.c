#include "barcode_tuning.h"
#include "comms.h"
#include "robot_packed.h"
#include "pin_config.h"
#include "buddy2_motion.h"

/* Packed type 12, version 1. */
void barcode_tuning_publish (uint32_t now)
{
    static uint32_t last;
    static uint8_t sensor;
    barcode_tuning_state_t state;
    robot_packed_writer_t writer;
    uint8_t payload[14];
    bool slowdown;
    uint8_t effective_speed;
    if ((now - last < 200u) || (COMMS_LINK_MQTT_UP != comms_get_link_state()))
    {
        return;
    }
    ir_tuning_get(sensor, &state);
    buddy2_get_barcode_speed(&slowdown, &effective_speed);
    robot_packed_writer_init(&writer, payload, sizeof(payload));
    robot_packed_put(&writer, 0xC1u, 8u);
    robot_packed_put(&writer, now, 32u);
    robot_packed_put(&writer, state.low, 12u);
    robot_packed_put(&writer, state.high, 12u);
    robot_packed_put(&writer, state.raw, 12u);
    robot_packed_put(&writer, state.enabled, 1u);
    robot_packed_put(&writer, state.inverted, 1u);
    robot_packed_put(&writer, state.black, 1u);
    robot_packed_put(&writer, sensor, 2u);
    robot_packed_put(&writer, sensor == 0u, 1u); /* digital input */
    robot_packed_put(&writer, (sensor == 0u) ? BARCODE_DIGITAL_PIN : 0u, 5u);
    robot_packed_put(&writer, BARCODE_DIGITAL_BLACK_LEVEL, 1u);
    robot_packed_put(&writer, 0u, 16u); /* transport age, updated by MQTT */
    robot_packed_put(&writer, slowdown, 1u);
    robot_packed_put(&writer, effective_speed, 7u);
    if (comms_publish(COMMS_TOPIC_DEBUG, payload, 14u))
    {
        last = now;
        sensor = (uint8_t) ((sensor + 1u) % IR_TUNING_SENSOR_COUNT);
    }
}
