/* Update only the transport-age field of packed diagnostic messages.
 * Raw samples, timestamps, sequence numbers and classifications stay intact. */
#include "robot_packed.h"
#include "robot_wire.h"

void robot_wire_add_transport_age (uint8_t * data, uint32_t length,
                                    uint32_t age_ms)
{
    uint32_t offset;
    uint32_t lag;
    uint32_t bit;
    robot_packed_reader_t reader;

    if ((data == 0) || (length == 0u) ||
        ((data[0] & 15u) != ROBOT_WIRE_VERSION))
    {
        return;
    }
    switch (data[0] >> 4u)
    {
        case ROBOT_WIRE_TYPE_STEERING: offset = 104u; break;
        case ROBOT_WIRE_TYPE_BARCODE: offset = 155u; break;
        case ROBOT_WIRE_TYPE_IR: offset = 40u; break;
        case ROBOT_WIRE_TYPE_IR_TUNING: offset = 88u; break;
        default: return;
    }
    if ((offset + 16u) > length * 8u)
    {
        return;
    }
    robot_packed_reader_init(&reader, data, length);
    reader.bit_pos = offset;
    lag = robot_packed_get(&reader, 16u);
    lag = (age_ms > 65535u - lag) ? 65535u : lag + age_ms;
    for (bit = 0u; bit < 16u; ++bit)
    {
        uint32_t position = offset + bit;
        uint8_t mask = (uint8_t) (1u << (7u - position % 8u));
        data[position / 8u] &= (uint8_t) ~mask;
        if ((lag & (1u << (15u - bit))) != 0u)
        {
            data[position / 8u] |= mask;
        }
    }
}
