/** @file robot_packed.c
 *
 * @brief Buddy 1: pack and unpack values at their exact bit widths.
 *
 * See robot_packed.h.  Fields are moved one bit at a time: the messages are
 * a few hundred bits, so this costs microseconds, and it never makes a wide
 * access to the buffer (rule: byte-by-byte only on the Cortex-M0+).
 */
#include <stddef.h>

#include "robot_packed.h"

#define PACKED_BITS_PER_BYTE    (8u)
#define PACKED_TOP_BIT          (7u)
#define PACKED_SIGNED_MIN_WIDTH (2u)
#define PACKED_SIGNED_MAX_WIDTH (31u)

static uint32_t packed_field_max (uint32_t width);
static bool     packed_width_ok (uint32_t width);

void robot_packed_writer_init (robot_packed_writer_t * p_writer,
                               uint8_t * p_buffer, uint32_t capacity)
{
    uint32_t index = 0u;

    if (NULL != p_writer)
    {
        p_writer->p_data     = p_buffer;
        p_writer->capacity   = (NULL == p_buffer) ? 0u : capacity;
        p_writer->bit_pos    = 0u;
        p_writer->b_overflow = false;
        for (index = 0u; index < p_writer->capacity; index++)
        {
            p_buffer[index] = 0u;
        }
    }
}

void robot_packed_put (robot_packed_writer_t * p_writer, uint32_t value,
                       uint32_t width)
{
    uint32_t field_max = 0u;
    uint32_t bit       = 0u;
    uint32_t byte_pos  = 0u;
    uint32_t shift     = 0u;

    if ((NULL == p_writer) || !packed_width_ok(width))
    {
        /* Nothing to write into, or a programming error in the caller. */
    }
    else if ((p_writer->bit_pos + width) >
             (p_writer->capacity * PACKED_BITS_PER_BYTE))
    {
        p_writer->b_overflow = true;
    }
    else
    {
        field_max = packed_field_max(width);
        if (value > field_max)
        {
            value = field_max;
        }
        /* Most significant bit of the field first. */
        for (bit = width; bit > 0u; bit--)
        {
            if (0u != ((value >> (bit - 1u)) & 1u))
            {
                byte_pos = p_writer->bit_pos / PACKED_BITS_PER_BYTE;
                shift =
                    PACKED_TOP_BIT - (p_writer->bit_pos % PACKED_BITS_PER_BYTE);
                /* shift is 0..7, so the mask fits in one byte. */
                p_writer->p_data[byte_pos] |= (uint8_t) (1u << shift);
            }
            p_writer->bit_pos++;
        }
    }
}

void robot_packed_put_signed (robot_packed_writer_t * p_writer, int32_t value,
                              uint32_t width)
{
    int32_t  field_max = 0;
    int32_t  field_min = 0;
    uint32_t pattern   = 0u;

    if ((NULL == p_writer) || (width < PACKED_SIGNED_MIN_WIDTH) ||
        (width > PACKED_SIGNED_MAX_WIDTH))
    {
        /* Nothing to write into, or a programming error in the caller. */
    }
    else
    {
        /* width <= 31, so both limits fit in int32_t. */
        field_max = (int32_t) ((1u << (width - 1u)) - 1u);
        field_min = -field_max - 1;
        if (value > field_max)
        {
            value = field_max;
        }
        else if (value < field_min)
        {
            value = field_min;
        }
        else
        {
            /* Already inside the field. */
        }
        /* Two's complement: the conversion keeps the bit pattern, and the
         * mask keeps only the field's low bits. */
        pattern = ((uint32_t) value) & packed_field_max(width);
        robot_packed_put(p_writer, pattern, width);
    }
}

uint32_t robot_packed_length (const robot_packed_writer_t * p_writer)
{
    uint32_t length = 0u;

    if ((NULL != p_writer) && !p_writer->b_overflow)
    {
        length = (p_writer->bit_pos + (PACKED_BITS_PER_BYTE - 1u)) /
                 PACKED_BITS_PER_BYTE;
    }

    return length;
}

void robot_packed_reader_init (robot_packed_reader_t * p_reader,
                               const uint8_t * p_data, uint32_t length)
{
    if (NULL != p_reader)
    {
        p_reader->p_data    = p_data;
        p_reader->length    = (NULL == p_data) ? 0u : length;
        p_reader->bit_pos   = 0u;
        p_reader->b_overrun = false;
    }
}

uint32_t robot_packed_get (robot_packed_reader_t * p_reader, uint32_t width)
{
    uint32_t value    = 0u;
    uint32_t bit      = 0u;
    uint32_t byte_pos = 0u;
    uint32_t shift    = 0u;

    if ((NULL == p_reader) || !packed_width_ok(width))
    {
        /* Nothing to read from, or a programming error in the caller. */
    }
    else if ((p_reader->bit_pos + width) >
             (p_reader->length * PACKED_BITS_PER_BYTE))
    {
        p_reader->b_overrun = true;
    }
    else
    {
        for (bit = 0u; bit < width; bit++)
        {
            byte_pos = p_reader->bit_pos / PACKED_BITS_PER_BYTE;
            shift = PACKED_TOP_BIT - (p_reader->bit_pos % PACKED_BITS_PER_BYTE);
            value = (value << 1u) |
                    (((uint32_t) p_reader->p_data[byte_pos] >> shift) & 1u);
            p_reader->bit_pos++;
        }
    }

    return value;
}

int32_t robot_packed_get_signed (robot_packed_reader_t * p_reader,
                                 uint32_t                width)
{
    uint32_t pattern  = 0u;
    uint32_t sign_bit = 0u;
    int32_t  value    = 0;

    if ((width >= PACKED_SIGNED_MIN_WIDTH) &&
        (width <= PACKED_SIGNED_MAX_WIDTH))
    {
        pattern  = robot_packed_get(p_reader, width);
        sign_bit = 1u << (width - 1u);
        if (0u != (pattern & sign_bit))
        {
            /* Negative: value = pattern - 2^width.  Both parts fit in int32_t
             * because width <= 31. */
            value = (int32_t) (pattern & (sign_bit - 1u)) - (int32_t) sign_bit;
        }
        else
        {
            value = (int32_t) pattern; /* Below 2^30, so it fits. */
        }
    }

    return value;
}

bool robot_packed_rest_is_zero (const robot_packed_reader_t * p_reader)
{
    uint32_t bit_pos = 0u;
    uint32_t shift   = 0u;
    bool     b_zero  = false;

    if (NULL != p_reader)
    {
        b_zero = true;
        for (bit_pos = p_reader->bit_pos;
             b_zero && (bit_pos < (p_reader->length * PACKED_BITS_PER_BYTE));
             bit_pos++)
        {
            shift = PACKED_TOP_BIT - (bit_pos % PACKED_BITS_PER_BYTE);
            b_zero =
                (0u == (((uint32_t)
                             p_reader->p_data[bit_pos / PACKED_BITS_PER_BYTE] >>
                         shift) &
                        1u));
        }
    }

    return b_zero;
}

/* Largest unsigned value a field of "width" bits can hold. */
static uint32_t packed_field_max (uint32_t width)
{
    return (width >= ROBOT_PACKED_MAX_WIDTH) ? UINT32_MAX
                                             : ((1u << width) - 1u);
}

static bool packed_width_ok (uint32_t width)
{
    return ((width > 0u) && (width <= ROBOT_PACKED_MAX_WIDTH));
}

/*** end of file ***/

