/** @file robot_packed.h
 *
 * @brief Buddy 1: pack and unpack values at their exact bit widths.
 *
 * Change Requirement 1 asks every field to use only the bits its valid range
 * needs.  These helpers write fields back to back into a byte buffer, most
 * significant bit first, and read them back in the same order.
 *
 * Two rules every caller relies on:
 *   - A buffer is built and read ONE BYTE AT A TIME.  The Cortex-M0+ faults
 *     on a 16- or 32-bit access at an odd address, so a packed buffer must
 *     never be cast to a wider type.
 *   - A C bit-field struct is never sent as it is.  The compiler chooses its
 *     bit order and padding; the dashboard could read it differently.
 *
 * Portable C (no kernel headers), so the host tests can use it.
 */
#ifndef ROBOT_PACKED_H
#define ROBOT_PACKED_H

#include <stdbool.h>
#include <stdint.h>

/** Widest field one call can write or read. */
#define ROBOT_PACKED_MAX_WIDTH (32u)

/** Writes fields into a caller-owned buffer. */
typedef struct
{
    uint8_t * p_data;   /**< Output buffer, cleared by the init call. */
    uint32_t  capacity; /**< Size of p_data in bytes. */
    uint32_t  bit_pos;  /**< Next bit to write, counted from bit 7 of byte 0. */
    bool      b_overflow; /**< A field did not fit; the message is invalid. */
} robot_packed_writer_t;

/** Reads fields from a received buffer. */
typedef struct
{
    const uint8_t * p_data;    /**< Received bytes. */
    uint32_t        length;    /**< Number of received bytes. */
    uint32_t        bit_pos;   /**< Next bit to read. */
    bool            b_overrun; /**< A read went past the end. */
} robot_packed_reader_t;

/**
 * @brief Start writing into p_buffer and clear it to zero.
 * @param p_writer Writer to set up; NULL is ignored.
 * @param p_buffer Output buffer; NULL makes every write overflow.
 * @param capacity Size of p_buffer in bytes.
 */
void robot_packed_writer_init (robot_packed_writer_t * p_writer,
                               uint8_t * p_buffer, uint32_t capacity);

/**
 * @brief Append the low "width" bits of an unsigned value.
 *
 * A value too large for the field is saturated to the field's maximum, so a
 * reading outside the documented range shows up as the largest value rather
 * than as a wrapped small one.
 *
 * @param p_writer Writer from robot_packed_writer_init().
 * @param value    Value to write.
 * @param width    Field width in bits, 1..ROBOT_PACKED_MAX_WIDTH.
 */
void robot_packed_put (robot_packed_writer_t * p_writer, uint32_t value,
                       uint32_t width);

/**
 * @brief Append a signed value as a two's-complement field.
 *
 * Values outside the field's range are saturated to its minimum or maximum.
 *
 * @param p_writer Writer from robot_packed_writer_init().
 * @param value    Value to write.
 * @param width    Field width in bits, 2..ROBOT_PACKED_MAX_WIDTH.
 */
void robot_packed_put_signed (robot_packed_writer_t * p_writer, int32_t value,
                              uint32_t width);

/**
 * @brief Bytes used so far (a partly filled last byte counts as one).
 * @param p_writer Writer from robot_packed_writer_init().
 * @return Length in bytes, or 0 when a write overflowed.
 */
uint32_t robot_packed_length (const robot_packed_writer_t * p_writer);

/**
 * @brief Start reading "length" bytes from p_data.
 * @param p_reader Reader to set up; NULL is ignored.
 * @param p_data   Received bytes; NULL makes every read overrun.
 * @param length   Number of bytes in p_data.
 */
void robot_packed_reader_init (robot_packed_reader_t * p_reader,
                               const uint8_t * p_data, uint32_t length);

/**
 * @brief Read the next unsigned field.
 * @param p_reader Reader from robot_packed_reader_init().
 * @param width    Field width in bits, 1..ROBOT_PACKED_MAX_WIDTH.
 * @return The field, or 0 when the read overran (b_overrun is then set).
 */
uint32_t robot_packed_get (robot_packed_reader_t * p_reader, uint32_t width);

/**
 * @brief Read the next two's-complement field and sign-extend it.
 * @param p_reader Reader from robot_packed_reader_init().
 * @param width    Field width in bits, 2..ROBOT_PACKED_MAX_WIDTH.
 * @return The field, or 0 when the read overran.
 */
int32_t robot_packed_get_signed (robot_packed_reader_t * p_reader,
                                 uint32_t                width);

/**
 * @brief True when every bit after the last field read is zero.
 *
 * Fixed-size messages leave their padding bits clear, so a set bit means the
 * message is not the one its first byte claims.
 *
 * @param p_reader Reader from robot_packed_reader_init().
 */
bool robot_packed_rest_is_zero (const robot_packed_reader_t * p_reader);

#endif /* ROBOT_PACKED_H */

/*** end of file ***/

