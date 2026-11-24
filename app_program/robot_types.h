/** @file robot_types.h
 *
 * @brief Project-wide fixed-width floating-point type names (BARR-C 5.4.b.i).
 *
 * C99 has no float32_t, so the project defines it here.  The array below
 * fails to compile if float is not 32 bits wide on the target.
 */
#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>

/** 32-bit IEEE-754 single-precision floating point. */
typedef float float32_t;

/** Compile-time check: a negative array size is an error. */
typedef uint8_t robot_types_float32_check_t[(4u == sizeof(float32_t)) ? 1 : -1];

#endif /* ROBOT_TYPES_H */

/*** end of file ***/

