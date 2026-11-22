/** @file code39.h
 *
 * @brief Decoder for a single framed Code 39 letter (*A* to *Z*) from a list
 *        of measured element widths.
 */
#ifndef CODE39_H
#define CODE39_H

#include <stdbool.h>
#include <stdint.h>
#include "robot_types.h"

/* Single uppercase letter, framed as *LETTER*. Each character has nine
 * alternating black/white elements, separated by one narrow white gap. */
#define CODE39_CHARACTER_ELEMENTS 9u
#define CODE39_FRAME_ELEMENTS 29u

/** Decode result; code39_status_name() gives the log name of each value. */
typedef enum
{
    CODE39_OK,
    CODE39_BAD_COUNT,
    CODE39_BAD_WIDTH,
    CODE39_BAD_FRAME,
    CODE39_BAD_SYMBOL,
    CODE39_BAD_QUIET
} code39_status_t;

/** A decoded letter and the measured geometry of its frame. */
typedef struct
{
    char      symbol;
    bool      reversed;
    float32_t narrow_width;
    float32_t wide_ratio;
    float32_t frame_width;
} code39_result_t;

/**
 * @brief Decode one *LETTER* frame.  Width units are arbitrary but must be
 *        consistent.  Both quiet zones must be at least 10 narrow widths.
 *        No motor actions are taken.
 * @param p_widths       CODE39_FRAME_ELEMENTS element widths, in scan order.
 * @param count          Number of widths; must be CODE39_FRAME_ELEMENTS.
 * @param leading_quiet  White width before the frame.
 * @param trailing_quiet White width after the frame.
 * @param p_result       Receives the letter; cleared on rejection.
 * @return CODE39_OK or the reason the frame was rejected.
 */
code39_status_t code39_decode (const float32_t * p_widths, uint32_t count,
                               float32_t         leading_quiet,
                               float32_t         trailing_quiet,
                               code39_result_t * p_result);

/**
 * @brief True when nine element widths are a start/stop '*', read either
 *        way (a reversed '*' looks different).  Used to resynchronise.
 * @param p_widths Nine element widths, first one black.
 * @return True for '*' or a reversed '*'.
 */
bool code39_is_star (const float32_t * p_widths);

/** Classify nine widths in scan order and return the pattern (first bar in bit 8). */
bool code39_character_pattern (const float32_t * p_widths, uint16_t * p_pattern);

/** Look up A–Z or '*'. Reverse the pattern for a scan in reverse print order. */
char code39_character_symbol (uint16_t pattern, bool reversed);

/**
 * @brief Short log name for a decode status.
 * @param status Status from code39_decode().
 * @return Constant string, "unknown" for out-of-range values.
 */
const char * code39_status_name (code39_status_t status);

#endif /* CODE39_H */

/*** end of file ***/

