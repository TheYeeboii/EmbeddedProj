/** @file buddy3_barcode.h
 *
 * @brief Buddy 3 barcode reader: turns black/white samples and wheel travel
 *        into Code 39 element widths and decodes the framed letter.
 */
#ifndef BUDDY3_BARCODE_H
#define BUDDY3_BARCODE_H

#include <stdbool.h>
#include <stdint.h>
#include "robot_types.h"
#include "code39.h"

/* Decode by default; 0 disables. */
#ifndef BUDDY3_BARCODE_MIN_ELEMENT_MM
#define BUDDY3_BARCODE_MIN_ELEMENT_MM 3.0f
#endif

/* Sensor offset from the axle centre line: left positive, right negative. */
#ifndef BUDDY3_BARCODE_OFFSET_LEFT_MM
#define BUDDY3_BARCODE_OFFSET_LEFT_MM 0.0f
#endif

/** Reader settings. */
typedef struct
{
    float32_t
        min_element_mm; /* Resolution bound, NOT the actual printed width. */
} buddy3_barcode_config_t;

/** What a call to buddy3_barcode_step() reported. */
typedef enum
{
    B3BC_NONE,
    B3BC_VALID,
    B3BC_FORMAT,
    B3BC_INTERRUPTED,
    B3BC_REVERSE,
    B3BC_SAMPLE_GAP,
    B3BC_STALL,
    B3BC_TIMEOUT
} buddy3_barcode_event_kind_t;

/** One reader event. */
typedef struct
{
    buddy3_barcode_event_kind_t kind;
    code39_status_t             format_status;
    uint32_t                    ts_ms;
    uint32_t dropped; /* Adapter's cumulative count of unread events dropped. */
    code39_result_t decoded;
} buddy3_barcode_event_t;

/** Reader memory kept between calls to buddy3_barcode_step(). */
typedef struct
{
    float32_t position;
    float32_t edge_position;
    float32_t white_distance;
    float32_t leading_quiet;
    float32_t max_sample_gap;
    float32_t widths[CODE39_FRAME_ELEMENTS];
    uint32_t  edge_ms[CODE39_FRAME_ELEMENTS];
    uint32_t  last_progress_ms;
    uint32_t  started_ms;
    uint8_t   count;
    bool      active;
    bool      previous_black;
    bool      have_sample;
} buddy3_barcode_decoder_t;

/**
 * @brief Forget any frame in progress and the white lead-in.
 * @param p_state Reader state to clear.
 */
void buddy3_barcode_reset (buddy3_barcode_decoder_t * p_state);

/**
 * @brief Feed one sample.  Forward vehicle travel only; a reversed PRINT
 *        orientation is supported.  No allocation, I/O or motor control.
 *        Emits at most one event per call.  Widths and edge timestamps
 *        remain available until a reset or the next frame.
 * @param p_state   Reader memory, updated in place.
 * @param p_cfg     Reader settings.
 * @param black     True when the sensor sees black.
 * @param travel_mm Sensor travel since the previous sample.
 * @param now       Current time in milliseconds.
 * @param permitted False aborts any frame (e.g. while turning).
 * @param p_event   Receives the event, if any.
 * @return True when p_event holds a new event.
 */
bool buddy3_barcode_step (buddy3_barcode_decoder_t *      p_state,
                          const buddy3_barcode_config_t * p_cfg, bool black,
                          float32_t travel_mm, uint32_t now, bool permitted,
                          buddy3_barcode_event_t * p_event);

/**
 * @brief Travel of an off-centre sensor from the two wheel distances.
 * @param left_mm        Left wheel travel.
 * @param right_mm       Right wheel travel.
 * @param track_mm       Distance between the wheels; 0 or less gives 0.
 * @param offset_left_mm Sensor offset, left positive.
 * @return Sensor travel in millimetres.
 */
float32_t buddy3_barcode_travel (float32_t left_mm, float32_t right_mm,
                                 float32_t track_mm, float32_t offset_left_mm);

/**
 * @brief Short log name for an event kind.
 * @param kind Event kind.
 * @return Constant string, "unknown" for out-of-range values.
 */
const char * buddy3_barcode_event_name (buddy3_barcode_event_kind_t kind);

#endif /* BUDDY3_BARCODE_H */

/*** end of file ***/

