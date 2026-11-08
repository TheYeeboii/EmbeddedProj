#ifndef CODE39_H
#define CODE39_H

#include <stdbool.h>

/* Single uppercase letter, framed as *LETTER*. Each character has nine
 * alternating black/white elements, separated by one narrow white gap. */
#define CODE39_FRAME_ELEMENTS 29u
typedef enum
{
    CODE39_OK,
    CODE39_BAD_COUNT,
    CODE39_BAD_WIDTH,
    CODE39_BAD_FRAME,
    CODE39_BAD_SYMBOL,
    CODE39_BAD_QUIET
} code39_status_t;
typedef struct
{
    char  symbol;
    bool  reversed;
    float narrow_width;
    float wide_ratio;
    float frame_width;
} code39_result_t;

/* Width units are arbitrary, but must be consistent. Both quiet zones must
 * be >=10 narrow widths. Output is cleared on rejection. No motor actions. */
code39_status_t code39_decode (const float * widths, unsigned count,
                               float leading_quiet, float trailing_quiet,
                               code39_result_t * result);
const char *    code39_status_name (code39_status_t status);
#endif

/*** end of file ***/

