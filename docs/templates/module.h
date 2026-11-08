/** @file module.h
 *
 * @brief One line saying what this module is for.
 *
 * Template for a BARR-C:2018 header (rule 4.4.a).  Copy it, rename
 * "module"/"MODULE" to the lowercase file name, and delete what is unused.
 * Public names start with the module name (rules 5.1.c and 6.1.i).
 */
#ifndef MODULE_H
#define MODULE_H

#include <stdbool.h>
#include <stdint.h>
#include "robot_types.h"

/* Constants other modules need.  Unsigned constants end in u. */
#define MODULE_PERIOD_MS (10u)

/** What the values mean. */
typedef enum
{
    MODULE_STATE_IDLE,
    MODULE_STATE_RUNNING
} module_state_t;

/** What the structure holds. */
typedef struct
{
    uint32_t  count;
    float32_t level;
    bool      b_ready;
} module_status_t;

/**
 * @brief What the function does, in one sentence.
 * @param now_ms What the parameter is, with its unit.
 */
void module_update (uint32_t now_ms);

/**
 * @brief What the function does, in one sentence.
 * @param p_status Receives the status; NULL is ignored.
 * @return True when the status was copied.
 */
bool module_get_status (module_status_t * p_status);

#endif /* MODULE_H */

/*** end of file ***/

