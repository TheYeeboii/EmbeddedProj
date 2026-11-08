/** @file module.c
 *
 * @brief One line saying what this module is for.
 *
 * Template for a BARR-C:2018 source file (rule 4.4.a).  Sections appear in
 * the order of rule 4.3.b: comment block, includes, private constants and
 * types, static data, private prototypes, public functions, then private
 * functions.  Format it with tools/barr_c_format.sh.
 */
#include <tk/tkernel.h>

#include "module.h"

/* Private constants: no magic numbers in the code below. */
#define MODULE_LIMIT (100u)

/* File-scope data starts with g_ and is initialised here. */
static module_status_t g_status         = {0u, 0.0f, false};
static uint32_t        g_last_update_ms = 0u;

static bool module_is_due (uint32_t now_ms);

void module_update (uint32_t now_ms)
{
    UINT interrupt_state = 0u;

    if (module_is_due(now_ms))
    {
        g_last_update_ms = now_ms;
        DI(interrupt_state);
        if (g_status.count < MODULE_LIMIT)
        {
            g_status.count++;
        }
        g_status.b_ready = true;
        EI(interrupt_state);
    }
}

bool module_get_status (module_status_t * p_status)
{
    UINT interrupt_state = 0u;
    bool b_copied        = false;

    /* One exit point: decide, then return at the bottom (rule 6.2.c). */
    if (NULL != p_status)
    {
        DI(interrupt_state);
        *p_status = g_status;
        EI(interrupt_state);
        b_copied = true;
    }

    return b_copied;
}

/* Private functions are static and come after the public ones. */
static bool module_is_due (uint32_t now_ms)
{
    return ((now_ms - g_last_update_ms) >= MODULE_PERIOD_MS);
}

/*** end of file ***/

