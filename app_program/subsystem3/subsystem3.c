/**
 * @file    subsystem3.c
 * @brief   Subsystem 3 (line following and barcode): start-up and the line
 *          task.
 *
 * A cyclic handler wakes the line task every BUDDY3_BARCODE_PERIOD_MS (1 ms)
 * for the barcode sensor; buddy3_update() does the line and junction work
 * every BUDDY3_SAMPLE_PERIOD_MS (10 ms) of those wake-ups.
 */

#include <tk/tkernel.h>

#include "app.h"
#include "subsystem3.h"

static ID g_line_task_id = 0;

/** @brief Run Buddy 3's periodic line-sensor update. */
static void line_task (INT stacd, void * p_exinf)
{
    (void) stacd;
    (void) p_exinf;
    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR);
        buddy3_update(app_now_ms());
    }
}

void subsystem3_init (void)
{
    buddy3_init();
}

void subsystem3_start (void)
{
    g_line_task_id = app_start_task("line task", line_task, APP_PRI_LINE);
    app_start_cyclic("line cyclic", &g_line_task_id,
                     (UINT) BUDDY3_BARCODE_PERIOD_MS);
}

/*** end of file ***/
