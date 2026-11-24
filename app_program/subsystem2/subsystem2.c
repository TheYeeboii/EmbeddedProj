/**
 * @file    subsystem2.c
 * @brief   Subsystem 2 (motion control): start-up and the motion task.
 *
 * A cyclic handler wakes the motion task every BUDDY2_CONTROL_PERIOD_MS; the
 * task then updates motor control, encoder distance and active moves.
 */

#include <tk/tkernel.h>

#include "app.h"
#include "subsystem2.h"

static ID g_motion_task_id = 0;

/** @brief Run Buddy 2's periodic motor/encoder update. */
static void motion_task (INT stacd, void * p_exinf)
{
    (void) stacd;
    (void) p_exinf;
    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR);
        buddy2_update(app_now_ms());
    }
}

void subsystem2_init (void)
{
    if (!app_check("buddy2_motion_init", (INT) buddy2_motion_init()))
    {
        app_fail_forever("buddy2_motion_init");
    }
}

void subsystem2_start (void)
{
    g_motion_task_id =
        app_start_task("motion task", motion_task, APP_PRI_MOTION);
    app_start_cyclic("motion cyclic", &g_motion_task_id,
                     (UINT) BUDDY2_CONTROL_PERIOD_MS);
}

/*** end of file ***/
