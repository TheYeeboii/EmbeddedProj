/**
 * @file    subsystem5.c
 * @brief   Subsystem 5 (ultrasonic and obstacle avoidance): start-up and
 *          the sensors task.
 */

#include <tk/tkernel.h>

#include "app.h"
#include "subsystem5.h"

void subsystem5_init (void)
{
    buddy5_init();
}

void subsystem5_start (void)
{
    (void) app_start_task("sensor task", buddy5_avoidance_task,
                          APP_PRI_SENSORS);
}

/*** end of file ***/
