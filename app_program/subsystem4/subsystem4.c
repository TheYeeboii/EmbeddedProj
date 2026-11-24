/**
 * @file    subsystem4.c
 * @brief   Subsystem 4 (IMU): start-up and the IMU task.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "app.h"
#include "subsystem4.h"

void subsystem4_init (void)
{
    if (!buddy4_init())
    {
        /* Not fatal: telemetry will show "compass_error". */
        tm_printf((UB *) "[init] IMU (LSM303) not answering on I2C - check "
                         "Grove 3.\n");
    }
}

void subsystem4_start (void)
{
    (void) app_start_task("IMU task", buddy4_task, APP_PRI_IMU);
}

/*** end of file ***/
