/**
 * @file    buddy5_avoidance.h
 * @brief   Buddy 5 obstacle-avoidance task.
 *
 * This module combines the ultrasonic scanner with Buddy 2 motors, Buddy 3
 * line detection, and Buddy 4 heading feedback to carry out one complete
 * obstacle-avoidance route.  Low-level ultrasonic/servo access remains in
 * buddy5_ultrasonic.c.
 */

#ifndef BUDDY5_AVOIDANCE_H
#define BUDDY5_AVOIDANCE_H

#include <tk/tkernel.h>

/** Run the 100 ms Buddy 5 ultrasonic and avoidance task. */
void buddy5_avoidance_task (INT stacd, void * p_exinf);

#endif /* BUDDY5_AVOIDANCE_H */

/*** end of file ***/

