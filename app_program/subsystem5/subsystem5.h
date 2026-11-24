/**
 * @file    subsystem5.h
 * @brief   Subsystem 5 (ultrasonic and obstacle avoidance): public interface.
 *
 * Owner and pins: TEAM.md.  usermain() calls subsystem5_init() first (no
 * tasks running yet), then subsystem5_start().  Other subsystems use the
 * functions declared in the headers included below.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "buddy5_ultrasonic.h"
#include "buddy5_avoidance.h"

/** @brief Initialise this subsystem's hardware and kernel objects. */
void subsystem5_init (void);

/** @brief Create and start this subsystem's task(s). */
void subsystem5_start (void);

#endif /* SUBSYSTEM5_H */

/*** end of file ***/
