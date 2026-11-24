/**
 * @file    subsystem2.h
 * @brief   Subsystem 2 (motion control): public interface.
 *
 * Owner and pins: TEAM.md.  usermain() calls subsystem2_init() first (no
 * tasks running yet), then subsystem2_start().  Other subsystems use the
 * functions declared in the headers included below.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "buddy2_motion.h"
#include "robot_action_log.h"

/** @brief Initialise this subsystem's hardware and kernel objects. */
void subsystem2_init (void);

/** @brief Create and start this subsystem's task(s). */
void subsystem2_start (void);

#endif /* SUBSYSTEM2_H */

/*** end of file ***/
