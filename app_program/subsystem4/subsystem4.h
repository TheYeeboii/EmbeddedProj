/**
 * @file    subsystem4.h
 * @brief   Subsystem 4 (IMU): public interface.
 *
 * Owner and pins: TEAM.md.  usermain() calls subsystem4_init() first (no
 * tasks running yet), then subsystem4_start().  Other subsystems use the
 * functions declared in the headers included below.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "buddy4_imu.h"

/** @brief Initialise this subsystem's hardware and kernel objects. */
void subsystem4_init (void);

/** @brief Create and start this subsystem's task(s). */
void subsystem4_start (void);

#endif /* SUBSYSTEM4_H */

/*** end of file ***/
