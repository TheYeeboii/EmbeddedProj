/**
 * @file    subsystem3.h
 * @brief   Subsystem 3 (line following and barcode): public interface.
 *
 * Owner and pins: TEAM.md.  usermain() calls subsystem3_init() first (no
 * tasks running yet), then subsystem3_start().  Other subsystems use the
 * functions declared in the headers included below.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "buddy3_ir.h"
#include "buddy3_barcode.h"
#include "barcode_tuning.h"
#include "robot_steering_trace.h"
#include "robot_barcode_trace.h"
#include "robot_ir_stream.h"

/** @brief Initialise this subsystem's hardware and kernel objects. */
void subsystem3_init (void);

/** @brief Create and start this subsystem's task(s). */
void subsystem3_start (void);

#endif /* SUBSYSTEM3_H */

/*** end of file ***/
