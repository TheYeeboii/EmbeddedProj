/**
 * @file    subsystem1.h
 * @brief   Subsystem 1 (Wi-Fi, MQTT bridge, telemetry): public interface.
 *
 * Owner and pins: TEAM.md.  usermain() calls subsystem1_init() first (no
 * tasks running yet), then subsystem1_start().  Other subsystems use the
 * functions declared in the headers included below.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "comms.h"
#include "robot_telemetry.h"
#include "robot_wire.h"
#include "robot_diagnostics_mqtt.h"

/** @brief Initialise this subsystem's hardware and kernel objects. */
void subsystem1_init (void);

/** @brief Create and start this subsystem's task(s). */
void subsystem1_start (void);

#endif /* SUBSYSTEM1_H */

/*** end of file ***/
