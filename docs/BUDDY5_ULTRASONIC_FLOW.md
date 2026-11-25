# Buddy 5: Ultrasonic Sensor and Obstacle Avoidance Flow

Buddy 5 consists of the HC-SR04+ ultrasonic sensor and its scanning servo.
The sensor module is implemented in `app_program/subsystem5/buddy5_ultrasonic.c`.
The higher-level obstacle-avoidance sequence is implemented by
`buddy5_avoidance_task()` in `app_program/subsystem5/buddy5_avoidance.c`.

## Normal forward measurement

1. At startup, `buddy5_init()` configures GP0 as the Trigger output, GP1 as
   the Echo input (both on Grove 1), and GP15 as the servo PWM output.
2. The servo is set to 90 degrees, so the ultrasonic sensor faces forward.
3. Every sensor-task cycle, `buddy5_measure_cm(now_ms)` triggers the sensor.
4. The Trigger pin is held HIGH for 10 microseconds, then returned LOW.
5. The code waits for Echo to rise, then measures how long Echo stays HIGH.
6. Distance is calculated as `echo_pulse_us / 58` in centimetres.
7. If Echo never rises, or stays HIGH for more than 40 ms, the measurement is
   `BUDDY5_NO_ECHO` (`0 cm`) and a timeout reason is printed.
8. A normal forward measurement is saved in `g_last`, which is the value sent
   to telemetry as **Ultrasonic distance**.

## Obstacle trigger

1. While the rover is line following (or driving forward), the code watches
   the saved front-facing distance.
2. A valid front distance of 15 cm or less (`APP_OBSTACLE_STOP_CM`) starts
   avoidance.
3. The controller stops the motors and line following, and remembers that
   the run came from line following so it can hand control back at the end.
4. The rover backs away 80 mm (`APP_BACKUP_MM`) to make room for the sweep
   and the turn.

Every avoidance move runs at a fixed 60 % (`APP_AVOID_SPEED_PCT`), not the
dashboard speed limit.  A dashboard command (stop included) cancels the run.

The route is a **fixed sequence of moves** whose lengths are set by the
constants at the top of `buddy5_avoidance.c` (the obstacle is a known box).
The sweep only decides which way round to go, how far away the box is and,
if its edge is further out than expected, how far to step sideways.

## Obstacle width sweep and side choice

1. The servo sweeps 30, 35, ... 150 degrees (90 is straight ahead): 500 ms
   to reach 30 degrees, then 100 ms per 5-degree step.  At each angle it
   pings up to three times (60 ms apart) and keeps the nearest of two valid
   echoes.  These pings never change the dashboard ultrasonic distance.
   The sweep takes about 5 seconds.
2. Each reading is split into a forward part (`d * cos(off-axis angle)`) and
   a sideways part (`d * sin(off-axis angle)`).
3. The obstacle's **front face** is the nearest forward part of the readings
   within 10 degrees of straight ahead.  With no echo there, the box is
   assumed to be 250 mm away (`APP_FALLBACK_FACE_MM`).
4. Walking outward on each side, a reading stays on the obstacle while it
   has an echo and its forward part is within 50 mm of the face
   (`APP_FACE_TOLERANCE_MM`).  A reading much **nearer** than the face is
   something else beside the obstacle: that side is not used.  The HC-SR04
   beam still echoes off the corner one step past it
   (`APP_SWEEP_BEAM_STEPS`), so the edge is taken one step inside the last
   echo on the face.
5. **Width** = left edge + right edge, shown on the dashboard (Buddy 5 tab),
   with "≥" if one edge was not found.  If the dashboard reads too wide for a
   box of known width, raise `APP_SWEEP_BEAM_STEPS`; too narrow, lower it.
6. **Side choice:** a side is usable if its edge was found, nothing is
   nearer than the box there, and
   `edge + rover reach on the passing side + 120 mm gap` is at most 560 mm.
   Both usable: the side with the **nearer edge**; a tie goes left, which
   keeps the barcode arm (left side, 75 mm reach against 55 mm on the
   right) away from the box.
7. **Neither side usable:** the side with more room at 45/135 degrees.  If
   both of those read under 20 cm (`APP_MIN_CLEARANCE_CM`), the rover stays
   stopped ("both sides under 20 cm").
8. **Sidestep:** `APP_SIDESTEP_MM` (400 mm), or more if the chosen edge
   needs it (edge + reach + gap).

## Route round the obstacle and back onto the line

**Heading.**  Every turn is measured by the **wheel encoders**, relative to
the heading the rover had on the line: the turn is the two wheels'
difference over the track width.  On the spot the wheels scrub, so a pivot
counts only 85 % of that (`APP_PIVOT_WHEEL_SLIP`).  Each turn out is
matched by a turn back, so an error in that factor largely cancels.  The
compass is not used for control (on the track it dropped out and the motor
current upset it during pivots); its change over the route is only printed
at the end, next to the encoder heading, for comparison.

**Pivots** turn on the spot at `APP_PIVOT_FAST_PWM`, then
`APP_PIVOT_SLOW_PWM` for the last 20 degrees, and stop within 4 degrees of
the target or as soon as they overshoot it.

**Straight legs** hold their heading by **speeding up the wheel that lags**
(never slowing the other), so both wheels get at least
`APP_FORWARD_BASE_PWM`.  Slowing one wheel took the weaker right motor low
enough to stall, which Buddy 2 then reported as an encoder fault.

1. **Extra reverse (right side only).**  Pivoting right swings the
   left-front barcode IR arm toward the obstacle, so the rover backs away
   another 100 mm first (`APP_ARM_EXTRA_BACKUP_MM`).
2. **Pivot out** 90 degrees away from the line.
3. **Sidestep** the planned distance.  The front sonar watches this leg;
   15 cm or less stops the run.
4. **Pivot back** to the line's heading.
5. **Pass** a fixed distance:
   `sonar ahead of the axle (100 mm) + distance to the box + APP_BOX_DEPTH_MM
   (250 mm) + APP_PASS_TAIL_MM (400 mm)`, plus 100 mm going right (the extra
   reverse).  The axle then ends well past the box's far side, so the tail
   and cable clear it in the next pivot.  The front sonar watches this leg.
6. **Pivot in** 90 degrees toward the line.  **The front sonar keeps
   watching** while turning back and on the way back: the box within 25 cm
   (turning) or 20 cm (driving back) means the rover is not past it yet.
   It turns back along the line, drives 200 mm further
   (`APP_PASS_RETRY_MM`) and tries again, at most 3 times, then stops.
7. **Return** toward the line until a front IR sensor crosses it.
   - Black only counts once the leg has come within 150 mm of the sidestep
     distance (`APP_RETURN_EARLY_MM`); black earlier is another line on the
     mat.
   - Crossing the line, black ends within 50 mm.  Black that lasts longer
     is a line running the rover's way, such as a junction's branch: it is
     ignored until the sensors are on white again.
   - No line within the sidestep + 250 mm (`APP_RETURN_EXTRA_MM`) stops the
     run ("line not found on the way back").
8. **Align.**  Drive on `APP_ALIGN_FORWARD_MM` (150 mm, the front sensors'
   distance ahead of the axle) less the black already crossed, so the axle
   is on the line, then pivot back to the line's heading.
9. **Hand back or search.**  If a front sensor is on the line when that
   pivot ends, line following takes over.  Otherwise the rover searches at
   once: 35 degrees toward where it came from, then 35 degrees the other
   way (`APP_SEARCH_SWEEP_DEG`), handing back the moment a sensor sees
   black.  While searching the task runs every 10 ms instead of 100 ms
   (`APP_SEARCH_PERIOD_MS`), so it stops on the line rather than a whole
   step past it.  Not found: it stops ("line not found after searching both
   ways").

The dashboard shows the steps as *reversing*, *scanning*, *turning*,
*forward* (sidestep), *passing* and *rejoining*.

**Limit of the fixed route.**  The rover assumes the box is no longer than
`APP_BOX_DEPTH_MM`.  A longer box whose far end lies just behind where the
pass ends is not in the front sonar's view when the rover turns back, and
the last pivot on the line can swing the tail into it.  Keep
`APP_BOX_DEPTH_MM` at least the real box's length.

### Safety stops

The run stops (motors off, no line following) and reports why on the USB
console and in the avoidance status when:

- an encoder faults (after a leg's first second), or the wheels move under
  15 mm in a second while driving or turning (stuck),
- a pivot takes over 6 s or a leg over 15 s,
- the sidestep or the pass is blocked, the box is still there after three
  retries, or the line is not found.

## Tuning on the floor

| Constant | Now | Meaning |
|---|---|---|
| `APP_SIDESTEP_MM` | 400 mm | Sideways leg (grows if the measured edge needs more). |
| `APP_BOX_DEPTH_MM` | 250 mm | Box length along the line; at least the real box. |
| `APP_PASS_TAIL_MM` | 400 mm | How far past the box's far side the axle goes before turning back. |
| `APP_ALIGN_FORWARD_MM` | 150 mm | Front IR sensors to axle: drive-on after crossing the line. |
| `APP_PIVOT_WHEEL_SLIP` | 0.85 | Raise if 90-degree pivots come out short, lower if long. |
| `APP_ROVER_LEFT_REACH_MM` / `_RIGHT_REACH_MM` | 75 / 55 mm | Centreline to the outermost part on each side. |
| `APP_SIDE_GAP_MM` | 120 mm | Gap kept from the box edge when the sidestep grows. |
| `APP_SWEEP_BEAM_STEPS` | 1 | Beam correction for the width (see above). |
| `APP_PIVOT_FAST_PWM` / `_SLOW_PWM` | 14000 / 11000 | Pivot drive; raise if a pivot stalls. |

## Telemetry rule

Only `buddy5_measure_cm()` updates telemetry.  It represents the last normal
front measurement.  `buddy5_sample_with_result_cm()` is used only for
avoidance pings (the width sweep) and never replaces the telemetry
ultrasonic-distance value.
