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
4. The rover backs away 80 mm (`APP_AVOID_REVERSE_DISTANCE_MM`) to make room
   for the sweep and the turn.

Every avoidance move runs at a fixed 60 % (`APP_AVOID_SPEED_PCT`), not the
dashboard speed limit.  A dashboard command (stop included) cancels the run.

## Obstacle width sweep and side choice

1. The servo sweeps 30, 35, ... 150 degrees (90 is straight ahead): 500 ms
   to reach 30 degrees, then 100 ms per 5-degree step.  At each angle it
   pings up to three times (60 ms apart) and keeps the nearest of two valid
   echoes.  These pings never change the dashboard ultrasonic distance.
   The sweep takes about 5 seconds.
2. Each reading is split into a forward part (`d * cos(off-axis angle)`) and
   a sideways part (`d * sin(off-axis angle)`).
3. The obstacle's front face is the nearest forward part of the readings
   within 10 degrees of straight ahead.  With no echo there, the fixed route
   of step 7 is used.
4. Walking outward on each side, a reading stays on the obstacle while it
   has an echo and its forward part is within 50 mm of the face, nearer or
   farther.  A reading much **nearer** than the face is something else
   beside the obstacle (on the track: things 23-30 cm away at 45 degrees
   made a 15 cm box read up to 34 cm): it ends the face for the width, but
   that side is not used for the route.  The
   HC-SR04 beam still echoes off the corner while it points one step past it
   (`APP_SWEEP_BEAM_STEPS`), so the edge is taken one step inside the last
   echo on the face.  If the obstacle has not ended by 30 or 150 degrees,
   that side is unusable.
5. **Width** = left edge + right edge, shown on the dashboard (Buddy 5 tab),
   with "≥" if one edge was not found.  To calibrate it, measure a box of
   known width: if the dashboard reads too wide, raise
   `APP_SWEEP_BEAM_STEPS`; too narrow, lower it.
6. **Side choice:** each side's sidestep is
   `edge + rover reach on the passing side + 120 mm margin` (at least 120 mm).
   The rover goes round the side with the **smaller** sidestep, i.e. the
   narrower way round.  A side needing more than 560 mm is unusable.  The
   barcode arm makes the rover's left reach (75 mm) bigger than its right
   (55 mm), which counts in that comparison.
7. If there is no echo ahead, or neither side is usable, it falls back to a
   fixed 300 mm sidestep toward the side with the larger 45/135-degree
   reading.  It stays stopped if one of those has no echo or both are under
   20 cm.

## Route round the obstacle and back onto the line

All turns are pivots on the spot measured with the Buddy 4 compass against
the heading the rover had on the line; the straight legs hold their heading
(a small PWM difference per tenth of a degree of error).

**Heading.**  The compass is the heading, but on the track it drops out and
the motors upset it during pivots.  Between readings the heading is carried
on from the wheel encoders (the wheels' difference over the track width;
on the spot only 85 % of that, `APP_PIVOT_WHEEL_SLIP`, as the wheels
scrub).  A compass reading more than 25 degrees from that estimate is a
glitch and is ignored, unless five in a row agree.  So a compass drop-out
no longer stops the run, and with no compass at all the turns are made on
the encoders alone.

**Position tracking.**  From the first pivot on, every 100 ms the rover adds
its centre travel (average of the two encoders) to its position, split by
the compass heading relative to the line: `x += d cos θ` along the line,
`y += d sin θ` sideways.  So it always knows how far it is from the line,
even when a leg's heading was a little off.  In pivots the wheels slip, but
pivots barely move the centre, and the direction always comes from the
compass.

1. **Extra reverse (right side only).**  Pivoting right swings the
   left-front barcode IR arm toward the obstacle, so the rover backs away
   another 100 mm first.
2. **Pivot out** 90 degrees away from the obstacle.  Pivots run fast, then
   slow for the last 20 degrees, and stop within 5 degrees of the target or
   as soon as they overshoot it (the next leg's heading hold removes the
   rest).
3. **Sidestep** until the tracked position is the planned distance from the
   line.  The front sonar watches this leg; 15 cm or less stops the run.
4. **Pivot back** to the line's heading.  The servo turns to look 70 degrees
   toward the obstacle (20 or 160 degrees) during this pivot.
5. **Pass.**  Drive along the line's heading, pinging the obstacle side
   every 100 ms.  A reading nearer than the original line + 100 mm is the
   obstacle.  Two such readings mean it is alongside; then five clear
   readings, at least 100 mm past its front face, mean it has ended (on the
   track a few missed echoes once ended a pass too early and the turn back
   pushed into the box); then the rover drives 400 mm more
   (`APP_PASS_TAIL_MM`) so the whole rover is past it.  **While the obstacle
   is alongside the rover keeps its planned gap from it:** nearer than
   planned steers away, farther steers back in (0.3 degrees per mm of
   error, at most 25 degrees), so the rover holds a set distance from it.
   The sideways distance allows for the rover's angle to the obstacle
   (from the heading): assuming it was parallel made a turned-away rover
   read the obstacle nearer and turn away further, and on the track it ran
   off 25 degrees.  If the side sonar never
   sees the obstacle within 300 mm of its face, the rover assumes it is
   350 mm deep.  The side reading shows on the dashboard as the scan
   left/right value while passing.  Still alongside after 1200 mm means a wall: it stops.
6. **Pivot in** 90 degrees toward the line; the servo faces forward again.
   **The front sonar keeps watching** while turning back and on the way
   back: the obstacle within 25 cm (turning) or 20 cm (driving back) means
   the rover is not past it yet.  It turns back along the line, drives
   200 mm further and tries again (at most 3 times, then it stops).
7. **Return** toward the line.  **Black is ignored until this leg:** the
   line continues right next to an obstacle that sits on it, and the rover
   starts on it, so only black seen on the way back counts.
   - The rover drives **across** the original line, so its black ends
     within 50 mm: that is the line, if the tracked position puts the front
     sensors within 150 mm of it.  A crossing farther out is another line
     on the mat, and the rover keeps going.  The axle is then 100 mm minus the
     black's length from it, which also corrects the tracked position.
   - Black that lasts longer is a line running the rover's way, such as a
     junction's branch beside the box (on the track the rover once took one
     for the line and followed it off the mat): it is ignored until the
     sensors are on white again.
   - The tracked position says the sensors are over the line but the IR saw
     nothing: drive 60 mm more for the IR to confirm it, and at least
     300 mm back in all, then trust the position.
   - No line and no position to trust within the sidestep + 250 mm stops
     the run.
8. **Align.**  Drive on until the axle is on the line (100 mm after the IR
   found it, or what the tracked position says is left), pivot back to the
   line's heading, and hand back to line following.
9. **Line search.**  If no front sensor touched the line during that last
   pivot, the rover may be beside it rather than on it (line following
   would then see white on both sides and drive straight off).  It sweeps
   35 degrees toward where the line should be, then 35 degrees the other
   way, and hands back the moment a sensor touches black.  Not found: it
   stops ("line not found after searching both ways").

The dashboard shows the steps as *reversing*, *scanning*, *turning*,
*forward* (sidestep), *passing* and *rejoining*.

### Safety stops

The run stops (motors off, no line following) and reports why on the USB
console and in the avoidance status when:

- an encoder faults, or the wheels move under 15 mm in a second while
  driving (stuck),
- a pivot takes over 6 s or a leg over 15 s,
- the sidestep is blocked, the obstacle is too long, or the line is not
  found on the way back.

## Tuning on the floor

| Constant | Meaning |
|---|---|
| `APP_ROVER_LEFT_EXTENT_MM` / `_RIGHT_EXTENT_MM` | Sonar centre to the outermost part on each side. |
| `APP_AVOID_SIDE_MARGIN_MM` | Gap kept between the body and the edge. |
| `APP_SWEEP_BEAM_STEPS` | Beam correction for the width (see above). |
| `APP_PASS_TAIL_MM` | How far past the obstacle's end before turning back. |
| `APP_PIVOT_FAST_PWM` / `_SLOW_PWM` | Pivot drive; raise if a pivot stalls. |
| `APP_ALIGN_FORWARD_MM` | Front sensors to axle. |

## Telemetry rule

Only `buddy5_measure_cm()` updates telemetry.  It represents the last normal
front measurement.  `buddy5_sample_with_result_cm()` is used only for
avoidance pings (width sweep, side watch) and never replaces the telemetry
ultrasonic-distance value.
