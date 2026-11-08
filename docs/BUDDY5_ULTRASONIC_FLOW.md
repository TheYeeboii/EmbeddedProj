# Buddy 5: Ultrasonic Sensor and Obstacle Avoidance Flow

Buddy 5 consists of the HC-SR04+ ultrasonic sensor and its scanning servo.
The sensor module is implemented in `firmware/app_program/buddy5_ultrasonic.c`.
The higher-level obstacle-avoidance sequence is implemented by
`buddy5_avoidance_task()` in `firmware/app_program/buddy5_avoidance.c`.

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

1. While the rover is actively driving forward, line following, or completing
   a forward distance move, the code watches the saved front-facing distance.
2. A valid front distance of 10 cm or less starts avoidance.
3. The controller stops the motors and stops line following so line-following
   motor commands cannot override avoidance commands.
4. The displayed telemetry value remains the last forward-facing reading.

## Obstacle width sweep and side choice

1. After backing away, the servo sweeps 30, 45, ... 150 degrees (90 is
   straight ahead).  It waits 500 ms before the first angle and 200 ms
   between the 15-degree steps.
2. At each angle `buddy5_sample_with_result_cm()` pings up to four times and
   keeps the nearest of up to two valid echoes.  These pings never change
   the dashboard ultrasonic distance.
3. Each reading is split into a forward part (`d * cos(off-axis angle)`) and
   a sideways part (`d * sin(off-axis angle)`).
4. The obstacle's front face is the nearest forward part of the 75, 90 and
   105-degree readings.  With no echo at any of them the rover stays
   stopped.
5. Walking outward from the centre on each side, a reading stays on the
   obstacle while it has an echo and its forward part is within 100 mm of
   the face.  The sideways part of the last reading on the face is that
   side's edge.  If the obstacle has not ended by 30 or 150 degrees, that
   side is unusable.
6. Each side's diagonal leg is sized so the rover clears the edge:
   `diagonal = (edge + rover reach on the passing side + margin - arc shift) / sin(turn-out angle)`,
   at least 150 mm.  The turn-out angle is 30 degrees on the left and
   60 degrees on the right (see step 9).  A side needing more than 600 mm
   is unusable.
7. The rover goes round the usable side with the shorter diagonal (the
   narrower way round).
8. If there is no echo ahead, or neither side is usable (often because
   something else near the obstacle hides its edges), it falls back to the
   fixed route used before the sweep: a 280 mm diagonal toward the side
   with the larger 45/135-degree reading.  It only stays stopped if one of
   those readings has no echo or both are under 20 cm.
9. Before a **right** turn-out the rover backs away another 100 mm
   (`APP_AVOID_ARM_SIDE_EXTRA_REVERSE_MM`): the right arc swings the
   left-front barcode IR arm toward the obstacle.  For the same reason a
   right turn-out turns 60 degrees (`APP_AVOID_ARM_SIDE_TURN_DEG`) instead
   of 30 before the diagonal starts; the turn-in mirrors it.
10. **Stuck check.**  While turning, driving a leg or recovering the
   heading, the rover checks every second (`APP_AVOID_STALL_WINDOW_MS`)
   how far its wheels moved.  Under 15 mm (`APP_AVOID_STALL_MIN_MM`) means
   it is pushing against something, often the barcode arm caught on the
   obstacle.  It then stops, backs away 80 mm and runs the width sweep
   again from where it is.  After 3 such rescans in one avoidance
   (`APP_AVOID_MAX_STALL_RETRIES`) it stops and reports blocked.
11. The console prints every sweep reading, the measured width and both
   diagonals.  The 45 and 135-degree readings still go to the dashboard as
   the right/left scan values.

Tune these in `buddy5_avoidance.c` on the floor:

| Constant | Meaning |
|---|---|
| `APP_ROVER_LEFT_EXTENT_MM` / `_RIGHT_EXTENT_MM` | Sonar centre to the outermost part on each side (the barcode arm makes the left wider). |
| `APP_AVOID_SIDE_MARGIN_MM` | Gap kept between the body and the edge. |
| `APP_AVOID_ARCS_LATERAL_MM` | Sideways shift of the turn-out plus recovery arcs. |
| `APP_SWEEP_FACE_TOLERANCE_MM` | How much deeper a reading may be and still count as the obstacle. |
| `APP_AVOID_DIAGONAL_MIN_MM` / `_MAX_MM` | Limits on the diagonal legs. |
| `APP_AVOID_ARM_SIDE_TURN_DEG` | Right (barcode-arm side) turn-out angle; change `APP_AVOID_ARM_SIDE_TURN_SIN` with it. |

The HC-SR04 beam is about 15 degrees wide, so edges are found to roughly
one sweep step.  Smooth faces seen at a steep angle may not echo; the
obstacle-side monitors during the turn-out and first diagonal still extend
the route if the rover passes too close.

## Outward turn and custom forward move

1. The rover commands a left or right turn using Buddy 2 motor/encoder code.
2. A five-second turn timeout stops the rover if that turn never completes.
3. Once the outward turn completes, Buddy 5 starts its own forward routine:
   `app_forward_start()`.
4. This forward routine does not call `buddy2_move_forward()`.
5. It saves the current left and right encoder distances, then commands both
   motors forward using the configured drive PWM.
6. On each sensor cycle, `app_forward_update()` compares the distance travelled
   by the left and right wheels.
7. If the left wheel is ahead, its PWM is reduced and the right PWM is raised.
   If the right wheel is ahead, the correction is reversed.
8. The routine stops only after both wheel encoders reach the requested
   distance, within the configured tolerance.
9. A newly detected front obstacle immediately stops this forward move and
   restarts the side-scan sequence.

## Opposite-side check and final forward move

1. After the outward move completes, the servo scans the side opposite the
   original outward turn.
2. That reading is temporary and does not change telemetry.
3. The rover turns back by the same configured turn angle to restore its
   original heading.
4. If the opposite-side reading is valid and farther than the obstacle limit,
   Buddy 5 starts the same custom encoder-synchronised forward routine for the
   final configured distance.
5. If the opposite-side reading is invalid or too close, the rover remains
   stopped at its restored heading.
6. After the final move completes, the servo returns to 90 degrees and the
   controller returns to `idle`.

## Telemetry rule

Only `buddy5_measure_cm()` updates telemetry.  It represents the last normal
front measurement.  `buddy5_sample_cm()` is used only for temporary avoidance
decisions and never replaces the telemetry ultrasonic-distance value.
