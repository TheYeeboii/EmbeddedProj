# Buddy 5: Ultrasonic Sensor and Obstacle Avoidance Flow

Buddy 5 consists of the HC-SR04+ ultrasonic sensor and its scanning servo.
The sensor module is implemented in `firmware/app_program/buddy5_ultrasonic.c`.
The higher-level obstacle-avoidance sequence is implemented by `sensor_task()`
in `firmware/app_program/app_main.c`.

## Normal forward measurement

1. At startup, `buddy5_init()` configures GP17 as the Trigger output, GP16 as
   the Echo input, and GP15 as the servo PWM output.
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

## Left/right decision scan

1. The servo moves to the configured first-side angle (normally 45 degrees).
2. The code waits for the configured side-scan dwell time.
3. `buddy5_sample_cm()` takes one temporary scan reading and stores it in
   `scan_45_cm`.
4. The servo moves to the configured second-side angle (normally 135 degrees).
5. After the same dwell time, another temporary reading is stored in
   `scan_135_cm`.
6. These two values are compared.  The larger valid distance selects the side
   used for the outward rover turn.
7. `buddy5_sample_cm()` deliberately does not overwrite `g_last`, so these
   side values never replace the dashboard ultrasonic distance.
8. If both side readings time out, the rover remains stopped.

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
