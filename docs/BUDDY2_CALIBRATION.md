# Buddy 2 Motion Calibration

## Current implementation

`buddy2_motion.c` provides signed motor control, encoder snapshots, speed and distance estimates, fixed-period control updates, distance moves, encoder-based turns, and a bounded PI/PID-style speed correction.

The current defaults are starting values only:

| Parameter | Default | Replace with measured value |
|---|---:|---|
| Wheel diameter | 65.0 mm | Measure loaded wheel diameter |
| Track width | 150.0 mm | Measure center-to-center wheel spacing |
| Left ticks/revolution | 20 | Count one wheel revolution |
| Right ticks/revolution | 20 | Count one wheel revolution |
| PWM limit | 32767 | Signed API limit; increase only if the API type is widened |
| PWM deadband | 0 | Minimum PWM that reliably moves each wheel |

The current hardware exposes one encoder channel per wheel. Direction is inferred from the commanded motor direction. Add encoder B channels for direction-independent quadrature feedback.

## Calibration procedure

1. Lift the rover so the wheels are safe to rotate.
2. Mark each wheel and rotate it exactly 10 revolutions by hand or at low PWM.
3. Record left and right rising-edge tick counts and divide by 10.
4. Measure a straight commanded move over at least 500 mm. Update effective wheel diameter or ticks/revolution until measured distance is within the target tolerance.
5. Command a 360-degree in-place turn. Measure the actual angle and update `track_width_mm` using:

   `new_track_width = old_track_width * actual_angle / commanded_angle`

6. Measure the minimum PWM that starts each wheel moving and set the larger value as `pwm_deadband`.
7. Repeat straight and turn tests at low, medium, and high PWM.

## Tuning and evaluation record

Record these values during bench testing before relying on autonomous motion:

| Test | Command | Measured | Error | Notes |
|---|---|---:|---:|---|
| Straight distance | 500 mm |  |  |  |
| Straight distance | 1000 mm |  |  |  |
| Left turn | 90 deg |  |  |  |
| Right turn | 90 deg |  |  |  |
| Full turn | 360 deg |  |  |  |

The motion controller currently uses a conservative proportional-plus-integral correction. Tune the output response only after ticks/revolution, wheel diameter, track width, and deadband are measured. The motor command watchdog stops active motion after one second without a fresh command.


## micro T-Kernel notes

- Code: `firmware/app_program/buddy2_motion.c`. The test run is `buddy2_motion_test.c`, started with the **GP20** button.
- PWM units: `32767` now means **100 % duty**. The PWM counter wrap is set explicitly (10 kHz), so the old "PWM limit = 50 %" note no longer applies.
- `buddy2_update()` runs every 20 ms in its own task, woken by a kernel cyclic handler.
