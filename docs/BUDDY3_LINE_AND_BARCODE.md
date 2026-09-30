# Buddy 3 Line and Barcode Setup

## Line following

Buddy 3 samples the left, center, and right IR sensors every 10 ms. The left and center sensors are used for line detection and position estimation; the right sensor is reserved for barcode reading, matching the hardware layout. It normalizes readings to `0..1000`, detects two-sensor junctions, and applies proportional/derivative steering correction through Buddy 2 signed motor control.

Enable it from the dashboard or MQTT:

```json
{"cmd":"line_follow"}
```

Stop it with:

```json
{"cmd":"line_stop"}
```

The default calibration matches the current module response: the normal floor produces low raw values and is not inverted. Set `line_is_dark` to `true` only if your measured black-line response is lower than the floor response.

## IR calibration

Record raw readings with the sensors over the floor and over the line. Set each sensor's `minimum` and `maximum` in `buddy3_ir.c`, then verify that the line produces normalized values above the line threshold. Tune `proportional_gain`, `derivative_gain`, and `base_pwm` at low speed first.

## Barcode protocol

The current decoder uses a compact timing-independent digital frame sampled by the right barcode sensor:

```text
8-bit start 0xA5, followed by 8-bit command
```

Commands are:

- `1`: turn left
- `2`: turn right
- `3`: go straight

The barcode must produce one stable binary sample per 10 ms. If the physical barcode uses variable-width bars, the decoder must be extended with measured bar-width timing before relying on it.

Decoded commands are translated into Buddy 2 navigation actions: 90-degree left/right turns or a 300 mm straight move. Validate these values on the actual track.


## micro T-Kernel notes

- Code: `firmware/app_program/buddy3_ir.c`. `buddy3_update()` runs every 10 ms in the `line` task, woken by a cyclic handler.
- Start/stop line following with the **GP21** button, or with `line_follow` / `line_stop` from the dashboard.
- Settings: Kp 9, Kd 2.5, base 12000 (about 37 % duty, since 32767 now means 100 %). The old base of 12000 gave only about 18 % duty, which was too weak to move the car.
- If the line is not under the sensors, the console prints `[line] no line under the sensors ...` once and waits.
- After a barcode action (turn or 300 mm straight) the controller resumes line following automatically (`robot_controller.c`).

## Sensor layout (as built)

- ADC0 and ADC1 are the two **front** sensors on the black bracket. ADC2 is on the white arm beside the car and reads the **barcodes**.
- The track line is **18 mm** wide, narrower than the gap between the front sensors, so normally **both front sensors see floor** (the line is between them). The black start area makes all three read black.
- Steering compares the two front sensors: only the left one on black means steer left, only the right one means steer right. Both on black is a crossing line or the start box, so drive straight.
- Safety stops: no front sensor touches the line for 6 s, or all three on black for 3 s (lifted, or a dark desk).
- While line following, the console prints a `[run]` line at every sensor change with the steering decision. Compare it with what the car actually did.
- Current setting: `BUDDY3_FRONT_LEFT 0`, `BUDDY3_FRONT_RIGHT 1`. If `[run]` says "steer LEFT" and the car turns right, swap them.
