# Buddy 3 Line and Barcode Setup

## Focused USB console logging

Build from `firmware/build_make` with `BUDDY3_DEBUG=1` to suppress periodic
status/encoder summaries, console JSON, successful sonar ping messages and the
avoidance telemetry summary. Buddy 3 `[sensor]`/`[line]` messages, controller
events, startup information, errors and safety messages remain visible.
MQTT publication and sensor/control tasks continue normally.

Buddy 3 also prints a numerical snapshot approximately once per second from
the telemetry task (not the steering loop):

```text
[buddy3] raw L=150 R=180 side=130 | norm L=36 R=43 side=31 | ADC L=1 R=0 side=2
```

Values above are illustrative. L/R follow the rover's configured left/right
mapping; the ADC labels identify which sensor calibration entry to adjust.
Keep the rover stationary with line following off to collect floor and black
readings at the normal mounting height. Raw values are 0..4095; normalized
values are 0..1000. A one-second snapshot is for calibration, not measurement
of steering latency or fast barcode transitions.

```sh
make CONSOLE=usb_cdc PICO_SDK_PATH="$PICO_SDK_PATH" BUDDY3_DEBUG=1 -j8
```

Reflash the generated UF2 to use this mode. Rebuild with `BUDDY3_DEBUG=0`
(the default) to restore normal output. Changing this option invalidates the
build profile so old objects are rebuilt. Keep any Wi-Fi build options you
normally use. Reduced printing is a diagnostic aid, not proof that logging
caused a steering delay. Sonar faults can still produce repeated messages.

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
