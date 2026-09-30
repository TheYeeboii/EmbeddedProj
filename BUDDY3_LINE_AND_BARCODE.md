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
