# Adaptive Code 39 reader

## Format and boundaries

Supported frames are `*LETTER*`, with exactly one uppercase A–Z payload.
The asterisks are framing markers, not returned characters. Each character has
nine alternating black/white widths (three wide, six narrow), followed by a
narrow inter-character white gap. There are 29 measured elements per frame.
Both bars and spaces carry information. Digit, multi-character, checksum and
extended Code 39 formats are outside this reader's scope.

`code39.c` is independent of the board, RTOS and motors. It estimates narrow
and wide widths separately for each character, validates their consistency,
and accepts nominal 2:1 and 3:1 ratios with measurement tolerance. Accepted
elements must stay within 25% of their class mean, narrow character means
within a factor of 1.3, and gaps within 30% of the frame's narrow mean.
Both quiet zones must cover at least ten of the largest character narrow widths.
Mixed marker orientation, incomplete frames and invalid elements are rejected.
Reverse print order is canonicalized before decoding; the letter is unchanged.

The two supplied photo patterns decode to F and Y. A/Z are supported too, but
no letter is assigned a turn yet. The photos are not physical IR trace evidence.
The single-letter format has no checksum: framing and width checks reduce
false reads but cannot prove every optically corrupted letter is correct.

## Acquisition

`buddy3_barcode.c` captures black/white transition widths from encoder travel,
with midpoint edge estimates and bounded arrays of 29 widths and timestamps.
`buddy3_ir.c` supplies normalized side-sensor readings with 400/600 hysteresis,
independent wheel travel, and steering/encoder validity. There is no allocation,
console output or MQTT publishing in the capture/decoder path.

Sensor forward travel is estimated as:

```text
sensor_delta = (left_delta + right_delta)/2
             - offset_left * (right_delta-left_delta)/wheel_track_width
```

Measure offset sideways from the midpoint between the wheels to the sensing
spot. Left is positive; right is negative. This compensates small corrections,
not diagonal traversal. Net heading changes above about 10 degrees, reverse
wheel movement, encoder faults, crossing/corner/recovery phases and stopping
invalidate an active read. Flipped prints are supported while driving forward;
driving backward through a barcode is not supported.

Stationary flicker adds no widths. A sample jump above a quarter of the
configured minimum element, a 1.5-second stall, a 30-second frame or a missing
edge over 10 metres aborts capture. Acceptance requires the entire closing
marker and trailing quiet zone. A second read needs another complete frame.

## Calibration and configuration

Measure wheel diameter, wheel track and encoder edges per wheel revolution.
Current defaults of 65 mm and 387 edges/revolution imply about 0.53 mm per edge.
The smallest allowed element must span at least four edges on BOTH wheels;
more resolution may be needed with sensor offset and task scheduling jitter.
The actual barcode must also be sampled at least four times per narrow width.
Do not change encoder calibration just to bypass the resolution checks.

While stopped:

```text
barcode_config MIN_ELEMENT_MM OFFSET_LEFT_MM
barcode_mode 1
```

JSON uses `min_element_mm`, replacing the obsolete `cell_mm` field:

```json
{"cmd":"barcode_config","min_element_mm":3,"offset_left_mm":-60}
```

The numbers above are examples, not measured rover settings. A minimum of 3 mm
allows adaptive detection of wider narrow elements; it does not force a 3 mm
printed cell. Uniform print scaling needs no new exact-width setting provided
it remains above this resolution bound and below the capture limits. There is
no fixed 141 mm length assumption. A smaller print may need better encoder
resolution, slower sampling travel, or different hardware.

Settings are RAM-only. For button-only use, set measured defaults via
`BUDDY3_BARCODE_MIN_ELEMENT_MM` and `BUDDY3_BARCODE_OFFSET_LEFT_MM`.
The minimum defaults to zero, keeping unconfigured reading disabled.
A zero minimum disables reading. Use USB, not Wi-Fi, for initial calibration.

## Results and limits

The controller prints `Decoded: F (forward)` or `Decoded: F (reverse)`, measured
scale/ratio, timestamp and a cumulative dropped-event count. Rejections include
a capture or format reason. The one-event handoff preserves the first unread
event and reports subsequent drops. Stop/start clears pending reads. Mode 0
consumes without printing; mode 1 observes. Neither changes motor ownership.

The dashboard's existing barcode snapshot displays ASCII letters instead of
numeric direction labels. It is transient; durable MQTT barcode events/history,
raw left/right encoder trace export and physical qualification remain separate
work in issues #12, #15 and #16. USB logs are currently the reliable result view.

## Software verification

From the repository root:

```sh
sh tests/run_barcode_tests.sh
```

Tests cover A–Z, five scales, 2:1/3:1 ratios, bounded width noise, photo-derived
F/Y patterns, both print orientations, malformed markers/widths/gaps/quiet
zones, changing speed, stationary noise and abort guards. The actual IR adapter
is exercised with simulated ADC/encoder values. These are software tests, not
physical sensor or track qualification.

Code 39 encoding reference:
[ZXing Code39Reader](https://github.com/zxing/zxing/blob/master/core/src/main/java/com/google/zxing/oned/Code39Reader.java).
