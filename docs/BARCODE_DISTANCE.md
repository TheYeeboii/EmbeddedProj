# Distance-based barcode reader

## Supported physical format

This reader retains the repository's CUSTOM format, not Code 39/EAN/QR:

- Two or more white cells before a frame.
- 16 equal-distance bit cells, MSB first: start byte 0xA5 (10100101),
  followed by command 1 (left), 2 (right), or 3 (straight).
- Black is 1; white is 0. Adjacent identical bits form a wider bar/space.
- A fresh white lead-in is required before another frame.

There is no checksum in the existing format. A valid start byte and command
filter mistakes but do not provide strong error detection. Do not assume an
ordinary printed product barcode is compatible.

## What changed

IR observations still occur every 10 ms. Bit samples now occur at the centres
of measured spatial cells, not once per task invocation. Speed changes do not
change the intended cell width. Repeated observations while stopped do not
advance the frame. The decoder rejects reverse motion, steps larger than one
quarter cell, stationary frames after 1500 ms, invalid headers/commands, and
samples during recovery/turning.

Differential wheel travel estimates the side sensor's local forward travel:

sensor_delta = (left_delta + right_delta)/2
               - offset_left * (right_delta-left_delta)/wheel_track_width

Offset is measured sideways from the midpoint BETWEEN THE WHEELS to the barcode
sensor. Left is positive; right is negative. This compensates small heading
corrections, not arbitrary diagonal traversal. More than about 10 degrees of
net heading change during a frame invalidates it. Mount/read the strip along a
straight section; this is not an absolute position system.

## Hardware/calibration prerequisite

Main's Buddy 2 update now reads each encoder independently and reports
left/right encoder faults. The reader rejects samples while either fault is
reported; a barcode move interrupted by a fault stops without resuming tracking.
The obsolete mirror-right workaround has been removed from pin_config.h.

Wheel diameter, track width and actual rising-edge counts PER WHEEL REVOLUTION
must be measured in buddy2_motion.c. Main now supplies 387 edges/revolution,
measured on the left wheel, and a 65 mm default diameter: about 0.53 mm per edge.
At that resolution, 5-10 mm cells pass the minimum resolution check, but the
right wheel calibration and actual frame sampling still need track validation.
The reader requires at least four encoder edges per cell;
more may be needed for sensor offset and sampling delays. Do not fake the
calibration numbers to pass the check. Repair/increase encoder resolution,
use physically wider cells, or choose a different validated sensing strategy.

## Configuration and read-only test

BUDDY3_BARCODE_CELL_MM defaults to zero (not measured), so no frame is accepted
on an unconfigured board. Set the measured defaults in buddy3_barcode.h to
avoid entering them after every reboot. Offset defaults to zero and must also
be measured for compensation. Alternatively, while stopped, send:

```text
barcode_config CELL_MM OFFSET_LEFT_MM
```

Example only: barcode_config 50 -60 means 50 mm per bit and sensor 60 mm to
the right of wheel-centre. These are not measurements of this robot.
Settings are RAM-only. Configuration is rejected while line following.
JSON: {"cmd":"barcode_config","cell_mm":50,"offset_left_mm":-60}.

After calibration succeeds, GP21 starts the existing gentle steering test.
Each decode logs [barcode] decoded command=N (distance frame). During the
isolated steering tests, decoded route actions stay disabled until explicitly
enabled in the movement integration. Keep the side sensor over the barcode
strip, and test every command separately at two steady speeds, then with
gentle corrections. The 15-second steering test limit still applies.

## Host regression test

```sh
cc -std=c11 -Wall -Wextra -Werror -I firmware/app_program tests/test_buddy3_barcode.c firmware/app_program/buddy3_barcode.c -o /tmp/test_buddy3_barcode
/tmp/test_buddy3_barcode
```

Software tests do not validate wheel calibration, printed barcode format,
optical detection, or the physical track.
