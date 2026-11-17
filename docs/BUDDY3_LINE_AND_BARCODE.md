# Buddy 3 line and barcode setup

## GP21 steering tests

The default is stage 1: 60% base and 27 percentage-point curve power. It slows
the inner wheel and speeds up the outer wheel without reversing either. GP21
starts/stops it; there is no fixed duration cutoff.
Use [STEERING_TESTS.md](STEERING_TESTS.md) for staged bench/track tests.
Sensor ADC mapping remains left=1, right=0, barcode=2; verify physical wiring.

Both front sensors normally see white when straddling the line. Stages 1..3
do not automatically spin after a white timeout. During ordinary tracking,
persistent both-black now commits to bounded, encoder-guided straight crossing.
See [CROSS_JUNCTIONS.md](CROSS_JUNCTIONS.md) for exit checks and limits.
Cross versus T-junction classification remains ambiguous with two sensors.
Stage 0 selects the old steering algorithm, not the new reader integration.

## Sensor calibration and logging

Build with BUDDY3_DEBUG=1 to reduce routine telemetry printing and show a
numerical sensor snapshot about once per second. Sensor values are normalized
to 0..1000. Read floor and black at the actual mounting height and adjust the
per-sensor endpoints/inversion in buddy3_ir.c. A 1-second debug snapshot cannot
capture barcode transitions.

The two front sensors steer; the side sensor is independent and reads barcodes.
Their use of the same 10 ms task does not mean each task tick is a barcode bit.

## Barcode reader

See [BARCODE_DISTANCE.md](BARCODE_DISTANCE.md) for adaptive Code 39 decoding,
sensor offset and encoder requirements, and [BARCODE_ACTIONS.md](BARCODE_ACTIONS.md)
for staged tests and issue status.

The supplied photos contain single-letter Code 39 frames `*F*` and `*Y*`.
The reader supports uppercase A–Z and either print orientation. It infers
narrow/wide widths per frame; no fixed barcode length or exact bit width is
required. The former `0xA5 + numeric command` decoder has been removed.

The reader is configured at boot (3 mm minimum element, 0 mm left offset;
`barcode_config` changes them). It decodes in two situations:

- **Line following** (dashboard Start or GP21), while tracking straight.
- **Manual forward driving** (hold W, or the drive form with both wheels
  forward), with no line following needed. The controller enables it with
  `buddy3_set_manual_barcode()` and the IR task restarts the reader at its
  next sample. Holding W keeps one scan going; turning (A/D), reversing,
  stopping or starting line following ends it.

Both use the same checks: healthy encoders, forward travel, and less than
about 10 degrees of heading change across the barcode. Drive straight past
the barcode so it passes under the side sensor; speed need not be steady
because widths are measured in encoder distance. Valid letters and rejected
frames are logged from the controller task, outside sensor capture.
Letter-to-turn mapping (`robot_controller.c`): a valid letter read while line
following sets the move at the next junction. `A` turns left, `C` turns right,
any other letter goes straight on, and the latest letter wins. At that
junction Buddy 3 stops instead of crossing, the controller drives
`ROBOT_JUNCTION_AXLE_MM` (front sensors to wheel axle, 100 mm until measured),
pivots 90 degrees and starts line following again. An encoder fault or a stop
ends the turn. Letters read while driving by hand only log.
