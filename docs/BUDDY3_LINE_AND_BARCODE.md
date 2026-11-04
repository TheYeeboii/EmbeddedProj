# Buddy 3 line and barcode setup

## GP21 steering tests

The default is stage 1: 35% base and 27 percentage-point curve power. It slows
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

Reading is disabled until the minimum resolvable element and sensor offset
are configured. GP21 starts/stops tracking with the reader. Valid letters and
rejected frames are logged from the controller task, outside sensor capture.
Letter-to-turn mapping remains disabled until the demo mapping and physical
qualification are available; `barcode_mode 2` is rejected.
