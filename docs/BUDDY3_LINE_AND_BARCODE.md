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

## Barcode reader and movement

See [BARCODE_DISTANCE.md](BARCODE_DISTANCE.md) for the decoder format,
distance/offset calibration, and encoder prerequisites.
See [BARCODE_ACTIONS.md](BARCODE_ACTIONS.md) for read-only and movement tests.

The software currently supports the project's custom fixed-cell format:
0xA5 followed by command 1 (left), 2 (right), or 3 (straight), MSB first.
A two-cell white lead-in precedes each frame. This is not Code 39/EAN/QR.
The physical track's printed barcode format is not yet confirmed.

Samples are assigned by measured sensor travel, with small-turn compensation
for the side sensor's lateral offset. Large turns, reverse motion, coarse
position jumps, stalled frames and recovery invalidate the read. Calibration
must provide sufficient resolution: at least four wheel encoder edges per cell.

Main's Buddy 2 update reads independent encoders at 387 edges/revolution and
reports encoder faults. These changes are preserved; faults invalidate reads
and prevent automatic resume of fault-stopped barcode moves.
Cell width is still unmeasured, so reading stays disabled until configured.
Read-only mode is the default once calibration is valid.

With barcode_mode 2 explicitly enabled, a valid read acts immediately:
pause tracking, turn left/right 90 degrees or move straight 300 mm, then resume
tracking. A rejected move or 8-second motion timeout stops without resuming.
GP21/stop can cancel a move. The reader resets at each handoff so a stale
frame cannot retrigger. These motion distances/angles require track validation.

For button-only operation after validating the physical format and calibration,
set the measured defaults in buddy3_barcode.h and the default mode in
robot_barcode_action.h. GP21 still controls starting and stopping.
