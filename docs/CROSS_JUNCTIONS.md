# Going straight across junctions

The GP21 default (stage 1) includes this crossing controller. Stages 2/3 use it
too; stage 0 is legacy steering. No extra command is required after flashing.

## Behaviour

1. Both front sensors black: immediately cancel the previous correction and
   command straight travel, with encoder feedback to hold the entry heading.
2. Black persists for 30 ms: commit to crossing. Brief left/right readings
   from the transverse branch no longer trigger steering.
3. Cross at least 25 mm from first detection, and require both front sensors
   white over at least 5 mm before releasing to normal tracking.
4. The normal 35/14 steering correction then resumes. GP21/stop can interrupt.

Heading hold compares how far each wheel travelled since entry. It slows the
wheel that travelled farther, never reverses either wheel or boosts base power.
Both wheels need independent calibrated encoders. Buddy 2's implementation,
calibration and fault flags are preserved.

## Bounded failure

Stop and require an explicit restart if either encoder reports a fault, travel
goes backwards/resets, the heading-distance error exceeds 20 mm, either wheel
travels 100 mm without release, or the crossing takes 2500 ms without release.
The existing all-three-black protection also remains. These are per-crossing
bounds, not a fixed circuit run limit.

A detected black patch shorter than 30 ms does not commit: normal steering
resumes when it clears. During an existing tight corner or seeking manoeuvre,
the crossing controller does not take over; existing recovery rules apply.

Barcode decoding is invalidated during crossing so the transverse black area
is not mistaken for a barcode. Mode 2 barcode actions decoded before crossing
still have immediate priority, as configured separately.

## Track qualification

With two front sensors this is a straight-through POLICY for persistent
both-black, not proof of a cross-junction. A T-junction, broad dark patch or
thick line entered diagonally may give identical readings. White on exit also
does not prove a forward line exists, since centred straddling is white.
The algorithm assumes the tested cross-junction has a straight outgoing path.
It cannot guarantee correct behaviour for arbitrary junction types.

Start with wheels lifted. Left-black should produce a left correction;
then both-black must cancel it and command straight. Without encoder travel,
a crossing stops within 2.5 seconds. Repeat mirrored.

On the track, test a single cross-junction from an aligned approach in each
direction before running the circuit. Watch [junction] confirm -> straight ->
cleared, then [steer] straight/micro. Verify that branch edges do not cause a
turn. Repeat at the intended speed, battery level and both track widths.

If the rover stops before clearing, read the logs and measure the crossing:
tune B3_JUNCTION_MIN_MM, CLEAR_MM, MAX_MM and MAX_MS in buddy3_junction.h
against the real geometry. The values are initial settings, not measured
dimensions of this track. Verify equal wheel travel/trim before raising bounds.

## Regression checks

```sh
cc -std=c11 -Wall -Wextra -Werror -I firmware/app_program tests/test_buddy3_junction.c firmware/app_program/buddy3_junction.c -o /tmp/test_buddy3_junction
/tmp/test_buddy3_junction

cc -std=c11 -Wall -Wextra -Werror -I tests/stubs -I firmware/app_program tests/test_buddy3_ir_junction.c firmware/app_program/buddy3_junction.c firmware/app_program/buddy3_steering.c firmware/app_program/buddy3_barcode.c -o /tmp/test_buddy3_ir_junction
/tmp/test_buddy3_ir_junction
```

The second check compiles the real IR processing module against fake ADC and
motor hardware, validating that crossing takes priority over old corrections
and releases back to steering. Track reliability still requires physical tests.
