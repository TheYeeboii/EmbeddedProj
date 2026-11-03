# Staged steering tests (test/fix-and-diagnose-steering)

These tests exercise steering before barcode navigation. Start at stage 1 when
isolating the basic steering behaviour. The normal Start button uses stage 3.
The default after reboot is stage 3; selection is not saved to flash.
No physical sensor/motor mapping was swapped: verify the actual wiring first.
Stages 1..3 now include straight cross-junction traversal; see
[CROSS_JUNCTIONS.md](CROSS_JUNCTIONS.md) for the policy, bounds and track tests.

## Build and flash

From firmware/build_make on this Mac:

```sh
make CONSOLE=usb_cdc PICO_SDK_PATH=/Users/noahwongnamsuen/.pico-sdk/sdk/2.3.1 -j4
```

Flash firmware/build_make/mtk3pico_smp0_usb_cdc.uf2.
This test build has USB serial and no Wi-Fi requirement. It replaces the
existing firmware on the Pico when flashed. Use your own space-free SDK path
on other computers (this Makefile does not handle spaces in SDK paths).

## Controls

For normal stage-3 operation, no serial commands are required: after flashing,
press GP21. The boot defaults are stage 3, 35% base PWM and 27 percentage-point curve
power. Press GP21 again to stop; a new press after stopping starts a
fresh run. There is no fixed run-duration cutoff; GP21/stop ends a run.

In the USB serial monitor, send each command with Enter:

```text
line_stage 1
line_speed 35 27
line_follow
stop
```

line_stage stops all motion before selecting a stage. line_follow explicitly
starts a new run. Runs continue until stopped or a fault occurs. stop or
line_stop stops it; GP21 toggles line following.
JSON equivalent: {"cmd":"line_stage","stage":2}.

Stages 1..3 default to observing barcodes without movement. The separate
barcode_mode 2 command enables immediate actions only after reader calibration;
see BARCODE_DISTANCE.md and BARCODE_ACTIONS.md.
Existing ultrasonic avoidance remains enabled: use clear
space, keep hands away from the ultrasonic beam, and distinguish [avoid] logs
from [steer] logs. On-board safety is not proof that a test is physically safe.

Stage 0 explicitly restores the original steering algorithm.
The new distance reader requires stage 1..3; stage 0 does not enable it.
It retains the original both-white search timeout. Do not select stage 0
until the isolated stages have been checked.

## 1. Verify mapping with wheels lifted

Send line_stage 1, line_speed 35 27, then line_follow.
Use black tape under one FRONT sensor at a time; covering a reflective sensor
with a hand is not necessarily equivalent to a black track.

- Physical left black/right white: [steer] micro L less than R.
  The physical left wheel must slow down while the right wheel speeds up.
- Physical right black/left white: mirror of the above.
- Both white: normally equal wheel commands; if one encoder gets ahead, its
  wheel is slowed to preserve the current heading. There is no blind search
  while the line is centred between the sensors.
- Left-only immediately followed by both-black: cancel the old left turn,
  command straight and commit after 30 ms. Mirror for right.
- Both-black from startup: try straight crossing; without encoder movement,
  expect a latched stop after 2.5 seconds.
- After a latched stop, removing tape cannot restart it; send line_follow.

Do not proceed if physical wheel behaviour disagrees with [steer] L/R:
check motor mapping. If the printed sensor side is wrong, check ADC mapping.
The ADC0/ADC1 assignments remain as they were; no blind reversal was applied.

## 2. Gentle tracking only

Keep stage 1. Place the sensors just outside the widest line so both see white
when centred. Start on a short straight, then a gentle curve; repeat on thin
and thick tape. Confirm corrections are small and symmetric without hunting.
Change only one setting at a time.

line_speed 35 27 requests base 35% and curve power 27 percentage points:
straight 35/35; left correction 8/62; right correction 62/8.
These are API commands BEFORE Buddy 2's global speed limit and trim.
The controller slows the inner wheel and boosts the outer wheel, clamping
both to the forward PWM range. Main's increased curve-power tuning is preserved.
Junction crossing instead uses the base with encoder heading correction.
If it cannot turn at all, verify traction/trim before changing timings.
Both-white cannot distinguish centred from off-track; GP21/stop is the manual
stop, and corner/recovery/junction timeout stops remain enabled.

Record thin/thick line width, sensor gap, base/correction and observed result.

## 3. Tight-turn escalation, without recovery

Send stop, line_stage 2, line_speed 35 27, line_follow.
First check with wheels lifted: hold left black for 300 ms.
Expect micro then corner, with left wheel stopped and right at base.
Mirror for right. A continuously held corner keeps turning while that sensor
continues to see the line; this supports long circular track sections.
If both become white during cornering, stage 2 stops rather than guessing.

Only then try a single left 90-degree corner, followed by a right corner.
Record whether corner engages too early/late and where the vehicle stops.
This is a sustained-correction heuristic, not proof of a 90-degree corner.

## 4. Bounded recovery

Send stop, line_stage 3, line_speed 35 27, line_follow.
After a sustained correction enters corner mode and loses black, it continues
a one-wheel turn toward that remembered side for at most 1200 ms.
It must see black again and then both-white for 100 ms before returning to
normal tracking. During that white confirmation it uses half base speed.
No reverse wheel command or full-circle spin is used.

Test left and right corners separately, followed by a deliberate missed line.
No reacquisition: expect stopped, and no automatic restart.
Both-black during active corner/recovery still uses the 150 ms ambiguous stop;
ordinary tracking uses the separate bounded straight-crossing controller.
A perpendicular re-entry may still be ambiguous: this heuristic does not prove
heading alignment or distinguish a junction from a wide line.

## Limits and next test

The timing values in app_program/buddy3_steering.h are initial test settings,
not calibrated physical angles. Do not treat a successful software test as a
successful track test. Cross-junctions now use a straight-through policy; the
two front sensors still cannot reliably classify cross versus T-junctions.
The separate distance decoder and movement modes are documented in
BARCODE_DISTANCE.md and BARCODE_ACTIONS.md. Stage 0 selects legacy steering.

A barcode strip along a straight can be used in stage 1 to check that steering
does not unexpectedly spin the rover, but barcode correctness is a separate
test. Only re-enable route actions after the steering tests pass.

## Repeatable host check

From the repository root:

```sh
cc -std=c11 -Wall -Wextra -Werror -I firmware/app_program \
  tests/test_buddy3_steering.c firmware/app_program/buddy3_steering.c \
  -o /tmp/test_buddy3_steering
/tmp/test_buddy3_steering
```

This checks the actual controller with synthetic sensor sequences: mirrored
wheel commands, centred-white behaviour, ambiguous-black stops, corner
escalation, recovery confirmation, timeout stops, and timestamp wrap.
