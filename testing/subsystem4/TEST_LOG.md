# Subsystem 4 — test log

**Subsystem:** IMU: heading, tilt, humps, lift  ·  **Student ID:** 2501665  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.
Requirement IDs (FR/NFR) are from the Week 6 report; interface IDs (IF-xx)
are from `TEAM.md` section 5.

## Test cases

Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | lift / overturn detection, humps and spikes ignored | FR16, FR23, IF-06 | host | `test_buddy4_lift.c` | `PASS:` line |
| S4-U02 | heading at rest and after turns | FR15, IF-06 | manual | Rover still on the floor: read the heading on the dashboard for 30 s. Then turn it by hand 90° four times (use floor tape marks) and read the heading at each mark. | Steady at rest (within ±2°); each 90° step reads 90° ± 10° (Tribunal 1 criterion), including across the 359°→0° wrap. |
| S4-U03 | pitch and roll | FR15 | manual | Put the rover on a ramp of known angle (measure it with a phone inclinometer), nose up, then sideways. | Pitch and roll within ±3° of the measured angle. |
| S4-U04 | motor noise on the compass | Tribunal 1 test plan (motors-off vs motors-on) | manual | Log heading for 10 s with the motors off, then 10 s with the wheels spinning off the ground. | Heading spread with motors on is no more than a few degrees larger than with motors off; note both spreads. |
| S4-U05 | hump detected and its peak reported | FR16, FR23 | manual | Line follow over the track hump. Screenshot the dashboard IMU card after the hump. | Hump count rises by 1; the peak tilt and height are reported; flat driving does not count a hump. |
| S4-U06 | lifted rover stops | FR23, IF-06 (SYS-06) | manual | While line following, lift the rover clear of the floor, hold it 2 s, put it back down. | Motors stop when lifted and stay stopped when put down; the lift count on the dashboard rises. |

*Method:* **host** = automatic test on the laptop, run by
`CC=gcc sh testing/run_barcode_tests.sh` (prints one `PASS:` line per test
program); **manual** = a person observes the result on the rover or the
dashboard.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Commit author | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|---|
| 2026-11-26 | S4-U01 | `8aeafb2` | TanJiaYiCharmain | terakaiser | PASS | `test_buddy4_lift.c` printed its `PASS:` line and exited with code 0 | host, Windows 11, gcc 15.2.0 |
