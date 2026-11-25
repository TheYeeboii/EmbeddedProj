# Subsystem 2 — test log

**Subsystem:** Motion: motors, encoders, PID  ·  **Student ID:** 2501208  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.
Requirement IDs (FR/NFR) are from the Week 6 report; interface IDs (IF-xx)
are from `TEAM.md` section 5.

## Test cases

Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | motor observer, barcode slowdown speed, trim/calibration preserved | FR8, NFR2, IF-03 | host | `test_buddy2_action_log.c` | `PASS:` lines, exit code 0 |
| S2-U02 | action log stream: direction, start/stop order, retries | FR8, IF-01 | host | `test_robot_action_log.c` | `PASS:` line, exit code 0 |
| S2-U03 | motor direction and encoder-to-wheel mapping | FR7, FR8 | manual | Wheels off the ground. From the dashboard drive forward, reverse, left, right. Watch which wheel turns which way and which encoder count rises on the dashboard. | Both wheels turn the commanded way; the left count rises only with the left wheel, the right count only with the right wheel. |
| S2-U04 | encoder counts per wheel revolution | FR7 | manual | Mark each wheel. Turn it 10 full turns by hand and read the encoder count on the dashboard or console. | Count = 10 × the calibrated ticks/rev, within ±1 % (Tribunal 1 promise). |
| S2-U05 | distance move accuracy | FR9, NFR7 | manual | On the floor, send a 500 mm and a 1000 mm forward move. Measure the distance travelled with a tape. 10 trials each; write every measurement in the notes. | Every trial stops within 5 % of the requested distance. |
| S2-U06 | turn accuracy | FR9, NFR7 | manual | Send 90° left and 90° right turns, 10 each. Measure the angle with a protractor or floor tape marks. | Every turn within ±5° of 90°. |
| S2-U07 | straight-line drift | FR21, NFR7 | manual | Start on a straight tape line, move 1000 mm forward, measure how far the axle centre ends up from the line. 10 trials. | Sideways drift ≤ 30 mm in every trial. |
| S2-U08 | stop response time | NFR8 | manual | Drive forward, send Stop from the dashboard. Print a console timestamp when the Controller receives the stop and when `buddy2_stop()` sets the PWM to 0 (temporary prints if needed). 10 trials. | Motors at 0 within 50 ms of the command reaching the Controller. |
| S2-U09 | raw-drive watchdog | NFR3 | manual | Hold an arrow key on the dashboard, then close the dashboard tab while the rover is driving. | Rover stops on its own ≤ 1000 ms after the last drive command (`ROBOT_DRIVE_TIMEOUT_MS`). |
| S2-U10 | motion loop period | NFR2 | manual | Build with a temporary timestamp print in the motion task; log 100 periods while driving. | Every period 20 ms ± 1 ms. |
| S2-U11 | stall and encoder-fault flags | Tribunal 1 fault byte (bits 4/5), IF-07 | manual | Start a 1000 mm move from the dashboard and hold one wheel still for over 1 s. Repeat holding both wheels. Then press *Reset everything* on the dashboard. | One silent wheel sets the encoder fault (bit 4), both set the stall fault (bit 5), after 800 ms without ticks (`BUDDY2_ENCODER_FAULT_MS`); the move stops. The bit stays set until *Reset everything* clears it. |

*Method:* **host** = automatic test on the laptop, run by
`CC=gcc sh testing/run_barcode_tests.sh` and
`CC=gcc sh testing/run_ir_diagnostics_tests.sh` (each prints one `PASS:`
line per test program); **manual** = a person observes the result on the
rover or the dashboard.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Commit author | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|---|
| 2026-11-26 | S2-U01 | `8aeafb2` | TanJiaYiCharmain | Faiz0909 | PASS | `test_buddy2_action_log.c` printed all 4 of its `PASS:` lines and exited with code 0 | host, Windows 11, gcc 15.2.0 |
| 2026-11-26 | S2-U02 | `8aeafb2` | TanJiaYiCharmain | Faiz0909 | PASS | `test_robot_action_log.c` printed its `PASS:` line and exited with code 0 | host, Windows 11, gcc 15.2.0 |
