# Subsystem 3 — test log

**Subsystem:** Line following, junctions, barcode  ·  **Student ID:** 2502896  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.
Requirement IDs (FR/NFR) are from the Week 6 report; interface IDs (IF-xx)
are from `TEAM.md` section 5.

## Test cases

Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | dashboard barcode settings | FR6, FR13 | host | `test_barcode_config.cjs` | `PASS:` line |
| S3-U02 | dashboard barcode history/model | FR6, FR13 | host | `test_barcode_history.cjs` | `PASS:` line |
| S3-U03 | barcode slowdown starts on black, ends after 3 s of white | FR13, NFR9 | host | `test_barcode_slowdown.c` | exit code 0 (asserts only, prints nothing) |
| S3-U04 | digital barcode input, front IR thresholds | FR10, FR13 | host | `test_barcode_tuning.c` | exit code 0 (asserts only, prints nothing) |
| S3-U05 | IR tuning report packet | FR5, IF-01 | host | `test_barcode_tuning_mqtt.c` | exit code 0 (asserts only, prints nothing) |
| S3-U06 | dashboard decodes the barcode stream | FR6, IF-01 | host | `test_barcode_wire.cjs` (reads the file S3-U18 writes) | `PASS:` line |
| S3-U07 | Code 39 A-Z in both directions, noise and malformed scans | FR13 | host | `test_buddy3_barcode.c` | `PASS:` line |
| S3-U08 | GP28 digital barcode input, full Code 39 pass | FR13, NFR9 | host | `test_buddy3_ir_digital.c` | `PASS:` lines |
| S3-U09 | IR integration: barcode reads, junction hand-over, end-of-line stop | FR11, FR13, FR22, IF-05 | host | `test_buddy3_ir_junction.c` | `PASS:` line |
| S3-U10 | junction debounce, straight crossing, stops | FR22 | host | `test_buddy3_junction.c` | `PASS:` line |
| S3-U11 | steering: micro/corner/recovery, both-black stop | FR10, FR11, FR12 | host | `test_buddy3_steering.c` | `PASS:` line |
| S3-U12 | dashboard barcode view in a browser (needs playwright) | FR6 | host | `test_dashboard_barcode_browser.cjs` | `PASS:` line |
| S3-U13 | dashboard IR graph in a browser (needs playwright) | FR6 | host | `test_dashboard_ir_render_browser.cjs` | `PASS:` line |
| S3-U14 | dashboard IR stream in a browser (needs playwright) | FR6 | host | `test_dashboard_ir_stream_browser.cjs` | `PASS:` line |
| S3-U15 | dashboard IR thresholds in a browser (needs playwright) | FR6 | host | `test_dashboard_ir_threshold_browser.cjs` | `PASS:` line |
| S3-U16 | dashboard IR history | FR6 | host | `test_ir_stream_history.cjs` | `PASS:` line |
| S3-U17 | dashboard decodes the IR stream | FR6, IF-01 | host | `test_ir_stream_wire.cjs` (reads the file S3-U19 writes) | `PASS:` line |
| S3-U18 | barcode diagnostic stream | FR5, IF-01 | host | `test_robot_barcode_trace.c` | `PASS:` line |
| S3-U19 | IR diagnostic stream | FR5, IF-01 | host | `test_robot_ir_stream.c` | `PASS:` line |
| S3-U20 | steering diagnostic stream | FR5, IF-01 | host | `test_robot_steering_trace.c` | `PASS:` line |
| S3-U21 | dashboard steering history | FR6 | host | `test_steering_history.cjs` | `PASS:` line |
| S3-U22 | line following on the track | FR10 | manual | Start line following on a straight, then through the curves of the track. | Follows the whole line without losing it. |
| S3-U23 | lost line: search, then stop | FR11 | manual | While line following, lift the rover off the line and put it down on white floor away from any line. | It turns toward the side it last saw the line; if no front sensor finds a line within 600 mm of wheel travel (`BUDDY3_NO_LINE_STOP_MM`; 3 s if an encoder has failed), it stops with the motors off. |
| S3-U24 | end of the line stop | FR11 (SYS-04) | manual | Line follow off the end of a line. Measure how far past the end it stops. | Stops within 600 mm of the end (`BUDDY3_NO_LINE_STOP_MM`, raised from 300 mm on 24 Nov after track measurements); mode Idle. |
| S3-U25 | barcode read on the track | FR13 | manual | Line follow over each test barcode, forwards and then the other way. Screenshot the dashboard barcode card. | The right letter is shown both ways. |
| S3-U26 | barcode-mapped action at the next junction | FR13, FR14, FR22, IF-05 | manual | Map a letter to left, right, straight and U-turn on the dashboard, one run each; run over that barcode into a junction. | Rover takes the mapped action at the junction and keeps line following. |
| S3-U27 | all-black stop | FR12 | manual | Put the rover on a wide black area for over 3 s while line following. | Motors stop. |
| S3-U28 | line loop period | NFR1 | manual | Temporary timestamp print in the line task; log 100 periods while line following. | Every period 10 ms ± 1 ms. |

*Method:* **host** = automatic test on the laptop, run by
`CC=gcc sh testing/run_barcode_tests.sh` and
`CC=gcc sh testing/run_ir_diagnostics_tests.sh` (each prints one `PASS:`
line per test program; S3-U03 to S3-U05 print nothing and pass with exit
code 0); **manual** = a person observes the result on the rover or the
dashboard.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Commit author | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|---|
| 2026-11-26 | S3-U01 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | PASS | `test_barcode_config.cjs` printed its `PASS:` line and exited with code 0 | host, Windows 11, node v24.22.1 |
| 2026-11-26 | S3-U02 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | PASS | `test_barcode_history.cjs` printed its `PASS:` line and exited with code 0 | host, Windows 11, node v24.22.1 |
| 2026-11-26 | S3-U03 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | PASS | `test_barcode_slowdown.c` ran every assertion and exited with code 0 (it prints no `PASS:` line) | host, Windows 11, gcc 15.2.0 |
| 2026-11-26 | S3-U04 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | PASS | `test_barcode_tuning.c` ran every assertion and exited with code 0 (it prints no `PASS:` line) | host, Windows 11, gcc 15.2.0 |
| 2026-11-26 | S3-U05 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | PASS | `test_barcode_tuning_mqtt.c` ran every assertion and exited with code 0 (it prints no `PASS:` line) | host, Windows 11, gcc 15.2.0 |
| 2026-11-26 | S3-U06 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | PASS | `test_barcode_wire.cjs` printed its `PASS:` line and exited with code 0 | host, Windows 11, node v24.22.1 |
| 2026-11-26 | S3-U07 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | PASS | `test_buddy3_barcode.c` printed its `PASS:` line and exited with code 0 | host, Windows 11, gcc 15.2.0 |
| 2026-11-26 | S3-U08 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | PASS | `test_buddy3_ir_digital.c` printed both of its `PASS:` lines and exited with code 0 | host, Windows 11, gcc 15.2.0 |
| 2026-11-26 | S3-U09 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | PASS | `test_buddy3_ir_junction.c` printed its `PASS:` line and exited with code 0 | host, Windows 11, gcc 15.2.0 |
| 2026-11-26 | S3-U10 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | PASS | `test_buddy3_junction.c` printed its `PASS:` line and exited with code 0 | host, Windows 11, gcc 15.2.0 |
| 2026-11-26 | S3-U11 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | PASS | `test_buddy3_steering.c` printed its `PASS:` line and exited with code 0 | host, Windows 11, gcc 15.2.0 |
| 2026-11-26 | S3-U12 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | NOT RUN | Not run: the playwright package is not installed on the test laptop | dashboard browser test; run `npm install playwright` first |
| 2026-11-26 | S3-U13 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | NOT RUN | Not run: the playwright package is not installed on the test laptop | dashboard browser test; run `npm install playwright` first |
| 2026-11-26 | S3-U14 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | NOT RUN | Not run: the playwright package is not installed on the test laptop | dashboard browser test; run `npm install playwright` first |
| 2026-11-26 | S3-U15 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | NOT RUN | Not run: the playwright package is not installed on the test laptop | dashboard browser test; run `npm install playwright` first |
| 2026-11-26 | S3-U16 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | PASS | `test_ir_stream_history.cjs` printed its `PASS:` line and exited with code 0 | host, Windows 11, node v24.22.1 |
| 2026-11-26 | S3-U17 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | PASS | `test_ir_stream_wire.cjs` printed its `PASS:` line and exited with code 0 | host, Windows 11, node v24.22.1 |
| 2026-11-26 | S3-U18 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | PASS | `test_robot_barcode_trace.c` printed its `PASS:` line and exited with code 0 | host, Windows 11, gcc 15.2.0 |
| 2026-11-26 | S3-U19 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | PASS | `test_robot_ir_stream.c` printed its `PASS:` line and exited with code 0 | host, Windows 11, gcc 15.2.0 |
| 2026-11-26 | S3-U20 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | PASS | `test_robot_steering_trace.c` printed its `PASS:` line and exited with code 0 | host, Windows 11, gcc 15.2.0 |
| 2026-11-26 | S3-U21 | `8aeafb2` | TanJiaYiCharmain | DisposableMask | PASS | `test_steering_history.cjs` printed its `PASS:` line and exited with code 0 | host, Windows 11, node v24.22.1 |
