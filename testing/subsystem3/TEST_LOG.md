# Subsystem 3 — test log

**Subsystem:** Line following, junctions, barcode  ·  **Student ID:** 2502896  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | dashboard barcode settings | | host | `test_barcode_config.cjs` | `PASS:` line |
| S3-U02 | dashboard barcode history/model | | host | `test_barcode_history.cjs` | `PASS:` line |
| S3-U03 | barcode slowdown starts on black, ends after 3 s of white | | host | `test_barcode_slowdown.c` | `PASS:` line |
| S3-U04 | digital barcode input, front IR thresholds | | host | `test_barcode_tuning.c` | `PASS:` line |
| S3-U05 | IR tuning report packet | | host | `test_barcode_tuning_mqtt.c` | `PASS:` line |
| S3-U06 | dashboard decodes the barcode stream | | host | `test_barcode_wire.cjs` | `PASS:` line |
| S3-U07 | Code 39 A-Z in both directions, noise and malformed scans | | host | `test_buddy3_barcode.c` | `PASS:` line |
| S3-U08 | GP28 digital barcode input, full Code 39 pass | | host | `test_buddy3_ir_digital.c` | `PASS:` line |
| S3-U09 | IR integration: barcode reads, junction hand-over, end-of-line stop | | host | `test_buddy3_ir_junction.c` | `PASS:` line |
| S3-U10 | junction debounce, straight crossing, stops | | host | `test_buddy3_junction.c` | `PASS:` line |
| S3-U11 | steering: micro/corner/recovery, both-black stop | | host | `test_buddy3_steering.c` | `PASS:` line |
| S3-U12 | dashboard barcode view in a browser (needs playwright) | | host | `test_dashboard_barcode_browser.cjs` | `PASS:` line |
| S3-U13 | dashboard IR graph in a browser (needs playwright) | | host | `test_dashboard_ir_render_browser.cjs` | `PASS:` line |
| S3-U14 | dashboard IR stream in a browser (needs playwright) | | host | `test_dashboard_ir_stream_browser.cjs` | `PASS:` line |
| S3-U15 | dashboard IR thresholds in a browser (needs playwright) | | host | `test_dashboard_ir_threshold_browser.cjs` | `PASS:` line |
| S3-U16 | dashboard IR history | | host | `test_ir_stream_history.cjs` | `PASS:` line |
| S3-U17 | dashboard decodes the IR stream | | host | `test_ir_stream_wire.cjs` | `PASS:` line |
| S3-U18 | barcode diagnostic stream | | host | `test_robot_barcode_trace.c` | `PASS:` line |
| S3-U19 | IR diagnostic stream | | host | `test_robot_ir_stream.c` | `PASS:` line |
| S3-U20 | steering diagnostic stream | | host | `test_robot_steering_trace.c` | `PASS:` line |
| S3-U21 | dashboard steering history | | host | `test_steering_history.cjs` | `PASS:` line |

*Method:* **host** = automatic test on the laptop, run by
`sh testing/run_barcode_tests.sh` and `sh testing/run_ir_diagnostics_tests.sh`
(each prints one `PASS:` line per test program); **manual** = a person observes
the result on the rover or the dashboard.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
