# Subsystem 2 — test log

**Subsystem:** Motion: motors, encoders, PID  ·  **Student ID:** 2501208  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | motor observer, barcode slowdown speed, trim/calibration preserved | | host | `test_buddy2_action_log.c` | `PASS:` line |
| S2-U02 | action log stream: direction, start/stop order, retries | | host | `test_robot_action_log.c` | `PASS:` line |

*Method:* **host** = automatic test on the laptop, run by
`sh testing/run_barcode_tests.sh` and `sh testing/run_ir_diagnostics_tests.sh`
(each prints one `PASS:` line per test program); **manual** = a person observes
the result on the rover or the dashboard.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
