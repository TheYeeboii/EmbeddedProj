# Subsystem 4 — test log

**Subsystem:** IMU: heading, tilt, humps, lift  ·  **Student ID:** 2501665  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | lift / overturn detection, humps and spikes ignored | | host | `test_buddy4_lift.c` | `PASS:` line |

*Method:* **host** = automatic test on the laptop, run by
`sh testing/run_barcode_tests.sh` and `sh testing/run_ir_diagnostics_tests.sh`
(each prints one `PASS:` line per test program); **manual** = a person observes
the result on the rover or the dashboard.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
