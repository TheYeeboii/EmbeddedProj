# Subsystem 5 — test log

**Subsystem:** Ultrasonic + servo: obstacle width, avoidance  ·  **Student ID:** 2502647  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | avoidance route end to end, obstacle width reported | | host | `test_buddy5_avoidance.c` | `PASS:` line |
| S5-U02 | return to the line after avoidance, stops on failures | | host | `test_buddy5_line_return.c` | `PASS:` line |

*Method:* **host** = automatic test on the laptop, run by
`sh testing/run_barcode_tests.sh` and `sh testing/run_ir_diagnostics_tests.sh`
(each prints one `PASS:` line per test program); **manual** = a person observes
the result on the rover or the dashboard.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
