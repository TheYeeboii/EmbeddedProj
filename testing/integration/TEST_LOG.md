# Integration tests

Tests across two or more subsystems (TEAM.md section 5).

| ID | What is tested | Interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | controller: barcode mapping, junction moves, U-turn, GP21, safety stops | IF-02, IF-03, IF-05 | host | `test_robot_barcode_controller.c` | `PASS:` line |
| INT-02 | dashboard ↔ rover packed messages | IF-01, IF-02 | host | `testing/subsystem1/test_robot_wire.c` + `test_rover_wire.cjs` | both pass with the same golden bytes |

*Method:* **host** = automatic test on the laptop, run by
`sh testing/run_barcode_tests.sh` and `sh testing/run_ir_diagnostics_tests.sh`
(each prints one `PASS:` line per test program); **manual** = a person observes
the result on the rover or the dashboard.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
