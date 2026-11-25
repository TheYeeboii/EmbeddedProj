# Integration tests

Tests across two or more subsystems (TEAM.md section 5). INT-03 to INT-07
are the cross-subsystem tests in the Tribunal 1 subsystem test plan.

| ID | What is tested | Interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | controller: barcode mapping, junction moves, U-turn, GP21, safety stops | IF-02, IF-03, IF-05 | host | `test_robot_barcode_controller.c` | `PASS:` line |
| INT-02 | dashboard ↔ rover packed messages | IF-01, IF-02 | host | `testing/subsystem1/test_robot_wire.c` + `test_rover_wire.cjs` | both pass with the same golden bytes |
| INT-03 | every packed message round-trips with min, max and out-of-range values (S1 + all) | IF-01, IF-02 | host | Covered by the INT-02 golden-byte tests; add any missing min/max/out-of-range field to `test_robot_wire.c`. | Every value identical after the round trip; out-of-range values clamped or rejected. |
| INT-04 | shared fault byte (S1, S2, S4, S5): each fault sets only its own bit | IF-07 | manual | One at a time: hold both wheels during a distance move (stall), hold one wheel (encoder), unplug the IMU, point the sonar at open space (no echo), stop the broker (MQTT), turn the hotspot off (Wi-Fi). Watch the fault bits on the dashboard and the rover. | Only the matching bit sets; a stall or encoder fault stops a distance/turn move; Wi-Fi loss stops line following after 2 s (`ROBOT_LINK_LOSS_GRACE_MS`). |
| INT-05 | line seen during the avoidance return (S3 + S5): no lost event | IF-04, IF-05 | manual | Run the obstacle route 20 times with a barcode near the return point. Save the console. | The return finds the line in all 20 runs; no line or barcode event is lost. |
| INT-06 | loop timing with packing on (S2 + S3) | NFR1, NFR2 | manual | Timestamp the line loop and motion loop (temporary prints) while line following with telemetry on. Turn a wheel 10 times by hand and compare the count. | Line loop 10 ms ± 1 ms, motion loop 20 ms ± 1 ms; 10 hand turns = expected count ± 1 %. |
| INT-07 | valid, truncated and out-of-range dashboard commands (S1 + S2) | IF-02, IF-03 | manual | Send each with `mosquitto_pub` to `rover/commands`. | Valid ones run; bad ones are ignored; the motors never get an invalid value. |
| INT-08 | barcode letter → mapped turn at the next junction (S3 → controller → S2) | IF-03, IF-05 | manual | Map a letter on the dashboard, line follow over its barcode into a junction. Repeat for left, right, straight, U-turn. | Rover takes the mapped action at the junction and keeps line following. |
| INT-09 | obstacle avoidance hands back to line following (S5 → controller → S3) | IF-03, IF-04, IF-06 | manual | Put a box on the line ahead of a line-following rover. | Rover goes round the box, finds the line again and line following resumes on its own. |

*Method:* **host** = automatic test on the laptop, run by
`CC=gcc sh testing/run_barcode_tests.sh` and
`CC=gcc sh testing/run_ir_diagnostics_tests.sh` (each prints one `PASS:`
line per test program); **manual** = a person observes the result on the
rover or the dashboard.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Commit author | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|---|
| 2026-11-26 | INT-01 | `8aeafb2` | TanJiaYiCharmain | Faiz0909 | PASS | `test_robot_barcode_controller.c` printed its `PASS:` line and exited with code 0 | host, Windows 11, gcc 15.2.0 |
| 2026-11-26 | INT-02 | `8aeafb2` | TanJiaYiCharmain | Faiz0909 | PASS | `test_robot_wire.c` and `test_rover_wire.cjs` both printed their `PASS:` lines with the same golden bytes | C side and dashboard side both pass |
