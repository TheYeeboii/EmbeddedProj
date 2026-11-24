# Subsystem 1 — test log

**Subsystem:** Wi-Fi, MQTT bridge, telemetry, dashboard  ·  **Student ID:** 2501134  ·  **Code:** `app_program/subsystem1/`, `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`

See `testing/README.md` for how to run tests and record results.

## Test cases

Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | dashboard action log: corrections, stops, snapshots | | host | `test_dashboard_actions.cjs` | `PASS:` line |
| S1-U02 | dashboard diagnostics in a browser (needs playwright) | | host | `test_dashboard_diagnostics_browser.cjs` | `PASS:` line |
| S1-U03 | MQTT + mailbox retry, ordering, back-pressure, reconnect | | host | `test_mqtt_backpressure.c` | `PASS:` line |
| S1-U04 | packed telemetry/debug/IMU layouts and command decoding (golden bytes) | | host | `test_robot_wire.c` | `PASS:` line |
| S1-U05 | dashboard decodes the same golden bytes and packs the same commands | | host | `test_rover_wire.cjs` | `PASS:` line |

*Method:* **host** = automatic test on the laptop, run by
`sh testing/run_barcode_tests.sh` and `sh testing/run_ir_diagnostics_tests.sh`
(each prints one `PASS:` line per test program); **manual** = a person observes
the result on the rover or the dashboard.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
