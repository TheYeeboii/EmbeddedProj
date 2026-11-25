# Subsystem 1 — test log

**Subsystem:** Wi-Fi, MQTT bridge, telemetry, dashboard  ·  **Student ID:** 2501134  ·  **Code:** `app_program/subsystem1/`, `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`

See `testing/README.md` for how to run tests and record results.
Requirement IDs (FR/NFR) are from the Week 6 report; interface IDs (IF-xx)
are from `TEAM.md` section 5.

## Test cases

Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | dashboard action log: corrections, stops, snapshots | FR6 | host | `test_dashboard_actions.cjs` | `PASS:` line |
| S1-U02 | dashboard diagnostics in a browser (needs playwright) | FR6 | host | `test_dashboard_diagnostics_browser.cjs` (run `npm install playwright` first) | `PASS:` line |
| S1-U03 | MQTT + mailbox retry, ordering, back-pressure, reconnect | FR2, IF-01 | host | `test_mqtt_backpressure.c` | `PASS:` line |
| S1-U04 | packed telemetry/debug/IMU layouts and command decoding (golden bytes) | FR5, IF-01, IF-02 | host | `test_robot_wire.c` | `PASS:` line |
| S1-U05 | dashboard decodes the same golden bytes and packs the same commands | FR6, IF-01, IF-02 | host | `test_rover_wire.cjs` | `PASS:` line |
| S1-U06 | Wi-Fi join and automatic rejoin | FR1 | manual | Power the rover with the hotspot on; note the time it shows *online*. Turn the hotspot off for 20 s, then on again. Save the broker log (`mosquitto -v`) and the console. | Rover joins at boot, goes offline when the hotspot is off, and rejoins and reconnects to MQTT by itself within about 10 s of the hotspot coming back. |
| S1-U07 | MQTT reconnect when the broker restarts | FR2 | manual | With the rover online, stop the broker for 10 s and start it again. Save the broker log. | Rover reconnects and publishes `rover/status` again without a reset. |
| S1-U08 | heartbeat and telemetry rate | FR4, NFR5 | manual | Record `mosquitto_sub -t 'rover/#' -F '%I %t'` for 60 s with the rover idle. Count the messages per topic. | One `rover/heartbeat` every 10 s; telemetry at the rate in `subsystem1.c`; the dashboard shows the last heartbeat time. |
| S1-U09 | dashboard commands drive the rover | FR3, FR6, IF-02 | manual | From the dashboard send forward, reverse, left, right, stop, line follow start/stop. | Each command does what its button says; Stop always stops. |
| S1-U10 | live telemetry on the dashboard | FR5, FR6 | manual | Move the rover by hand over the line, tilt it, put an object in front of the sonar. Screenshot the dashboard. | IR, encoder, heading/pitch/roll and ultrasonic values change to match. |
| S1-U11 | bad commands are ignored | Tribunal 1 test plan (Buddy 1 + 2) | manual | Publish a truncated and an out-of-range command to `rover/commands` with `mosquitto_pub`. | Rover ignores them: no motion, no reset, and it keeps answering valid commands afterwards. |

*Method:* **host** = automatic test on the laptop, run by
`CC=gcc sh testing/run_barcode_tests.sh` and
`CC=gcc sh testing/run_ir_diagnostics_tests.sh` (each prints one `PASS:`
line per test program); **manual** = a person observes the result on the
rover or the dashboard.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Commit author | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|---|
| 2026-11-26 | S1-U01 | `8aeafb2` | TanJiaYiCharmain | TheYeeboii | PASS | `test_dashboard_actions.cjs` printed its `PASS:` line and exited with code 0 | host, Windows 11, node v24.22.1 |
| 2026-11-26 | S1-U02 | `8aeafb2` | TanJiaYiCharmain | TheYeeboii | NOT RUN | Not run: the playwright package is not installed on the test laptop | dashboard browser test; run `npm install playwright` first |
| 2026-11-26 | S1-U03 | `8aeafb2` | TanJiaYiCharmain | TheYeeboii | PASS | `test_mqtt_backpressure.c` printed its `PASS:` line and exited with code 0 | host, Windows 11, gcc 15.2.0 |
| 2026-11-26 | S1-U04 | `8aeafb2` | TanJiaYiCharmain | TheYeeboii | PASS | `test_robot_wire.c` printed its `PASS:` line and exited with code 0 | host, Windows 11, gcc 15.2.0 |
| 2026-11-26 | S1-U05 | `8aeafb2` | TanJiaYiCharmain | TheYeeboii | PASS | `test_rover_wire.cjs` printed its `PASS:` line and exited with code 0 | host, Windows 11, node v24.22.1 |
