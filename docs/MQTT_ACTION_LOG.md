# MQTT steering action log

The dashboard Serial monitor now has a dedicated **Steering actions** panel.
It displays:

- Trying to turn left
- Trying to turn right
- Stopped turning left
- Stopped turning right

The badge shows the latest commanded direction. These events describe issued
motor commands, not measured wheel rotation or proof of a completed turn.
Straight travel and a full stop both end a turn; the PWM values show which
command replaced it.

## Firmware capture and transport

Buddy 2 has observational hooks at accepted raw/steering commands, planned
distance/turn moves and motor stops. Calibration (387 edges/revolution), trim,
speed scaling, PID, encoder-fault handling and drive behaviour are preserved.
Trim differences and straight-move PID balancing do not invent turn actions.
For raw PWM, a requested wheel difference greater than 256 API units counts
as steering; smaller differences are treated as straight to reduce chatter.

Repeated commands in the same direction update the state but do not generate
repeated events. A direct left->right switch emits stop-left, then turn-right
with the same rover timestamp. All callers of Buddy 2 are covered: GP21 line
following, corner/recovery, junction heading correction, manual/dashboard drive,
barcode moves and obstacle avoidance. A fault/completion stop ends the active
turn as well.

The fast path only writes a bounded 32-event RAM queue. Packing and the network
mailbox are handled by the controller task, at most two events per loop
(about 20 ms without pending commands). No formatting, network calls or blocking
publication is added to the motor-control critical section.

Topic: rover/actions (non-retained), `DIAGNOSTICS=1` builds only. Each event is
a 16-byte packed message (type 8, [MQTT_BINARY_FORMAT.md](MQTT_BINARY_FORMAT.md));
the dashboard decodes it into:

```json
{"type":"action","seq":1,"ts_ms":4200,"action":"turn_left","direction":"left","source":"pwm","left_pwm":2621,"right_pwm":20316,"dropped":0}
```

source is pwm, motion or stop. Requested PWM values use the Buddy 2 API range;
raw PWM values are before trim/global speed scaling. For motion events, values
are the planned signed max PWM before speed scaling. A stop-turn event carries
the replacement command, so switching left->right may show right-turn PWM on
the stop-left row.

A type=action_state snapshot is published about once per second, behind queued
events. It updates the badge when a dashboard connects mid-turn and never
creates fake start/stop history. Rover uptime is shown separately from local
receipt time, so delayed buffered events can be identified.

If the network is offline or the mailbox is full, queued events wait for a
later loop. If the bounded queue overflows, oldest entries are replaced and a
cumulative dropped count increases. MQTT uses the existing QoS 0 transport;
delivery is best effort. Sequence gaps and firmware buffer losses are surfaced
in the dashboard. This is a live diagnostic stream, not a persistent audit log.

## Use

1. Rebuild/reflash with your real Wi-Fi configuration, WIFI_MQTT=1 and
   DIAGNOSTICS=1. A USB-only UF2 cannot publish to MQTT, and the default
   DIAGNOSTICS=0 build leaves this stream out.
2. Restart/reload the dashboard so action-log.js and the updated app.js load.
3. Connect to the existing broker. No extra topic configuration is needed:
   the dashboard subscribes to rover/actions automatically.
4. Press GP21. Verify a left-only sensor reading starts a left action, and
   returning to straight ends it; repeat for right. A cross-junction should
   end the previous correction; small heading corrections may log their own
   turn intentions during the crossing.

Pause/Clear affects both sensor and action histories. The badge stays live while
paused. Action history is limited to 100 displayed rows. A disconnected/stale
robot marks the badge unavailable instead of presenting an old turn as current.

## Checks

```sh
CC=gcc sh tests/run_ir_diagnostics_tests.sh
```

Three of its checks cover this log. `test_robot_action_log.c` runs the actual
queue and publisher and mocks MQTT mailbox acceptance. `test_buddy2_action_log.c`
runs the real Buddy 2 motion code against fake HAL/kernel calls, checking trim,
completion and encoder-fault events. `test_dashboard_actions.cjs` exercises the
real dashboard renderer against a small DOM fixture, including pause, clear,
state snapshots, lost messages, reboot and malformed payloads.
