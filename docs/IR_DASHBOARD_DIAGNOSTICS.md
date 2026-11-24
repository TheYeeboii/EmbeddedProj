# IR dashboard diagnostics — issue #18

## Steering evidence

The new panel displays commanded left/right arrows, a duration timeline and
front-IR observations. Select a timeline segment or move the event slider to
freeze and inspect its sensor input, calibration, phase, requested/applied PWM,
encoder distance/speed and timestamps. Freeze does not stop the rover. Live
arrows continue updating during autonomous control. During manual control the
D-pad remains available and the duplicate arrows collapse; history continues
recording. Use GP21 or the normal Stop control to stop motion.
Export preserves the frozen/live history as JSON; Import displays it as a
replay without publishing any movement commands. Old firmware still supports
the action stream and delivery warnings; the new panel requires
`rover/steering` messages. Routine left/right/stop-turn rows are omitted from
the text log. Diagnostic events retain control-owner handovers, actual stop
commands, encoder faults/recovery, delivery gaps and dashboard input rejections.
Firmware-side command rejections remain available through their existing
reporting paths; browser input errors are labelled explicitly as dashboard rejections.

Green timeline segments mean left, blue right, grey straight/stopped and
orange unavailable intervals. The slider/detail identifies stopped versus
straight explicitly. Short events retain their actual timestamps and duration;
one-pixel markers are visibility aids, not an assertion of a minimum duration.
The final state has no known end time until a new event arrives. Dots show
normalized front left/right IR observations; exact readings are in the detail.

Requested steering is not measured heading. Applied PWM is sampled under the
same Buddy 2 mutex that accepts the motor command. Encoder values are the
latest measured observation, with their own timestamp; they are not proof the
vehicle physically turned in the requested direction.

## Capture and transport

Buddy 3 passes a stack-owned input snapshot directly into Buddy 2's observed
command API. The snapshot is copied at the accepted-command boundary; there
is no global pending sensor snapshot that another task could accidentally
consume. Existing trim, speed limit, PID and stop behavior are unchanged.
The physical mapping follows the steering definitions: front L is ADC1/index1,
front R ADC0/index0, barcode ADC2. The older dashboard's reversed front labels
are corrected too. Front detection has one threshold; barcode hysteresis is
separate. Calibration endpoints and inversion flags travel with each input.

`robot_steering_trace.c` keeps at most 64 fixed-size records. It preserves
direction, input classification, phase, owner and fault changes, with 10 Hz
steady observations at accepted commands. No network or formatting occurs in
the motor lock. Newest records replace the oldest on overflow; cumulative
drops and sequence gaps remain visible. The controller drains at most two
records per 20 ms iteration, through the existing nonblocking MQTT mailbox.
When caught up it sends a state snapshot once per second. A delayed event
over 500 ms appears in history but does not light a live arrow. Missing input
or unavailable state is never inferred from a later sensor sample.

The queue has a finite offline window, not guaranteed lossless logging. The
shared four-message mailbox and MQTT QoS0 can lose data; sequence, drops,
queue lag and stale/disconnect markers make those limits explicit. Each
connection must receive a current snapshot before assuming the state is live.
After 2.5 seconds without steering data the arrows become unavailable. History
is limited to 600 records; exports/imports are capped at 2 MB. JSON import is
schema/ordering checked and never interpreted as HTML or robot commands.

## Protocol v1 (`rover/steering`, non-retained)

`type` is `steering` or `steering_state`. `boot` is a startup-only oscillator
noise identifier (not a security token); `seq`, `ts_ms`, `lag_ms`, and `dropped`
describe event order, rover uptime, queue delay and cumulative firmware drops.
All new trace timestamps use kernel uptime, including accepted motor commands;
the legacy action log retains its hardware timer. Unsigned time/sequence wrap is supported. A changed boot ID starts a new
session; delayed messages from recently retired sessions are rejected.

`owner`: 0 stopped, 1 manual, 2 tracking, 3 junction, 4 search, 5 avoidance,
6 motion. `phase`: 0 straight, 1 micro, 2 corner, 3 seek, 4 align,
5 both-black, 6 stopped, 7 crossing, 8 legacy, 9 dark surface.
`direction`: 0 none, 1 left, 2 right, using requested R-L PWM difference >256.
`requested` and `applied` are `[left,right]` signed PWM API values.
`encoder_ms`, `distance` (0.1 mm), `speed` (0.1 mm/s) and `faults` (L bit0,
R bit1) describe the latest odometry observation. Speeds saturate at +/-3276.7 mm/s, independently of motor calibration.

`has_input` identifies a causal IR decision; `sample_ms` is that sample's
uptime. `ir` contains, in order: raw L/R, normalized L/R, black threshold,
L min/max, R min/max, inversion flags (L bit0/R bit1). Non-IR commands have
`has_input=0`; their zero IR fields are not interpreted as a real measurement.
Every message fits the existing 512-byte payload bound, including maximal
integer values; the C serialization test checks this.

## Verification

Run `sh testing/run_ir_diagnostics_tests.sh` from the repository root. The suite
checks actual motor/IR/controller integration, unchanged action logs, short
pulses, queue overflow/backpressure, stale ack, maximum payload bounds,
ordering/reboot/wrap, freeze and export/import. Pico USB CDC firmware builds
with SDK 2.3.1. For real-browser tests, serve `dashboard/` on localhost:8765,
provide Playwright via Node's module path, and run
`node testing/subsystem1/test_dashboard_diagnostics_browser.cjs`. Set `PLAYWRIGHT_CHROME_PATH`
to a local Chrome executable if bundled Chromium is unavailable. Browser
tests mock MQTT, use no rover, and check desktop/mobile layout and replay.

Physical direction, radio bandwidth, sensing/control jitter and stop latency
still need measurement on the rover with #4/#5. This PR supplies the software
instrumentation and test procedure; it does not establish hardware qualification
or close all physical acceptance criteria of #18. Before enabling on-track,
record a baseline and a diagnostic-enabled run at the same speed, compare
actual update timing, drops, motor ownership and GP21 stop response, and save
the exported session with video. A teammate should reproduce a deliberately
wrong or delayed correction from that evidence.
