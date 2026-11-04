# Barcode scan evidence — issue #19

## Two independent evidence views

The default **Continuous IR** tab records everything the side sensor observes,
including when the decoder is disabled, suspended or has never found a barcode.
It also works when white/black paper is moved under a stationary rover. No reader
configuration, encoder travel or leading quiet zone is required for this view.
The live black/white indicator and time strip use the sensor's hysteresis state;
the raw and normalized plots expose optical/calibration problems. A sample can
show black without becoming a valid barcode element. Firmware still needs to be
running and connected to MQTT; classification is evidence, not ground truth.

The **Barcode attempts** tab retains the distance-based decoder evidence described
below, including rejected and interrupted attempts. Reader setup is in this tab.
Switching tabs only changes the display: both streams continue to record, and
neither tab sends motor commands. Freeze/replay, clear, export and import are
independent for the two histories. Select a time window and inspect individual
continuous observations; export before clearing or leaving the page.

The continuous strip uses **time**, not printed millimetres. Grey means unknown
or missing observations, never assumed white. Narrow black pulses are retained
between MQTT updates rather than replaced by the latest white reading. A black
transition that was never sampled cannot be reconstructed. Live state becomes
unavailable on disconnect or stale data, while recorded evidence remains.

## What the barcode-attempt panel shows

The dashboard now reports disabled/waiting/capturing/trailing-quiet/accepted/
rejected/suspended states and grows a measured black/white strip during a scan.
The strip uses distance, not one graphical bit per task tick. Finished and
interrupted attempts remain selectable. Each completed bar/space exposes its
width, detection timestamp and independent wheel position observations. Raw
ADC, normalized signal and hysteresis state are retained alongside encoder
travel and the active calibration. Raw curves break across missing samples.

Select the expected letter FROM THE PRINT independently of the decoded result.
The reference supports A–Z, 2:1 or 3:1 widths, and either orientation. Only a
validated result may supply the automatic reference orientation; for a partial
scan use Forward/Flipped explicitly. Before the first nine complete elements,
the reference is normalized and cannot establish an actual millimetre scale.
Afterward, measured first-character widths estimate the shared scale. No
5/10 mm element sizes or 141 mm total length are imposed.

The measured strip shows the contiguous confirmed prefix. Orange means an
unknown transport interval; later received widths remain inspectable by index
but are not placed at invented physical positions. An unfinished segment keeps its observed black/white colour with a dashed
outline; its width is provisional and it is not a validated barcode element. Leading/trailing white distances appear in
the scan summary. Narrow/wide classes require complete nine-element groups.
Reference differences highlight mismatched classes, not a proven root cause.

A firmware-accepted letter remains labelled as such even if its diagnostic
trace is incomplete; the view explicitly distinguishes these facts. No prefix
or partial capture is labelled a decoded letter. Barcode movement remains
disabled; an import/replay never sends commands.

## Enabling a disabled reader

`Disabled / unconfigured` means capture has not been enabled: the firmware
minimum element defaults to zero. Use the stopped-only Reader setup form in
the panel instead of needing a serial command. The form requires fresh idle
telemetry, validates both fields, requests observe-only mode, and waits for
the firmware state. Exact active minimum/offset values are confirmed when
available; older firmware reports enabled with an explicit unconfirmed-values
notice. Lost/stale state, timeout and reboot do not appear as success.

Measure sensor offset from the midpoint BETWEEN the wheels to the sensing
spot. Left positive, right negative. An offset beyond the body edge must also
include the distance from that edge to the wheel midpoint; do not treat a
1–3 cm body-edge clearance as a 10–30 mm centre offset. Settings are RAM-only
and must be reapplied after reboot (or set as measured firmware defaults).

## Resource and transport bounds

The diagnostic recorder is separate from the hardware-independent Code 39
parser. The parser's acceptance/reset logic is unchanged. Capture now uses a fresh
non-mutating encoder-position snapshot rather than the 20 ms cached motor
state; see the cadence regression in BARCODE_DISTANCE.md. The adapter copies
new authoritative edges after each step and retains them before an abort can
discard the evidence. GP21/stop/reconfiguration explicitly finalizes active
traces before reset. Raw observations continue while stationary without adding
spatial elements. Encoder references are observations at edge detection; the
width's midpoint estimate is not an exact physical edge/pose measurement.

At most two frames, 29 completed elements and 512 raw observations per frame
are stored in fixed arrays (about 25 KB for frame storage). On sample overflow,
the earliest 512 observations remain; additional sample losses are counted.
The complete width stream may still decode successfully. On frame pressure,
the oldest pending frame is replaced and cumulative scan drops increase.
The dashboard retains at most 20 scans, with a 4 MB import limit.

MQTT formatting occurs in the controller, never in sensor capture. At most two
packets are offered per 20 ms controller iteration, in batches of four rows,
through the existing nonblocking mailbox. Backpressure preserves publisher
cursors for retry; stale acknowledgements cannot consume an overwritten frame.
The shared outbox/QoS0 transport is finite and can lose data. Sequence gaps,
missing row indices, conflicting rows, missing metadata, sample drops and stale
reader state are visible. A reconnect does not promise replay of data already
accepted by the mailbox. Connect BEFORE the passage for complete evidence.

## Continuous protocol v1 (`rover/ir`, non-retained)

A separate fixed ring retains the newest 256 observations (about 2.56 seconds
at the nominal 10 ms sampling period). Overflow increments a drop counter.
The controller publishes up to two batches per iteration, normally four samples
per packet (about 25 packets/second). A final short batch waits at most 40 ms
before being offered to the nonblocking mailbox. It shares finite MQTT capacity
with other diagnostics; sequence/timing gaps remain visible rather than hidden.

Packets contain `v:1`, `type:"ir_stream"`, shared diagnostic `boot`, newest-row
`ts_ms`, queue `lag_ms`, cumulative `dropped`, `period_ms:10`, `white:400`,
`black:600` and up to four `rows`:

`[sample_seq, sample_ms, raw, normalized, hysteresis_black, left_0.1mm,
right_0.1mm, decoder_blocked_flags, raw_minimum, raw_maximum, inverted]`.

Decoder blocked flags are informational and never gate this recorder. Calibration
travels with each observation. The browser retains at most 6000 samples (about
60 seconds), rejects duplicates/out-of-order data, marks missing observations,
and supports a bounded 4 MB JSON export/import (`rover-ir-stream-v1`). Older
firmware without this topic leaves the continuous view unavailable, not white.

## Protocol v1 (`rover/barcode`, non-retained)

All packets contain `v`, `type`, diagnostic `boot`, packet `seq`, `scan`, kernel
uptime `ts_ms`, queue age `lag_ms`, and cumulative overwritten-frame count
`dropped`. Buffered data older than 500 ms remains inspectable but is not
presented as the live reader state. A boot ID
matches the steering diagnostic stream; sequences identify barcode packets.

- `barcode_start`: `leading` white distance in 0.01 mm and `config`:
  raw minimum/maximum, inversion flag, normalized white/black thresholds,
  minimum resolvable element (0.01 mm), sensor-left offset (0.1 mm), wheel track
  (0.1 mm), wheel diameter (0.1 mm), left/right edges per wheel revolution
  (0.1 edge). The minimum is a resolution bound, not the actual print width.
- `barcode_edges`: up to four `rows` of
  `[index, detection_ms, width_0.01mm, left_0.1mm, right_0.1mm]`.
  Index zero starts black; following entries alternate black/white. No missing
  transition is manufactured. White gaps count as elements too.
- `barcode_samples`: up to four `rows` of
  `[index, sample_ms, raw, normalized, left_0.1mm, right_0.1mm,
  estimated_local_sensor_0.01mm, hysteresis_black]`.
- `barcode_progress` / `barcode_state`: reader `state`, completed `count`,
  retained `samples`, `sample_drops`, unfinished `partial` (0.01 mm), and
  `blocked` reason flags. Idle state snapshots are published about once a second and include the
  active `config` array even before a scan. State zero reports disabled settings
  as zeros; blocked flags explain configured-but-suspended operation.
- `barcode_end`: the same progress fields plus result `reason` and `format`,
  ASCII `symbol` (zero when rejected), `reversed`, `leading`, `trailing`,
  measured `narrow` (0.01 mm), `ratio` (0.01 ratio), and largest accepted
  sample spacing `max_gap` (0.01 mm). Only a core-validated complete result
  carries a letter. Quiet zones and current partial widths are distinct.

States: 0 disabled, 1 leading quiet, 2 capture, 3 trailing quiet, 4 accepted,
5 rejected/interrupted, 6 suspended. Result reasons follow
`buddy3_barcode_event_kind_t`; format reasons follow `code39_status_t`.
Blocked bits: 1 stopped/unconfigured, 2 encoder fault, 4 excessive heading
change, 8 reverse wheel/sensor travel, 16 junction/recovery/unsupported phase,
32 excessive sampling distance. A pre-capture spacing rejection is retained
in status for one second so a following zero-tick sample cannot hide it.

Idle snapshots optionally include `readiness`: side raw, normalized, hysteresis
black flag, observed white lead-in (0.01 mm), current sensor travel (0.01 mm),
allowed travel (0.01 mm), last rejected oversized step (0.01 mm), and cumulative
oversized-step count. These observations are shown before a scan starts.
Older firmware simply reports that pre-scan evidence is unavailable.


Packet sizes are tested at numeric bounds and stay below the existing 512-byte
mailbox limit. No broker authentication/configuration changes are included.

## Testing

Continuous regressions cover black pulses with stationary encoders and blocked
or unconfigured decoding, queue pressure/retry/wrap, calibration serialization,
and actual C-generated packets through the JS history. The browser test
`tests/test_dashboard_ir_stream_browser.cjs` accepts continuous and barcode
NDJSON fixture paths; it verifies black without barcode frames, independent
buffers/tabs, unknown gaps, partial black elements, export/import, disconnect,
mobile layout and that tab/dropdown keyboard navigation sends no motor command.
Compile `tests/test_robot_ir_stream.c` and run it with an output path to generate
the continuous fixture. These tests are included in the diagnostics suite except
for the browser test, which needs the local server and Playwright described below.


`sh tests/run_ir_diagnostics_tests.sh` covers firmware capture/serialization,
bounded raw storage, aborted evidence, overflow/backpressure/stale ack, real
C-generated JSON into the dashboard model, reference A–Z, independent F/Y/A/Z
fixtures at five scales and both orientations, missing rows, conflicting and
out-of-order data, reboot and export/import. `sh tests/run_barcode_tests.sh`
also verifies the underlying decoder, real IR adapter and GP21/controller.

For browser tests, compile `tests/test_robot_barcode_trace.c` with
`buddy3_barcode.c` and `code39.c`, then run the executable with a temporary
NDJSON output path. Serve `dashboard/` on localhost:8765 and run
`node tests/test_dashboard_barcode_browser.cjs /path/to/output.ndjson` with
Playwright available. `PLAYWRIGHT_CHROME_PATH` can select installed Chrome.
The browser consumes the actual C serializer output through mocked MQTT and
checks growing scans, no premature result, freeze, F in both directions,
independent reference selection, raw-sample inspection, interrupted evidence,
JSON download/import, missing batches and mobile overflow. No rover is driven.

Both USB CDC and complete Wi-Fi/DHCP/MQTT firmware compile with SDK 2.3.1.
The network compile used a temporary dummy credential fixture, removed after
verification. The latest continuous-stream build output files are labelled
`ISSUE19_CONTINUOUS_IR_BUILD_CHECK_ONLY_20261104`
and are NOT suitable for flashing. Configure real local credentials and rebuild
before a real radio test. No credentials are committed.

## Branch dependency and hardware checks

The issue #19 branch is based on the issue #18 steering branch and includes
the existing `feature/code39-reading` reader prerequisite (`d072ec2`). Its PR
targets the #18 branch so the steering code is not duplicated in the review
diff. Review/merge #18 first, then retarget the barcode PR to main. Both PRs
are left unmerged for the user. The old custom 0xA5 reader is retired by the
reader prerequisite, not interpreted as Code 39.

Rover evidence remains outstanding: actual F/Y prints from each end, repeat
passes, supported speeds/scales/lighting, a deliberate failure, radio load,
sensor update timing and GP21 stop response. Export a scan and compare it with
video/the physical print. A wrong reconstruction alone cannot establish
whether the cause was print quality, optical calibration, motion, sampling or
transport; use the recorded raw values, positions, settings and explicit gaps.
These checks remain open under #12/#15/#16/#19; no physical success is inferred
from synthetic tests, and #17 action mapping remains disabled.
