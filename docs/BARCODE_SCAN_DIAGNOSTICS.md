# Barcode scan evidence — issue #19

## What the panel shows

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
but are not placed at invented physical positions. A grey unfinished segment
is not a classified/validated bar. Leading/trailing white distances appear in
the scan summary. Narrow/wide classes require complete nine-element groups.
Reference differences highlight mismatched classes, not a proven root cause.

A firmware-accepted letter remains labelled as such even if its diagnostic
trace is incomplete; the view explicitly distinguishes these facts. No prefix
or partial capture is labelled a decoded letter. Barcode movement remains
disabled; an import/replay never sends commands.

## Resource and transport bounds

The diagnostic recorder is separate from the hardware-independent Code 39
parser. The parser's acceptance/reset logic is unchanged. The adapter copies
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
  `blocked` reason flags. Idle state snapshots are published about once a second.
- `barcode_end`: the same progress fields plus result `reason` and `format`,
  ASCII `symbol` (zero when rejected), `reversed`, `leading`, `trailing`,
  measured `narrow` (0.01 mm), `ratio` (0.01 ratio), and largest accepted
  sample spacing `max_gap` (0.01 mm). Only a core-validated complete result
  carries a letter. Quiet zones and current partial widths are distinct.

States: 0 disabled, 1 leading quiet, 2 capture, 3 trailing quiet, 4 accepted,
5 rejected/interrupted, 6 suspended. Result reasons follow
`buddy3_barcode_event_kind_t`; format reasons follow `code39_status_t`.
Blocked bits: 1 stopped/unconfigured, 2 encoder fault, 4 excessive heading
change, 8 reverse wheel travel, 16 junction/recovery/unsupported phase.

Packet sizes are tested at numeric bounds and stay below the existing 512-byte
mailbox limit. No broker authentication/configuration changes are included.

## Testing

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
verification. Its output files are labelled `ISSUE19_BUILD_CHECK_ONLY_20261104`
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
