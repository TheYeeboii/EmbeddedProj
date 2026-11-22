# Binary MQTT format (Change Requirement 1)

Change Requirement 1 says MQTT telemetry and command payloads use compact
binary fields instead of text, each field using only the bits its valid range
needs. This page covers the shared and Buddy 1 part: the telemetry, debug,
heartbeat, status and command messages, the comms mailboxes, the fault byte,
and the dashboard decoder.

| Where | File |
|---|---|
| Bit packing helpers (byte by byte, MSB first) | `firmware/app_program/robot_packed.c/.h` |
| Message layouts, command decoder | `firmware/app_program/robot_wire.c/.h` |
| Shared fault byte | `firmware/app_program/robot_fault.c/.h` |
| Binary-safe mailboxes | `firmware/app_program/comms.c/.h`, `robot_mqtt.c` |
| Dashboard decoder / command packer | `dashboard/rover-wire.js` |
| Tests | `tests/test_robot_wire.c`, `tests/test_rover_wire.cjs` |

`robot_wire.c` and `rover-wire.js` must change in the same commit. Raise
`ROBOT_WIRE_VERSION` (and `VERSION` in `rover-wire.js`) whenever a layout
changes. A dashboard that receives another version shows "Firmware format vN,
dashboard vM" instead of decoding garbage.

## Rules

- Bits needed = ceil(log2(number of valid values)).
- Byte 0 of every rover-to-dashboard message = type (high nibble) + version
  (low nibble). The fields follow back to back, **most significant bit
  first**. Signed fields are two's complement. A value outside its field is
  saturated to the field's limit, never wrapped.
- C bit-field structs are never sent: the compiler chooses their bit order and
  padding. Messages are built with explicit shifts and masks.
- Packed buffers are read and written **one byte at a time**. The Cortex-M0+
  HardFaults on a 16/32-bit access at an odd address, so a packed buffer is
  never cast to a wider type.
- Values stay native (`int32_t` ticks, PID floats) inside the control loops
  and ISRs, and are packed at the boundary: when they are put in a message.
- Type 7 and opcode 7 are reserved. 0x7B is `{`, so byte 0 tells a packed
  message from JSON text, i.e. a robot still running pre-binary firmware.
  Every message the firmware sends now, including the diagnostic streams,
  is packed binary.
- `rover/telemetry` carries two packed messages, telemetry (type 1) and
  Buddy 4's IMU report (type 6). The type nibble tells them apart.

## Data representation table

### rover/telemetry: type 1, 28 bytes (224 bits), every 1 s

| Field | Valid range | Before | Bits | Representation |
|---|---|---|---|---|
| Header | type 1, version 1 | none | 8 | `0x11` |
| Uptime | seconds since boot | `ts_ms` text | 20 | Whole seconds; saturates after about 12 days. Replaces `ts_ms` and the `robot_id` string |
| Controller mode | idle, drive, motion, line, avoid | text name | 3 | Index of `robot_mode_t` |
| Sensor state | online, compass error, IMU error | text name, sent twice | 2 | 0 / 1 / 2 |
| Fault mask | 6 faults | separate fields | 8 | `robot_fault.h` bits (below) |
| IR raw x3 | 0-4095 | text | 12 each | ADC value |
| Distance | 0 (no echo), 2-400 cm | text | 9 | 0 still means no echo |
| Servo angle | 0-180 deg | text | 8 | Plain value |
| Heading | 0-3599 tenths of a degree | text `263.4` | 12 | Tenths |
| Pitch, max pitch | about +/-90.0 deg | text | 11 each | Signed tenths (+/-102.3 deg) |
| Encoder ticks L/R | 32-bit counters | text | 16 each | Low 16 bits; the dashboard unwraps them (`TickUnwrapper`) |
| Motor command L/R | 0-65535 dashboard units | text | 16 each | Plain value |
| Trim L/R | 50-100 % | text | 6 each | Stored as value - 50 |
| Line base speed | 15-80 % | text | 7 | Plain value |
| Line curve power | 0-60 % | text | 6 | Plain value |
| Speed limit | 20-100 % | text | 7 | Plain value |

### rover/debug: type 2, 25 bytes (198 bits), every 1 s

| Field | Valid range | Before | Bits | Representation |
|---|---|---|---|---|
| Header | type 2, version 1 | none | 8 | `0x21` |
| IR raw x3 | 0-4095 | text | 12 each | ADC value |
| IR normalised x3 | 0-1000 | text | 10 each | Plain value |
| Line position | -1000 to +1000 | text | 11 | Signed |
| Line / junction / barcode flags | yes / no | `true`/`false` | 1 each | Bits |
| Barcode letter | 0 or `A`-`Z` | text number | 7 | ASCII |
| Side scan L/R | 0 (no echo) to 400 cm | text | 9 each | Plain value |
| Avoidance decision | none, left, right | text name | 2 | 0 / 1 / 2 |
| Avoidance status | 12 states | text name | 4 | `ROBOT_AVOID_STATUS_*` |
| Heading change | +/-180.0 deg | text tenths | 12 | Signed tenths (+/-204.7) |
| Heading target | 0-359.9 deg | text tenths | 12 | Tenths |
| Turn PWM L/R | +/-32767 | text | 16 each | Signed |
| Forward progress / target | 0-2047 mm | text | 11 each | Plain value; saturates |
| GPIO barcode reader | yes / no | padding | 1 | Set when the barcode letter is the latest confirmed GPIO scan. Older firmware sends zero. |

### rover/telemetry: type 6, Buddy 4's IMU / hump report, 14 bytes (107 bits)

Sent every second, and at once after a hump or a lift. It replaced a
283-byte JSON report that also carried its own 9-byte packet as hex text.

| Field | Valid range | Before | Bits | Representation |
|---|---|---|---|---|
| Header | type 6, version 1 | none | 8 | `0x61` |
| IMU fault / calibrated / height valid | yes / no | text numbers | 1 each | Bits |
| Sample interval | ms between IMU samples | text | 10 | Saturates at 1023 (a gap) |
| Roll | -180.0 to +180.0 deg | text tenths | 12 | Signed tenths |
| Hump state | flat, climbing, top, descending | text | 2 | `BUDDY4_FLAT..` |
| Hump sequence | completed humps | 16-bit text | 8 | Low 8 bits (wraps after 255) |
| Hump peak / highest peak | about +/-90.0 deg | text tenths | 11 each | Signed tenths |
| Hump height / highest | 0-255 mm | text | 8 each | Team decision: 8 bits |
| Motion class | 8 classes | text | 3 | `BUDDY4_MOTION_*` |
| Turn rate | +/-900.0 deg/s | text tenths | 15 | Signed tenths (+/-180 deg in a 200 ms window) |
| Lifted / overturned | 0-255 | text | 8 | Saturating count of lift stops (this slot carried impacts before the requirement change; same position and width) |

### Diagnostic streams (`DIAGNOSTICS=1` builds only): types 8-11

The tuning views' streams are packed the same way (`robot_diagnostics_mqtt.c`)
and `rover-wire.js` turns them back into the objects the views always used.
Sequence numbers, the boot id and millisecond timestamps genuinely use 32
bits and keep them. Drop counts and lags saturate at 16 bits. Distances
(x10 or x100 mm) use 24 signed bits (+/-838 m).

| Topic | Type | Fields (bits) | Size | Was (JSON) |
|---|---|---|---|---|
| `rover/actions` | 8 | header 8, state flag 1, seq 32, time 32, dropped 16, action 2, direction 2, source 2, PWM L/R 16 each | 16 B | ~139 B |
| `rover/steering` | 9 | header 8, state flag 1, boot/seq/time 32 each, lag 16, dropped 16, owner 3, phase 4, direction 2, requested 2x16, applied 2x16, encoder time 32, distance 2x24, speed 2x16, faults 2, sample time 32, IR raw 2x12, norm 2x10, threshold 10, calibration 4x12, dark 2, has input 1 | 58 B | ~268 B |
| `rover/barcode` | 10 | header 8, packet type 3, boot/seq/scan/time 32 each, dropped 16, lag 16, then by type: start (leading 24 + config 145), edges/samples (count 3, first index 5/9, up to 4 rows), progress/end/state (state 3, edges 5, samples 10, sample drops 16, partial 24, blocked 6; end adds reason 3, format 3, symbol 7, reversed 1, 5x24; state adds config 145 and readiness 135) | 22-87 B | up to ~405 B |
| `rover/ir` | 11 | header 8, boot 32, lag 16, dropped 16, rows 3, first sequence 32 and time 32, then per row: time step 16 (rows 2-4), raw 12, normalized 10, black 1, distances 2x24, blocked 6, calibration 2x12, inverted 1 | 75 B (4 rows) | ~295-439 B |

Barcode config (145 bits): raw endpoints 2x12, inversion 1, white/black
thresholds 2x10, minimum element x100 17 (up to 1000 mm), left offset x10
14 signed, track width / wheel diameter / ticks per revolution x10 4x16.
IR rows are consecutive samples, so only the first carries its sequence
number and full time.

### One-byte messages

| Message | Before | After | Representation |
|---|---|---|---|
| `rover/heartbeat` | `{"robot_id":"buddy1","state":"online"}` | `0x31` | Header byte only |
| `rover/status` online (retained) | same JSON | `0x51` | Type 5 |
| `rover/status` offline (Last Will) | same JSON with `offline` | `0x41` | Type 4; online/offline is the 1 bit that differs |

### rover/commands: 4-bit opcode + values

The dashboard sends 14 commands, so the opcode needs 4 bits. 0 and 7 are
reserved, which means an all-zero byte or an old text/JSON command is
rejected.

| Opcode | Command | Values (bits) | Bytes |
|---|---|---|---|
| 1 | stop | none | 1 |
| 2 | drive preset | forward / reverse / left / right (2) | 1 |
| 3 | drive | left (16), right (16), 0-65535 | 5 |
| 4 | line follow | none | 1 |
| 5 | line stop | none | 1 |
| 6 | IMU reset | none | 1 |
| 8 | reset all | none | 1 |
| 9 | motor trim | left - 50 (6), right - 50 (6) | 2 |
| 10 | speed limit | percent (7) | 2 |
| 11 | servo angle | degrees (8) | 2 |
| 12 | barcode mode + sensor levels | 0 off / 1 read (1), barcode sensor white raw (12), black raw (12); both 0 = levels unchanged | 4 |
| 13 | barcode config | min element mm (10), left offset mm signed (10) | 3 |
| 14 | line stage | 0-3 (2) | 1 |
| 15 | line speed | base % (7), curve % (6) | 3 |

The old `forward`, `reverse`, `left` and `right` text commands became one
"drive preset" opcode. That kept the opcode at 4 bits, and the fixed preset
speeds stay in the firmware.

A command is used only if all of these hold:

1. The opcode is known.
2. The length is exactly the one that opcode needs.
3. The padding bits after the last value are zero.
4. The controller's existing range check accepts every value. It still prints
   why it rejected a value on the USB console.

Text parsing (`robot_find`, `robot_json_int`, ...) is gone from the
controller.

## Fault byte (`robot_fault.h`)

| Bit | Fault | Owner | Set / cleared |
|---|---|---|---|
| 0 | Wi-Fi loss | Buddy 1, radio task | Set while not joined (`comms_set_link_state`) |
| 1 | MQTT loss | Buddy 1, radio task | Set while there is no broker connection |
| 2 | IMU fault | Buddy 4, IMU task (`buddy4_report_fault`) | Set while `imu_fault` is set: a compass or accelerometer read failed or went stale |
| 3 | No echo | Buddy 5, sensor task (`buddy5_report_fault`) | Set after 3 forward pings in a row the sensor did not answer (Echo never rose, or stuck high before the trigger). Cleared by one valid reading. "Nothing in range" (Echo high for the full 40 ms) is a normal answer and does not count |
| 4 | Encoder fault | Buddy 2, motion task (`buddy2_report_faults`) | Exactly one wheel is powered with no ticks for 800 ms (`BUDDY2_ENCODER_FAULT_MS`) |
| 5 | Motor stall | Buddy 2, motion task | Both wheels powered with no ticks for 800 ms |

All six bits are live and sent in telemetry. Each bit has one writer, which
only touches the byte when its bit changes. The dashboard's main panel shows
the IMU, no-echo, encoder and stall faults next to the sensor state.

Each owner already had the detection; these bits mirror it. Buddy 2's
threshold is its existing 800 ms, not the 500 ms suggested at the meeting.

**The controller's reaction (team decision):** line following stops once the
Wi-Fi-loss bit has been set for 2 s (`robot_check_link_loss`). Only a link
that was up and then dropped counts. A robot that never had a network (GP21
start, or a build without Wi-Fi) keeps following. After such a stop, a
deliberate GP21 press starts line following again. Raw drive commands
already stop after 1 s without a refresh.

## Safe access to packed shared data

| Data | Writers | Rule |
|---|---|---|
| Fault byte | Several tasks | Changed only through `robot_fault_set()`, `robot_fault_clear()` or `robot_fault_update()` (clear and set in one step), which do the read-modify-write inside `DI()`/`EI()`. The M0+ has no atomic bit set on RAM. Reading is one byte load, so readers need no lock |
| Controller state (`g_controller_state`, 8 bits: 3-bit mode + 5 flags) | Controller task and Buddy 5's avoidance task | Every change goes through `robot_ctl_update()` under `DI()`/`EI()`. `robot_ctl_enter(mode, flags)` changes the mode and its run flags in one step, so the avoidance task never sees a new mode with old flags |
| Link state (`uint8_t`, 2 bits used) | Radio task only | Single writer; an aligned byte read is atomic |
| GP21 debounce count (2 bits), barcode mode (1 bit), no-echo count (2 bits) | One task each | Single writer, one byte each, no lock |
| Mailbox messages | Producer task / radio task | The kernel message buffer copies each message whole, with its length |
| Encoder counters | Encoder ISR | Stay native 32-bit; telemetry packs a snapshot copy outside the ISR |

## Binary-safe mailboxes

Binary payloads contain 0x00 bytes, and three places assumed text:

1. `comms_publish()` measured the payload with `comms_strlen`. It is now
   `comms_publish(topic, data, len)`; nothing sends text any more.
2. `robot_mqtt.c` published with `strlen(g_outgoing)`. It now uses the length
   the mailbox returns. The Last Will passes its length explicitly
   (`will_msg_len` = 1).
3. `comms_wait_command()` handed the controller a string. It now returns
   `(bytes, len)`, and a command longer than `COMMS_COMMAND_MAX` is dropped,
   never truncated into a different command.

## Message size comparison

Before = the old JSON from the old format strings, filled with typical values
(computed, not yet captured with `mosquitto_sub`). After = exact, fixed by the
layout and checked by the tests.

| Message | Before (bytes) | After (bytes) | Reduction |
|---|---|---|---|
| Telemetry | 379 | 28 | 93 % |
| Debug stream | 424 | 25 | 94 % |
| Heartbeat | 38 | 1 | 97 % |
| Status / Last Will | 38 / 39 | 1 | 97 % |
| Buddy 4 IMU report | 283 (measured) | 14 | 95 % |
| `stop` | 14 | 1 | 93 % |
| `forward` | 17 | 1 | 94 % |
| `drive` | 42 | 5 | 88 % |
| `motor_trim` | 42 | 2 | 95 % |
| `line_speed` | 40 | 3 | 93 % |
| `barcode_config` | 62 | 3 | 95 % |
| Actions (tuning) | 139 | 16 | 88 % |
| Steering (tuning) | 268 | 58 | 78 % |
| Barcode packet (tuning, largest) | 405 | 87 | 79 % |
| IR batch, 4 samples (tuning) | 295-439 | 75 | 75-83 % |

At one telemetry, one debug and one IMU message per second, the rover sends
about 67 B/s of payload instead of about 1.1 KB/s. Topic names are unchanged, so
`rover/telemetry` (15 bytes) now costs more than its payload. Shortening the
topics is still an open team decision.

## RAM usage comparison

Wi-Fi build (`make CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1
WIFI_DHCP=1 WIFI_MQTT=1`, Pico SDK 2.3.0, arm-none-eabi-gcc 15.2),
`arm-none-eabi-size` and `arm-none-eabi-nm -S`:

| | Before | After | Change |
|---|---|---|---|
| `.text` | 386,417 | 385,061 | -1,356 B (text parsers removed) |
| `.data` | 3,264 | 3,264 | 0 |
| `.bss` | 90,064 | 89,464 | -600 B |
| Telemetry build buffer | 512 | 32 | -480 B |
| `g_incoming` (MQTT command) | 128 | 8 | -120 B |
| Link state | 4 | 1 | -3 B |
| Fault byte | n/a | 1 | +1 B (replaces scattered flags) |
| Inbox message buffer (kernel pool, not in `.bss`) | 4 x (128 + 8) = 544 | 4 x (8 + 8) = 64 | -480 B |
| Command copy on the controller stack | 128 | 8 | -120 B |
| `g_outgoing` (MQTT publish copy), `DIAGNOSTICS=0` | 512 | 32 | -480 B |
| Outbox message buffer (kernel pool), `DIAGNOSTICS=0` | 4 x (512 + 1 + 8) = 2,084 | 4 x (32 + 1 + 8) = 164 | -1,920 B |
| Outbox copy on each publisher's stack, `DIAGNOSTICS=0` | 513 | 33 | -480 B |
| Controller mode + 5 flags | 4 + 5 = 9 | 1 | -8 B |

With every default message packed, `COMMS_PAYLOAD_MAX` is 32
(`ROBOT_WIRE_MESSAGE_MAX`); a `DIAGNOSTICS=1` build needs 96
(`ROBOT_WIRE_DIAG_MESSAGE_MAX`, the largest barcode packet), down from 512
when its streams were JSON. `comms.c` fails to compile if a packed message
or command could ever be larger than its mailbox.

## Diagnostic streams: the `DIAGNOSTICS` build switch

`rover/actions`, `rover/steering`, `rover/barcode` and `rover/ir` exist only
for the dashboard's tuning views (packed binary, types 8-11 above).
`rover/ir` alone sends about 25 batches a second. Their five modules
(`robot_action_log`, `robot_steering_trace`, `robot_barcode_trace`,
`robot_ir_stream`, and `robot_diagnostics_mqtt`, which publishes all four)
only record what the robot did; no control code reads anything back from
them.

| Build | What is linked | MQTT |
|---|---|---|
| `make ... DIAGNOSTICS=0` (default) | `robot_diagnostics_off.c`: empty stand-ins with the same functions, so callers are unchanged | Packed binary only |
| `make ... DIAGNOSTICS=1` | The five real modules | Adds the four packed tuning streams |

The switch is part of the build profile, so changing it rebuilds everything.
The USB banner prints which one is running. The host tests still compile the
real modules.

| Wi-Fi build | `.text` | `.data` | `.bss` |
|---|---|---|---|
| Before Change Requirement 1 | 386,417 | 3,264 | 90,064 |
| Binary MQTT, `DIAGNOSTICS=1` (tuning, all packed) | 383,693 | 3,264 | 87,640 |
| Binary MQTT, `DIAGNOSTICS=0` (default) | 376,645 | 3,256 | 49,312 |
| Saving vs before, default build | -9,772 B | -8 B | **-40,752 B (-45 %)** |

Most of the RAM was the trace queues: `g_frames` (25.7 KB of barcode frames),
two `g_queue` rings (4 KB and 7 KB), and the action and IR buffers. The
outbox's kernel pool (-1.9 KB above) comes on top, since `size` does not
count the kernel's memory pool.

## Correctness evidence

- `tests/test_robot_wire.c`: round-trips every telemetry and debug field at
  its minimum, maximum and edge values (trim 50 and 100, distance 0 and 400,
  negative pitch, tick wrap, -1000 line position, +/-32767 PWM). It also
  checks saturation, overflow and overrun, and accepts or rejects commands
  (unknown opcode, opcode 7, old JSON and text, wrong length, stray padding
  bit, and a valid command containing 0x00).
- `tests/test_rover_wire.cjs`: decodes the C encoder's golden bytes in the
  dashboard and checks every value. It checks that the dashboard packs the
  exact command bytes the firmware accepts, and that the encoder counts
  unwrap across 0xFFFF in both directions.
- `tests/test_robot_barcode_controller.c`: the controller's command, button
  and barcode tests now run on packed commands. It also checks that the
  packed state byte holds mode + flags, and the Wi-Fi-loss stop. That stop
  must not fire within the 2 s grace and must fire after it. A robot that
  never had a network is never stopped, and GP21 works again afterwards.
- The IMU report (type 6) has its own golden message, decoded by both
  `test_robot_wire.c` and `test_rover_wire.cjs`.
- Diagnostic streams: `test_robot_barcode_trace.c` and `test_robot_ir_stream.c`
  write every packed message they publish as hex; `test_barcode_wire.cjs` and
  `test_ir_stream_wire.cjs` decode those with `rover-wire.js` and feed the
  dashboard models (both orientations of "F", a rejected scan, replay).
  The action and steering tests check the packed fields and lengths.
- Manual barcode reading: `test_buddy3_ir_junction.c` drives the real IR code
  over a Code 39 "F" three ways. Line following and a manual forward drive
  each read it once, and with neither nothing is read. The controller test
  checks a held W key keeps the reader running, that turning or reversing
  ends it, and that a stop only pauses it.
- The dashboard was checked in the browser by feeding it the golden messages:
  every tile showed the expected value. Its commands were captured and
  matched the table above; the version-mismatch and offline badges worked.

Run them with:

```sh
CC=gcc sh tests/run_barcode_tests.sh
CC=gcc sh tests/run_ir_diagnostics_tests.sh
```

Both scripts pass end to end.

## Still to do

- On-robot check: flash, then `mosquitto_sub -h 192.168.137.1 -t "rover/#"
  -F "%t %x"` to capture the real byte counts. Also confirm a payload
  containing 0x00 arrives whole (assumption A1), and re-run the Week 8
  cross-subsystem tests.
- Timing: time `robot_telemetry_build()` with `hal_time_us()` against the old
  `tm_sprintf` build.
- Each buddy's own internal structs (Buddy 2 motion status and move
  requests, Buddy 3 status flags and line-follow state, Buddy 4 IMU values,
  Buddy 5 scan and avoidance state). Each buddy agreed to do their own.
- Open team decision: shorten the topic names.
