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
  message from the JSON diagnostic streams that still share some topics
  (Buddy 4's IMU report on `rover/telemetry`).

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

### rover/debug: type 2, 25 bytes (197 bits), every 1 s

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
| 12 | barcode mode | 0 off / 1 observe (1) | 1 |
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
| 2 | IMU fault | Buddy 4 | Planned: I2C error or `mag_ok` false |
| 3 | No echo | Buddy 5 | Planned: N timeouts in a row |
| 4 | Encoder fault | Buddy 2 | Planned: one wheel ticks, the other doesn't, for 500 ms |
| 5 | Motor stall | Buddy 2 | Planned: power applied, no ticks for 500 ms |

Bits 0 and 1 are live and sent in telemetry. Bits 2-5 are reserved for their
owners to set. The controller does not react to the mask yet: the agreed
2-second Wi-Fi-loss grace stop is still to do.

## Safe access to packed shared data

| Data | Writers | Rule |
|---|---|---|
| Fault byte | Several tasks | Changed only through `robot_fault_set()` / `robot_fault_clear()`, which do the read-modify-write inside `DI()`/`EI()`. The M0+ has no atomic bit set on RAM. Reading is one byte load, so readers need no lock |
| Link state (`uint8_t`, 2 bits used) | Radio task only | Single writer; an aligned byte read is atomic |
| Mailbox messages | Producer task / radio task | The kernel message buffer copies each message whole, with its length |
| Encoder counters | Encoder ISR | Stay native 32-bit; telemetry packs a snapshot copy outside the ISR |

## Binary-safe mailboxes

Binary payloads contain 0x00 bytes, and three places assumed text:

1. `comms_publish()` measured the payload with `comms_strlen`. It is now
   `comms_publish(topic, data, len)`. The JSON streams call
   `comms_publish_text()`.
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
| `stop` | 14 | 1 | 93 % |
| `forward` | 17 | 1 | 94 % |
| `drive` | 42 | 5 | 88 % |
| `motor_trim` | 42 | 2 | 95 % |
| `line_speed` | 40 | 3 | 93 % |
| `barcode_config` | 62 | 3 | 95 % |

At one telemetry and one debug message per second, the rover sends about
53 B/s of payload instead of about 800 B/s. Topic names are unchanged, so
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

The outbox is still 4 x 521 bytes, and `g_outgoing` is still 512. Packed
telemetry needs only 32, but the JSON diagnostic streams (`rover/actions`,
`rover/steering`, `rover/barcode`, `rover/ir`, Buddy 4's IMU report) share
the outbox and are up to 439 bytes. When those are packed,
`COMMS_PAYLOAD_MAX` can drop to 32. That saves about 1.9 KB of kernel pool
and 480 B of `.bss`, and each publish stops copying 513 bytes onto the
caller's stack.

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
  and barcode tests now run on packed commands.
- The dashboard was checked in the browser by feeding it the golden messages:
  every tile showed the expected value. Its commands were captured and
  matched the table above; the version-mismatch and offline badges worked.

Run them with:

```sh
CC=gcc sh tests/run_barcode_tests.sh
```

## Still to do

- On-robot check: flash, then `mosquitto_sub -h 192.168.137.1 -t "rover/#"
  -F "%t %x"` to capture the real byte counts. Also confirm a payload
  containing 0x00 arrives whole (assumption A1), and re-run the Week 8
  cross-subsystem tests.
- Timing: time `robot_telemetry_build()` with `hal_time_us()` against the old
  `tm_sprintf` build.
- Pack the remaining JSON streams, then shrink `COMMS_PAYLOAD_MAX`.
- Controller state as a packed byte (3-bit mode + flags). Fault bits 2-5 are
  set by their owners. Wi-Fi-loss stop after a 2 s grace period.
