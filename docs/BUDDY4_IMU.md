# Buddy 4 implementation and rover validation

Software tests and both firmware builds pass; this is not a claim of completed physical calibration or track acceptance.

## What changed

- `buddy4_imu.c/.h`: LSM303 I2C driver, accelerometer block-data-update enabled
  (`CTRL_REG4_A = 0x88`), configuration re-sent every second, queued calibration,
  independent IMU task, freshness checks and telemetry.
- `buddy4_processing.c/.h`: portable processing, calibration offsets/scales,
  median-of-three plus time-aware low-pass tilt filtering, hump lifecycle,
  height estimation, motion classification, sudden-impact detection, compass
  turn rate, compact serialization. No kernel or SDK dependencies.
- `app_main.c`: starts the IMU task at priority 9 with a target 50 ms period.
  Buddy 5 (priority 10) no longer performs I2C updates. Motion, line and controller
  retain priorities 5, 6 and 7. This adds a 4096-byte task stack from the kernel pool.
- `robot_telemetry.c`: accelerometer failures no longer appear as all sensors OK.
- Dashboard: separate IMU/hump panel; independent IMU reports do not clear the
  existing heading, distance, motor or line tiles.

Later changes: a single implausible accelerometer sample no longer cancels a
hump (see below); motion classification, impact detection, turn rate and
highest hump peak were added; the shared snapshot uses bit-fields (Change
Requirement 1); both files follow Barr-C (braces, one declaration per line,
module prefixes, named constants). The unused `buddy4_reset_max_pitch()` and the
unread raw-axis, rejected-count and timestamp fields were removed.

Register reference: [ST LSM303DLHC datasheet](https://www.st.com/resource/en/datasheet/lsm303dlhc.pdf).
The BDU bit prevents a high/low-byte pair changing halfway through its read.
It does not prove a whole reading is physically accurate.

## Sampling, ownership and validity

`usermain()` initializes the driver. Only `buddy4_task()` calls
`buddy4_update()` after startup. Other tasks use `buddy4_get_state()`.
Snapshots and pending requests use short DI/EI sections, matching the existing
single-core (`SMP=0`) application. Do not enable SMP without replacing this
protection with an appropriate cross-core synchronization scheme.

The task sleeps for the remainder of its 50 ms period; overruns yield for 1 ms.
This is a target, not a hard-real-time guarantee. `sample_interval_ms` reports
the measured processing interval. The accelerometer runs at 100 Hz and the
magnetometer at 15 Hz, so consecutive 20 Hz polls may reuse a magnetic sample.
Timestamps identify successful software reads, not hardware data-ready events.

Read failures, magnetic overflow/out-of-range values and a zero horizontal
magnetic field are rejected. Acceleration magnitude must lie within 0.8–1.2 g.
Three plausible accelerometer samples are required to prime the median filter.
The low-pass time constant is 100 ms. A single isolated pitch spike is rejected
by the median filter; sustained vehicle acceleration can still bias tilt.
The getter invalidates readings older than 250 ms. Gaps over 250 ms or duplicate
timestamps cancel an unfinished hump. Up to four implausible accelerometer
samples in a row (a jolt at the hump edge, motor vibration) are skipped: the
filter, the hump state and the travel since the last good sample are kept. A
fifth one in a row (about the 250 ms staleness limit) resets the filter and
cancels the hump. Before this change one jolt cancelled the hump.

Both sensors' configuration is re-sent every second, not only after an I2C
failure. On the rover the compass sometimes stopped updating while I2C reads
still succeeded: the heading stayed at exactly one value, with no fault shown,
sometimes from boot before any motor had run. A Pico reset cleared it on one
try and not on the next, which fits the compass being left in its sleep
default. Re-sending costs four short register writes per second and should
bring it back within a second; if the heading still freezes, the cause is
elsewhere (wiring or the module).

`mag_ok` / `accel_ok` now mean usable readings under these checks, not just ACKs.
`imu_fault` is their combined failure flag. Buddy 5 still uses compass validity
specifically; a tilt-filter warmup does not unnecessarily disable a valid compass.
The fault is local Buddy 4 state, not a new project-wide shared fault byte.

## Calibration

Defaults are identity offsets/scales and **uncalibrated**. Height validity stays
false until both sensor and wheel-distance calibration are explicitly confirmed.
Call `buddy4_set_calibration(&cal)` with measured values; the owner task applies
the request at the next update and resets filters and run statistics.

```c
buddy4_calibration_t cal = {0};
cal.mag_scale[0] = cal.mag_scale[1] = cal.mag_scale[2] = 1.0f;
/* Fill measured accel_bias_mg[], mag_bias[], mag_scale[], pitch_zero_x10,
   roll_zero_x10 and heading_offset_x10 here. Never invent measurements. */
cal.calibrated = true;          /* only AFTER IMU/mounting validation */
cal.distance_calibrated = true; /* only AFTER Buddy 2 wheel validation */
bool accepted = buddy4_set_calibration(&cal);
```

Settings are in RAM; repeat this call with measured constants after startup to
retain them across boots. No flash-storage format or dashboard calibration
command was introduced. Calibration accepts finite positive scales and bounded
offsets. It does not perform an automatic calibration experiment.

The LSM303 is mounted turned 90 degrees: tilting the rover's nose moves the
sensor's Y axis and tilting it sideways moves X. Before this was found, pitch
showed the sideways tilt, roll showed the nose tilt, and leaning the rover
sideways counted as a hump. `buddy4_imu.c` now rotates both sensors (rover
X = sensor Y, rover Y = minus sensor X), so processing sees level Z
approximately +1000 mg and nose-up negative X, as before. Because it is a
rotation and not a swap, the compass still turns the same way; only its
fixed offset changes. Roll is positive with the left side down. This
implementation preserves `atan2(mag_y, mag_x)` heading, with hard-iron offsets,
axis scales and a heading offset. It is **not tilt-compensated** and does not
correct general cross-axis soft-iron distortion. Validate heading on the surface
where Buddy 5 turns; mounting errors and motor fields remain physical concerns.

## Hump detection and height

The track hump sits under **one wheel**. With one wheel up the body leans
sideways and slightly nose-down (measured on the track: pitch -3 degrees, roll
+5.4 degrees, about 6 degrees in total); the lean holds while the wheel is on
top and goes back to level after it. The pitch never swings positive then
negative, so detection uses the **total tilt** from the flat attitude,
`sqrt(dpitch^2 + droll^2)`, not the sign of the pitch.

The flat attitude follows the floor and the sensor mounting while the rover is
level (time constant about 2 s), so zero offsets are not needed to detect a
hump and a slowly sloping floor is not one. Entry requires three filtered
readings at or above 5 degrees of tilt, and not while turning at 25 degrees/s
or more (a pivot turn leans the body outwards). After entry the state shows
climbing while the tilt rises, top when it holds, and descending once it is
2 degrees below the peak. The hump completes after eight readings (400 ms)
within 2.5 degrees of flat; this is long enough that a two-wheel crossing, which
is level for a moment on the crest, still counts once. A 15-second timeout
cancels a hump that never comes back to level, such as a rover stuck on it.
These first thresholds come from one track hump and need tuning from logs.
Manually tilting the rover can also produce a hump.

`hump_count` saturates at seven for the 3-bit representation. `hump_sequence`
increments on every completed hump, so telemetry consumers can detect further
events. The peak tilt and the last completed height are retained; `highest_hump_mm`
retains the largest valid height estimate since initialization/calibration reset.
`highest_peak_deg_x10` is the largest peak tilt of any **completed** hump. It
answers "highest hump peak experienced during the run" even before height is
calibrated, and unlike `max_pitch_deg_x10` it ignores a climb that never comes
back to level.

Height uses average wheel travel and pitch:

`elevation += delta_distance * sin(average(previous_pitch, current_pitch))`

It records the maximum elevation during the event, including the entry debounce
window. This is a slope-and-distance estimate, not double-integrated acceleration.
Both wheel speeds must indicate forward movement and odometry must be fresh.
Negative/reset distance, steps above 100 mm per sample, reverse/pivot/stopped
samples, or absent calibration invalidate the event's height. Buddy 2 signs
distance from commanded direction; wheel slip, coasting, external pushes,
wheelbase/chassis geometry and filter lag still affect accuracy. Starting the
integration at the entry threshold can underestimate a gradual approach.
This estimator assumes both wheels climb the hump. For the one-wheel track
hump it is not valid: the body pitches slightly nose-down while one wheel
rises. A one-wheel height would come from the roll instead, roughly
`track_width * sin(droll)` (150 mm * sin(5.4 deg), about 14 mm, on the
measured hump); that is not implemented yet.

## Motion events, impacts and turn rate

`motion_class` (3 bits) is one class per sample; when several apply the first
in this list wins:

| Value | Class | Rule (first estimates, tune on the track) |
|---:|---|---|
| 7 | Sudden impact | Acceleration below 0.5 g or above 1.5 g, held 500 ms |
| 6 | On a hump | Hump state is not flat (`hump_state` says climbing/top/descending) |
| 5 | Turning | Compass turn rate at least 25 deg/s |
| 0 | Unknown | No fresh Buddy 2 odometry |
| 1 | Stationary | Forward and pivot wheel speed both under 5 mm/s |
| 3 | Accelerating | Speed up by more than 60 mm/s since the last 400 ms window |
| 4 | Decelerating | Speed down by more than 60 mm/s |
| 2 | Cruising | Moving at a steady speed |

- **Impacts:** `impact_count` (8 bits, saturating) counts sudden impacts; one
  collision gives several wild samples, so samples within 500 ms count once.
  The IMU only reads the newest of the 100 Hz samples every 50 ms, so a very
  short knock can be missed. A hard hump edge can also count as an impact.
  Buddy 4 only reports impacts; deciding to stop belongs to the controller.
- **Turn rate:** `turn_rate_dps_x10` is the compass heading change over a 200 ms
  window (the 15 Hz compass repeats samples at the 20 Hz poll), shortest way
  round 359 to 0. Positive means the heading number increases; check which way
  that is on the rover. The compass is not tilt-compensated and motors disturb
  it, so check the rate against a timed 90-degree turn.
- **Speed:** 400 ms windows of Buddy 2 odometry. One encoder tick at the current
  20 ticks/rev default is 25 mm/s over 400 ms, so the 60 mm/s threshold needs
  more than two ticks. Pivot speed `(right - left) / 2` keeps an on-the-spot
  turn from reading as stationary. Re-check thresholds after Buddy 2 calibrates.

## Telemetry and compact representation

The IMU task publishes a separate small JSON object on `rover/telemetry` every
second, immediately after a completed hump and immediately after an impact. It
carries fault/calibration, roll, hump state/count/sequence/peak, highest peak,
last and highest height, height validity, `motion`, `turn_rate_x10`, `impacts`,
sample interval and `imu_packed`. Completed humps and impacts are also printed
on USB.
If the nonblocking mailbox drops a report, the retained sequence and results
are repeated in subsequent periodic reports. This is not guaranteed event
delivery or an event history queue.

`buddy4_pack()` serializes nine bytes explicitly, least-significant bit first
within each byte. Signed fields use the listed width's two's complement. It
returns false and sets the overflow bit if any field required saturation.

| Bit offset | Width | Meaning |
|---|---:|---|
| 0 | 4 | Version = 2 |
| 4, 5, 6, 7 | 1 each | Compass OK, accel OK, height valid, calibrated |
| 8 | 12 | Heading in tenths, 0–3599 |
| 20 | 11 | Signed pitch in tenths, -900–900 |
| 31 | 12 | Signed roll in tenths, -1800–1800 |
| 43 | 11 | Hump peak tilt in tenths (signed field) |
| 54 | 2 | Hump state |
| 56 | 3 | Saturating hump count |
| 59 | 8 | Last height, saturated at 255 mm |
| 67 | 1 | Packing overflow |
| 68 | 1 | IMU fault |
| 69 | 3 | Motion class (version 2; was reserved zero in version 1) |

`imu_packed` contains these bytes as 18 hexadecimal characters. The existing
Buddy 1 transport uses NUL-terminated JSON, so binary bytes must not be passed
to `comms_publish()`. This implementation supplies Buddy 4's compact payload
without changing the team's entire MQTT protocol. Full binary transport,
one-byte binary event messages, and a shared system fault byte require Buddy 1
integration. Runtime control values stay unpacked.

### Change Requirement 1: Buddy 4 data representation

The shared snapshot `buddy4_imu_state_t` (copied by other tasks under DI/EI)
keeps flags, states and counters in one 32-bit bit-field word. Only the IMU task
writes it, and readers copy it whole, so packed read-modify-write is safe on
the single-core build.

| Field | Valid range | Bits | Representation |
|---|---|---:|---|
| `mag_ok`, `accel_ok`, `height_valid`, `imu_fault`, `calibrated` | 0–1 | 1 each | bit-field |
| `hump_state` | 0–3 | 2 | bit-field |
| `hump_count` | 0–7 (saturating) | 3 | bit-field |
| `motion_class` | 0–7 | 3 | bit-field |
| `impact_count` | 0–255 (saturating) | 8 | bit-field |
| `heading_deg_x10` | 0–3599 | 12 | `int16_t` in RAM, 12 bits in the packet |
| `pitch_deg_x10`, `hump_peak_deg_x10` | -900–900 | 11 | `int16_t` in RAM, 11 bits in the packet |
| `roll_deg_x10` | -1800–1800 | 12 | `int16_t` in RAM, 12 bits in the packet |
| `hump_height_mm` | 0–255 sent | 8 | `uint16_t` in RAM, saturated to 8 bits in the packet |
| `hump_sequence`, `sample_interval_ms` | 0–65535 | 16 | `uint16_t` (was `uint32_t`) |
| `*_timestamp_ms` | uptime | 32 | `uint32_t` (16 bits would wrap after 65 s) |

Angles stay `int16_t` in RAM because they are signed values used in maths;
signed bit-fields are not allowed by Barr-C. They are packed to their minimum
width in `imu_packed`.

| Measurement (arm-none-eabi-gcc, Cortex-M0+) | Before | After |
|---|---:|---|
| `buddy4_imu_state_t` (copied on every `buddy4_get_state()`) | 60 bytes | 36 bytes |
| `buddy4_processor_t` (IMU task only) | 168 bytes | 196 bytes (speed/turn windows, flat attitude for humps) |
| Wi-Fi build `.bss` (whole firmware) | 49 980 bytes | 49 972 bytes |
| USB build `.text` (whole firmware) | 63 244 bytes | 64 292 bytes (new features) |
| Buddy 4 packet | 9 bytes | 9 bytes (motion class uses the 3 spare bits) |

## Validation performed

A host test of `buddy4_processing.c` (kept outside the repository, like the
earlier one) passes. It checks: stationary and cruising
classes; a complete hump with valid height and highest peak; one 1.8 g jolt in
the climb keeps the hump and counts one impact; five bad samples in a row
cancel the hump; a 90-degree turn in 1 s gives 90 deg/s and the turning class;
accelerating and decelerating; stale odometry gives unknown; a slow pivot is not
stationary; an uncalibrated hump has no valid height; packet version 2, motion
bits and the overflow bit. The same jolt check against the previous code
showed the old behaviour: the hump was cancelled.

A code review then led to these fixes, each with its own host check: a stopped
or reversing step during a skipped sample now spoils the height; travel over several skipped samples is checked one step
at a time (100 mm per sample), not as one total; impacts are counted at most
once per 500 ms from the counted one, so a later impact is not hidden; a
sampling gap no longer brings back an old turn rate.

Both profiles build with `-Wall -Wextra` and no Buddy 4 warnings: USB (`SMP=0`, `CONSOLE=usb_cdc`) and Wi-Fi MQTT. Nothing has been
flashed or tested on the rover.

Before accepting Buddy 4 on the rover:

1. Log raw acceleration, filtered tilt and heading while level, motors off/on.
2. Confirm nose-up sign and measured ±90-degree turns, including 359→0 wrap.
3. Measure both wheels and update Buddy 2 calibration; validate forward distance.
4. Apply measured IMU calibration. Compare estimates with several measured humps
   at intended speeds; record error and tune filtering/thresholds as needed.
5. Check no hump on flat starts/stops, one event per hump, and no completed event
   after sensor disconnection or an interrupted ascent.
6. Disconnect the IMU during Buddy 5 turning and verify its existing fault-stop
   path. Log actual IMU sampling intervals during ultrasonic scans and Wi-Fi load.
7. Confirm the dashboard updates through a real broker and that reconnects show
   the latest retained-in-RAM sequence/results. Power cycling resets the run.
