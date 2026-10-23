# Buddy 4 implementation and rover validation

Implemented 23 October 2026. Software tests and the USB firmware build pass;
this is not a claim of completed physical calibration or track acceptance.

## What changed

- `buddy4_imu.c/.h`: LSM303 I2C driver, accelerometer block-data-update enabled
  (`CTRL_REG4_A = 0x88`), configuration retry after disconnect, queued calibration
  and peak reset, independent IMU task, freshness checks and telemetry.
- `buddy4_processing.c/.h`: portable processing, calibration offsets/scales,
  median-of-three plus time-aware low-pass tilt filtering, hump lifecycle,
  height estimation, compact serialization. No kernel or SDK dependencies.
- `app_main.c`: starts the IMU task at priority 9 with a target 50 ms period.
  Buddy 5 (priority 10) no longer performs I2C updates. Motion, line and controller
  retain priorities 5, 6 and 7. This adds a 4096-byte task stack from the kernel pool.
- `robot_telemetry.c`: accelerometer failures no longer appear as all sensors OK.
- Dashboard: separate IMU/hump panel; independent IMU reports do not clear the
  existing heading, distance, motor or line tiles.

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
The getter invalidates readings older than 250 ms. Gaps over 250 ms, duplicate
timestamps, or invalid acceleration cancel an unfinished hump. Configuration
is retried every second after an I2C failure.

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

Preserve the original mounting convention: level Z approximately +1000 mg,
and nose-up produces negative X. Verify Y/roll direction on the rover. This
implementation preserves `atan2(mag_y, mag_x)` heading, with hard-iron offsets,
axis scales and a heading offset. It is **not tilt-compensated** and does not
correct general cross-axis soft-iron distortion. Validate heading on the surface
where Buddy 5 turns; mounting errors and motor fields remain physical concerns.

## Hump detection and height

The four states are flat, climbing, top, descending. Entry requires three
filtered readings at or above +6 degrees. Climbing becomes top below +3 degrees;
three readings at or below -6 degrees confirm descending. Three readings within
±3 degrees complete the hump. A 15-second timeout cancels incomplete events.
These initial thresholds require track tuning. An isolated ascent/ramp does not
count as a completed hump. Detection describes the observed tilt sequence;
manually tilting the sensor can also produce that sequence.

`hump_count` saturates at seven for the 3-bit representation. `hump_sequence`
increments on every completed hump, so telemetry consumers can detect further
events. Peak pitch and the last completed height are retained; `highest_hump_mm`
retains the largest valid height estimate since initialization/calibration reset.

Height uses average wheel travel and pitch:

`elevation += delta_distance * sin(average(previous_pitch, current_pitch))`

It records the maximum elevation during the event, including the entry debounce
window. This is a slope-and-distance estimate, not double-integrated acceleration.
Both wheel speeds must indicate forward movement and odometry must be fresh.
Negative/reset distance, steps above 100 mm per sample, reverse/pivot/stopped
samples, or absent calibration invalidate the event's height. Buddy 2 signs
distance from commanded direction; wheel slip, coasting, external pushes,
wheelbase/chassis geometry and filter lag still affect accuracy. Starting the
integration at the +6-degree threshold can underestimate a gradual approach.
The estimator is intended for slow, forward traversal of known test humps.

## Telemetry and compact representation

The IMU task publishes a separate small JSON object on `rover/telemetry` every
second and immediately after a completed hump. It carries fault/calibration,
roll, hump state/count/sequence/peak, last and highest height, height validity,
sample interval and `imu_packed`. A completed event is also printed on USB.
If the nonblocking mailbox drops a report, the retained sequence and results
are repeated in subsequent periodic reports. This is not guaranteed event
delivery or an event history queue.

`buddy4_pack()` serializes nine bytes explicitly, least-significant bit first
within each byte. Signed fields use the listed width's two's complement. It
returns false and sets the overflow bit if any field required saturation.

| Bit offset | Width | Meaning |
|---|---:|---|
| 0 | 4 | Version = 1 |
| 4, 5, 6, 7 | 1 each | Compass OK, accel OK, height valid, calibrated |
| 8 | 12 | Heading in tenths, 0–3599 |
| 20 | 11 | Signed pitch in tenths, -900–900 |
| 31 | 12 | Signed roll in tenths, -1800–1800 |
| 43 | 11 | Signed hump peak pitch in tenths |
| 54 | 2 | Hump state |
| 56 | 3 | Saturating hump count |
| 59 | 8 | Last height, saturated at 255 mm |
| 67 | 1 | Packing overflow |
| 68 | 1 | IMU fault |
| 69 | 3 | Reserved zero |

`imu_packed` contains these bytes as 18 hexadecimal characters. The existing
Buddy 1 transport uses NUL-terminated JSON, so binary bytes must not be passed
to `comms_publish()`. This implementation supplies Buddy 4's compact payload
without changing the team's entire MQTT protocol. Full binary transport,
one-byte binary event messages, and a shared system fault byte require Buddy 1
integration. Runtime control values and raw diagnostic arrays stay unpacked;
raw magnetic readings must not be blindly truncated to 12 bits.

## Validation performed

The host test sources and runner were removed after validation at the user's
request. The completed tests covered flat readings, isolated spikes, complete/incomplete/repeated humps,
height validity and calibration gating, rejected acceleration, magnetic
overflow, I2C failures and recovery, stale snapshots, timer wraparound, distance
reset/reverse movement, packed-field boundaries, and dashboard report isolation.

The USB (`SMP=0`, `CONSOLE=usb_cdc`) firmware was built using ARM GCC 15.2 and
Pico SDK 2.3.0. Output: `firmware/build_make/mtk3pico_smp0_usb_cdc.uf2`.
It has not been flashed. The MQTT profile additionally needs the project's
local `firmware/config/wifi_credentials.h`; that file was absent during this work.

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
