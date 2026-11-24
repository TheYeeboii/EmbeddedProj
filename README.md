# CSC04 Autonomous Robot Car

Robot car for the INF2004 Embedded Systems project: a Raspberry Pi Pico W on a
Cytron Robo Pico, running **micro T-Kernel 3.0** in **C** with the **Pico C SDK**.
It follows a line, reads barcodes, detects humps, avoids obstacles and reports
telemetry over Wi-Fi (MQTT) to a browser dashboard.

## Repository layout

The robot and the dashboard exchange packed binary MQTT messages; the layouts
are in [MQTT_BINARY_FORMAT.md](docs/MQTT_BINARY_FORMAT.md). Four diagnostic
streams feed the dashboard's evidence views, such as the **Steering actions** log
([MQTT_ACTION_LOG.md](docs/MQTT_ACTION_LOG.md)).

The layout follows the course template
([sirfonzie/INF2004_ProjectTemplate](https://github.com/sirfonzie/INF2004_ProjectTemplate)):
the micro T-Kernel project is the repository root, with one folder per
subsystem (buddy) in `app_program/` and `testing/`.  Owners are in
[TEAM.md](TEAM.md).

| Folder | What it is |
|---|---|
| `app_program/` | **All of our code.** Shared files at the top, one folder per subsystem (`subsystem1/` … `subsystem5/`). Every `.c` file here and one folder down is compiled automatically. |
| `config/` | Kernel settings, MQTT broker IP (`mqtt_config.h`), Wi-Fi credentials (`wifi_credentials.h`, not committed). |
| `build_make/` | Makefiles. Plain `make` builds the rover's full setup. |
| `lib/liblwip/sysdepend/pico_rp2040/` | lwIP port and the MQTT bridge (`mqtt_utk.c`), run by the kernel's Wi-Fi task. |
| `kernel/`, `include/`, `lib/`, `device/`, `etc/` | The micro T-Kernel itself (kernel, headers, libraries, drivers). Leave alone. |
| `testing/` | Host tests, one folder per subsystem plus `integration/` and `system/`; `run_*.sh` run them on the laptop, no robot needed. |
| `dashboard/` | Browser dashboard (runs on the laptop). |
| `docs/` | Per-buddy design, calibration and test notes; kernel port notes. |
| `tools/` | Mosquitto (MQTT broker) config for the laptop; BARR-C formatter and checker; `elf2uf2`. |

## Code in `app_program/`

All of it follows the BARR-C:2018 embedded C coding standard; see
[CODING_STANDARD.md](docs/CODING_STANDARD.md) for the formatter, checker and
the few deliberate deviations.

| File | Subsystem | What it does |
|---|---|---|
| `app_main.c/.h` | shared | `usermain()`: initialises everything and creates the tasks |
| `robot_controller.c/.h` | shared | Vehicle controller: modes, dashboard commands, barcode moves at junctions, GP21 button, safety stops |
| `robot_hal.c/.h` | shared | **The only file that includes Pico SDK `hardware/*.h`** |
| `gpio_irq.c/.h` | shared | The single shared GPIO interrupt handler |
| `robot_fault.c/.h` | shared | The shared fault byte |
| `pin_config.h` | shared | All pin numbers |
| `robot_types.h`, `usb_console_compat.h` | shared | `float32_t` (BARR-C fixed-width float type); USB console helper |
| `subsystem1/comms.c/.h` | 1 | Message buffers between robot tasks and the network |
| `subsystem1/robot_telemetry.c/.h` | 1 | Collects the values the dashboard shows |
| `subsystem1/robot_wire.c/.h`, `robot_packed.c/.h`, `robot_wire_transport.c` | 1 | Binary message layouts and bit packing |
| `subsystem1/robot_diagnostics_mqtt.c/.h` | 1 | Diagnostic streams for the dashboard's tuning views |
| `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c/.h` | 1 | MQTT client (runs inside the kernel's Wi-Fi task, not in `app_program/`) |
| `subsystem2/buddy2_motion.c/.h` | 2 | Motors, encoders, PID, `move_forward` / `turn_left` / ... |
| `subsystem2/robot_action_log.c/.h` | 2 | Motor action log for the dashboard |
| `subsystem3/buddy3_ir.c/.h` | 3 | IR sensors, line following, end-of-line stop; feeds the barcode reader |
| `subsystem3/buddy3_steering.c/.h`, `buddy3_junction.c/.h` | 3 | Steering state machine; straight across junctions |
| `subsystem3/buddy3_barcode.c/.h`, `code39.c/.h` | 3 | Barcode reader: black/white and wheel travel to a Code 39 letter |
| `subsystem3/barcode_slowdown.c/.h`, `barcode_tuning*.c/.h` | 3 | Barcode slowdown and IR threshold tuning |
| `subsystem3/robot_steering_trace`, `robot_barcode_trace`, `robot_ir_stream` | 3 | Line/barcode diagnostic streams |
| `subsystem4/buddy4_imu.c/.h`, `buddy4_processing.c/.h` | 4 | LSM303 IMU: heading, tilt, filtering, hump detection, lifted/overturned motor stop |
| `subsystem5/buddy5_ultrasonic.c/.h` | 5 | Ultrasonic distance + scanning servo |
| `subsystem5/buddy5_avoidance.c/.h` | 5 | Obstacle avoidance, width measurement and return to the line |

### RTOS tasks

| Task | Priority (lower = more urgent) | Runs | Job |
|---|---|---|---|
| usb (kernel) | 3 | on demand | USB serial console |
| cyw43 (kernel) | 4 | about every 2 ms | Wi-Fi + lwIP + MQTT (Wi-Fi builds only) |
| motion | 5 | every 20 ms (cyclic handler) | Buddy 2 motor and encoder control |
| line | 6 | every 2 ms (cyclic handler) | Buddy 3: barcode sensor every 2 ms; line sensors and steering every 10 ms |
| controller | 7 | every 20 ms, or on command | commands, barcodes, GP21 button, safety |
| imu | 9 | every 50 ms | Buddy 4 IMU and hump processing |
| sensors | 10 | every 100 ms | Buddy 5 ultrasonic and avoidance |
| telemetry | 15 | every 1 s | console status + MQTT publish |

### Buttons and testing

- **GP21**: start/stop line following.
- Barcodes are read while line following, or while driving straight forward
  with W. While line following, **A** turns left and **C** turns right at the
  next junction; any other letter goes straight on
  ([BARCODE_ACTIONS.md](docs/BARCODE_ACTIONS.md)).
- Distance/turn moves have no dashboard control; obstacle avoidance and the
  barcode junction turns use them.
- Host tests (no robot needed): `CC=gcc sh testing/run_barcode_tests.sh` and
  `CC=gcc sh testing/run_ir_diagnostics_tests.sh`.

## Getting started (Windows laptop)

What each tool is for:

| Tool | Runs on | Why we need it |
|---|---|---|
| **Ubuntu (WSL)** | Windows, as a Linux app | Compiles the code into a `.uf2` file for the Pico. The course template builds with `make` and Linux commands, so it must run in Linux. |
| **Pico SDK** | inside Ubuntu | Raspberry Pi's library for pins, PWM, ADC, Wi-Fi. |
| **Mosquitto** | Windows | The MQTT broker (server). The Pico and the dashboard both talk to it. |
| **Python** | Windows | Serves the dashboard web page. |
| **PuTTY** (or VS Code Serial Monitor) | Windows | Shows the Pico's console (status lines, errors). |

> The Raspberry Pi Pico VS Code extension's **Compile Project** button does
> **not** work here: it only builds CMake projects, and this project uses the
> template's makefiles. VS Code is still fine for editing, and you can open an
> Ubuntu terminal inside it (**View → Terminal**, the **⌄** next to **+**,
> pick **Ubuntu (WSL)**).

### 1. One-time setup

**a) Install Ubuntu.** "WSL" is the Windows feature; Ubuntu is the Linux you
install inside it. In PowerShell:

```powershell
wsl --install -d Ubuntu
```

Restart if asked, then open **Ubuntu** from the Start menu (not "WSL") and
create a username and password (the password is invisible while typing).
If a terminal shows `-sh: sudo: not found`, you are in a different Linux, not
Ubuntu.

**b) Install the compiler and download the Pico SDK.** In Ubuntu:

```sh
sudo apt update
sudo apt install gcc-arm-none-eabi build-essential git
git clone -b 2.2.0 https://github.com/raspberrypi/pico-sdk.git ~/pico/pico-sdk
cd ~/pico/pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
```

**c) On Windows,** install [Mosquitto](https://mosquitto.org/download/)
(default folder), [Python](https://www.python.org/downloads/) and PuTTY.

**d) Wi-Fi details.** Copy `config/wifi_credentials.example.h` to
`config/wifi_credentials.h`, fill in your laptop hotspot's name and
password, and set `UTK_WIFI_CREDENTIALS_SET` to `1`. This file is ignored by
Git: **never commit it**.

### 2. Build the `.uf2` (every time the code changes)

Copy `config/mqtt_config.example.h` to
`config/mqtt_config.h` and set `ROBOT_MQTT_BROKER_IP` to your laptop's
IP on the rover's Wi-Fi network. The local MQTT configuration is ignored by Git.

In Ubuntu (change `<you>` to your Windows user name; `/mnt/c/` is your C: drive):

```sh
cd "/mnt/c/Users/<you>/Desktop/Git Desktop/EmbeddedProj/build_make"
make PICO_SDK_PATH=~/pico/pico-sdk -j8
```

It is done when the prompt comes back with no `Error`. The result is
`build_make/mtk3pico_smp0_usb_cdc_wifi_mqtt.uf2`. Plain `make` builds
the rover's full setup: USB console, Wi-Fi + MQTT and the dashboard's
diagnostic streams.

**Windows without WSL**, in Git Bash, with the toolchain the Raspberry Pi
Pico VS Code extension installs and MSYS2's `mingw32-make`:

```sh
export PATH="$HOME/.pico-sdk/toolchain/15_2_Rel1/bin:/c/msys64/ucrt64/bin:$PATH"
export TMP="$(cygpath -m "$LOCALAPPDATA/Temp")" TEMP="$(cygpath -m "$LOCALAPPDATA/Temp")"
cd build_make
mingw32-make PICO_SDK_PATH="C:/Users/<you>/.pico-sdk/sdk/2.3.1" -j8
~/.pico-sdk/picotool/2.3.1/picotool/picotool.exe uf2 convert mtk3pico_smp0_usb_cdc_wifi_mqtt.elf mtk3pico_smp0_usb_cdc_wifi_mqtt.uf2 --family rp2040
```

The make step ends with `Error 193`: the template's `tools/elf2uf2` is a Linux
program. The `.elf` is complete by then, and the `picotool` line makes the
`.uf2`. Setting `TMP`/`TEMP` stops gcc trying to write its temporary files to
`C:\Windows`.

### 3. Flash the Pico

Hold **BOOTSEL**, plug the Pico into USB, and drag the `.uf2` file onto the
`RPI-RP2` drive. The Pico restarts by itself.

### 4. Start the laptop side (Windows)

1. **Hotspot:** Settings → Network → Mobile hotspot. Set the band to
   **2.4 GHz** (the Pico W cannot use 5 GHz) and turn it on. The laptop is
   then `192.168.137.1`, which the firmware and dashboard expect (broker IP
   is in `config/mqtt_config.h`).
2. **Broker:** PowerShell in the `EmbeddedProj` folder:
   ```powershell
   & "C:\Program Files\Mosquitto\mosquitto.exe" -c "$PWD\tools\mosquitto-local.conf" -v
   ```
   Click **Allow** if the firewall asks. If it cannot use the address, the
   hotspot is not on yet. It allows anyone on the hotspot to connect, so use
   it only on your own private hotspot.
3. **Dashboard:** a second PowerShell in the same folder:
   ```powershell
   python -m http.server 8000 --directory dashboard
   ```
   Open <http://localhost:8000>. The laptop needs internet (the page loads
   two scripts online). For a different broker:
   `http://localhost:8000/?broker=HOST_OR_IP`.

### 5. Check it works

- **Console:** PuTTY → Serial, the Pico's COM port (Device Manager → Ports),
  speed 115200. You should see Wi-Fi join, then MQTT connect.
- **Mosquitto window:** a client `buddy1` connects.
- **Dashboard:** shows online and the numbers update.

### MQTT topics

Every payload is packed binary; byte 0 tells the dashboard which message it is
([MQTT_BINARY_FORMAT.md](docs/MQTT_BINARY_FORMAT.md)).

| Topic | Direction | Content |
|---|---|---|
| `rover/commands` | dashboard → robot | 1-5 byte commands: stop, drive, line follow / stop, trim, speed limit, line speed, servo, barcode settings, reset |
| `rover/telemetry` | robot → dashboard | Every 1 s: mode, faults, IR, ultrasonic, heading, tilt, encoders, motor settings (28 B); Buddy 4's IMU/hump report (14 B) |
| `rover/debug` | robot → dashboard | Every 1 s: line-sensor detail, barcode letter, avoidance scan and status (25 B) |
| `rover/heartbeat` | robot → dashboard | 1 byte every 10 s |
| `rover/status` | robot → dashboard | Retained 1 byte: online, or the "offline" Last Will |
| `rover/actions`, `rover/steering`, `rover/barcode`, `rover/ir` | robot → dashboard | Diagnostic streams for the dashboard's tuning views |

A raw drive command stops the motors if it is not refreshed within one second.

## Rules for adding code

1. **Never include `hardware/*.h` and `<tk/tkernel.h>` in the same `.c` file.**
   They clash (`GPIO_IN`, `SIO_BASE`, `size_t`, and more). Put SDK calls in
   `robot_hal.c` and expose a small `hal_...()` function in `robot_hal.h`. If
   the SDK function lives in a `.c` file, add it to `build_make/robot.mk`.
2. **GPIO interrupts go through `gpio_irq_register()`.** All pins share one interrupt.
3. **No `sleep_ms()` or `printf()`.** Use `tk_dly_tsk()` and `tm_printf((UB *)"...")`.
   `tm_printf` has no `%f`, so print tenths as integers (see `robot_telemetry.c`).
4. **Don't include `<math.h>` or `<string.h>` next to `<tk/tkernel.h>`.** They
   clash on `size_t`. Declare the one function you need instead.
5. **Data shared between tasks** is copied under `DI()` / `EI()` (short structs)
   or a kernel mutex (see `buddy2_motion.c`).
6. **Never call lwIP from a robot task.** Use `comms_publish()` and `comms_wait_command()`.
7. **Don't return from `usermain()`.** Returning shuts the kernel down.
8. Follow the **Barr-C Embedded C Coding Standard (2018)**.

### Kernel settings we changed

| File | Change | Why |
|---|---|---|
| `config/config.h` | `USE_SDEV_DRV 0` | The kernel's sample I2C driver claimed GP8/GP9 (left motor, M1). We use the SDK for ADC/I2C instead |
| `config/config.h` | `CNF_TIMER_PERIOD 1` (was 10) | 1 ms kernel tick, so the barcode sensor can be read every 2 ms |
| `build_make/pico_rp2040.mk` | `include robot.mk`, `WIFI_MQTT` option | Pico SDK libraries; MQTT build profile |
| `build_make/makefile` | links `$(LIBS_ROBOT)` (`-lm`) | maths functions for the IMU |
| `lib/liblwip/sysdepend/pico_rp2040/include/lwipopts.h` | TCP on for MQTT, bigger MQTT buffer | MQTT needs TCP |
| `lib/libwifi/.../cyw43_utk.c` | calls `robot_mqtt_poll()` | MQTT runs in the radio task |

## Documentation

Per-buddy notes are in [`docs/`](docs): motion calibration (Buddy 2), line and
barcode (Buddy 3), IMU and humps (Buddy 4), ultrasonic and avoidance (Buddy 5).
