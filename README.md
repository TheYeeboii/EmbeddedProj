# CSC04 Autonomous Robot Car

Robot car for the INF2004 Embedded Systems project: a Raspberry Pi Pico W on a
Cytron Robo Pico, running **micro T-Kernel 3.0** in **C** with the **Pico C SDK**.
It follows a line, reads barcodes, detects humps, avoids obstacles and reports
telemetry over Wi-Fi (MQTT) to a browser dashboard.

## Repository layout

| Folder | What it is |
|---|---|
| `firmware/` | The course template ([sirfonzie/mtk3smp-rp2040](https://github.com/sirfonzie/mtk3smp-rp2040)) with our robot code in `firmware/app_program/`. Everything that runs on the Pico. |
| `firmware/app_program/` | **All of our code.** Every `.c` file here is compiled automatically. |
| `firmware/config/` | Kernel settings, MQTT broker IP (`mqtt_config.h`), Wi-Fi credentials (`wifi_credentials.h`, not committed). |
| `firmware/build_make/` | Makefiles. `robot.mk` adds the Pico SDK parts we use. |
| other `firmware/` folders | The micro T-Kernel itself (kernel, headers, libraries, drivers). Leave alone. |
| `dashboard/` | Browser dashboard (runs on the laptop). |
| `docs/` | Per-buddy design, calibration and test notes. |
| `tools/` | Mosquitto (MQTT broker) config for the laptop. |

## Code in `firmware/app_program/`

| File | Buddy | What it does |
|---|---|---|
| `app_main.c/.h` | all | `usermain()`: initialises everything and creates the tasks |
| `robot_controller.c/.h` | all | Vehicle controller: modes, dashboard commands, barcode actions, buttons, safety stop |
| `comms.c/.h` | 1 | Message buffers between robot tasks and the network |
| `robot_mqtt.c/.h` | 1 | MQTT client (runs inside the kernel's Wi-Fi task) |
| `robot_telemetry.c/.h` | 1 | Builds the messages the dashboard shows |
| `buddy2_motion.c/.h` | 2 | Motors, encoders, PID, `move_forward` / `turn_left` / ... |
| `buddy3_ir.c/.h` | 3 | IR line following, line search, barcode decoder |
| `buddy4_imu.c/.h`, `buddy4_processing.c/.h` | 4 | LSM303 IMU: heading, tilt, filtering, hump detection |
| `buddy5_ultrasonic.c/.h` | 5 | Ultrasonic distance + scanning servo |
| `buddy5_avoidance.c/.h` | 5 | Obstacle avoidance and return to the line |
| `robot_hal.c/.h` | - | **The only file that includes Pico SDK `hardware/*.h`** |
| `gpio_irq.c/.h` | - | The single shared GPIO interrupt handler |
| `pin_config.h` | - | All pin numbers |
| `usb_console_compat.h` | - | USB console helper |

### RTOS tasks

| Task | Priority (lower = more urgent) | Runs | Job |
|---|---|---|---|
| usb (kernel) | 3 | on demand | USB serial console |
| cyw43 (kernel) | 4 | about every 2 ms | Wi-Fi + lwIP + MQTT (Wi-Fi builds only) |
| motion | 5 | every 20 ms (cyclic handler) | Buddy 2 motor and encoder control |
| line | 6 | every 10 ms (cyclic handler) | Buddy 3 line sensors |
| controller | 7 | every 20 ms, or on command | commands, barcodes, GP21 button, safety |
| imu | 9 | every 50 ms | Buddy 4 IMU and hump processing |
| sensors | 10 | every 100 ms | Buddy 5 ultrasonic and avoidance |
| telemetry | 15 | every 1 s | console status + MQTT publish |

### Buttons and testing

- **GP21**: start/stop line following.
- Distance/turn moves have no dashboard control (the Motion panel was removed on 26 Oct). They run only for barcode actions and Buddy 5's obstacle avoidance.

## Build and flash (Windows, using WSL)

**One-time setup** (Ubuntu/WSL terminal):

```sh
sudo apt update
sudo apt install gcc-arm-none-eabi build-essential git
git clone -b 2.2.0 https://github.com/raspberrypi/pico-sdk.git ~/pico/pico-sdk
cd ~/pico/pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
```

**Wi-Fi settings:** copy `firmware/config/wifi_credentials.example.h` to
`firmware/config/wifi_credentials.h` and fill in the hotspot name and password
(never commit this file). The broker (laptop) IP is in `firmware/config/mqtt_config.h`.

**Build** (from `firmware/build_make`):

```sh
cd "/mnt/c/Users/<you>/Desktop/Git Desktop/EmbeddedProj/firmware/build_make"

# Without Wi-Fi
make CONSOLE=usb_cdc PICO_SDK_PATH=~/pico/pico-sdk -j8

# With Wi-Fi + MQTT (dashboard)
make CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_MQTT=1 \
     PICO_SDK_PATH=~/pico/pico-sdk -j8
```

The output is a `.uf2` file in `build_make/`. Hold BOOTSEL, plug in the Pico,
and copy the file to the `RPI-RP2` drive. Always build with `CONSOLE=usb_cdc`;
the serial console is the USB port.

## Laptop side: MQTT broker and dashboard

Start the broker (Mosquitto) from the repository root:

```powershell
& "C:\Program Files\Mosquitto\mosquitto.exe" -c "$PWD\tools\mosquitto-local.conf" -v
```

It listens on `192.168.137.1:1883` (robot) and `192.168.137.1:9001`
(WebSockets, dashboard). It allows anonymous access, so use it only on the
private hotspot.

Start the dashboard in another terminal and open <http://localhost:8000>:

```powershell
python -m http.server 8000 --directory dashboard
```

To use a different broker, open `http://localhost:8000/?broker=HOST_OR_IP`.

### MQTT topics

| Topic | Direction | Content |
|---|---|---|
| `rover/commands` | dashboard → robot | Commands, e.g. `{"cmd":"stop"}`, `{"cmd":"drive","left":30000,"right":30000}`, `line_follow`, `line_stop`, `motor_trim`, `speed_limit`, `line_speed` |
| `rover/telemetry` | robot → dashboard | Mode, IR, ultrasonic, heading, tilt, encoders, motor settings; IMU/hump report |
| `rover/debug` | robot → dashboard | Line-sensor detail, barcode state, avoidance scan and status |
| `rover/heartbeat` | robot → dashboard | Periodic "online" message |
| `rover/status` | robot → dashboard | Retained "offline" Last Will |

A raw drive command stops the motors if it is not refreshed within one second.

## Rules for adding code

1. **Never include `hardware/*.h` and `<tk/tkernel.h>` in the same `.c` file.**
   They clash (`GPIO_IN`, `SIO_BASE`, `size_t`, and more). Put SDK calls in
   `robot_hal.c` and expose a small `hal_...()` function in `robot_hal.h`. If
   the SDK function lives in a `.c` file, add it to `firmware/build_make/robot.mk`.
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
| `firmware/config/config.h` | `USE_SDEV_DRV 0` | The kernel's sample I2C driver claimed GP8/GP9 (left motor, M1). We use the SDK for ADC/I2C instead |
| `firmware/build_make/pico_rp2040.mk` | `include robot.mk`, `WIFI_MQTT` option | Pico SDK libraries; MQTT build profile |
| `firmware/build_make/makefile` | links `$(LIBS_ROBOT)` (`-lm`) | maths functions for the IMU |
| `firmware/lib/libnet/lwip/include/lwipopts.h` | TCP on for MQTT, bigger MQTT buffer | MQTT needs TCP |
| `firmware/lib/libwifi/.../cyw43_utk.c` | calls `robot_mqtt_poll()` | MQTT runs in the radio task |

## Documentation

Per-buddy notes are in [`docs/`](docs): motion calibration (Buddy 2), line and
barcode (Buddy 3), IMU and humps (Buddy 4), ultrasonic and avoidance (Buddy 5).
