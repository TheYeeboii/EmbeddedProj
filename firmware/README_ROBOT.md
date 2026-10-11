# Robot car firmware (micro T-Kernel 3.0)

This folder is the whole robot firmware: the micro T-Kernel 3.0 port for the
Pico W ([sirfonzie/mtk3smp-rp2040](https://github.com/sirfonzie/mtk3smp-rp2040))
plus our application in `app_program/`.

**Where to put code:** `app_program/`. Every `.c` file there is compiled
automatically. Everything else in this folder is the kernel; leave it alone
unless a section below tells you to change something.

---

## What's in `app_program/`

| File | Buddy | What it does |
|---|---|---|
| `app_main.c/.h` | all | `usermain()`: initialises everything and creates the tasks |
| `robot_controller.c/.h` | all | The vehicle controller: dashboard commands, barcode actions, buttons, safety stop |
| `buddy2_motion.c/.h` | 2 | Motors, encoders, PID, `move_forward` / `turn_left` / ... |
| `buddy2_motion_test.c/.h` | 2 | GP20 button motion test (calibration) |
| `buddy3_ir.c/.h` | 3 | IR line following + barcode decoder |
| `buddy4_imu.c/.h` | 4 | LSM303 compass heading + accelerometer tilt |
| `buddy5_ultrasonic.c/.h` | 5 | Ultrasonic distance (interrupt-timed) + scanning servo |
| `comms.c/.h` | 1 | Message buffers between robot tasks and the network |
| `robot_mqtt.c/.h` | 1 | MQTT client (runs inside the kernel's Wi-Fi task) |
| `robot_telemetry.c/.h` | 1 | Builds the JSON the dashboard shows |
| `robot_hal.c/.h` | - | **The only file that includes Pico SDK `hardware/*.h`** |
| `gpio_irq.c/.h` | - | The single shared GPIO interrupt handler |
| `pin_config.h` | - | All pin numbers |
| `demo_tasks.c/.h`, `usb_console_compat.h` | - | GP16 "alive" blink task (kept from the kernel demo) |

### Tasks

| Task | Priority | Runs | Job |
|---|---|---|---|
| (usb) | 3 | kernel | USB serial console |
| (cyw43) | 4 | every 2 ms, kernel | Wi-Fi + lwIP + MQTT (Wi-Fi builds only) |
| motion | 5 | every 20 ms (cyclic handler) | `buddy2_update()` |
| line | 6 | every 10 ms (cyclic handler) | `buddy3_update()` |
| controller | 7 | every 20 ms, or on command | commands, barcodes, GP21 button, safety |
| sensors | 10 | every 100 ms | IMU; ultrasonic + servo step every 500 ms |
| telemetry | 15 | every 1 s | console print + MQTT publish |
| motion_test | 20 | GP20 button | Buddy 2 calibration run |
| blink | 30 | 250 ms | GP16 LED |

Lower number = higher priority.

### Buttons

- **GP20**: motion test (forward 500 mm, left 90°, right 90°, back 500 mm)
- **GP21**: start/stop line following

---

## Build (Windows, using WSL)

**One-time setup** (Ubuntu/WSL terminal):

```sh
sudo apt update
sudo apt install gcc-arm-none-eabi build-essential git
git clone -b 2.2.0 https://github.com/raspberrypi/pico-sdk.git ~/pico/pico-sdk
cd ~/pico/pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
```

**Build** (from `firmware/build_make`):

```sh
cd "/mnt/c/Users/ahmed/Desktop/Git Desktop/EmbeddedProj/firmware/build_make"
# cd /mnt/c/Users/MewXe/Desktop/Embedded_First/EmbeddedProj/firmware/build_make/

# Without Wi-Fi
make CONSOLE=usb_cdc PICO_SDK_PATH=~/pico/pico-sdk -j8

# With Wi-Fi + MQTT (dashboard)
make CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_MQTT=1 \
     PICO_SDK_PATH=~/pico/pico-sdk -j8
```

The output is a `.uf2` file in `build_make/`. Hold BOOTSEL, plug in the Pico, and
copy the file to the `RPI-RP2` drive.

Ready-built copies are in `prebuilt/`:

| File | What |
|---|---|
| `robot_usb.uf2` | Everything except Wi-Fi |
| `robot_wifi_mqtt.uf2` | Everything, with Wi-Fi + MQTT. Joins `WEISONG 4706` and uses the broker at `192.168.137.1` (the settings from the old CMake project) |

### Wi-Fi settings

- **Network name and password:** `config/wifi_credentials.h`. This file is not
  committed; copy `config/wifi_credentials.example.h` to create it.
- **Broker (laptop) IP:** `config/mqtt_config.h`

Always build with `CONSOLE=usb_cdc`. The ultrasonic sensor uses GP0/GP1, which
are also the kernel's UART console pins, so the UART console stops working
once the robot starts.

---

## Rules for adding code

1. **Never include `hardware/*.h` and `<tk/tkernel.h>` in the same `.c` file.**
   They clash (`GPIO_IN`, `SIO_BASE`, `size_t`, and more). Put SDK calls in
   `robot_hal.c` and expose a small `hal_...()` function in `robot_hal.h`. If
   the SDK function lives in a `.c` file, add it to `build_make/robot.mk`.
2. **GPIO interrupts go through `gpio_irq_register()`.** All pins share one
   interrupt.
3. **No `sleep_ms()` or `printf()`.** Use `tk_dly_tsk()` and `tm_printf((UB *)"...")`.
   `tm_printf` has no `%f`, so print tenths as integers (see `robot_telemetry.c`).
4. **Don't include `<math.h>` or `<string.h>` next to `<tk/tkernel.h>`.** They
   clash on `size_t`. Declare the one function you need instead (see `buddy4_imu.c`).
5. **Data shared between tasks** is copied under `DI()` / `EI()` (short structs)
   or a kernel mutex (see `buddy2_motion.c`).
6. **Never call lwIP from a robot task.** Use `comms_publish()` and
   `comms_wait_command()`.
7. **Don't return from `usermain()`.** Returning shuts the kernel down.

### Kernel settings we changed

| File | Change | Why |
|---|---|---|
| `config/config.h` | `USE_SDEV_DRV 0` | The kernel's sample I2C driver claimed GP8/GP9 (right motor, M1). We use the SDK for ADC/I2C instead |
| `build_make/pico_rp2040.mk` | `include robot.mk`, `WIFI_MQTT` option | Pico SDK libraries; MQTT build profile |
| `build_make/makefile` | links `$(LIBS_ROBOT)` (`-lm`) | `atan2f` / `sqrtf` for the compass |
| `lib/libnet/lwip/include/lwipopts.h` | TCP on for MQTT, bigger MQTT buffer | MQTT needs TCP; telemetry is about 350 bytes |
| `lib/libwifi/.../cyw43_utk.c` | calls `robot_mqtt_poll()` | MQTT runs in the radio task |

---

## Status

| Subsystem | Ported | Tested |
|---|---|---|
| Buddy 2 motion | Yes | On the car |
| Buddy 3 line + barcode | Yes, same algorithm | Emulator (line tracking, steering, barcode "turn left" then resume) |
| Buddy 4 IMU | Compass, plus new accelerometer and tilt | Emulator (fake sensor: heading 45.0°, pitch 26.5°) |
| Buddy 5 ultrasonic + servo | Yes, echo now interrupt-timed | Emulator (20 cm echo reads 18–19 cm) |
| Buddy 1 Wi-Fi + MQTT | Yes, same topics and JSON fields | **Compiles only. Needs a test on the car with the dashboard** |
| Controller commands | Same commands as the old `main.c` | Parser unit-tested on PC |

**Behaviour changes from the old `main.c`:** "stop" also stops line following.
A raw drive command stops line following. After a barcode action the robot
resumes line following. Dashboard PWM values are halved internally, so the motor
duty is the same as before the PWM fix.

**Not done yet (project requirements):** hump detection and height (Buddy 4),
obstacle profiling, avoidance and line recovery (Buddy 5), the U-turn barcode,
straight-line correction and PID tuning (Buddy 2), and Wi-Fi reconnection after
the link drops.
