# Team and subsystems

Team CSC04. This is the single place that says **who owns what**, so that five
people can work in one firmware image without clashing over pins, priorities,
or kernel objects.

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review. Shared
files (`app_program/app_main.c`, `app_program/robot_controller.c`,
`app_program/pin_config.h`, this file) are changed only by agreement of the
team.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | MQTT, Wi-Fi, telemetry (Yew Wei Song) | 2501134 | TheYeeboii | `app_program/subsystem1/`, `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` | `testing/subsystem1/` |
| 2 | Motion control (Mohamed Faizal Ahmed Fairaj) | 2501208 | Faiz0909 | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Line following and barcode decoder (Noah Wong Nam Suen) | 2502896 | DisposableMask | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU (Chin Chee Yuan) | 2501665 | terakaiser | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Ultrasonic sensor, obstacle avoidance (Tan Jia Yi Charmain) | 2502647 | TanJiaYiCharmain | `app_program/subsystem5/` | `testing/subsystem5/` |

Shared (all): `app_program/app_main.c` (tasks), `robot_controller.c` (modes
and commands), `robot_hal.c`, `gpio_irq.c`, `robot_fault.c`, `pin_config.h`.

## 2. Pins

One owner per pin; the numbers are defined in `app_program/pin_config.h`.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | S5 | Ultrasonic Trig (Grove 1) |
| GP1 | S5 | Ultrasonic Echo (Grove 1) |
| GP2 | S2 | Left wheel encoder (Grove 2) |
| GP3 | | Free (released as an input at boot) |
| GP4 | S4 | IMU I2C0 SDA (Grove 3) |
| GP5 | S4 | IMU I2C0 SCL (Grove 3) |
| GP8, GP9 | S2 | Left motor M1A / M1B (PWM) |
| GP10, GP11 | S2 | Right motor M2A / M2B (PWM) |
| GP15 | S5 | Scanning servo (PWM, 50 Hz) |
| GP16 | | Free (released as an input at boot) |
| GP17 | S2 | Right wheel encoder (Grove 4) |
| GP21 | shared | GP21 button: line following start/stop |
| GP26 | S3 | Front IR sensor (ADC0) |
| GP27 | S3 | Front IR sensor (ADC1) |
| GP28 | S3 | Barcode IR sensor, digital DO (HIGH = black) |

## 3. Peripherals

Each peripheral is driven by exactly one task, its *owning task*.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| I2C0 | S4 | imu task | GP4, GP5 |
| ADC | S3 | line task | GP26, GP27 |
| PWM slices 4, 5 | S2 | motion task (others call its mutex-protected API) | GP8–GP11 |
| PWM slice for GP15 | S5 | sensors task | GP15 |
| GPIO interrupts | shared | `gpio_irq.c` dispatcher (encoders, echo) | GP2, GP17, GP1 |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT

No application task calls `cyw43_...`, lwIP or MQTT functions. The MQTT
client (`lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`) runs inside the
kernel's Wi-Fi service task. Application tasks exchange data with it only
through the message buffers in `app_program/subsystem1/comms.c` (TX:
`comms_publish`, never blocking; RX: `comms_wait_command`). Subscriptions are
restored whenever the client reconnects.

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4).

| Subsystem | Tasks (name – priority – period) |
|---|---|
| 2 | motion – 5 – 20 ms cyclic |
| 3 | line – 6 – 1 ms cyclic (front sensors every 10 ms) |
| shared | controller – 7 – waits up to 20 ms for a command |
| 4 | imu – 9 – 50 ms target |
| 5 | sensors (ultrasonic + avoidance) – 10 – 100 ms |
| 1 | telemetry – 15 – 1 s |

## 5. Interfaces between subsystems

| ID | From → To | Mechanism | Object / function | Data / meaning |
|---|---|---|---|---|
| IF-01 | all → S1 | message buffer (TX, owned by S1) | `comms_publish()` | packed binary messages, see `docs/MQTT_BINARY_FORMAT.md` |
| IF-02 | S1 → shared controller | message buffer (RX, owned by S1) | `comms_wait_command()` | packed dashboard commands (1–5 bytes) |
| IF-03 | S3, S5, controller → S2 | priority-inheritance mutex | `buddy2_set_motor_pwm()`, `buddy2_move_*()`, `buddy2_turn_*()` | motor commands; S2 alone writes the motors |
| IF-04 | S2 → S3, S4, S5 | shared snapshot under DI/EI | `buddy2_get_motion_state()` | wheel distances, encoder faults |
| IF-05 | S3 → controller | shared bytes | `buddy3_take_barcode()`, `buddy3_take_junction_turn()` | decoded letter; turn reached at a junction |
| IF-06 | S4 → S5, controller | timestamped snapshot under DI/EI | `buddy4_get_state()` | heading, tilt, lift/overturn |
| IF-07 | all → S1 | packed flags under DI/EI | `robot_fault.c` | shared fault byte |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).
