# Abbreviations

BARR-C:2018 rule 1.5.b asks for one version-controlled table of the
abbreviations and acronyms used in the code.  Add a row when you introduce a
new one; avoid abbreviations that are not widely understood.

## Hardware and protocols

| Abbreviation | Meaning |
|---|---|
| ADC, ADC0..ADC2 | Analogue-to-digital converter (input channels 0..2) |
| ASCII | American Standard Code for Information Interchange |
| BOOTSEL | Pico boot-select button (USB mass-storage flashing mode) |
| GND | Ground |
| GPIO, GPn | General-purpose input/output; GPn is RP2040 pin n |
| HC-SR04 | Ultrasonic distance sensor model |
| I2C | Inter-Integrated Circuit bus (IMU) |
| IMU | Inertial measurement unit (LSM303 accelerometer + compass) |
| IR | Infrared (line and barcode sensors) |
| IRQ | Interrupt request |
| ISR | Interrupt service routine |
| JSON | JavaScript Object Notation (MQTT payloads) |
| LED | Light-emitting diode |
| LSM303 | ST accelerometer + magnetometer chip (Buddy 4) |
| M1A, M1B, M2A, M2B | Robo Pico motor-driver outputs |
| MQTT | Message Queuing Telemetry Transport (dashboard link) |
| NVIC | Nested vectored interrupt controller (Cortex-M) |
| PWM | Pulse-width modulation (motors, servo) |
| SDK | Software development kit (Pico C SDK) |
| UART | Universal asynchronous receiver/transmitter |
| USB, USB-CDC | Universal Serial Bus; communications-device class (serial console) |

## Software

| Abbreviation | Meaning |
|---|---|
| API | Application programming interface |
| DI / EI | micro T-Kernel disable / enable interrupts |
| ER, ID, INT, UINT, UB, UW, SZ, TMO | micro T-Kernel types: error code, object id, int, unsigned int, unsigned byte, unsigned word, size, timeout |
| NaN | Not a number (floating point) |
| PID | Proportional-integral-derivative controller |
| RTOS | Real-time operating system (micro T-Kernel) |

## Naming suffixes and prefixes

| Abbreviation | Meaning |
|---|---|
| `_x10`, `_x100` | Value in tenths / hundredths of the unit (e.g. `heading_x10`) |
| `_mm`, `_cm`, `_ms`, `_us`, `_deg`, `_pct` | Millimetres, centimetres, milliseconds, microseconds, degrees, percent |
| `_mm_s`, `_dps` | Millimetres per second, degrees per second |
| `g_`, `p_`, `pp_`, `b_` | Global, pointer, pointer-to-pointer, Boolean (BARR-C 7.1) |
| `B3`, `B3BC`, `B3J` | Buddy 3 steering, barcode and junction constants |
| `BC_` | Barcode trace packet types |
| `cfg`, `cal` | Configuration, calibration |
