# Temporary README — interrupt-driven rover barcode scan

Port the timing scanner from `Pico-W-Line-Tracing-Car/app_program/subsystem3/barcode_reading.c` to the rover's default digital side sensor. The same reader works with a moving print or moving rover. Keep enough speed through each character to distinguish narrow and wide elements by duration.

## Bring over

- [x] Replace the placeholder in `firmware/app_program/barcode_reading.c` with the edge record, 32-slot FIFO, edge-loss reset, and one nine-width capture buffer. Add `barcode_reading.h` for the init and task entry declarations.
- [x] Use the rover's `gpio_irq_register()`, `hal_time_us()`, `BARCODE_DIGITAL_PIN`, and `BARCODE_DIGITAL_BLACK_LEVEL`. Register both edges on the side sensor and create the event flag before arming the pin. The digital build defaults to GP28 with black high.
- [x] Reuse `code39.c/.h` to decode each character as soon as nine widths are available. Search for an opening marker in either direction, then require a letter and a closing marker in that direction. Skip the white gaps. Enable `BUDDY3_DEBUG=1` for the nine width lines per character on the console.
- [x] Wire the reader into `app_main.c` after GPIO IRQ and Buddy 3 setup. Pin the worker to core 0 on SMP builds so it can copy FIFO entries with local interrupts disabled. Keep the callback limited to capturing and signalling.
- [x] Use the IRQ reader by default. Remove legacy polling, wheel-distance decoding, and automatic barcode slowdown from the digital path. Keep the wheel-distance reader for builds that explicitly select `BARCODE_DIGITAL_INPUT=0`.
- [x] Retain the latest confirmed letter for `rover/debug`. During digital scan testing, decoded letters do not request junction turns and GP28 does not feed floor detection.

## Verify

- [x] Run the host tests for both scan directions, queue loss, timeout, controller delivery, and the dashboard letter.
- [x] Build the default firmware and the SMP USB Wi-Fi/MQTT profile. Use a clean build when changing `ROBOT_CFLAGS` because that variable is not part of the build profile.
- [ ] With the rover idle, slide `*A*` past GP28 both ways and confirm the dashboard shows `A`. Scan another letter and try an incomplete scan. With width logging enabled, check the nine durations per character on USB.
- [ ] Drive the rover over the print and check that it continues line following and reports the letter after the closing marker.

Example USB and MQTT build from the firmware build directory, after setting local Wi-Fi and broker configuration:

```sh
make -j8 PICO_SDK_PATH=/path/to/pico-sdk CONSOLE=usb_cdc SMP=1 WIFI=cyw43 \
  WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_MQTT=1 BUDDY3_DEBUG=1
```

Remove this temporary README once port and hardware verification are accepted.
