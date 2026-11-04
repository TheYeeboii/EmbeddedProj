# Barcode qualification and action mapping

Reading currently reports letters only. `barcode_mode 0` suppresses logs and
`barcode_mode 1` observes. `barcode_mode 2` is rejected because the demo mapping
is unknown and physical qualification has not been completed. The unsupported
numeric 1/2/3 motor-action path and its helper module have been removed.

## Test one stage at a time

1. Run `sh tests/run_barcode_tests.sh`. These tests need no board or network.
2. Check both encoders and calibrate the side sensor over black and white at its
   mounted height. Measure its sideways offset from the wheel midpoint.
3. Configure the minimum resolvable element and offset while stopped, following
   [BARCODE_DISTANCE.md](BARCODE_DISTANCE.md). Start with a straight approach.
4. Press GP21. Pass F and Y prints individually at low speed. USB should report
   exactly one correct letter and orientation per complete passage. GP21 stops.
5. Flip each print end-for-end. It should return the same letter, with reversed
   orientation. Uniformly scale prints and repeat above the resolution limit.
6. Try incomplete patterns, inadequate quiet space, a mid-read stop/reversal,
   turns and encoder faults. None may produce a valid letter or a barcode turn.
7. Record trials across speeds, lighting and scales before enabling any actions.

## Outstanding issue gates

- [#12](https://github.com/TheYeeboii/EmbeddedProj/issues/12): distance capture is
  implemented with raw per-wheel diagnostic export; real IR traces remain.
- [#13](https://github.com/TheYeeboii/EmbeddedProj/issues/13): adaptive single-letter
  Code 39 is implemented with host tests; physical evidence remains outstanding.
- [#14](https://github.com/TheYeeboii/EmbeddedProj/issues/14): both print orientations
  are tested in software; real flipped-print trials remain outstanding.
- [#15](https://github.com/TheYeeboii/EmbeddedProj/issues/15): USB results/rejections
  and loss counts are implemented. MQTT scan history, gap/reconnect handling and
  export/replay are now implemented; physical radio tests remain.
- [#16](https://github.com/TheYeeboii/EmbeddedProj/issues/16): physical qualification
  remains open. Target at least 95% successful reads and no wrong-letter accepts.
- [#17](https://github.com/TheYeeboii/EmbeddedProj/issues/17): mapping remains
  disabled until the actual letters/actions are announced and qualification
  passes. Then act immediately after a validated frame, independent of scan
  orientation. Do not assume A=left, Z=right, or F/Y navigation meanings.

Buddy 2 odometry, Buddy 4 IMU, Buddy 5 avoidance and GP21 steering behavior are
preserved. Barcode events do not acquire the motors or resume a stopped rover.

## Implementation verification (4 November 2026)

All five suites in `sh tests/run_barcode_tests.sh` pass. The existing Buddy 2
action-log, MQTT action-log and dashboard action-log tests also pass. Pico W
USB CDC firmware compiles and links with SDK 2.3.1:

```sh
cd firmware/build_make
make CONSOLE=usb_cdc PICO_SDK_PATH=/path/to/pico-sdk/2.3.1 -j4
```

A broader run of the existing `test_buddy5_avoidance.c` fails its PWM assertion
at line 90. The avoidance source and this test are unchanged by the barcode
work. This failure remains outstanding; it is not counted as a passing check.
The optional Address/Undefined Sanitizer executable did not complete within
20 seconds in the local environment, so sanitizer verification is unconfirmed.
No board has been flashed or physically qualified by these software checks.
