# Barcode qualification and action mapping

Historical notes for the retired wheel-distance reader. Subsystem 3 now reports
barcode letters to the dashboard as scan results.

While line following, a decoded letter sets the turn at the next junction:
**A** turns left, **C** turns right, any other letter goes straight on (team
mapping, `ROBOT_BARCODE_LETTER_LEFT` / `_RIGHT` in `robot_controller.c`). The
latest letter wins. At the junction the rover stops, drives its
front-sensor-to-axle distance, pivots 90 degrees and follows the line again.
Letters read while driving by hand are printed but move nothing.
`barcode_mode 0` suppresses the letters; `barcode_mode 1` (the default) reads
them. The old numeric 1/2/3 motor-action path has been removed.

## Test one stage at a time

1. Run `sh tests/run_barcode_tests.sh`. These tests need no board or network.
2. Check both encoders and calibrate the side sensor over black and white at its
   mounted height. Measure its sideways offset from the wheel midpoint.
3. Configure the minimum resolvable element and offset while stopped, following
   [BARCODE_DISTANCE.md](BARCODE_DISTANCE.md). Start with a straight approach.
4. Press GP21, or drive forward with W. Pass F and Y prints individually at
   low speed. USB should report exactly one correct letter and orientation per
   complete passage. GP21 stops.
5. Flip each print end-for-end. It should return the same letter, with reversed
   orientation. Uniformly scale prints and repeat above the resolution limit.
6. Try incomplete patterns, a mid-read reversal, turns and encoder faults.
   None may produce a valid letter or a barcode turn. No white space is needed
   after the barcode: the letter is decoded as the closing `*` ends.
7. Record trials across speeds, lighting and scales, then try the A and C
   junction turns on the track.

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
- [#17](https://github.com/TheYeeboii/EmbeddedProj/issues/17): the team mapping
  (A left, C right, at the next junction) is implemented, independent of scan
  orientation; physical track qualification remains.

Buddy 2 odometry, Buddy 4 IMU, Buddy 5 avoidance and GP21 steering behaviour are
preserved. A letter only takes the motors at the junction it was read for, and
it never restarts a stopped rover.

## Implementation verification (4 November 2026)

All five suites in `sh tests/run_barcode_tests.sh` pass. The existing Buddy 2
action-log, MQTT action-log and dashboard action-log tests also pass. Pico W
USB CDC firmware compiles and links with SDK 2.3.1:

```sh
cd firmware/build_make
make CONSOLE=usb_cdc PICO_SDK_PATH=/path/to/pico-sdk/2.3.1 -j4
```

The older `test_buddy5_avoidance.c` predates the obstacle width sweep and fails
its first stub check. It is not in the test scripts and is not counted as a
passing check; `test_buddy5_line_return.c` covers the return to the line.
The optional Address/Undefined Sanitizer executable did not complete within
20 seconds in the local environment, so sanitizer verification is unconfirmed.
No board has been flashed or physically qualified by these software checks.
