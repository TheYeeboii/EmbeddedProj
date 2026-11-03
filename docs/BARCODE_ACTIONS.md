# Barcode movement handoff

Branch: feature/distance-barcode-navigation. Reading and movement are separate
commits so the decoder can be tested before motor actions.

The track requirement is IMMEDIATE action after reading, not "next junction".
Only the custom 0xA5/command format is implemented until the printed barcode is
identified. A clear photo of the complete physical barcode is needed to confirm
compatibility; don't assume Code 39 or another standard is supported.

## Sequence

GP21 -> gentle tracking + distance reader -> valid command ->
pause tracking/reader -> Buddy 2 executes -> resume tracking.

Commands keep the existing meanings: 1 left 90 degrees, 2 right 90 degrees,
3 forward 300 mm. These are calibrated encoder moves, not line-reacquisition
turns. Track geometry and calibration must match. Completion does not prove the
front sensors have reacquired the intended line.

## Test one stage at a time

1. Read docs/BARCODE_DISTANCE.md. Verify both independent encoders,
   calibrate wheel travel, measure cell width and side offset, and confirm the
   physical barcode format first. Main's new 387-edge calibration and fault
   reporting are preserved. An unmeasured cell width still disables decoding.
2. Configure the measured cell width/offset while stopped. Default mode is
   observe (1). Press GP21 and verify correct decoded commands at multiple
   speeds; no barcode move is performed. Steering tests still stop after 15 s.
3. Only after correct reads, stop and select barcode_mode 2. Press GP21.
   A valid frame pauses line following, immediately starts one move, and resumes
   following only after that move reports completion. Each resumed tracking
   segment starts a fresh 15-second test window. A fresh two-cell quiet
   lead-in is required before another barcode.
4. Test left, right and straight separately. Confirm GP21/stop can cancel each
   action and a missing completion causes a stop within 8 seconds. Buddy 2's
   encoder-fault flags also cancel a barcode move without resuming tracking.
   A rejected motion request stops the rover rather than resuming blindly.

Commands (sent while stopped):

```text
barcode_mode 0
barcode_mode 1
barcode_mode 2
```

0 ignores decoded commands; 1 logs without moving; 2 acts immediately.
JSON equivalent: {"cmd":"barcode_mode","mode":2}.
Configuration/mode selection is RAM-only. For button-only operation after
calibration, set measured BUDDY3_BARCODE_CELL_MM and
BUDDY3_BARCODE_OFFSET_LEFT_MM in buddy3_barcode.h, and set
ROBOT_BARCODE_DEFAULT_MODE to 2 in robot_barcode_action.h. GP21 remains the
start/stop button. Current defaults intentionally remain unmeasured/read-only.

Stage 0 is the legacy steering algorithm and is not supported by the new
distance-reader integration. Use stage 1 initially; stages 2/3 invalidate reads
during tight turns or recovery. Sampling/decoding continues independently of
network access. No Wi-Fi or broker is required for these USB/GP21 tests.

Manual drive cancels barcode ownership. Obstacle avoidance cancels a barcode
move without an automatic barcode-triggered resume. Fault/timeout stops require
a fresh explicit start. Sensor/decoder events are cleared on tracking stop/start.

## Tests

The action controller is exercised with real robot_controller.c and fake motor,
button and reader hardware:

```sh
cc -std=c11 -Wall -Wextra -Werror -I tests/stubs -I firmware/app_program tests/test_robot_barcode_controller.c firmware/app_program/robot_barcode_action.c -o /tmp/test_robot_barcode_controller
/tmp/test_robot_barcode_controller
```

Run the existing tests/test_buddy3_barcode.c and tests/test_buddy3_steering.c
checks as well. These are software sequence checks, not physical validation.
