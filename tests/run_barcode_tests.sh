#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/barcode-tests.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM
compiler=${CC:-cc}
run() {
    name=$1
    shift
    "$compiler" -std=c11 -Wall -Wextra -Werror -Itests/stubs \
        -Ifirmware/app_program -Ifirmware/config "$@" -lm -o "$build_dir/$name"
    "$build_dir/$name"
}
run tuning tests/test_barcode_tuning.c firmware/app_program/barcode_tuning.c
run tuning_mqtt tests/test_barcode_tuning_mqtt.c firmware/app_program/barcode_tuning.c firmware/app_program/barcode_tuning_mqtt.c firmware/app_program/robot_packed.c
run irq_scan -DBUDDY3_DEBUG=1 tests/test_barcode_reading.c firmware/app_program/subsystem3/barcode_reading.c
run irq_scan_low -DBARCODE_DIGITAL_BLACK_LEVEL=0 tests/test_barcode_reading.c firmware/app_program/subsystem3/barcode_reading.c
run controller tests/test_robot_barcode_controller.c firmware/app_program/robot_steering_trace.c firmware/app_program/robot_packed.c firmware/app_program/robot_wire.c firmware/app_program/robot_fault.c firmware/app_program/barcode_tuning.c
run ir tests/test_buddy3_ir_junction.c firmware/app_program/buddy3_junction.c firmware/app_program/buddy3_steering.c firmware/app_program/robot_ir_stream.c firmware/app_program/barcode_tuning.c
run steering tests/test_buddy3_steering.c firmware/app_program/buddy3_steering.c
run junction tests/test_buddy3_junction.c firmware/app_program/buddy3_junction.c
run lift tests/test_buddy4_lift.c firmware/app_program/buddy4_processing.c
run wire tests/test_robot_wire.c firmware/app_program/robot_packed.c firmware/app_program/robot_wire.c
node tests/test_rover_wire.cjs
run avoid_line tests/test_buddy5_line_return.c
run avoid_route tests/test_buddy5_avoidance.c
