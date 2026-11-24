#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/barcode-tests.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM
compiler=${CC:-cc}
run() {
    name=$1
    shift
    "$compiler" -std=c11 -Wall -Wextra -Werror -Itesting/stubs \
        -Iapp_program -Iapp_program/subsystem1 -Iapp_program/subsystem2 -Iapp_program/subsystem3 -Iapp_program/subsystem4 -Iapp_program/subsystem5 -Iconfig "$@" -lm -o "$build_dir/$name"
    "$build_dir/$name"
}
run slowdown testing/subsystem3/test_barcode_slowdown.c app_program/subsystem3/barcode_slowdown.c
run tuning testing/subsystem3/test_barcode_tuning.c app_program/subsystem3/barcode_tuning.c
run tuning_mqtt testing/subsystem3/test_barcode_tuning_mqtt.c app_program/subsystem3/barcode_tuning.c app_program/subsystem3/barcode_tuning_mqtt.c app_program/subsystem1/robot_packed.c
run decoder testing/subsystem3/test_buddy3_barcode.c app_program/subsystem3/buddy3_barcode.c app_program/subsystem3/code39.c
run controller testing/integration/test_robot_barcode_controller.c app_program/subsystem3/buddy3_barcode.c app_program/subsystem3/code39.c app_program/subsystem3/robot_steering_trace.c app_program/subsystem1/robot_packed.c app_program/subsystem1/robot_wire.c app_program/robot_fault.c app_program/subsystem3/barcode_tuning.c
run ir testing/subsystem3/test_buddy3_ir_junction.c app_program/subsystem3/buddy3_barcode.c app_program/subsystem3/code39.c app_program/subsystem3/buddy3_junction.c app_program/subsystem3/buddy3_steering.c app_program/subsystem3/robot_barcode_trace.c app_program/subsystem3/robot_ir_stream.c app_program/subsystem3/barcode_tuning.c app_program/subsystem3/barcode_slowdown.c
run ir_digital testing/subsystem3/test_buddy3_ir_digital.c app_program/subsystem3/buddy3_barcode.c app_program/subsystem3/code39.c app_program/subsystem3/buddy3_junction.c app_program/subsystem3/buddy3_steering.c app_program/subsystem3/robot_barcode_trace.c app_program/subsystem3/robot_ir_stream.c app_program/subsystem3/barcode_tuning.c app_program/subsystem3/barcode_slowdown.c
run steering testing/subsystem3/test_buddy3_steering.c app_program/subsystem3/buddy3_steering.c
run junction testing/subsystem3/test_buddy3_junction.c app_program/subsystem3/buddy3_junction.c
run lift testing/subsystem4/test_buddy4_lift.c app_program/subsystem4/buddy4_processing.c
run wire testing/subsystem1/test_robot_wire.c app_program/subsystem1/robot_packed.c app_program/subsystem1/robot_wire.c
node testing/subsystem1/test_rover_wire.cjs
run avoid_line testing/subsystem5/test_buddy5_line_return.c
run avoid_route testing/subsystem5/test_buddy5_avoidance.c
