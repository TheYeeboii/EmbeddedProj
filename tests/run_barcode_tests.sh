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
run decoder tests/test_buddy3_barcode.c firmware/app_program/buddy3_barcode.c firmware/app_program/code39.c
run controller tests/test_robot_barcode_controller.c firmware/app_program/buddy3_barcode.c firmware/app_program/code39.c firmware/app_program/robot_steering_trace.c firmware/app_program/robot_packed.c firmware/app_program/robot_wire.c
run ir tests/test_buddy3_ir_junction.c firmware/app_program/buddy3_barcode.c firmware/app_program/code39.c firmware/app_program/buddy3_junction.c firmware/app_program/buddy3_steering.c firmware/app_program/robot_barcode_trace.c firmware/app_program/robot_ir_stream.c
run steering tests/test_buddy3_steering.c firmware/app_program/buddy3_steering.c
run junction tests/test_buddy3_junction.c firmware/app_program/buddy3_junction.c
run wire tests/test_robot_wire.c firmware/app_program/robot_packed.c firmware/app_program/robot_wire.c
node tests/test_rover_wire.cjs
