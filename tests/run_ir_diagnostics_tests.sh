#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/ir-diagnostics.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM
compiler=${CC:-cc}
run() {
    name=$1; shift
    "$compiler" -std=c11 -Wall -Wextra -Werror -Itests/stubs -Ifirmware/app_program -Ifirmware/config "$@" -lm -o "$build_dir/$name"
    "$build_dir/$name"
}
run steering-trace tests/test_robot_steering_trace.c
run motor-observer tests/test_buddy2_action_log.c firmware/app_program/robot_steering_trace.c
run ir tests/test_buddy3_ir_junction.c firmware/app_program/buddy3_junction.c firmware/app_program/buddy3_steering.c firmware/app_program/buddy3_barcode.c
run controller tests/test_robot_barcode_controller.c firmware/app_program/robot_barcode_action.c firmware/app_program/robot_steering_trace.c
run actions tests/test_robot_action_log.c
run steering tests/test_buddy3_steering.c firmware/app_program/buddy3_steering.c
run junction tests/test_buddy3_junction.c firmware/app_program/buddy3_junction.c
node --check dashboard/app.js
node --check dashboard/steering-view.js
node tests/test_dashboard_actions.cjs
node tests/test_steering_history.cjs
