#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/ir-diagnostics.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM
compiler=${CC:-cc}
run() {
    name=$1; shift
    "$compiler" -std=c11 -Wall -Wextra -Werror -Itesting/stubs -Iapp_program -Iapp_program/subsystem1 -Iapp_program/subsystem2 -Iapp_program/subsystem3 -Iapp_program/subsystem4 -Iapp_program/subsystem5 -Iconfig "$@" -lm -o "$build_dir/$name"
    "$build_dir/$name"
}
run slowdown testing/subsystem3/test_barcode_slowdown.c app_program/subsystem3/barcode_slowdown.c
run mqtt-backpressure -Itesting/mqtt_stubs testing/subsystem1/test_mqtt_backpressure.c app_program/subsystem1/robot_packed.c app_program/subsystem1/robot_wire_transport.c
run steering-trace testing/subsystem3/test_robot_steering_trace.c app_program/subsystem2/robot_action_log.c app_program/subsystem3/robot_barcode_trace.c app_program/subsystem3/buddy3_barcode.c app_program/subsystem3/code39.c app_program/subsystem3/robot_ir_stream.c
run motor-observer testing/subsystem2/test_buddy2_action_log.c app_program/subsystem3/robot_steering_trace.c app_program/subsystem3/buddy3_barcode.c app_program/subsystem3/code39.c app_program/robot_fault.c
run ir testing/subsystem3/test_buddy3_ir_junction.c app_program/subsystem3/buddy3_junction.c app_program/subsystem3/buddy3_steering.c app_program/subsystem3/buddy3_barcode.c app_program/subsystem3/code39.c app_program/subsystem3/robot_barcode_trace.c app_program/subsystem3/robot_ir_stream.c app_program/subsystem3/barcode_tuning.c app_program/subsystem3/barcode_slowdown.c
run ir_digital testing/subsystem3/test_buddy3_ir_digital.c app_program/subsystem3/buddy3_junction.c app_program/subsystem3/buddy3_steering.c app_program/subsystem3/buddy3_barcode.c app_program/subsystem3/code39.c app_program/subsystem3/robot_barcode_trace.c app_program/subsystem3/robot_ir_stream.c app_program/subsystem3/barcode_tuning.c app_program/subsystem3/barcode_slowdown.c
run controller testing/integration/test_robot_barcode_controller.c app_program/subsystem3/buddy3_barcode.c app_program/subsystem3/code39.c app_program/subsystem3/robot_steering_trace.c app_program/subsystem1/robot_packed.c app_program/subsystem1/robot_wire.c app_program/robot_fault.c app_program/subsystem3/barcode_tuning.c
run actions testing/subsystem2/test_robot_action_log.c app_program/subsystem3/robot_steering_trace.c app_program/subsystem3/robot_barcode_trace.c app_program/subsystem3/buddy3_barcode.c app_program/subsystem3/code39.c app_program/subsystem3/robot_ir_stream.c
run steering testing/subsystem3/test_buddy3_steering.c app_program/subsystem3/buddy3_steering.c
run junction testing/subsystem3/test_buddy3_junction.c app_program/subsystem3/buddy3_junction.c
node --check dashboard/app.js
node --check dashboard/steering-view.js
node testing/subsystem1/test_dashboard_actions.cjs
node testing/subsystem3/test_steering_history.cjs
"$compiler" -std=c11 -Wall -Wextra -Werror -Itesting/stubs -Iapp_program -Iapp_program/subsystem1 -Iapp_program/subsystem2 -Iapp_program/subsystem3 -Iapp_program/subsystem4 -Iapp_program/subsystem5 -Iconfig testing/subsystem3/test_robot_barcode_trace.c app_program/subsystem3/buddy3_barcode.c app_program/subsystem3/code39.c app_program/subsystem2/robot_action_log.c app_program/subsystem3/robot_steering_trace.c app_program/subsystem3/robot_ir_stream.c -lm -o "$build_dir/barcode-trace"
"$build_dir/barcode-trace" "$build_dir/barcode-wire.ndjson"
node testing/subsystem3/test_barcode_history.cjs
node testing/subsystem3/test_barcode_wire.cjs "$build_dir/barcode-wire.ndjson"
node --check dashboard/barcode-view.js
node --check dashboard/barcode-config.js
node testing/subsystem3/test_barcode_config.cjs
node --check dashboard/ir-stream-model.js
node --check dashboard/ir-stream-view.js
node testing/subsystem3/test_ir_stream_history.cjs
"$compiler" -std=c11 -Wall -Wextra -Werror -Itesting/stubs -Iapp_program -Iapp_program/subsystem1 -Iapp_program/subsystem2 -Iapp_program/subsystem3 -Iapp_program/subsystem4 -Iapp_program/subsystem5 -Iconfig testing/subsystem3/test_robot_ir_stream.c app_program/subsystem2/robot_action_log.c app_program/subsystem3/robot_steering_trace.c app_program/subsystem3/robot_barcode_trace.c app_program/subsystem3/buddy3_barcode.c app_program/subsystem3/code39.c -lm -o "$build_dir/ir-stream"
"$build_dir/ir-stream" "$build_dir/ir-stream.ndjson"
node testing/subsystem3/test_ir_stream_wire.cjs "$build_dir/ir-stream.ndjson"
