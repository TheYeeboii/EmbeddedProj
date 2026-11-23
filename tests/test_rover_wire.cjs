'use strict';
/* Dashboard side of the binary MQTT format. The golden bytes are printed by
 * "test_robot_wire --print" (tests/test_robot_wire.c) from the firmware's own
 * encoder, so this proves both sides agree on every bit. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const W = require('../dashboard/rover-wire.js');

const bytes = text => Uint8Array.from(text.split(' ').map(h => parseInt(h, 16)));

const TELEMETRY = bytes('11 ff ff f8 90 80 07 ff c0 06 42 d3 83 e3 e3 84 ff ff 30 39 ff ff 00 00 03 2a 1e 14');
const DEBUG = bytes('21 ff f0 01 00 0f a0 00 7d 20 c5 b4 00 c8 57 1f 1c 1f 00 02 ff fe 3e bf f8');

/* Telemetry: every field at an edge of its range. */
{
  const m = W.decode(TELEMETRY);
  assert.equal(m.kind, 'telemetry');
  const d = m.data;
  assert.equal(d.uptime_s, 1048575);
  assert.equal(d.mode, 'avoid');
  assert.equal(d.state, 'compass_error');
  assert.deepEqual(d.faults, ['wifi_loss', 'motor_stall']);
  assert.deepEqual([d.left_ir, d.center_ir, d.right_ir], [0, 4095, 2048]);
  assert.equal(d.distance_cm, 400);
  assert.equal(d.servo_angle_deg, 180);
  assert.equal(d.heading_deg, 359.9);
  assert.equal(d.pitch_deg, -90);
  assert.equal(d.max_pitch_deg, 90);
  assert.equal(d.left_ticks_raw, 0xffff);
  assert.equal(d.right_ticks_raw, 12345);
  assert.deepEqual([d.motor_left, d.motor_right], [65535, 0]);
  assert.deepEqual([d.trim_left, d.trim_right], [50, 100]);
  assert.deepEqual([d.line_base, d.line_turn, d.speed], [80, 60, 20]);
}

/* Debug: negative signed fields and edges. */
{
  const m = W.decode(DEBUG);
  assert.equal(m.kind, 'debug');
  const d = m.data;
  assert.deepEqual([d.raw_left, d.raw_center, d.raw_right], [4095, 1, 0]);
  assert.deepEqual([d.norm_left, d.norm_center, d.norm_right], [1000, 0, 500]);
  assert.equal(d.position, -1000);
  assert.deepEqual([d.line, d.junction, d.barcode], [true, false, true]);
  assert.equal(String.fromCharCode(d.command), 'Z');
  assert.deepEqual([d.ultra_left, d.ultra_right], [0, 400]);
  assert.equal(d.decision, 'right');
  assert.equal(d.avoid_status, 'reversing');
  assert.equal(d.avoid_heading_change_x10, -1800);
  assert.equal(d.avoid_heading_target_x10, 3599);
  assert.deepEqual([d.avoid_motor_left_pwm, d.avoid_motor_right_pwm], [-32767, 32767]);
  assert.deepEqual([d.avoid_forward_mm, d.avoid_forward_target_mm], [250, 2047]);
  assert.equal(d.barcode_irq, false);
  const irq = DEBUG.slice();
  irq[24] |= 0x04;
  assert.equal(W.decode(irq).data.barcode_irq, true);
  assert.equal(W.decode(irq).data.command, d.command);
}

/* A GPIO result updates the letter card without a distance trace. */
{
  const irq = DEBUG.slice();
  irq[24] |= 0x04;
  const result = W.decode(irq).data;
  const fields = new Map();
  const find = key => {
    if (!fields.has(key)) fields.set(key, {dataset: {}});
    return fields.get(key);
  };
  const card = {querySelector: find, dataset: {}};
  const RoverBarcode = require('../dashboard/barcode-model.js');
  const context = {RoverBarcode, document: {getElementById: () => card}};
  vm.runInNewContext(fs.readFileSync(require.resolve('../dashboard/barcode-view.js'), 'utf8'), context);
  const view = Object.create(context.RoverBarcodeView.prototype);
  Object.assign(view, {find, interruptResult: null, history: new RoverBarcode.BarcodeHistory()});
  view.renderLive();
  assert.equal(find('[data-letter="reader"]').textContent, 'Reader: waiting for telemetry');
  view.history.state = 0;
  view.renderLive();
  assert.equal(find('[data-letter="reader"]').textContent, 'Reader: off');
  view.showInterruptResult(result.barcode_irq, String.fromCharCode(result.command));
  assert.equal(find('[data-letter="symbol"]').textContent, 'Z');
  assert.equal(find('outcome').textContent, 'Latest confirmed letter: Z');
  assert.equal(card.dataset.result, 'ok');
  view.unavailable();
  assert.equal(find('[data-letter="symbol"]').textContent, '–');
}

/* Buddy 4's IMU report (type 6): every field at an edge. */
{
  const m = W.decode(bytes('61 bf fc 7c 7f f1 f1 c2 7f 80 7b 9b 19 00'));
  assert.equal(m.kind, 'imu');
  const d = m.data;
  assert.deepEqual([d.imu_fault, d.imu_calibrated, d.height_valid], [1, 0, 1]);
  assert.equal(d.imu_interval_ms, 1023);
  assert.equal(d.roll_x10, -1800);
  assert.equal(d.hump_state, 3);
  assert.equal(d.hump_seq, 255);
  assert.deepEqual([d.hump_peak_x10, d.highest_peak_x10], [-900, 900]);
  assert.deepEqual([d.hump_height_mm, d.highest_hump_mm], [255, 0]);
  assert.equal(d.motion, 7);
  assert.equal(d.turn_rate_x10, -9000);
  assert.equal(d.lifts, 200);
}

/* One-byte messages, the JSON diagnostic streams, and bad input. */
assert.equal(W.decode(bytes('31')).kind, 'heartbeat');
assert.deepEqual(W.decode(bytes('51')).data, { state: 'online' });
assert.deepEqual(W.decode(bytes('41')).data, { state: 'offline' });
assert.deepEqual(W.decode(new TextEncoder().encode('{"hump_state":0}')), { kind: 'json', data: { hump_state: 0 } });
assert.equal(W.decode(bytes('12 00')).kind, 'version_mismatch');
assert.equal(W.decode(TELEMETRY.slice(0, 27)).kind, 'invalid');
assert.equal(W.decode(new Uint8Array(0)).kind, 'invalid');

/* Commands: the same bytes tests/test_robot_wire.c accepts. */
const hex = (name, values) => W.hex(W.encodeCommand(name, values));
assert.equal(hex('stop'), '10');
assert.equal(hex('reverse'), '24');
assert.equal(hex('forward'), '20');
assert.equal(hex('drive', { left: 65535, right: 0 }), '3f ff f0 00 00');
assert.equal(hex('motor_trim', { left: 50, right: 50 }), '90 00');
assert.equal(hex('barcode_config', { min_element_mm: 1000, offset_left_mm: -500 }), 'df a2 0c');
assert.equal(hex('line_speed', { base: 80, turn: 60 }), 'fa 1e 00');
assert.equal(hex('reset_all'), '80');
assert.equal(W.encodeCommand('ir_thresholds', {
  sensor: 3, inverted: false, white: 200, black: 300
}), null);
const irCommand = W.encodeCommand('ir_thresholds', {
  sensor: 2, inverted: true, white: 220, black: 260
});
const irReader = new W.Reader(irCommand);
assert.deepEqual([irReader.get(4), irReader.get(2), irReader.get(1),
  irReader.get(12), irReader.get(12)], [0, 2, 1, 220, 260]);
const frontReport = new W.Writer(11).put(0xc1, 8).put(200, 32).put(220, 12)
  .put(260, 12).put(200, 12).put(1, 1).put(1, 1).put(1, 1).put(2, 2).bytes;
assert.equal(W.decode(frontReport).data.sensor, 2);
const agedReport = new W.Writer(13).put(0xc1,8).put(2000,32)
  .put(220,12).put(260,12).put(250,12).put(1,1).put(0,1).put(1,1)
  .put(1,2).put(0,1).put(0,5).put(0,1).put(3000,16).bytes;
assert.equal(W.decode(agedReport).data.lag_ms,3000);
assert.equal(W.decode(agedReport).data.sensor,1);
const speedReport = new W.Writer(14).put(0xc1,8).put(2000,32)
  .put(0,12).put(0,12).put(1,12).put(0,1).put(0,1).put(1,1)
  .put(0,2).put(1,1).put(28,5).put(1,1).put(3000,16)
  .put(1,1).put(10,7).bytes;
assert.equal(W.decode(speedReport).data.lag_ms,3000);
assert.equal(W.decode(speedReport).data.slowdown,true);
assert.equal(W.decode(speedReport).data.effective_speed,10);
const digitalReport = new W.Writer(11).put(0xc1,8).put(2000,32)
  .put(0,12).put(0,12).put(1,12).put(0,1).put(0,1).put(1,1)
  .put(0,2).put(1,1).put(28,5).put(1,1).bytes;
assert.deepEqual(W.decode(digitalReport).data, {
  ts_ms:2000, low:0, high:0, raw:1, enabled:false, inverted:false,
  black:true, sensor:0, digital:true, pin:28, black_level:1
});

assert.equal(hex('barcode_mode', { mode: 1, white: 1300, black: 3900 }), 'c8 a2 9e 78');
assert.equal(hex('barcode_mode', { mode: 1, sensor: 1, white: 200, black: 1300 }), 'ca 19 0a 28');
assert.equal(hex('barcode_mode', { mode: 1, sensor: 3, white: 200, black: 1300 }), 'ce 19 0a 28');
const tuning = new W.Writer(10).put(0xc1, 8).put(200, 32).put(220, 12)
  .put(260, 12).put(200, 12).put(1, 1).put(1, 1).put(1, 1).bytes;
assert.deepEqual(W.decode(tuning), { kind: 'ir_tune', data: {
  ts_ms: 200, low: 220, high: 260, raw: 200, enabled: true, inverted: true, black: true
} });
assert.equal(W.encodeCommand('barcode_mode', { mode: 1, sensor: 4, white: 200, black: 1300 }), null);
assert.equal(hex('barcode_mode', { mode: 1 }), 'c8 00 00 00');
assert.equal(W.encodeCommand('barcode_mode', { mode: 1, white: 4096, black: 0 }), null);
assert.equal(W.encodeCommand('motor_trim', { left: 49, right: 100 }), null);
assert.equal(W.encodeCommand('drive', { left: 70000, right: 0 }), null);
assert.equal(W.encodeCommand('no_such_command'), null);
/* Every command fits in 5 bytes (the old JSON was up to ~45). */
assert(['stop', 'left', 'line_follow', 'imu_reset'].every(c => W.encodeCommand(c).length === 1));

/* Encoder counts: 16 bits on the wire, unwrapped across rollovers. */
{
  const u = new W.TickUnwrapper();
  assert.equal(u.update(0xfffe), -2);
  assert.equal(u.update(0x0003), 3);
  assert.equal(u.update(0x7ff0), 0x7ff0);
  assert.equal(u.update(0xeff0), 0xeff0);
  assert.equal(u.update(0x0010), 0x10010); /* forward past 0xffff */
  assert.equal(u.update(0xfff0), 0xfff0);  /* and back again */
}

console.log('PASS: dashboard decodes the firmware golden telemetry (28 B), debug (25 B) and IMU (14 B), and packs the same command bytes');
