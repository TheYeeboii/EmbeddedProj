'use strict';
/* Dashboard side of the binary MQTT format. The golden bytes are printed by
 * "test_robot_wire --print" (tests/test_robot_wire.c) from the firmware's own
 * encoder, so this proves both sides agree on every bit. */
const assert = require('node:assert/strict');
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
}

/* One-byte messages, the JSON reports that still share topics, and bad input. */
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

console.log('PASS: dashboard decodes the firmware golden telemetry (28 B) and debug (25 B), and packs the same command bytes');
