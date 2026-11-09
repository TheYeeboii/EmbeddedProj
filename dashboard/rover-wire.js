/* Binary MQTT format shared with the rover (firmware/app_program/robot_wire.c,
 * docs/MQTT_BINARY_FORMAT.md). Change both sides together and raise VERSION.
 *
 * Rover -> dashboard: byte 0 = type (high nibble) + version (low nibble), then
 * the fields back to back at their bit widths, most significant bit first.
 * Dashboard -> rover: a 4-bit opcode, then the command's values.
 *
 * The decoder returns objects with the same field names the JSON messages
 * used, so the rest of the dashboard did not have to change. */
(function (root) {
  'use strict';

  const VERSION = 1;
  const TYPE = { telemetry: 1, debug: 2, heartbeat: 3, statusOffline: 4, statusOnline: 5, imu: 6 };
  /* Byte 0 of the JSON diagnostic streams (DIAGNOSTICS=1 builds): '{' = type 7. */
  const JSON_FIRST_BYTE = 0x7b;
  const LENGTH = { 1: 28, 2: 25, 3: 1, 4: 1, 5: 1, 6: 14 };

  /* Indexed by the firmware's enums. */
  const MODES = ['idle', 'drive', 'motion', 'line', 'avoid'];
  const SENSOR_STATES = ['online', 'compass_error', 'imu_error'];
  const DECISIONS = ['none', 'left', 'right'];
  const AVOID_STATUS = ['idle', 'scanning', 'turning', 'complete', 'blocked', 'imu_unavailable',
    'imu_lost', 'timeout', 'cancelled', 'forward', 'recovering', 'reversing'];
  /* robot_fault.h bit order. */
  const FAULTS = ['wifi_loss', 'mqtt_loss', 'imu', 'no_echo', 'encoder', 'motor_stall'];

  const OP = {
    stop: 1, drive_preset: 2, drive: 3, line_follow: 4, line_stop: 5, imu_reset: 6,
    reset_all: 8, motor_trim: 9, speed_limit: 10, servo_angle: 11, barcode_mode: 12,
    barcode_config: 13, line_stage: 14, line_speed: 15
  };
  const PRESETS = { forward: 0, reverse: 1, left: 2, right: 3 };
  const TRIM_OFFSET = 50;

  class Reader {
    constructor(bytes) { this.bytes = bytes; this.bit = 0; }
    get(width) {
      let value = 0;
      for (let i = 0; i < width; i++, this.bit++) {
        const byte = this.bytes[this.bit >> 3];
        value = value * 2 + ((byte >> (7 - (this.bit & 7))) & 1);
      }
      return value;
    }
    signed(width) {
      const value = this.get(width);
      return value >= 2 ** (width - 1) ? value - 2 ** width : value;
    }
    flag() { return this.get(1) === 1; }
  }

  class Writer {
    constructor(bytes) { this.bytes = new Uint8Array(bytes); this.bit = 0; }
    put(value, width) {
      for (let i = width - 1; i >= 0; i--, this.bit++) {
        if (Math.floor(value / 2 ** i) % 2) this.bytes[this.bit >> 3] |= 0x80 >> (this.bit & 7);
      }
      return this;
    }
    signed(value, width) { return this.put(value < 0 ? value + 2 ** width : value, width); }
  }

  function faultList(mask) {
    return FAULTS.filter((name, bit) => mask & (1 << bit));
  }

  function telemetry(r) {
    const data = { uptime_s: r.get(20) };
    data.ts_ms = data.uptime_s * 1000;
    data.mode = MODES[r.get(3)] ?? 'unknown';
    data.state = SENSOR_STATES[r.get(2)] ?? 'unknown';
    data.status = data.state;
    data.fault_mask = r.get(8);
    data.faults = faultList(data.fault_mask);
    data.left_ir = r.get(12);
    data.center_ir = r.get(12);
    data.right_ir = r.get(12);
    data.distance_cm = r.get(9);
    data.servo_angle_deg = r.get(8);
    data.heading_deg = r.get(12) / 10;
    data.pitch_deg = r.signed(11) / 10;
    data.max_pitch_deg = r.signed(11) / 10;
    data.left_ticks_raw = r.get(16);
    data.right_ticks_raw = r.get(16);
    data.motor_left = r.get(16);
    data.motor_right = r.get(16);
    data.trim_left = r.get(6) + TRIM_OFFSET;
    data.trim_right = r.get(6) + TRIM_OFFSET;
    data.line_base = r.get(7);
    data.line_turn = r.get(6);
    data.speed = r.get(7);
    return data;
  }

  function debug(r) {
    const data = { raw_left: r.get(12), raw_center: r.get(12), raw_right: r.get(12) };
    data.norm_left = r.get(10);
    data.norm_center = r.get(10);
    data.norm_right = r.get(10);
    data.position = r.signed(11);
    data.line = r.flag();
    data.junction = r.flag();
    data.barcode = r.flag();
    data.command = r.get(7);
    data.ultra_left = r.get(9);
    data.ultra_right = r.get(9);
    data.decision = DECISIONS[r.get(2)] ?? 'none';
    data.avoid_status = AVOID_STATUS[r.get(4)] ?? 'idle';
    data.avoid_heading_change_x10 = r.signed(12);
    data.avoid_heading_target_x10 = r.get(12);
    data.avoid_motor_left_pwm = r.signed(16);
    data.avoid_motor_right_pwm = r.signed(16);
    data.avoid_forward_mm = r.get(11);
    data.avoid_forward_target_mm = r.get(11);
    return data;
  }

  /* Buddy 4's IMU / hump report (type 6). Same field names as its old JSON. */
  function imu(r) {
    const data = { imu_fault: r.get(1), imu_calibrated: r.get(1), height_valid: r.get(1) };
    data.imu_interval_ms = r.get(10);
    data.roll_x10 = r.signed(12);
    data.hump_state = r.get(2);
    data.hump_seq = r.get(8);
    data.hump_peak_x10 = r.signed(11);
    data.highest_peak_x10 = r.signed(11);
    data.hump_height_mm = r.get(8);
    data.highest_hump_mm = r.get(8);
    data.motion = r.get(3);
    data.turn_rate_x10 = r.signed(15);
    data.impacts = r.get(8);
    return data;
  }

  /* Returns { kind, data } where kind is 'json' (a diagnostic text stream),
   * 'telemetry', 'debug', 'imu', 'heartbeat', 'status', 'version_mismatch' or
   * 'invalid'. */
  function decode(input) {
    const bytes = input instanceof Uint8Array ? input : new Uint8Array(input);
    if (bytes.length === 0) return { kind: 'invalid', data: null };
    if (bytes[0] === JSON_FIRST_BYTE) {
      try { return { kind: 'json', data: JSON.parse(new TextDecoder().decode(bytes)) }; }
      catch { return { kind: 'invalid', data: null }; }
    }
    const type = bytes[0] >> 4;
    const version = bytes[0] & 0x0f;
    if (!LENGTH[type]) return { kind: 'invalid', data: null };
    if (version !== VERSION) return { kind: 'version_mismatch', data: { type, version, expected: VERSION } };
    if (bytes.length !== LENGTH[type]) return { kind: 'invalid', data: null };
    const r = new Reader(bytes);
    r.get(8);
    if (type === TYPE.telemetry) return { kind: 'telemetry', data: telemetry(r) };
    if (type === TYPE.debug) return { kind: 'debug', data: debug(r) };
    if (type === TYPE.imu) return { kind: 'imu', data: imu(r) };
    if (type === TYPE.heartbeat) return { kind: 'heartbeat', data: {} };
    return { kind: 'status', data: { state: type === TYPE.statusOnline ? 'online' : 'offline' } };
  }

  const inRange = (n, min, max) => Number.isInteger(n) && n >= min && n <= max;

  /* Pack a dashboard command. Returns a Uint8Array, or null when the name is
   * unknown or a value cannot be represented (the firmware would reject it). */
  function encodeCommand(name, values = {}) {
    if (name in PRESETS) return new Writer(1).put(OP.drive_preset, 4).put(PRESETS[name], 2).bytes;
    switch (name) {
      case 'stop': case 'line_follow': case 'line_stop': case 'imu_reset': case 'reset_all':
        return new Writer(1).put(OP[name], 4).bytes;
      case 'drive':
        if (!inRange(values.left, 0, 65535) || !inRange(values.right, 0, 65535)) return null;
        return new Writer(5).put(OP.drive, 4).put(values.left, 16).put(values.right, 16).bytes;
      case 'motor_trim':
        if (!inRange(values.left, 50, 100) || !inRange(values.right, 50, 100)) return null;
        return new Writer(2).put(OP.motor_trim, 4).put(values.left - TRIM_OFFSET, 6)
          .put(values.right - TRIM_OFFSET, 6).bytes;
      case 'speed_limit':
        if (!inRange(values.percent, 0, 127)) return null;
        return new Writer(2).put(OP.speed_limit, 4).put(values.percent, 7).bytes;
      case 'servo_angle':
        if (!inRange(values.angle, 0, 255)) return null;
        return new Writer(2).put(OP.servo_angle, 4).put(values.angle, 8).bytes;
      case 'barcode_mode':
        if (!inRange(values.mode, 0, 1)) return null;
        return new Writer(1).put(OP.barcode_mode, 4).put(values.mode, 1).bytes;
      case 'barcode_config':
        if (!inRange(values.min_element_mm, 0, 1023) || !inRange(values.offset_left_mm, -512, 511)) return null;
        return new Writer(3).put(OP.barcode_config, 4).put(values.min_element_mm, 10)
          .signed(values.offset_left_mm, 10).bytes;
      case 'line_stage':
        if (!inRange(values.stage, 0, 3)) return null;
        return new Writer(1).put(OP.line_stage, 4).put(values.stage, 2).bytes;
      case 'line_speed':
        if (!inRange(values.base, 0, 127) || !inRange(values.turn, 0, 63)) return null;
        return new Writer(3).put(OP.line_speed, 4).put(values.base, 7).put(values.turn, 6).bytes;
      default:
        return null;
    }
  }

  /* Telemetry carries only the low 16 bits of each encoder count. Feed every
   * reading in order; it returns the full signed count, assuming the wheel
   * moved less than 32768 ticks between two messages. */
  class TickUnwrapper {
    constructor() { this.total = null; this.last = 0; }
    update(raw) {
      if (this.total === null) {
        this.total = raw >= 0x8000 ? raw - 0x10000 : raw;
      } else {
        let delta = raw - this.last;
        if (delta > 0x7fff) delta -= 0x10000;
        else if (delta < -0x8000) delta += 0x10000;
        this.total += delta;
      }
      this.last = raw;
      return this.total;
    }
    reset() { this.total = null; this.last = 0; }
  }

  function hex(bytes) {
    return Array.from(bytes, b => b.toString(16).padStart(2, '0')).join(' ');
  }

  root.RoverWire = { VERSION, TYPE, LENGTH, OP, FAULTS, decode, encodeCommand, TickUnwrapper, hex, Writer, Reader };
  if (typeof module !== 'undefined' && module.exports) module.exports = root.RoverWire;
})(globalThis);
