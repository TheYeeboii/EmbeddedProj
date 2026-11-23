const topics = {
  telemetry: 'rover/telemetry',
  heartbeat: 'rover/heartbeat',
  status: 'rover/status',
  commands: 'rover/commands',
  debug: 'rover/debug',
  actions: 'rover/actions',
  steering: 'rover/steering',
  barcode: 'rover/barcode',
  irStream: 'rover/ir'
};

const brokerHost = new URLSearchParams(window.location.search).get('broker') || '192.168.137.1';
const brokerUrl = `ws://${brokerHost}:9001`;

/* Firmware: raw drive stops after 1 s, heartbeat every 10 s, telemetry every 1 s. */
const DRIVE_REPEAT_MS = 300;
const STALE_AFTER_MS = 25000;
const SERIAL_MAX_LINES = 200;
const SPARK_POINTS = 40;
/* Buddy 4 BUDDY4_MOTION_* values (buddy4_processing.h). */
const MOTION_NAMES = ['Unknown', 'Stationary', 'Cruising', 'Accelerating', 'Decelerating', 'Turning', 'On a hump', 'Lifted / overturned'];
const MODE_TONES = { idle: 'muted', drive: 'accent', motion: 'accent', line: 'success' };
/* Fault-byte bits worth showing while the robot is connected. */
const FAULT_NAMES = { imu: 'IMU fault', no_echo: 'Ultrasonic not answering', encoder: 'Encoder fault', motor_stall: 'Motor stall' };
/* Buddy 2 defaults (buddy2_motion.c): 65 mm wheel, 387 encoder ticks per turn. */
const CM_PER_TICK = (Math.PI * 6.5) / 387;
/* Buddy 5 avoid_status names (robot_telemetry.c) and the badge tone for each. */
const AVOID_TONES = {
  idle: 'muted', scanning: 'accent', turning: 'accent', forward: 'accent', recovering: 'accent', reversing: 'accent',
  complete: 'success', cancelled: 'muted',
  blocked: 'warning', imu_unavailable: 'destructive', imu_lost: 'destructive', timeout: 'destructive'
};
const KEY_COMMANDS = {
  w: 'forward', arrowup: 'forward',
  s: 'reverse', arrowdown: 'reverse',
  a: 'left', arrowleft: 'left',
  d: 'right', arrowright: 'right'
};

const $ = (selector) => document.querySelector(selector);
const numberFormat = new Intl.NumberFormat(undefined, { maximumFractionDigits: 1 });
const timeFormat = new Intl.DateTimeFormat(undefined, { hour: '2-digit', minute: '2-digit', second: '2-digit' });
const relativeFormat = new Intl.RelativeTimeFormat(undefined, { numeric: 'auto', style: 'short' });

const connection = $('#connection');
const commandResult = $('#command-result');
const serialLog = $('#serial-log');
const actionLog = new window.RoverActionLog($('#action-log'), $('#steering-action'));
const steeringView = new window.RoverSteeringView($('#steering-visual'));
const barcodeView = new window.RoverBarcodeView($('#barcode-visual'));
const irStreamView = new window.RoverIRStreamView($('#barcode-visual'));
const barcodeConfiguration = new window.RoverBarcodeConfiguration($('#barcode-config-form'), {
  send: (command, values) => sendCommand(command, values, { quiet: true })
});

const irThresholdViews = ['ir-threshold-editor', 'ir-threshold-editor-left',
  'ir-threshold-editor-right'].map((id, sensor) => new window.RoverIRThresholdView(
    $('#' + id), (command, values) => sendCommand(command, values, { quiet: true }), sensor));

let client = null;
let serialPaused = false;
let robotOffline = false;
let lastSeen = 0;
let distanceHistory = [];
/* Set when the rover's binary format version differs from rover-wire.js. */
let wireMismatch = null;
/* Telemetry sends the low 16 bits of each encoder count. */
const leftTicks = new window.RoverWire.TickUnwrapper();
const rightTicks = new window.RoverWire.TickUnwrapper();
/* Uptime going backwards means the rover rebooted (its ticks restart at 0). */
let lastUptime = 0;

/* ---------- helpers ---------- */

function setText(id, value) {
  const element = $(`#${id}`);
  const text = value == null ? '' : String(value);
  if (element.textContent !== text) element.textContent = text;
}

function fmt(value) {
  return typeof value === 'number' && Number.isFinite(value) ? numberFormat.format(value) : '--';
}

function setBadge(id, text, tone) {
  const badge = $(`#${id}`);
  if (badge.textContent !== text) badge.textContent = text;
  if (badge.dataset.tone !== tone) badge.dataset.tone = tone;
}

function formatUptime(ms) {
  if (typeof ms !== 'number') return '--';
  const total = Math.floor(ms / 1000);
  const h = Math.floor(total / 3600);
  const m = Math.floor((total % 3600) / 60);
  const s = total % 60;
  const pad = (n) => String(n).padStart(2, '0');
  return h > 0 ? `${h}:${pad(m)}:${pad(s)}` : `${m}:${pad(s)}`;
}

function setConnection(state, text) {
  connection.dataset.state = state;
  connection.querySelector('.pill-label').textContent = text;
}

function markSeen() {
  lastSeen = Date.now();
  robotOffline = false;
  refreshRobotLink();
}

function refreshRobotLink() {
  if (robotOffline) {
    actionLog.unavailable();
    steeringView.unavailable();
    barcodeView.unavailable();
    irStreamView.unavailable();
    barcodeConfiguration.unavailable();
    setBadge('robot-link', 'Offline', 'destructive');
  } else if (wireMismatch) {
    /* The rover runs another message layout: its values would be garbage. */
    setBadge('robot-link', `Firmware format v${wireMismatch.version}, dashboard v${wireMismatch.expected}`, 'warning');
  } else if (!lastSeen) {
    setBadge('robot-link', 'Unknown', 'muted');
  } else if (Date.now() - lastSeen > STALE_AFTER_MS) {
    actionLog.unavailable();
    steeringView.unavailable();
    barcodeView.unavailable();
    irStreamView.unavailable();
    barcodeConfiguration.unavailable();
    setBadge('robot-link', 'Stale', 'warning');
  } else {
    setBadge('robot-link', 'Online', 'success');
  }
  if (lastSeen) {
    const seconds = Math.round((lastSeen - Date.now()) / 1000);
    setText('heartbeat', relativeFormat.format(seconds, 'second'));
  }
}

/* ---------- lift stop card (Buddy 4) ---------- */

/* What the lift card needs: the IMU's lifted state and count, and what the
 * motors were doing around each lift. A lift counts as "stopped" when the
 * mode is idle and both motor outputs are 0 within 1 s of the lift. */
const LIFT_STOP_WINDOW_MS = 1000;
/* IMU reports can reach the dashboard several seconds late over a busy
 * Wi-Fi link, so a lift stays shown for 10 s and is matched to whatever was
 * driving up to 10 s before it arrived. */
const LIFT_SHOW_MS = 10000;
const LIFT_FROM_MS = 10000;
const lift = { lifted: null, count: null, at: 0, stopped: null, from: null,
  mode: null, left: null, right: null, activeMode: null, activeAt: 0 };

function noteDrive(mode, left, right) {
  lift.mode = mode;
  lift.left = left;
  lift.right = right;
  const running = mode !== 'idle' || left !== 0 || right !== 0;
  if (running) { lift.activeMode = mode; lift.activeAt = Date.now(); }
  if (lift.at && lift.stopped === null && Date.now() - lift.at <= LIFT_STOP_WINDOW_MS && !running) lift.stopped = true;
  renderLift();
}

function noteLift(data) {
  const lifted = data.motion === 7;
  const counted = lift.count !== null && data.lifts > lift.count;
  if (counted || (lifted && lift.lifted === false)) {
    lift.at = Date.now();
    lift.stopped = null;
    lift.from = Date.now() - lift.activeAt < LIFT_FROM_MS ? lift.activeMode : null;
  }
  lift.lifted = lifted;
  lift.count = data.lifts;
  renderLift();
}

function renderLift() {
  const now = Date.now();
  if (lift.at && lift.stopped === null && now - lift.at > LIFT_STOP_WINDOW_MS) {
    lift.stopped = lift.mode === 'idle' && lift.left === 0 && lift.right === 0;
  }
  /* The rover reports "lifted" from the pick-up push until its IR sensors
   * see the floor again (or while tipped past 45°). Reports can arrive late,
   * so a lift that already ended still shows for a while. */
  const recent = lift.at && now - lift.at < LIFT_SHOW_MS;
  setText('lift-now', lift.lifted === null ? '--'
    : (lift.lifted ? 'In the air / overturned' : (recent ? 'Put back down' : 'On the ground')));
  const running = lift.mode !== null && (lift.mode !== 'idle' || lift.left !== 0 || lift.right !== 0);
  setText('lift-motors', lift.mode === null ? '--' : (running ? 'Running' : 'Stopped'));
  setText('lift-motors-sub', lift.mode === null ? '--' : `L ${fmt(lift.left)} · R ${fmt(lift.right)} · ${lift.mode}`);
  setText('lift-count', fmt(lift.count));
  if (lift.at) {
    setText('lift-last', relativeFormat.format(Math.round((lift.at - now) / 1000), 'second'));
    setText('lift-last-sub', lift.stopped === null ? 'checking motors…'
      : lift.stopped ? (lift.from ? `stopped ${lift.from === 'line' ? 'line following' : lift.from}` : 'rover was already idle')
        : 'motors did NOT stop');
  }
  if (lift.lifted) setBadge('lift-status', running ? 'Lifted: motors still running' : 'Lifted: motors stopped', running ? 'destructive' : 'warning');
  else if (lift.at && lift.stopped === false) setBadge('lift-status', 'Failed: motors kept running', 'destructive');
  else if (lift.at && lift.stopped && now - lift.at < 30000) setBadge('lift-status', 'Stopped on lift', 'success');
  else if (lift.lifted === false) setBadge('lift-status', 'Armed', 'muted');
}

setInterval(renderLift, 1000);

/* ---------- telemetry ---------- */

function showTelemetry(data) {
  // Buddy 4 reports independently; preserve the other sensor tiles.
  if (Object.prototype.hasOwnProperty.call(data, 'hump_state')) {
    if (data.imu_fault) setBadge('imu-health', 'Reading unavailable', 'destructive');
    else if (data.imu_calibrated) setBadge('imu-health', 'Calibrated', 'success');
    else setBadge('imu-health', 'Needs calibration', 'warning');
    setText('imu-interval', `${data.imu_interval_ms} ms`);
    setText('imu-roll', fmt(data.roll_x10 / 10));
    setText('hump-state', ['Flat', 'Climbing', 'Top', 'Descending'][data.hump_state] ?? 'Unknown');
    setText('hump-count', `${data.hump_seq} hump${data.hump_seq === 1 ? '' : 's'}`);
    // hump_peak_x10 also tracks climbs that never complete, so show it only
    // during a hump; once flat, show the steepest completed hump.
    const inHump = data.hump_state !== 0;
    setText('hump-peak', inHump ? fmt(data.hump_peak_x10 / 10) : (data.hump_seq > 0 ? fmt(data.highest_peak_x10 / 10) : '--'));
    setText('hump-peak-note', inHump ? 'this hump' : (data.hump_seq > 0 ? 'steepest' : 'none yet'));
    setText('hump-height', data.height_valid ? fmt(data.hump_height_mm) : (data.highest_hump_mm > 0 ? fmt(data.highest_hump_mm) : '--'));
    setText('hump-height-note', data.height_valid ? `max ${data.highest_hump_mm}`
      : (data.highest_hump_mm > 0 ? 'highest' : 'none yet'));
    setText('imu-motion', MOTION_NAMES[data.motion] ?? '--');
    setText('imu-turn-rate', fmt(data.turn_rate_x10 / 10));
    setText('imu-lifts', fmt(data.lifts));
    noteLift(data);
    return;
  }

  markSeen();
  /* Shared fault byte (robot_fault.h). Wi-Fi/MQTT loss cannot be seen while
   * the message arrives, so only the sensor and motor faults are shown. */
  const faults = (data.faults || []).filter(f => FAULT_NAMES[f]).map(f => FAULT_NAMES[f]);
  const sensorsOk = (data.state || data.status) === 'online' && faults.length === 0;
  const stateText = (data.state || data.status) === 'online' ? ''
    : (data.state === 'compass_error' ? 'Compass error' : (data.state || 'Unknown'));
  setText('state', sensorsOk ? 'All sensors OK' : [stateText, ...faults].filter(Boolean).join(', '));
  $('#state').style.color = sensorsOk ? '' : 'var(--warning)';

  barcodeConfiguration.robot(data.mode);
  const mode = data.mode || 'idle';
  noteDrive(mode, data.motor_left, data.motor_right);
  setBadge('mode', mode, MODE_TONES[mode] || 'muted');

  setText('distance', fmt(data.distance_cm));
  setText('heading', fmt(data.heading_deg));
  setText('pitch', fmt(data.pitch_deg));
  setText('left-ticks', fmt(data.left_ticks));
  setText('right-ticks', fmt(data.right_ticks));
  setText('odo-left', typeof data.left_ticks === 'number' ? fmt(data.left_ticks * CM_PER_TICK) : '--');
  setText('odo-right', typeof data.right_ticks === 'number' ? fmt(data.right_ticks * CM_PER_TICK) : '--');
  showImuHealth(data.state);
  setText('max-pitch', fmt(data.max_pitch_deg));
  setText('motor-left', fmt(data.motor_left));
  setText('motor-right', fmt(data.motor_right));
  setText('uptime', formatUptime(data.ts_ms));
  if (typeof data.trim_left === 'number') {
    setText('trim-current', `L ${data.trim_left}% R ${data.trim_right}%`);
    syncInputs('trim', { 'trim-left': data.trim_left, 'trim-right': data.trim_right });
  }
  if (typeof data.speed === 'number') {
    setText('speed-current', `${data.speed}%`);
    syncInputs('speed', { 'speed-limit': data.speed });
  }
  if (typeof data.servo_angle_deg === 'number') {
    setText('servo-current', `${data.servo_angle_deg}°`);
    syncInputs('servo', { 'servo-angle': data.servo_angle_deg });
  }
  setServoControlsDisabled(mode !== 'idle');
  if (typeof data.line_base === 'number') {
    setText('line-current', `${data.line_base}% / ${data.line_turn}%`);
    syncInputs('line', { 'line-base': data.line_base, 'line-turn': data.line_turn });
  }
  setText('updated', `Updated ${timeFormat.format(new Date())}`);

  /* Telemetry carries raw IR too; debug messages refresh the bars more often. */
  setText('raw-0', fmt(data.center_ir));
  setText('raw-1', fmt(data.left_ir));
  setText('raw-2', fmt(data.right_ir));

  if (typeof data.heading_deg === 'number') {
    $('#compass-needle').style.transform = `rotate(${data.heading_deg}deg)`;
  }
  if (typeof data.distance_cm === 'number') {
    distanceHistory = [...distanceHistory, data.distance_cm].slice(-SPARK_POINTS);
    drawSpark();
  }
}

/* Buddy 4 tab: the firmware reports "compass_error" when !mag_ok, else
 * "imu_error" when !accel_ok, else "online"; accel is not checked when the
 * compass has failed. */
function showImuHealth(state) {
  if (state === 'compass_error') {
    setBadge('mag-ok', 'Error', 'destructive');
    setBadge('accel-ok', 'Not checked', 'muted');
  } else if (state === 'imu_error') {
    setBadge('mag-ok', 'OK', 'success');
    setBadge('accel-ok', 'Error', 'destructive');
  } else if (state === 'online') {
    setBadge('mag-ok', 'OK', 'success');
    setBadge('accel-ok', 'OK', 'success');
  }
}

/* Keep tuning inputs equal to the robot's real values (another dashboard or a
 * reboot may change them), except while the user is editing that group. */
const SYNC_HOLD_MS = 8000;
const lastEdited = {};
function noteEdit(group) {
  lastEdited[group] = Date.now();
}
function syncInputs(group, values) {
  if (Date.now() - (lastEdited[group] || 0) < SYNC_HOLD_MS) return;
  for (const [id, value] of Object.entries(values)) {
    const input = $(`#${id}`);
    if (document.activeElement !== input) input.value = value;
  }
}

function drawSpark() {
  const values = distanceHistory;
  if (values.length < 2) return;
  const min = Math.min(...values);
  const max = Math.max(...values);
  const span = max - min || 1;
  const step = 200 / (SPARK_POINTS - 1);
  const offset = SPARK_POINTS - values.length;
  const points = values.map((v, i) => `${((i + offset) * step).toFixed(1)},${(38 - ((v - min) / span) * 36).toFixed(1)}`);
  $('#distance-spark polyline').setAttribute('points', points.join(' '));
}

/* ---------- line sensor debug ---------- */

function showDebug(data) {
  markSeen();
  showIrRaw('left', data.raw_center);
  showIrRaw('right', data.raw_left);
  showBarcodeRaw(data.raw_right);
  /* Firmware ADC order: 0 physical front right, 1 physical front left, 2 barcode. */
  [
    [data.norm_center, data.raw_center],
    [data.norm_left, data.raw_left],
    [data.norm_right, data.raw_right]
  ].forEach(([norm, raw], index) => {
    setText(`norm-${index}`, fmt(norm));
    setText(`raw-${index}`, fmt(raw));
    const ratio = Math.min(Math.max((norm ?? 0) / 1000, 0), 1);
    $(`#bar-${index}`).style.transform = `scaleX(${ratio})`;
  });

  const position = typeof data.position === 'number' ? data.position : 0;
  setText('position', data.line ? fmt(position) : 'No line');
  const marker = $('#position-marker');
  marker.style.left = `${50 + Math.max(-1000, Math.min(1000, position)) / 20}%`;
  marker.classList.toggle('on', Boolean(data.line));

  $('#chip-line').classList.toggle('on', Boolean(data.line));
  $('#chip-junction').classList.toggle('on', Boolean(data.junction));
  $('#chip-barcode').classList.toggle('on', Boolean(data.barcode));
  const barcodeName = Number.isInteger(data.command) && data.command >= 65 && data.command <= 90
    ? String.fromCharCode(data.command) : '';
  $('#barcode-cmd').textContent = data.barcode && barcodeName ? barcodeName : '';

  showAvoidance(data);
  appendSerial(data);
}

/* Buddy 5 tab: obstacle avoidance fields of rover/debug. Same rules as the
 * serial line: scan values only count once a decision was made or the
 * avoidance is blocked, and 0 cm means no valid echo. */
function showAvoidance(data) {
  if (typeof data.avoid_status !== 'string') return;
  setBadge('avoid-status', data.avoid_status.replace('_', ' '), AVOID_TONES[data.avoid_status] || 'muted');
  const scanned = (data.decision && data.decision !== 'none') || data.avoid_status === 'blocked';
  const distance = value => (value === 0 ? 'invalid' : fmt(value));
  setText('avoid-left', scanned ? distance(data.ultra_left) : '--');
  setText('avoid-right', scanned ? distance(data.ultra_right) : '--');
  setText('avoid-decision', data.decision && data.decision !== 'none' ? `Turn ${data.decision}` : '--');
  setText('avoid-pwm-left', fmt(data.avoid_motor_left_pwm));
  setText('avoid-pwm-right', fmt(data.avoid_motor_right_pwm));

  const change = typeof data.avoid_heading_change_x10 === 'number' ? data.avoid_heading_change_x10 / 10 : null;
  const target = typeof data.avoid_heading_target_x10 === 'number' && data.avoid_heading_target_x10 > 0
    ? data.avoid_heading_target_x10 / 10 : null;
  const holding = data.avoid_status === 'forward' || data.avoid_status === 'recovering';
  if (holding) {
    setText('avoid-heading', `${fmt(change)}°`);
    setText('avoid-heading-note', 'heading error');
  } else {
    setText('avoid-heading', target === null ? '--' : `${fmt(change ?? 0)} / ${fmt(target)}°`);
    setText('avoid-heading-note', 'turned / target');
  }

  const goal = data.avoid_forward_target_mm;
  const done = typeof goal === 'number' && goal > 0 && typeof data.avoid_forward_mm === 'number';
  setText('avoid-forward', done ? `${data.avoid_forward_mm} / ${goal} mm` : '--');
  $('#avoid-forward-bar').style.transform = `scaleX(${done ? Math.min(data.avoid_forward_mm / goal, 1) : 0})`;
}

function appendSerial(data) {
  if (serialPaused) return;
  const empty = serialLog.querySelector('.console-empty');
  if (empty) empty.remove();

  const line = document.createElement('div');
  const parts = [
    ['t', `${timeFormat.format(new Date())}  `],
    ['k', 'raw '], ['', `${data.raw_left},${data.raw_center},${data.raw_right}  `],
    ['k', 'norm '], ['', `${data.norm_left},${data.norm_center},${data.norm_right}  `],
    ['k', 'pos '], ['', `${data.position}  `],
    ['k', 'line '], [data.line ? 'y' : '', `${data.line ? 'yes' : 'no'}  `],
    ['k', 'junction '], [data.junction ? 'y' : '', `${data.junction ? 'yes' : 'no'}  `],
    ['k', 'barcode '], [data.barcode ? 'y' : '', `${data.barcode ? 'yes' : 'no'}  `],
    ['k', 'cmd '], ['', String(data.command)]
  ];
  if ((data.decision && data.decision !== 'none') ||
      (data.avoid_status && data.avoid_status === 'blocked')) {
    const distance = value => value === 0 ? 'invalid' : `${value} cm`;
    parts.push(
      ['k', '  ultra_left: '], ['', `${distance(data.ultra_left)}  `],
      ['k', 'ultra_right: '], ['', `${distance(data.ultra_right)}  `]
    );
    if (data.decision && data.decision !== 'none') {
      parts.push(['k', 'decision: '], [data.decision === 'left' ? 'y' : '', data.decision]);
    }
    if (data.avoid_status === 'blocked') {
      /* The firmware does not send why; its USB console prints the cause. */
      parts.push(['k', 'reason: '], ['', 'route stopped (cause in USB console)']);
    }
  }
  if (data.avoid_status && data.avoid_status !== 'idle') {
    parts.push(['k', '  avoid: '], [data.avoid_status === 'turning' ? 'y' : '', data.avoid_status]);
    if ((data.avoid_status === 'forward' || data.avoid_status === 'recovering') &&
        typeof data.avoid_heading_change_x10 === 'number') {
      const error = (Number(data.avoid_heading_change_x10) / 10).toFixed(1);
      parts.push(['k', '  heading error: '], ['', `${error} deg`]);
      if (typeof data.avoid_forward_mm === 'number' &&
          typeof data.avoid_forward_target_mm === 'number' &&
          data.avoid_forward_target_mm > 0) {
        parts.push(['k', '  forward: '], ['', `${data.avoid_forward_mm}/${data.avoid_forward_target_mm} mm`]);
      }
    } else if (typeof data.avoid_heading_target_x10 === 'number' && data.avoid_heading_target_x10 > 0) {
      const changed = (Number(data.avoid_heading_change_x10 || 0) / 10).toFixed(1);
      const target = (data.avoid_heading_target_x10 / 10).toFixed(1);
      parts.push(['k', '  heading: '], ['', `${changed}/${target} deg`]);
    }
    if (typeof data.avoid_motor_left_pwm === 'number' &&
        typeof data.avoid_motor_right_pwm === 'number') {
      parts.push(['k', '  motor PWM: '], ['', `L=${data.avoid_motor_left_pwm} R=${data.avoid_motor_right_pwm}`]);
    }
  }
  for (const [cls, text] of parts) {
    const span = document.createElement('span');
    if (cls) span.className = cls;
    span.textContent = text;
    line.append(span);
  }

  const stickToBottom = serialLog.scrollHeight - serialLog.scrollTop - serialLog.clientHeight < 24;
  serialLog.append(line);
  while (serialLog.childElementCount > SERIAL_MAX_LINES) serialLog.firstElementChild.remove();
  if (stickToBottom) serialLog.scrollTop = serialLog.scrollHeight;
}

/* ---------- commands ---------- */

function sendCommand(command, values = {}, { quiet = false } = {}) {
  if (!client || !client.connected) {
    commandResult.textContent = 'Broker offline. Check Mosquitto is running.';
    commandResult.style.color = 'var(--destructive)';
    return false;
  }
  /* Packed binary command (rover-wire.js); null = a value the format cannot
   * carry, which the input checks above should already have caught. */
  const payload = window.RoverWire.encodeCommand(command, values);
  if (!payload) {
    commandResult.textContent = `Not sent: ${command.replace('_', ' ')} has an invalid value.`;
    commandResult.style.color = 'var(--destructive)';
    return false;
  }
  client.publish(topics.commands, payload, { qos: 0 });
  if (!quiet) {
    commandResult.textContent = `Sent ${command.replace('_', ' ')} at ${timeFormat.format(new Date())}`;
    commandResult.style.color = '';
  }
  return true;
}

/* Hold-to-drive: re-send while held so the 1 s firmware timeout never trips. */
let held = null;

function startHold(command, button) {
  if (held && held.command === command) return;
  stopHold(false);
  if (!sendCommand(command)) return;
  held = {
    command,
    button,
    timer: setInterval(() => sendCommand(command, {}, { quiet: true }), DRIVE_REPEAT_MS)
  };
  button?.classList.add('is-held');
}

function stopHold(sendStop = true) {
  if (!held) return;
  clearInterval(held.timer);
  held.button?.classList.remove('is-held');
  held = null;
  if (sendStop) sendCommand('stop');
}

document.querySelectorAll('[data-command]').forEach((button) => {
  const command = button.dataset.command;
  if (command === 'stop') {
    button.addEventListener('click', () => { stopHold(false); sendCommand('stop'); });
    return;
  }
  button.addEventListener('pointerdown', (event) => {
    if (event.button !== 0) return;
    button.setPointerCapture(event.pointerId);
    startHold(command, button);
  });
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach((type) => {
    button.addEventListener(type, () => { if (held?.button === button) stopHold(); });
  });
  /* Keyboard activation (Enter/Space on a focused button) sends a single pulse. */
  button.addEventListener('click', (event) => { if (event.detail === 0) sendCommand(command); });
});

function isTyping(target) {
  return target instanceof HTMLInputElement || target instanceof HTMLTextAreaElement || target instanceof HTMLSelectElement || target?.isContentEditable;
}

document.addEventListener('keydown', (event) => {
  if (event.ctrlKey || event.metaKey || event.altKey || isTyping(event.target)) return;
  const key = event.key.toLowerCase();
  if (key === 'escape' || (key === ' ' && !(event.target instanceof HTMLButtonElement))) {
    event.preventDefault();
    stopHold(false);
    sendCommand('stop');
    return;
  }
  const command = KEY_COMMANDS[key];
  if (!command) return;
  event.preventDefault();
  if (!event.repeat) startHold(command, $(`[data-command="${command}"]`));
});

document.addEventListener('keyup', (event) => {
  const command = KEY_COMMANDS[event.key.toLowerCase()];
  if (command && held?.command === command) stopHold();
});

window.addEventListener('blur', () => stopHold());
document.addEventListener('visibilitychange', () => { if (document.hidden) stopHold(); });

/* ---------- forms ---------- */

function readInt(input, min, max) {
  const value = Number(input.value);
  const ok = input.value.trim() !== '' && Number.isInteger(value) && value >= min && value <= max;
  input.setAttribute('aria-invalid', String(!ok));
  return ok ? value : null;
}

function showError(id, message, input) {
  const el = $(`#${id}`);
  el.hidden = !message;
  el.textContent = message || '';
  if (message && !serialPaused) actionLog.append('Dashboard rejected input: ' + message, 'rejected');
  if (message && input) input.focus();
}

$('#drive-form').addEventListener('submit', (event) => {
  event.preventDefault();
  const leftInput = $('#left-speed');
  const rightInput = $('#right-speed');
  const left = readInt(leftInput, 0, 65535);
  const right = readInt(rightInput, 0, 65535);
  if (left === null || right === null) {
    showError('drive-error', 'PWM values must be whole numbers from 0 to 65535.', left === null ? leftInput : rightInput);
    return;
  }
  showError('drive-error', '');
  sendCommand('drive', { left, right });
});

/* IR sensor levels (Buddy 3 tab): white and black raw readings for each
 * sensor.  Wire numbering 0 barcode, 1 left line, 2 right line.  Debug
 * names: raw_center = front left, raw_left = front right, raw_right =
 * barcode (firmware ADC order). */
let barcodeDigital = false;
const IR_LEVEL_SENSORS = { barcode: 0, left: 1, right: 2 };
const irLive = { barcode: null, left: null, right: null };
function showIrRaw(key, raw) {
  if (typeof raw !== 'number') return;
  irLive[key] = raw;
  const row = document.querySelector(`[data-ir-levels="${key}"]`);
  if (row) row.querySelector('[data-ir-live]').textContent = String(raw);
}
function showBarcodeRaw(raw) { showIrRaw('barcode', raw); }
function sendIrLevels(key) {
  if (key === 'barcode' && barcodeDigital) {
    showError('levels-error', 'Barcode uses digital input; software levels are disabled.');
    return;
  }
  const row = document.querySelector(`[data-ir-levels="${key}"]`);
  const whiteInput = row.querySelector('[data-ir-white]');
  const blackInput = row.querySelector('[data-ir-black]');
  const white = readInt(whiteInput, 0, 4095);
  const black = readInt(blackInput, 0, 4095);
  if (white === null || black === null) {
    showError('levels-error', 'Levels must be whole numbers from 0 to 4095.', white === null ? whiteInput : blackInput);
    return;
  }
  if (black - white < 200) {
    showError('levels-error', 'Black must be at least 200 above white.', blackInput);
    return;
  }
  showError('levels-error', '');
  if (sendCommand('barcode_mode', { mode: 1, sensor: IR_LEVEL_SENSORS[key], white, black })) {
    commandResult.textContent = `${key} IR white ${white} / black ${black}`;
  }
}
for (const row of document.querySelectorAll('[data-ir-levels]')) {
  const key = row.dataset.irLevels;
  row.querySelector('[data-ir-apply]').addEventListener('click', () => sendIrLevels(key));
  for (const take of row.querySelectorAll('[data-ir-take]')) {
    take.addEventListener('click', () => {
      if (irLive[key] !== null) row.querySelector(`[data-ir-${take.dataset.irTake}]`).value = irLive[key];
    });
  }
}

/* Motor trim: percent of commanded PWM per motor, 50..100. */
function sendTrim() {
  const leftInput = $('#trim-left');
  const rightInput = $('#trim-right');
  const left = readInt(leftInput, 50, 100);
  const right = readInt(rightInput, 50, 100);
  if (left === null || right === null) {
    showError('trim-error', 'Trim must be a whole number from 50 to 100.', left === null ? leftInput : rightInput);
    return;
  }
  showError('trim-error', '');
  if (sendCommand('motor_trim', { left, right })) {
    commandResult.textContent = `Trim L ${left}% R ${right}%`;
  }
}

/* Line-follow speed: percent of full power for straight driving and curves.
 * Returns false when a box holds an invalid value (nothing is sent then). */
function sendLineSpeed() {
  const baseInput = $('#line-base');
  const turnInput = $('#line-turn');
  const base = readInt(baseInput, 15, 80);
  const turn = readInt(turnInput, 0, 60);
  if (base === null || turn === null) {
    showError('line-error', base === null ? 'Speed must be a whole number from 15 to 80.' : 'Curve power must be a whole number from 0 to 60.', base === null ? baseInput : turnInput);
    return false;
  }
  showError('line-error', '');
  if (sendCommand('line_speed', { base, turn })) {
    commandResult.textContent = `Line speed ${base}% curve ${turn}%`;
  }
  return true;
}

/* Global speed limit: scales every motor command on the robot. Only sent on Apply. */
function sendSpeedLimit() {
  const input = $('#speed-limit');
  const percent = readInt(input, 20, 100);
  if (percent === null) {
    showError('speed-error', 'Speed must be a whole number from 20 to 100.', input);
    return;
  }
  showError('speed-error', '');
  if (sendCommand('speed_limit', { percent })) {
    commandResult.textContent = `Speed limit ${percent}%`;
  }
}

function setServoControlsDisabled(disabled) {
  document.querySelectorAll('[data-servo-control]').forEach((control) => {
    control.disabled = disabled;
  });
}

function sendServoAngle(angle = readInt($('#servo-angle'), 0, 180)) {
  const input = $('#servo-angle');
  if (angle === null) {
    showError('servo-error', 'Servo angle must be a whole number from 0 to 180.', input);
    return;
  }
  input.value = angle;
  showError('servo-error', '');
  if (sendCommand('servo_angle', { angle })) {
    commandResult.textContent = `Servo set to ${angle}°`;
  }
}

/* Groups listed here send as soon as −/+ is pressed; others wait for Apply. */
const STEPPER_SENDERS = { trim: sendTrim, line: sendLineSpeed, servo: () => sendServoAngle() };

/* −/+ buttons: nudge the target input within its min/max, then send at once. */
document.querySelectorAll('[data-target]').forEach((button) => {
  button.addEventListener('click', () => {
    const input = $(`#${button.dataset.target}`);
    const min = Number(input.min);
    const max = Number(input.max);
    const current = Number.isFinite(Number(input.value)) && input.value !== '' ? Number(input.value) : Number(input.defaultValue);
    const next = Math.min(max, Math.max(min, Math.round(current) + Number(button.dataset.step)));
    const group = button.dataset.target.split('-')[0];
    noteEdit(group);
    if (next === Math.round(current)) {
      commandResult.textContent = `Already at the ${next === min ? 'minimum' : 'maximum'} (${next}%)`;
      commandResult.style.color = 'var(--warning)';
      return;
    }
    input.value = next;
    STEPPER_SENDERS[group]?.();
  });
});
['trim-left', 'trim-right', 'line-base', 'line-turn', 'speed-limit', 'servo-angle'].forEach((id) => {
  $(`#${id}`).addEventListener('input', () => noteEdit(id.split('-')[0]));
});
$('#trim-send').addEventListener('click', sendTrim);
$('#speed-form').addEventListener('submit', (event) => {
  event.preventDefault();
  sendSpeedLimit();
});
$('#servo-form').addEventListener('submit', (event) => {
  event.preventDefault();
  sendServoAngle();
});
document.querySelectorAll('[data-servo-angle]').forEach((button) => {
  button.addEventListener('click', () => sendServoAngle(Number(button.dataset.servoAngle)));
});
/* Reset to the firmware defaults (BUDDY2_TRIM_DEFAULT_LEFT/RIGHT). */
$('#trim-reset').addEventListener('click', () => {
  $('#trim-left').value = 83;
  $('#trim-right').value = 100;
  sendTrim();
});

/* Typed line speeds apply on Enter or when the box loses focus (−/+ already
 * send at once). Start sends the boxes first, so the run always uses what is
 * shown, even after a reboot reset the robot to its defaults. */
['line-base', 'line-turn'].forEach((id) => {
  $(`#${id}`).addEventListener('change', sendLineSpeed);
  $(`#${id}`).addEventListener('keydown', (event) => {
    if (event.key === 'Enter') {
      event.preventDefault();
      sendLineSpeed();
    }
  });
});
$('#line-follow-command').addEventListener('click', () => {
  if (sendLineSpeed()) sendCommand('line_follow');
});
$('#line-stop-command').addEventListener('click', () => sendCommand('line_stop'));
$('#imu-reset').addEventListener('click', () => sendCommand('imu_reset'));

/* Evidence cards hide their debug extras (freeze/export/import, sliders,
 * detail boxes, reference strips, reader setup) behind a Tools toggle. */
document.querySelectorAll('#steering-visual, #barcode-visual').forEach((card) => {
  const button = document.createElement('button');
  button.type = 'button';
  button.className = 'btn btn-ghost btn-sm tools-toggle';
  button.setAttribute('aria-pressed', 'false');
  button.textContent = 'Tools';
  button.addEventListener('click', () => {
    const on = card.classList.toggle('show-tools');
    button.setAttribute('aria-pressed', String(on));
    button.textContent = on ? 'Hide tools' : 'Tools';
  });
  card.querySelector('.card-header').append(button);
});

/* One button: rover back to boot defaults (stopped, faults/odometry/IMU
 * cleared) and every dashboard history, log and form field cleared. */
const resetInputs = [...document.querySelectorAll('.col-side input:not([type="file"])')];
$('#reset-all').addEventListener('click', () => {
  if (!window.confirm('Reset everything?\n\nStops the rover, clears encoder faults, odometry and IMU, restores default settings, and clears all histories and logs on this page (export first if you need them).')) return;
  sendCommand('reset_all');
  /* The rover zeroes its encoder counts: restart the unwrapped totals too. */
  leftTicks.reset();
  rightTicks.reset();
  document.querySelectorAll('[data-steering="clear"], [data-ir="clear"], [data-barcode="clear"], #clear-serial')
    .forEach((button) => button.click());
  resetInputs.forEach((input) => { input.value = input.defaultValue; });
});

/* ---------- serial controls ---------- */

$('#clear-serial').addEventListener('click', () => {
  actionLog.clear();
  serialLog.replaceChildren();
  const empty = document.createElement('span');
  empty.className = 'console-empty';
  empty.textContent = 'Cleared. Waiting for new data…';
  serialLog.append(empty);
});

$('#pause-serial').addEventListener('click', (event) => {
  const button = event.currentTarget;
  serialPaused = !serialPaused;
  button.setAttribute('aria-pressed', String(serialPaused));
  button.querySelector('span').textContent = serialPaused ? 'Resume' : 'Pause';
  const icon = button.querySelector('svg, i');
  if (icon && window.lucide) {
    const next = document.createElement('i');
    next.setAttribute('data-lucide', serialPaused ? 'play' : 'pause');
    next.setAttribute('aria-hidden', 'true');
    icon.replaceWith(next);
    window.lucide.createIcons();
  }
});

/* ---------- theme ---------- */

$('#theme-toggle').addEventListener('click', () => {
  const root = document.documentElement;
  const next = root.dataset.theme === 'light' ? 'dark' : 'light';
  root.dataset.theme = next;
  $('meta[name="theme-color"]').setAttribute('content', next === 'light' ? '#fbfbfc' : '#0b0b0c');
  try { localStorage.setItem('buddy1-theme', next); } catch {}
});
if (document.documentElement.dataset.theme === 'light') {
  $('meta[name="theme-color"]').setAttribute('content', '#fbfbfc');
}

/* ---------- tabs ---------- */

/* Built after every listener above, so moved cards keep their handlers. */
const tabs = new window.RoverTabs($('#tabbar'), { onChange: () => {
  window.scrollTo(0, 0);
  irStreamView.scheduleRender();
} });

/* ---------- MQTT ---------- */

setText('broker-host', `${brokerHost}:9001`);
setText('footer-broker', brokerUrl);
setInterval(refreshRobotLink, 1000);
window.lucide?.createIcons();

if (!window.mqtt) {
  setConnection('offline', 'MQTT library failed to load');
} else {
  client = mqtt.connect(brokerUrl, {
    clientId: `buddy1-dashboard-${Math.random().toString(16).slice(2)}`,
    reconnectPeriod: 2000,
    connectTimeout: 5000
  });
  window.roverClient = client;
  setConnection('pending', 'Connecting…');

  client.on('connect', () => {
    setConnection('online', 'Connected');
    client.subscribe([topics.telemetry, topics.heartbeat, topics.debug, topics.status, topics.actions, topics.steering, topics.barcode, topics.irStream]);
  });
  client.on('reconnect', () => setConnection('pending', 'Reconnecting…'));
  client.on('close', () => {
    stopHold(false);
    actionLog.unavailable();
    steeringView.unavailable();
    barcodeView.unavailable();
    irStreamView.unavailable();
    barcodeConfiguration.unavailable();
    setConnection('offline', 'Disconnected');
  });
  client.on('error', () => setConnection('offline', 'Broker error'));
  client.on('message', (topic, raw) => {
    const message = window.RoverWire.decode(raw);
    if (message.kind === 'version_mismatch') {
      wireMismatch = message.data;
      refreshRobotLink();
      return;
    }
    if (message.kind === 'invalid') return;
    if (message.kind !== 'json') wireMismatch = null;
    const data = message.data;
    if (message.kind === 'ir_tune') {
      irThresholdViews.forEach(view => view.handle(data));
      if ((data.sensor ?? 0) === 0) {
        barcodeDigital = !!data.digital;
        const row = document.querySelector('[data-ir-levels="barcode"]');
        row.querySelectorAll('input, button').forEach(control => {
          control.disabled = barcodeDigital;
        });
        $('#barcode-level-mode').textContent = barcodeDigital ?
          'Digital input: analogue calibration disabled.' : '';
      }
      markSeen();
      return;
    }
    if (message.kind === 'telemetry') {
      if (data.uptime_s < lastUptime) { leftTicks.reset(); rightTicks.reset(); }
      lastUptime = data.uptime_s;
      data.left_ticks = leftTicks.update(data.left_ticks_raw);
      data.right_ticks = rightTicks.update(data.right_ticks_raw);
    }
    if (topic === topics.telemetry) showTelemetry(data);
    else if (topic === topics.actions) {
      if (actionLog.handle(data, serialPaused)) markSeen();
    }
    else if (topic === topics.steering) {
      if (steeringView.handle(data)) {
        actionLog.diagnostic(data, serialPaused);
        markSeen();
      }
    }
    else if (topic === topics.barcode) {
      if (barcodeView.handle(data)) {
        barcodeConfiguration.reader(data);
        markSeen();
      }
    }
    else if (topic === topics.irStream) {
      if (irStreamView.handle(data)) {
        markSeen();
        /* 10 ms rows (DIAGNOSTICS=1 builds): faster live value, and the
         * levels the robot is using right now. */
        const row = Array.isArray(data.rows) ? data.rows[data.rows.length - 1] : null;
        if (row) {
          showBarcodeRaw(row[2]);
          setText('levels-robot', `white ${row[8]} / black ${row[9]}`);
        }
      }
    }
    else if (topic === topics.debug) showDebug(data);
    else if (topic === topics.heartbeat) markSeen();
    else if (topic === topics.status) {
      /* Retained Last Will: "offline" when the rover drops, "online" on reconnect. */
      robotOffline = data.state === 'offline';
      if (!robotOffline) lastSeen = Date.now();
      refreshRobotLink();
    }
  });
}
