const topics = {
  telemetry: 'rover/telemetry',
  heartbeat: 'rover/heartbeat',
  commands: 'rover/commands',
  debug: 'rover/debug'
};

const brokerHost = new URLSearchParams(window.location.search).get('broker') || '192.168.137.1';

const connection = document.querySelector('#connection');
const commandResult = document.querySelector('#command-result');
let serialPaused = false;

function setText(id, value) {
  document.querySelector(`#${id}`).textContent = value;
}

function setConnection(online, text) {
  connection.classList.toggle('online', online);
  connection.classList.toggle('offline', !online);
  connection.lastChild.textContent = text;
}

function showTelemetry(data) {
  setText('state', data.state || data.status || 'unknown');
  setText('robot', data.robot_id || 'buddy1');
  setText('distance', data.distance_cm ?? '--');
  setText('heading', data.heading_deg ?? '--');
  setText('left-ir', data.left_ir ?? '--');
  setText('center-ir', data.center_ir ?? '--');
  setText('right-ir', data.right_ir ?? '--');
  setText('ticks', `${data.left_ticks ?? '--'} / ${data.right_ticks ?? '--'}`);
  setText('motors', `${data.motor_left ?? '--'} / ${data.motor_right ?? '--'}`);
  setText('uptime', data.ts_ms ?? '--');
  setText('updated', `Updated ${new Date().toLocaleTimeString()}`);
}

function showDebug(data) {
  if (serialPaused) return;
  const log = document.querySelector('#serial-log');
  const line = `${new Date().toLocaleTimeString()}  Buddy3: raw=${data.raw_left},${data.raw_center},${data.raw_right} norm=${data.norm_left},${data.norm_center},${data.norm_right} position=${data.position} line=${data.line ? 'yes' : 'no'} junction=${data.junction ? 'yes' : 'no'} barcode=${data.barcode ? 'yes' : 'no'} cmd=${data.command}`;
  const lines = log.textContent === 'Waiting for Pico W sensing data...' ? [] : log.textContent.split('\n');
  lines.push(line);
  log.textContent = lines.slice(-80).join('\n');
  log.scrollTop = log.scrollHeight;
}

function sendCommand(command, values = {}) {
  if (!window.roverClient || !window.roverClient.connected) {
    commandResult.textContent = 'Broker offline';
    return;
  }
  window.roverClient.publish(topics.commands, JSON.stringify({ cmd: command, ...values }), { qos: 0 });
  commandResult.textContent = `Sent ${command}`;
}

const client = mqtt.connect(`ws://${brokerHost}:9001`, {
  clientId: `buddy1-dashboard-${Math.random().toString(16).slice(2)}`,
  reconnectPeriod: 2000,
  connectTimeout: 5000
});
window.roverClient = client;

client.on('connect', () => {
  setConnection(true, 'Connected');
  client.subscribe([topics.telemetry, topics.heartbeat, topics.debug]);
});
client.on('reconnect', () => setConnection(false, 'Reconnecting'));
client.on('close', () => setConnection(false, 'Disconnected'));
client.on('error', () => setConnection(false, 'Broker error'));
client.on('message', (topic, raw) => {
  let data;
  try { data = JSON.parse(raw.toString()); } catch { return; }
  if (topic === topics.telemetry) showTelemetry(data);
  if (topic === topics.heartbeat) {
    setText('heartbeat', new Date().toLocaleTimeString());
    setText('robot', data.robot_id || 'buddy1');
  }
  if (topic === topics.debug) showDebug(data);
});

document.querySelectorAll('[data-command]').forEach((button) => {
  button.addEventListener('click', () => sendCommand(button.dataset.command));
});
document.querySelector('#drive-command').addEventListener('click', () => {
  const left = Number(document.querySelector('#left-speed').value);
  const right = Number(document.querySelector('#right-speed').value);
  if (Number.isInteger(left) && Number.isInteger(right) && left >= 0 && right >= 0 && left <= 65535 && right <= 65535) {
    sendCommand('drive', { left, right });
  }
});

document.querySelectorAll('[data-motion]').forEach((button) => {
  button.addEventListener('click', () => {
    const command = button.dataset.motion;
    const field = command.startsWith('move_') ? '#motion-distance' : '#motion-angle';
    const key = command.startsWith('move_') ? 'distance_mm' : 'angle_deg';
    const value = Number(document.querySelector(field).value);
    if (Number.isInteger(value) && value > 0) {
      sendCommand(command, { [key]: value, max_pwm: 20000 });
    }
  });
});

document.querySelector('#line-follow-command').addEventListener('click', () => sendCommand('line_follow'));
document.querySelector('#line-stop-command').addEventListener('click', () => sendCommand('line_stop'));
document.querySelector('#clear-serial').addEventListener('click', () => {
  document.querySelector('#serial-log').textContent = 'Serial monitor cleared.';
});
document.querySelector('#pause-serial').addEventListener('click', (event) => {
  serialPaused = !serialPaused;
  event.currentTarget.textContent = serialPaused ? 'Resume' : 'Pause';
});
