(function (root) {
  'use strict';
  const LABELS = Object.freeze({
    turn_left: 'Trying to turn left',
    turn_right: 'Trying to turn right',
    stop_turn_left: 'Stopped turning left',
    stop_turn_right: 'Stopped turning right'
  });
  const SOURCES = Object.freeze({
    pwm: 'steering/manual command',
    motion: 'distance/turn move',
    stop: 'motor stop'
  });
  const validCount = n => Number.isInteger(n) && n >= 0 && n <= 0xffffffff;
  class RoverActionLog {
    constructor(log, badge, options = {}) {
      this.log = log;
      this.badge = badge;
      this.limit = options.limit || 100;
      this.clock = options.clock || (() => new Date().toLocaleTimeString());
      this.lastSequence = null;
      this.lastDropped = 0;
    }
    unavailable() {
      this.badge.textContent = 'Command state unavailable';
      this.badge.dataset.tone = 'muted';
    }
    clear() {
      this.log.replaceChildren();
      const empty = this.log.ownerDocument.createElement('span');
      empty.className = 'console-empty';
      empty.textContent = 'Waiting for rover/actions…';
      this.log.append(empty);
    }
    append(text, action = '') {
      const follow = this.log.scrollHeight - this.log.scrollTop - this.log.clientHeight < 24;
      this.log.querySelector('.console-empty')?.remove();
      const line = this.log.ownerDocument.createElement('div');
      line.className = 'console-action';
      line.dataset.action = action;
      line.textContent = text; // MQTT content must never be inserted as HTML.
      this.log.append(line);
      while (this.log.childElementCount > this.limit) this.log.firstElementChild.remove();
      if (follow) this.log.scrollTop = this.log.scrollHeight;
    }
    handle(data, paused = false) {
      if (!data || (data.type !== 'action' && data.type !== 'action_state') ||
          !validCount(data.seq) || !validCount(data.ts_ms) ||
          !validCount(data.dropped) || !['none', 'left', 'right'].includes(data.direction))
        return false;
      if (data.type === 'action' && !Object.hasOwn(LABELS, data.action)) return false;

      if (this.lastSequence !== null && data.seq < this.lastSequence) {
        // A restarted firmware begins a new sequence.
        this.lastSequence = null;
        this.lastDropped = 0;
      }
      const gap = this.lastSequence === null ? 0 : Math.max(0, data.seq - this.lastSequence - 1);
      const dropped = Math.max(0, data.dropped - this.lastDropped);
      this.lastSequence = data.seq;
      this.lastDropped = data.dropped;
      this.badge.textContent = data.direction === 'left' ? 'Left turn commanded' :
        data.direction === 'right' ? 'Right turn commanded' : 'No turn commanded';
      this.badge.dataset.tone = data.direction === 'none' ? 'muted' : 'accent';
      if (paused) return true; // Pause history only; the current badge stays live.

      if (gap || dropped) {
        this.append(this.clock() + '  Some action events were unavailable' +
          ' (sequence gap ' + gap + ', firmware buffer drops +' + dropped + ').', 'gap');
      }
      if (data.type === 'action') {
        const uptime = (data.ts_ms / 1000).toFixed(2) + 's';
        const source = SOURCES[data.source] || 'command';
        const pwm = Number.isInteger(data.left_pwm) && Number.isInteger(data.right_pwm) ?
          '; PWM L=' + data.left_pwm + ' R=' + data.right_pwm : '';
        this.append(this.clock() + '  [rover ' + uptime + '] ' + LABELS[data.action] +
          ' (' + source + pwm + ')', data.action);
      }
      return true;
    }
  }
  root.RoverActionLog = RoverActionLog;
  if (typeof module !== 'undefined' && module.exports) module.exports = RoverActionLog;
})(globalThis);
