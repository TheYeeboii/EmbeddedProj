(function (root) {
  'use strict';
  const ACTIONS = new Set(['turn_left', 'turn_right', 'stop_turn_left', 'stop_turn_right']);
  const OWNERS = ['Stopped', 'Manual', 'Tracking', 'Junction', 'Search', 'Avoidance', 'Motion'];
  const validCount = n => Number.isInteger(n) && n >= 0 && n <= 0xffffffff;
  class RoverActionLog {
    constructor(log, badge, options = {}) {
      this.log = log;
      this.badge = badge;
      this.limit = options.limit || 100;
      this.clock = options.clock || (() => new Date().toLocaleTimeString());
      this.lastSequence = null;
      this.lastDropped = 0;
      this.lastDiagnostic = null;
    }
    unavailable() {
      this.badge.textContent = 'Diagnostics unavailable';
      this.badge.dataset.tone = 'muted';
    }
    clear() {
      this.log.replaceChildren();
      const empty = this.log.ownerDocument.createElement('span');
      empty.className = 'console-empty';
      empty.textContent = 'No diagnostic events yet.';
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
    diagnostic(data, paused = false) {
      if (!data || !['steering', 'steering_state'].includes(data.type) ||
          !validCount(data.boot) || !validCount(data.seq) || !validCount(data.ts_ms) ||
          !Number.isInteger(data.owner) || data.owner < 0 || data.owner >= OWNERS.length ||
          !Number.isInteger(data.faults) || data.faults < 0 || data.faults > 3) return false;
      const previous = this.lastDiagnostic;
      const sameBoot = previous && previous.boot === data.boot;
      if (sameBoot) {
        const advance = (data.seq - previous.seq) >>> 0;
        if (advance >= 0x80000000) return false;
      }
      this.lastDiagnostic = { boot: data.boot, seq: data.seq, owner: data.owner, faults: data.faults };
      this.badge.textContent = data.lag_ms > 500 ? 'Buffered diagnostics' : 'Control: ' + OWNERS[data.owner];
      this.badge.dataset.tone = data.faults ? 'warning' : 'muted';
      if (sameBoot && previous.seq === data.seq) return true; // Restore badge without duplicate rows.
      if (paused) return true;
      const prefix = this.clock() + '  [rover ' + (data.ts_ms / 1000).toFixed(2) + 's] ';
      if (!sameBoot || previous.owner !== data.owner) {
        const stop = sameBoot && data.owner === 0;
        const message = stop ? 'Stop commanded' + (data.phase === 9 ? ': all sensors black' : '') :
          (previous && !sameBoot ? 'Rover restarted; ' : '') + 'Control owner: ' +
          (sameBoot ? OWNERS[previous.owner] + ' → ' : '') + OWNERS[data.owner];
        this.append(prefix + message, stop ? 'stop' : 'owner');
      }
      const oldFaults = sameBoot ? previous.faults : 0;
      if (data.faults !== oldFaults) {
        const wheels = [data.faults & 1 ? 'left' : '', data.faults & 2 ? 'right' : ''].filter(Boolean).join(' and ');
        this.append(prefix + (data.faults ? 'Encoder fault: ' + wheels : 'Encoder feedback restored'), 'fault');
      }
      return true;
    }
    handle(data, paused = false) {
      if (!data || (data.type !== 'action' && data.type !== 'action_state') ||
          !validCount(data.seq) || !validCount(data.ts_ms) ||
          !validCount(data.dropped) || !['none', 'left', 'right'].includes(data.direction))
        return false;
      if (data.type === 'action' && !ACTIONS.has(data.action)) return false;

      if (this.lastSequence !== null && data.seq < this.lastSequence) {
        // A restarted firmware begins a new sequence.
        this.lastSequence = null;
        this.lastDropped = 0;
      }
      const gap = this.lastSequence === null ? 0 : Math.max(0, data.seq - this.lastSequence - 1);
      const dropped = Math.max(0, data.dropped - this.lastDropped);
      this.lastSequence = data.seq;
      this.lastDropped = data.dropped;
      if (!this.lastDiagnostic) {
        this.badge.textContent = 'Action stream connected';
        this.badge.dataset.tone = 'muted';
      }
      if (paused) return true; // Pause history only; the current badge stays live.

      if (gap || dropped) {
        this.append(this.clock() + '  Some action events were unavailable' +
          ' (sequence gap ' + gap + ', firmware buffer drops +' + dropped + ').', 'gap');
      }
      // Routine turn transitions stay in the visual timeline, not this log.
      return true;
    }
  }
  root.RoverActionLog = RoverActionLog;
  if (typeof module !== 'undefined' && module.exports) module.exports = RoverActionLog;
})(globalThis);
