(function (root) {
  'use strict';
  const owners = ['Stopped', 'Manual', 'Tracking', 'Junction', 'Search', 'Avoidance', 'Motion'];
  const phases = ['Straight', 'Micro correction', 'Corner', 'Seeking', 'Aligning', 'Both black', 'Stopped', 'Crossing', 'Legacy', 'Dark surface'];
  const integer = (x, lo, hi) => Number.isInteger(x) && x >= lo && x <= hi;
  const u32 = x => integer(x, 0, 0xffffffff);
  const array = (x, size, lo, hi) => Array.isArray(x) && x.length === size && x.every(n => integer(n, lo, hi));
  function valid(d) {
    return d && d.v === 1 && ['steering', 'steering_state'].includes(d.type) &&
      ['boot', 'seq', 'ts_ms', 'lag_ms', 'dropped', 'encoder_ms', 'sample_ms'].every(k => u32(d[k])) &&
      integer(d.owner, 0, 6) && integer(d.phase, 0, 9) && integer(d.direction, 0, 2) &&
      integer(d.faults, 0, 3) && integer(d.has_input, 0, 1) &&
      array(d.requested, 2, -32767, 32767) && array(d.applied, 2, -32767, 32767) &&
      array(d.distance, 2, -2147483647, 2147483647) && array(d.speed, 2, -32767, 32767) &&
      array(d.ir, 10, 0, 4095) && d.ir.slice(2, 5).every(n => n <= 1000) && d.ir[9] <= 3;
  }
  class SteeringHistory {
    constructor(limit = 600) {
      this.limit = limit; this.records = []; this.live = null; this.retired = [];
      this.last = null; this.pendingGap = false; this.evicted = 0; this.receivedAt = 0;
    }
    unavailable() { this.live = null; this.pendingGap = true; }
    accept(input, now = Date.now()) {
      if (!valid(input)) return false;
      const d = JSON.parse(JSON.stringify(input));
      let gap = this.pendingGap;
      if (this.last && this.last.boot !== d.boot) {
        if (this.retired.includes(d.boot)) return false;
        this.retired.push(this.last.boot); this.retired = this.retired.slice(-8); gap = true;
      } else if (this.last) {
        const advance = (d.seq - this.last.seq) >>> 0;
        if (advance >= 0x80000000 || (!advance && d.type !== 'steering_state')) return false;
        if (advance > 1 || d.dropped !== this.last.dropped) gap = true;
        if (!advance) {
          this.receivedAt = now;
          this.live = d.lag_ms > 500 ? null : d;
          /* A reconnect snapshot may restore state, but cannot bridge offline time. */
          if (gap) this.records.push({ data: d, gap: true });
          this.pendingGap = false; this.trim(); return true;
        }
      }
      if (!this.last && d.dropped) gap = true;
      this.records.push({ data: d, gap }); this.trim();
      this.last = d; this.receivedAt = now; this.pendingGap = false;
      this.live = d.lag_ms > 500 ? null : d;
      return true;
    }
    trim() {
      while (this.records.length > this.limit) { this.records.shift(); ++this.evicted; }
    }
    export(records = this.records) {
      return JSON.stringify({ format: 'rover-steering-v1', evicted: this.evicted, records }, null, 2);
    }
    static read(text) {
      if (typeof text !== 'string' || text.length > 2000000) throw new Error('Session exceeds 2 MB');
      const file = JSON.parse(text);
      if (file.format !== 'rover-steering-v1' || !Array.isArray(file.records) ||
          file.records.length > 600 || !u32(file.evicted) ||
          !file.records.every(r => r && typeof r.gap === 'boolean' && valid(r.data)))
        throw new Error('Invalid steering session');
      const replay = new SteeringHistory();
      for (const record of file.records) {
        if (record.gap) replay.unavailable();
        if (!replay.accept(record.data)) throw new Error('Session is duplicated or out of order');
      }
      return JSON.parse(JSON.stringify(file.records));
    }
  }
  class SteeringView {
    constructor(element) {
      this.element = element; this.history = new SteeringHistory(); this.frozen = null; this.selected = null;
      this.find = name => element.querySelector(`[data-steering="${name}"]`);
      this.find('pause').addEventListener('click', () => this.pause());
      this.find('clear').addEventListener('click', () => {
        this.history.records = []; this.history.evicted = 0; this.frozen = null; this.selected = null; this.render();
      });
      this.find('cursor').addEventListener('input', event => {
        if (!this.frozen) this.frozen = this.history.records.slice();
        this.selected = Number(event.target.value); this.render();
      });
      this.find('export').addEventListener('click', () => {
        const blob = new Blob([this.history.export(this.frozen || this.history.records)], { type: 'application/json' });
        const url = URL.createObjectURL(blob), a = document.createElement('a');
        a.href = url; a.download = 'rover-steering.json'; a.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
      });
      this.find('import').addEventListener('change', async event => {
        try {
          const file = event.target.files[0]; if (!file) return;
          if (file.size > 2000000) throw new Error('Session exceeds 2 MB');
          this.frozen = SteeringHistory.read(await file.text()); this.selected = null; this.render();
          this.find('notice').textContent = 'Imported replay — live indicators still show the connected rover.';
        } catch (error) { this.find('notice').textContent = error.message; }
        event.target.value = '';
      });
      this.timer = setInterval(() => {
        if (Date.now() - this.history.receivedAt > 2500 && this.history.live) this.unavailable();
      }, 1000);
      this.render();
    }
    pause() {
      this.frozen = this.frozen ? null : this.history.records.slice(); this.selected = null; this.render();
    }
    unavailable() { this.history.unavailable(); this.renderLive(); }
    handle(d) {
      if (!this.history.accept(d)) return false;
      this.renderLive(); if (!this.frozen) this.render(); return true;
    }
    renderLive() {
      const d = this.history.live;
      const manual = !!d && d.owner === 1;
      this.element.dataset.manual = String(manual);
      this.element.querySelectorAll('.steering-arrow').forEach(arrow => { arrow.hidden = manual; });
      this.find('hint').textContent = manual ?
        'Steering arrows are collapsed during manual control. History continues recording.' :
        'Arrows show commands. Encoder measurements are separate evidence.';
      this.element.dataset.direction = d ? ['straight', 'left', 'right'][d.direction] : 'unknown';
      this.element.dataset.stopped = String(!!d && d.owner === 0);
      this.find('live').textContent = manual ? 'Manual control · use the D-pad' : d ? `${owners[d.owner]} · ${phases[d.phase]} · ${d.owner === 0 ? 'stopped' : ['straight', 'left commanded', 'right commanded'][d.direction]}` : 'State unavailable / catching up';
    }
    render() {
      this.renderLive();
      const records = this.frozen || this.history.records;
      const cursor = this.find('cursor'); cursor.max = String(Math.max(0, records.length - 1));
      cursor.disabled = !records.length;
      const selected = this.selected === null ? records.length - 1 : Math.min(this.selected, records.length - 1);
      cursor.value = String(Math.max(0, selected));
      this.find('pause').textContent = this.frozen ? 'Resume live history' : 'Freeze history';
      this.find('pause').setAttribute('aria-pressed', String(!!this.frozen));
      this.find('notice').textContent = `${records.length}/600 records · ${this.history.evicted} evicted · ${this.frozen ? 'Frozen; rover is still running' : 'Live history'} · shaded breaks mean unavailable data`;
      const selectedRecord = records[selected];
      if (!selectedRecord) {
        this.find('detail').textContent = 'Waiting for rover/steering. No new firmware? Existing action logs remain available.';
        this.find('timeline').replaceChildren(); return;
      }
      const d = selectedRecord.data, ir = d.ir;
      this.find('detail').textContent = `Boot ${d.boot} · event ${d.seq} · rover ${(d.ts_ms / 1000).toFixed(3)} s${selectedRecord.gap ? ' · GAP before this record' : ''}\n` +
        `${owners[d.owner]} / ${phases[d.phase]} · requested PWM L ${d.requested[0]} / R ${d.requested[1]} · applied L ${d.applied[0]} / R ${d.applied[1]}\n` +
        (d.has_input ? `Decision input at ${d.sample_ms} ms: front L raw ${ir[0]} / norm ${ir[2]}, front R raw ${ir[1]} / norm ${ir[3]}; black threshold ${ir[4]} (no hysteresis on front sensors).\nCalibration L ${ir[5]}–${ir[6]}, R ${ir[7]}–${ir[8]}, dark flags ${ir[9]}. ADC mapping: front L=1, R=0.\n` : 'No associated IR decision (manual/motion/stop command).\n') +
        `Encoder observation ${d.encoder_ms} ms: travel L ${d.distance[0] / 10} / R ${d.distance[1] / 10} mm; speed L ${d.speed[0] / 10} / R ${d.speed[1] / 10} mm/s; fault flags ${d.faults}; drops ${d.dropped}; queue lag ${d.lag_ms} ms.\nCommanded steering is not measured heading.`;
      this.draw(records, selected);
    }
    draw(records, selected) {
      const svg = this.find('timeline'); svg.replaceChildren();
      const boot = records[selected].data.boot;
      const series = records.map((r, i) => ({ ...r, index: i })).filter(r => r.data.boot === boot);
      const start = series[0].data.ts_ms;
      const span = Math.max(1, (series[series.length - 1].data.ts_ms - start) >>> 0);
      const ns = 'http://www.w3.org/2000/svg';
      const add = (tag, attrs, text) => {
        const node = document.createElementNS(ns, tag);
        for (const [key, value] of Object.entries(attrs)) node.setAttribute(key, String(value));
        if (text !== undefined) node.textContent = text;
        svg.append(node); return node;
      };
      this.find('axis').textContent = `Boot ${boot} · ${span} ms shown · green: left · blue: right · grey: straight/stopped · orange: unknown interval`;
      series.forEach((r, i) => {
        const x = 5 + (((r.data.ts_ms - start) >>> 0) / span) * 890;
        const next = series[i + 1];
        const duration = next ? (next.data.ts_ms - r.data.ts_ms) >>> 0 : 0;
        const width = Math.max(1, duration / span * 890);
        const color = next?.gap ? 'trace-gap' : ['trace-straight', 'trace-left', 'trace-right'][r.data.direction];
        const rect = add('rect', { x, y: 20, width, height: 22, class: `${color}${r.index === selected ? ' trace-selected' : ''}` });
        const title = document.createElementNS(ns, 'title');
        title.textContent = `${r.data.ts_ms} ms · ${duration ? `${duration} ms` : 'open state; duration unknown'} · ${next?.gap ? 'unavailable interval' : ['straight', 'left', 'right'][r.data.direction]}`;
        rect.append(title);
        rect.addEventListener('click', () => { if (!this.frozen) this.frozen = records.slice(); this.selected = r.index; this.render(); });
        if (r.data.has_input) {
          add('circle', { cx: x, cy: 58 - r.data.ir[2] * 0.012, r: 2, class: 'trace-ir-left' });
          add('circle', { cx: x, cy: 78 - r.data.ir[3] * 0.012, r: 2, class: 'trace-ir-right' });
        }
      });
      // Labels stay in HTML so they remain legible on narrow displays.
    }
  }
  root.RoverSteeringView = SteeringView;
  if (typeof module !== 'undefined' && module.exports) module.exports = { SteeringHistory, valid, owners, phases };
})(globalThis);
