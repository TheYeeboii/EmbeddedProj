(function (root) {
  'use strict';
  class IRThresholdView {
    constructor(element, send, sensor = 0) {
      this.element = element;
      this.send = send;
      this.sensor = sensor;
      this.applied = null;
      this.pending = null;
      this.seen = 0;
      this.find = name => element.querySelector('[data-threshold="' + name + '"]');
      for (const name of ['low', 'high']) {
        for (const kind of ['range', 'number']) {
          this.find(name + '-' + kind).addEventListener('input', event => {
            this.find(name + (kind === 'range' ? '-number' : '-range')).value = event.target.value;
            this.paint();
          });
        }
      }
      this.find('invert').addEventListener('change', () => this.paint());
      element.addEventListener('submit', event => {
        event.preventDefault();
        this.apply(false);
      });
      this.find('legacy').addEventListener('click', () => this.apply(true));
      this.timer = setInterval(() => this.refresh(), 250);
      this.paint();
    }
    refresh() {
      const age = Math.max(0, Date.now() - this.seen);
      const fresh = !!this.applied && age < 1500;
      if (this.pending && Date.now() - this.pending.at > 4000) {
        this.pending = null;
        this.find('status').textContent = 'No configuration confirmation received. Check the connection and apply again.';
      }
      this.find('apply').disabled = !fresh || !!this.pending || !!this.applied?.digital;
      this.find('legacy').disabled = !fresh || !!this.pending || !!this.applied?.digital;
      this.element.dataset.stale = String(!fresh);
      if (!this.applied) {
        this.find('live').textContent = 'Waiting for sensor reports';
        this.find('status').textContent = 'No sensor report received yet. Check the connection and firmware.';
      } else if (!fresh) {
        const label = this.applied.digital ?
          'Digital ' + (this.applied.raw ? 'HIGH (1)' : 'LOW (0)') :
          'Raw ' + this.applied.raw;
        this.find('live').textContent = 'Delayed · last ' + label + ' · ' +
          (this.applied.black ? 'BLACK' : 'WHITE');
        this.find('status').textContent = 'Last report is ' + (age / 1000).toFixed(1) +
          ' s old. Keeping the last reading; Apply waits for fresh data.';
      }
    }
    values() {
      return {
        low: Number(this.find('low-number').value),
        high: Number(this.find('high-number').value),
        inverted: this.find('invert').checked
      };
    }
    paint() {
      const values = this.values();
      if (this.applied && !this.pending) {
        this.find('status').textContent = 'Draft edited. Apply to update the rover.';
      }
      const valid = Number.isInteger(values.low) && Number.isInteger(values.high) &&
        values.low >= 0 && values.low < values.high && values.high <= 4095;
      if (valid) {
        const low = values.low / 4095 * 100;
        const high = values.high / 4095 * 100;
        const left = values.inverted ? '#161616' : '#fff';
        const right = values.inverted ? '#fff' : '#161616';
        this.find('strip').style.background =
          'linear-gradient(to right,' + left + ' 0%,' + left + ' ' + low +
          '%,#929292 ' + low + '%,#929292 ' + high + '%,' + right + ' ' +
          high + '%,' + right + ' 100%)';
      }
      this.find('legend').textContent = valid ?
        'Draft · ' + (values.inverted ? 'Black' : 'White') + ' ≤ ' + values.low +
        ' · Keep previous colour between stops · ' +
        (values.inverted ? 'White' : 'Black') + ' ≥ ' + values.high :
        'Enter whole numbers from 0 to 4095. The lower stop must be below the upper stop.';
    }
    apply(legacy) {
      if (this.applied?.digital) return;
      if (!this.applied || Date.now() - this.seen >= 1500 || this.pending) return;
      const values = this.values();
      const fields = ['low-number', 'high-number'];
      if (!legacy && fields.some(name => !this.find(name).value.trim() ||
          !this.find(name).checkValidity())) return;
      if (!legacy && values.low >= values.high) return;
      const request = legacy ? {low: 0, high: 0, inverted: false} : values;
      if (!this.send('ir_thresholds', {
        sensor: this.sensor, inverted: request.inverted,
        white: request.low, black: request.high
      })) {
        this.find('status').textContent = 'Settings were not sent. Check the broker connection.';
        return;
      }
      this.pending = {...request, at: Date.now()};
      this.find('status').textContent = 'Waiting for the rover to apply the settings…';
    }
    handle(data) {
      if ((data.sensor ?? 0) !== this.sensor) return;
      const wasStale = Date.now() - this.seen >= 1500;
      this.seen = Date.now() - Math.max(0, data.lag_ms ?? 0);
      this.applied = data;
      const digital = !!data.digital;
      const speed = this.find('speed');
      if (speed && typeof data.slowdown === 'boolean') {
        speed.hidden = false;
        speed.textContent = data.slowdown ?
          'Reported speed · barcode slowdown ' + data.effective_speed +
            '% · restores after more than 3 seconds of white' :
          'Reported speed · normal setting ' + data.effective_speed + '%';
      }
      this.find('hint').textContent = digital ?
        'Digital DO reports LOW or HIGH. Sensitivity is set on the sensor module.' :
        'Raw ADC · drag either stop or type a value. Grey retains the previous colour. Apply sends the draft to the rover.';
      for (const name of ['low-range', 'high-range', 'low-number', 'high-number', 'invert']) {
        this.find(name).disabled = digital;
      }
      for (const selector of ['.threshold-strip', '.threshold-handles', '.threshold-fields']) {
        this.element.querySelector(selector).hidden = digital;
      }
      this.find('legend').hidden = digital;
      if (digital) {
        this.pending = null;
        this.find('live').textContent = 'Digital ' + (data.raw ? 'HIGH (1)' : 'LOW (0)') +
          ' · ' + (data.black ? 'BLACK' : 'WHITE');
        this.find('active').textContent = 'Digital input · GP' + data.pin +
          ' · black = ' + (data.black_level ? 'HIGH' : 'LOW');
        this.find('status').textContent = 'Software thresholds disabled. Adjust sensitivity on the sensor module.';
        this.find('apply').disabled = true;
        this.find('legacy').disabled = true;
        this.refresh();
        return;
      }
      this.find('marker').style.left = data.raw / 4095 * 100 + '%';
      this.find('live').textContent = 'Raw ' + data.raw + ' · ' + (data.black ? 'BLACK' : 'WHITE');
      this.find('active').textContent = data.enabled ?
        'Reported settings · raw hysteresis ' + data.low + '–' + data.high +
        ' · black at ' + (data.inverted ? 'low' : 'high') + ' readings' :
        (this.sensor === 0 ? 'Reported settings · existing peak/valley classifier' :
          'Reported settings · existing normalized threshold');
      if (wasStale && !this.pending) {
        this.find('status').textContent = 'Rover connected. Edit and apply thresholds to update this sensor.';
      }
      const matches = Date.now() - this.seen < 1500 && this.pending &&
        data.enabled === (this.pending.high !== 0) &&
        (!data.enabled || (data.low === this.pending.low && data.high === this.pending.high &&
          data.inverted === this.pending.inverted));
      if (matches) {
        this.pending = null;
        const draft = this.values();
        const sameDraft = !data.enabled || (draft.low === data.low &&
          draft.high === data.high && draft.inverted === data.inverted);
        this.find('status').textContent = sameDraft ?
          'Confirmed by the rover. Settings reset on reboot.' :
          'Rover settings confirmed. Your edited draft has not been applied.';
      }
      this.refresh();
    }
  }
  root.RoverIRThresholdView = IRThresholdView;
})(globalThis);
