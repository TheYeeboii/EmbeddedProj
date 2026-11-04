(function(root) {
  'use strict';
  class BarcodeConfiguration {
    constructor(form, options) {
      this.form = form; this.send = options.send;
      this.clock = options.clock || Date.now;
      this.field = name => form.querySelector(`[data-barcode-config="${name}"]`);
      this.mode = null; this.seen = null; this.pending = null; this.enabled = null; this.config = null; this.boot = null;
      form.addEventListener('submit', event => { event.preventDefault(); this.submit(); });
      this.timer = (options.schedule || setInterval)(() => this.refresh(), 1000);
      this.refresh();
    }
    robot(mode) {
      const wasIdle = this.isIdle();
      this.mode = mode; this.seen = this.clock();
      if (mode === 'idle' && !wasIdle && !this.pending)
        this.field('status').textContent = 'Rover stopped. Enter the sensor measurement and apply the reader settings.';
      this.refresh();
    }
    unavailable() {
      this.seen = null; this.pending = null;
      this.field('status').textContent = 'Rover offline or stale. Reconnect before applying settings.';
      this.refresh();
    }
    isIdle() { return this.seen !== null && this.clock() - this.seen < 3500 && this.mode === 'idle'; }
    refresh() {
      if (this.pending && this.clock() - this.pending.at >= 5000) {
        this.pending = null;
        this.field('status').textContent = 'No configuration confirmation. Keep the rover stopped; check the USB configuration response and encoder calibration.';
      }
      this.field('apply').disabled = !this.isIdle() || !!this.pending;
      if (!this.pending && this.seen !== null && this.mode !== 'idle')
        this.field('status').textContent = 'Stop the rover with GP21 or Stop before configuring the reader.';
      if (this.seen !== null && this.clock() - this.seen >= 3500)
        this.field('status').textContent = 'Rover telemetry is stale. Waiting for a fresh stopped state.';
    }
    submit() {
      if (!this.isIdle() || this.pending) { this.refresh(); return false; }
      const minInput = this.field('minimum'), offsetInput = this.field('offset');
      const min = Number(minInput.value), offset = Number(offsetInput.value);
      if (!minInput.value.trim() || !Number.isInteger(min) || min < 1 || min > 1000) {
        this.field('status').textContent = 'Enter a whole-number minimum element size between 1 and 1000 mm.';
        minInput.focus(); return false;
      }
      if (!offsetInput.value.trim() || !Number.isInteger(offset) || offset < -500 || offset > 500) {
        this.field('status').textContent = 'Enter the measured sensor offset in mm: left positive, right negative (−500 to +500).';
        offsetInput.focus(); return false;
      }
      if (!this.send('barcode_config', { min_element_mm: min, offset_left_mm: offset })) {
        this.field('status').textContent = 'Configuration was not sent. Check the broker connection.';
        return false;
      }
      this.send('barcode_mode', { mode: 1 }); // Observe only; never starts motors.
      this.pending = { min, offset, at: this.clock() };
      this.field('status').textContent = 'Settings sent. Waiting for reader confirmation…';
      this.refresh(); return true;
    }
    reader(data) {
      if (!data || data.type !== 'barcode_state' || data.lag_ms > 500) return;
      if (this.boot !== null && this.boot !== data.boot) {
        this.pending = null;
        this.field('status').textContent = 'Rover restarted. Reader settings are RAM-only; apply them again if disabled.';
      }
      this.boot = data.boot;
      this.enabled = data.state !== 0;
      this.config = Array.isArray(data.config) && data.config.length === 11 ? data.config : null;
      this.field('active').textContent = !this.enabled ? 'Reader disabled — apply settings below.' : this.config ?
        `Active minimum ${this.config[5] / 100} mm · sensor offset ${this.config[6] / 10} mm (left +)` :
        'Reader enabled; this firmware does not report its active settings.';
      if (this.pending && this.enabled) {
        if (!this.config) {
          this.field('status').textContent = 'Reader reports enabled. Press GP21 to test; rebuild current firmware for confirmation of the active values.';
          this.pending = null;
        } else if (this.config[5] === this.pending.min * 100 && this.config[6] === this.pending.offset * 10) {
          this.pending = null;
          this.field('status').textContent = 'Reader settings confirmed. Press GP21 and drive slowly across the barcode.';
        }
      }
      this.refresh();
    }
  }
  root.RoverBarcodeConfiguration = BarcodeConfiguration;
  if (typeof module !== 'undefined' && module.exports) module.exports = BarcodeConfiguration;
})(globalThis);
