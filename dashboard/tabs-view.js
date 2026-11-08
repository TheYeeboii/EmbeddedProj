(function (root) {
  'use strict';
  const STORAGE_KEY = 'buddy1-tab';
  const DEFAULT_TAB = 'main';
  const NAV_KEYS = ['ArrowLeft', 'ArrowRight', 'Home', 'End'];

  /* Page tabs. "main" is the original page; every other tab is a panel of
   * empty [data-slot] boxes. Selecting a buddy tab moves the matching
   * [data-widget] element from the main page into its slot, and selecting
   * another tab puts it back. A widget is always the same DOM element, so the
   * code that renders it (by id, or the view object bound to it) feeds it
   * whichever tab shows it. */
  class RoverTabs {
    constructor(bar, options = {}) {
      this.bar = bar;
      this.doc = bar.ownerDocument;
      this.onChange = options.onChange || (() => {});
      this.buttons = [...bar.querySelectorAll('[data-tab]')];
      this.homes = new Map();
      this.current = null;
      for (const [index, button] of this.buttons.entries()) {
        button.addEventListener('click', () => this.select(button.dataset.tab, true));
        button.addEventListener('keydown', (event) => {
          if (!NAV_KEYS.includes(event.key)) return;
          /* Stop here: arrow keys also drive the rover. */
          event.preventDefault();
          event.stopPropagation();
          const last = this.buttons.length - 1;
          const next = event.key === 'Home' ? 0 : event.key === 'End' ? last
            : (index + (event.key === 'ArrowRight' ? 1 : -1) + this.buttons.length) % this.buttons.length;
          this.buttons[next].focus();
          this.select(this.buttons[next].dataset.tab, true);
        });
      }
      this.select(this.load(), false);
    }

    has(name) {
      return this.buttons.some(button => button.dataset.tab === name);
    }

    load() {
      try {
        const saved = root.localStorage.getItem(STORAGE_KEY);
        if (this.has(saved)) return saved;
      } catch {}
      return DEFAULT_TAB;
    }

    save(name) {
      try { root.localStorage.setItem(STORAGE_KEY, name); } catch {}
    }

    select(name, remember) {
      if (!this.has(name) || name === this.current) return;
      this.restore();
      for (const panel of this.doc.querySelectorAll('[data-tab-panel]')) {
        const shown = panel.dataset.tabPanel === name;
        panel.hidden = !shown;
        if (shown && name !== DEFAULT_TAB) this.place(panel);
      }
      for (const button of this.buttons) {
        const selected = button.dataset.tab === name;
        button.setAttribute('aria-selected', String(selected));
        button.tabIndex = selected ? 0 : -1;
      }
      this.doc.body.dataset.tab = name;
      this.current = name;
      if (remember) this.save(name);
      this.onChange(name);
    }

    /* Move each widget named by a slot in this panel into that slot. A comment
     * marks its place on the main page; data-reveal shows a widget the main
     * page keeps hidden. */
    place(panel) {
      for (const slot of panel.querySelectorAll('[data-slot]')) {
        const widget = this.doc.querySelector(`[data-widget="${slot.dataset.slot}"]`);
        if (!widget) continue;
        if (!this.homes.has(widget)) {
          const marker = this.doc.createComment(`tab home: ${slot.dataset.slot}`);
          widget.before(marker);
          this.homes.set(widget, { marker, hidden: widget.hidden });
        }
        slot.append(widget);
        if (slot.hasAttribute('data-reveal')) widget.hidden = false;
      }
    }

    /* Put every moved widget back where the main page had it. */
    restore() {
      for (const [widget, home] of this.homes) {
        home.marker.after(widget);
        widget.hidden = home.hidden;
      }
    }
  }

  root.RoverTabs = RoverTabs;
  if (typeof module !== 'undefined' && module.exports) module.exports = RoverTabs;
})(globalThis);
