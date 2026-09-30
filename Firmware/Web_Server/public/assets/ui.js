// -----------------------------------------------------------------------------
// Small DOM toolkit: element builder, icons, toasts, confirmation dialog and
// number formatting. No framework; views build DOM directly.
// -----------------------------------------------------------------------------
import { UNITS, toDisplay } from './protocol.js';

export function h(tag, attrs = {}, ...children) {
  const el = document.createElement(tag);
  for (const [key, value] of Object.entries(attrs ?? {})) {
    if (value === undefined || value === null || value === false) continue;
    if (key === 'class') el.className = value;
    else if (key === 'text') el.textContent = value;
    else if (key === 'style' && typeof value === 'object') Object.assign(el.style, value);
    else if (key.startsWith('on') && typeof value === 'function') el.addEventListener(key.slice(2).toLowerCase(), value);
    else if (key === 'dataset') Object.assign(el.dataset, value);
    else if (value === true) el.setAttribute(key, '');
    else el.setAttribute(key, value);
  }
  append(el, children);
  return el;
}

// replaceChildren that skips null/false parts and flattens arrays, like h().
export function mount(el, ...children) {
  el.replaceChildren();
  append(el, children);
  return el;
}

function append(el, children) {
  for (const child of children.flat(Infinity)) {
    if (child === null || child === undefined || child === false) continue;
    el.append(child instanceof Node ? child : document.createTextNode(String(child)));
  }
}

// ---- Icons (24×24 stroke) ------------------------------------------------------------
const PATHS = {
  usb: 'M12 3v13M8 7l4-4 4 4M7 12v2l5 3 5-3v-2M12 17v4',
  plug: 'M9 3v5M15 3v5M6 8h12v3a6 6 0 0 1-12 0V8ZM12 17v4',
  play: 'M7 5v14l11-7Z',
  stop: 'M6 6h12v12H6Z',
  power: 'M12 3v8M6.3 7.3a8 8 0 1 0 11.4 0',
  home: 'M4 11 12 4l8 7M6 10v10h12V10',
  target: 'M12 3v4M12 17v4M3 12h4M17 12h4M12 8a4 4 0 1 1 0 8 4 4 0 0 1 0-8Z',
  zero: 'M12 4a8 8 0 1 1 0 16 8 8 0 0 1 0-16ZM12 4v8',
  sliders: 'M4 6h10M18 6h2M4 12h4M12 12h8M4 18h12M20 18h0M14 4v4M8 10v4M16 16v4',
  activity: 'M3 12h4l3-7 4 14 3-7h4',
  alert: 'M12 4 2.5 20h19L12 4ZM12 10v5M12 17.5v.5',
  info: 'M12 3a9 9 0 1 1 0 18 9 9 0 0 1 0-18ZM12 11v6M12 7.5v.5',
  check: 'M5 12.5 10 17l9-10',
  x: 'M6 6l12 12M18 6 6 18',
  left: 'M15 5l-7 7 7 7',
  right: 'M9 5l7 7-7 7',
  refresh: 'M20 12a8 8 0 1 1-2.3-5.6M20 4v5h-5',
  download: 'M12 4v11M7 10l5 5 5-5M5 20h14',
  upload: 'M12 20V9M7 14l5-5 5 5M5 4h14',
  cpu: 'M8 8h8v8H8ZM4 10h2M4 14h2M18 10h2M18 14h2M10 4v2M14 4v2M10 18v2M14 18v2',
  bolt: 'M13 3 5 14h6l-1 7 8-11h-6Z',
  thermo: 'M10 14.5V5a2 2 0 1 1 4 0v9.5a4 4 0 1 1-4 0Z',
  gauge: 'M4 16a8 8 0 1 1 16 0M12 16l4-5',
  scan: 'M4 8V5h3M17 5h3v3M20 16v3h-3M7 19H4v-3M8 12h8',
  pencil: 'M4 20h4L19 9l-4-4L4 16Z',
  copy: 'M8 8h11v11H8ZM16 8V5H5v11h3',
  sun: 'M12 8a4 4 0 1 1 0 8 4 4 0 0 1 0-8ZM12 2v2M12 20v2M4.9 4.9l1.4 1.4M17.7 17.7l1.4 1.4M2 12h2M20 12h2M4.9 19.1l1.4-1.4M17.7 6.3l1.4-1.4',
  moon: 'M20 14.5A8 8 0 0 1 9.5 4a8 8 0 1 0 10.5 10.5Z',
  monitor: 'M3 5h18v11H3ZM8 20h8M12 16v4',
  grid: 'M4 4h7v7H4ZM13 4h7v7h-7ZM4 13h7v7H4ZM13 13h7v7h-7Z',
  bus: 'M3 12h18M6 12V7M10 12V7M14 12v5M18 12v5',
  compass: 'M12 3a9 9 0 1 1 0 18 9 9 0 0 1 0-18ZM15.5 8.5l-2 5-5 2 2-5Z',
  save: 'M5 4h11l3 3v13H5ZM8 4v5h7V4M8 20v-6h8v6',
};

export function icon(name, cls = '') {
  const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
  svg.setAttribute('viewBox', '0 0 24 24');
  svg.setAttribute('aria-hidden', 'true');
  if (cls) svg.setAttribute('class', cls);
  svg.style.fill = 'none';
  svg.style.stroke = 'currentColor';
  svg.style.strokeWidth = '1.8';
  svg.style.strokeLinecap = 'round';
  svg.style.strokeLinejoin = 'round';
  const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
  path.setAttribute('d', PATHS[name] ?? PATHS.info);
  svg.append(path);
  return svg;
}

// ---- Feedback ------------------------------------------------------------------------------
export function toast(text, tone = 'good', ms = 3600) {
  const host = document.getElementById('toasts');
  const el = h('div', { class: 'toast' }, h('span', { class: 'dot', dataset: { tone } }), h('div', {}, text));
  host.append(el);
  setTimeout(() => el.remove(), ms);
}

export function confirmDialog({ title, text, confirm = 'Confirm', tone = 'danger' }) {
  const modal = document.getElementById('modal');
  document.getElementById('modal-title').textContent = title;
  const body = document.getElementById('modal-text');
  body.replaceChildren(...[].concat(text).map(t => (t instanceof Node ? t : h('p', {}, t))));
  const button = document.getElementById('modal-confirm');
  button.textContent = confirm;
  button.className = tone === 'danger' ? 'btn btn-danger' : 'btn';
  modal.returnValue = '';
  modal.showModal();
  return new Promise(resolve => modal.addEventListener('close', () => resolve(modal.returnValue === 'confirm'), { once: true }));
}

// ---- Formatting ---------------------------------------------------------------------------------
export function fixed(value, digits) {
  if (!Number.isFinite(value)) return '—';
  const text = value.toFixed(digits);
  return text === `-${(0).toFixed(digits)}` ? (0).toFixed(digits) : text;
}

export function signed(value, digits) {
  const text = fixed(value, digits);
  return value > 0 && text !== (0).toFixed(digits) ? `+${text}` : text;
}

export const pos = (turns, unit) => fixed(toDisplay(turns, unit), UNITS[unit].digits);
export const vel = (turns, unit) => fixed(toDisplay(turns, unit), Math.max(1, UNITS[unit].digits - 1));

export function dialLabels(unit) {
  if (unit === 'deg') return ['0°', '90°', '180°', '270°'];
  if (unit === 'rad') return ['0', 'π/2', 'π', '3π/2'];
  return ['0', '¼', '½', '¾'];
}

// Jog step sizes that make sense in each unit.
export function jogSteps(unit) {
  if (unit === 'deg') return [0.1, 1, 10, 90, 360];
  if (unit === 'rad') return [0.01, 0.1, 1, Math.PI / 2, 2 * Math.PI];
  return [0.001, 0.01, 0.1, 0.25, 1];
}

export function stepLabel(step, unit) {
  if (unit === 'rad' && Math.abs(step - Math.PI / 2) < 1e-9) return 'π/2';
  if (unit === 'rad' && Math.abs(step - 2 * Math.PI) < 1e-9) return '2π';
  return `${+step.toPrecision(3)}${unit === 'deg' ? '°' : ''}`;
}

export function copyText(text) {
  navigator.clipboard?.writeText(text).then(() => toast('Copied', 'good', 1500), () => toast(text, 'accent'));
}

// ---- Live bindings ------------------------------------------------------------------------
// A region of the page (top bar, rail, stage) owns one Scope. Rendering the
// region clears it and registers fresh updaters: `text`/`attr` run ~12 times a
// second so numbers stay readable, `frame` runs every animation frame (canvases).
export class Scope {
  constructor() {
    this.clear();
  }
  clear() {
    const cleanups = this.cleanups ?? [];
    this.slow = [];
    this.fast = [];
    this.key = null;
    this.cleanups = [];
    for (const fn of cleanups) fn();
  }
  // Runs when the region is re-rendered or left (e.g. to end a hold-to-run).
  onClear(fn) {
    this.cleanups.push(fn);
  }
  // One keyboard handler per region; return true when the key was used.
  onKey(fn) {
    this.key = fn;
  }
  each(fn) {
    this.slow.push(fn);
    return fn;
  }
  frame(fn) {
    this.fast.push(fn);
    return fn;
  }
  text(el, fn) {
    this.slow.push(() => {
      const value = String(fn());
      if (el.textContent !== value) el.textContent = value;
    });
    return el;
  }
  attr(el, name, fn) {
    this.slow.push(() => {
      const value = fn();
      if (value === null || value === undefined || value === false) el.removeAttribute(name);
      else if (el.getAttribute(name) !== String(value)) el.setAttribute(name, value === true ? '' : value);
    });
    return el;
  }
  // Re-renders `el`'s children when `key()` changes.
  swap(el, key, render) {
    let last;
    this.slow.push(() => {
      const k = key();
      if (k === last) return;
      last = k;
      el.replaceChildren(...[].concat(render()).filter(Boolean));
    });
    return el;
  }
  run(slow, now) {
    if (slow) for (const fn of this.slow) fn(now);
    for (const fn of this.fast) fn(now);
  }
}
