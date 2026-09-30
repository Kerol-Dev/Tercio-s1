// -----------------------------------------------------------------------------
// Tercio Control: bootstrap, routing between pages, and the frame loop that
// keeps live values moving without re-rendering the page.
// -----------------------------------------------------------------------------
import * as store from './store.js';
import { state, subscribe } from './store.js';
import { SerialTransport } from './link.js';
import { Scope } from './ui.js';
import { renderTopbar, renderRail, renderWelcome, renderFleet, renderAdapter, renderImu } from './views.js';
import { renderAxis } from './axis.js';

const scopes = { top: new Scope(), rail: new Scope(), stage: new Scope() };
const stage = document.getElementById('stage');

function renderStage() {
  scopes.stage.clear();
  if (!state.bus) return renderWelcome(stage);
  const selected = state.selected;
  if (typeof selected === 'number' && state.nodes.has(selected)) return renderAxis(stage, scopes.stage, state.nodes.get(selected));
  if (selected === 'adapter') return renderAdapter(stage, scopes.stage);
  if (typeof selected === 'string' && selected.startsWith('imu:') && state.imus.has(Number(selected.slice(4)))) {
    return renderImu(stage, scopes.stage, Number(selected.slice(4)));
  }
  state.selected = 'fleet';
  return renderFleet(stage, scopes.stage);
}

function renderChrome() {
  renderTopbar(scopes.top);
  renderRail(scopes.rail);
}

// Several changes in one task collapse into one render.
const pending = new Set();
function schedule(what) {
  if (!pending.size) queueMicrotask(flush);
  pending.add(what);
}
function flush() {
  const jobs = new Set(pending);
  pending.clear();
  if (jobs.has('chrome')) renderChrome();
  if (jobs.has('stage')) {
    // Keep the scroll position and the focused field (and its caret) across the re-render.
    const scroll = window.scrollY;
    const active = document.activeElement;
    const selector = active?.id ? `#${CSS.escape(active.id)}` : active?.dataset?.key ? `[data-key="${CSS.escape(active.dataset.key)}"]` : null;
    let caret = null;
    try {
      caret = selector && active.selectionStart !== null ? [active.selectionStart, active.selectionEnd] : null;
    } catch {}  // number inputs have no selection API
    renderStage();
    const next = selector && stage.querySelector(selector);
    if (next) {
      next.focus({ preventScroll: true });
      try {
        if (caret) next.setSelectionRange(...caret);
      } catch {}
    }
    window.scrollTo({ top: scroll });
  }
}

let lastSelection = null;
subscribe((kind, detail) => {
  if (kind === 'structure' || kind === 'theme') {
    schedule('chrome');
    schedule('stage');
    const selection = `${state.selected}|${state.tab}`;
    if (selection !== lastSelection && kind === 'structure') window.scrollTo({ top: 0 });
    lastSelection = selection;
  } else if (kind === 'nodes') {
    schedule('chrome');
    if (state.selected === 'fleet' || state.selected === detail) schedule('stage');
  } else if (kind === 'params') {
    if (state.selected === detail) schedule('stage');
  }
});

// Live values: text ~12 times a second, canvases every frame.
let lastSlow = 0;
function frame(now) {
  const slow = now - lastSlow >= 80;
  if (slow) lastSlow = now;
  for (const scope of Object.values(scopes)) scope.run(slow, now);
  requestAnimationFrame(frame);
}

document.addEventListener('keydown', event => {
  if (event.defaultPrevented || document.getElementById('modal').open) return;
  if (event.key === 'Escape' && state.bus) {
    event.preventDefault();
    store.stopAll();
    return;
  }
  if (!event.altKey && !event.ctrlKey && !event.metaKey && scopes.stage.key?.(event)) event.preventDefault();
});

document.querySelector('.brand').addEventListener('click', event => {
  event.preventDefault();
  if (state.bus) store.select('fleet');
});

// Follow the operating system's theme while "system" is chosen.
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => state.theme === 'system' && store.changed('theme'));

if (SerialTransport.supported) {
  navigator.serial.addEventListener('connect', () => store.refreshKnownPort());
  navigator.serial.addEventListener('disconnect', () => store.refreshKnownPort());
}

if (state.theme !== 'system') document.documentElement.dataset.theme = state.theme;
store.refreshKnownPort();
lastSelection = `${state.selected}|${state.tab}`;
renderChrome();
renderStage();
requestAnimationFrame(frame);
