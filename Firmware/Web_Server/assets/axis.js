// -----------------------------------------------------------------------------
// One motor: header with power controls, fault and setup notices, and the
// Control tab (dial, readouts, moves, jog, hold-to-run, procedures).
// -----------------------------------------------------------------------------
import * as p from './protocol.js';
import * as store from './store.js';
import { state } from './store.js';
import { h, icon, fixed, pos, jogSteps, stepLabel, dialLabels, toast } from './ui.js';
import { Dial } from './widgets.js';
import { chip, fmtPos, fmtVel, isStale, panel, status, statusChip, unitInfo } from './views.js';
import { renderTuning } from './tuning.js';
import { renderSettings, renderDiagnostics } from './settings.js';

const TABS = [
  { key: 'control', label: 'Control' },
  { key: 'tuning', label: 'Tuning' },
  { key: 'settings', label: 'Settings' },
  { key: 'diagnostics', label: 'Diagnostics' },
];

const has = (node, flag) => Boolean(node.t && node.t.flags & flag);

// Every hold-to-run in progress; all of them end when the page loses focus.
const holds = new Set();
const releaseAll = () => holds.forEach(end => end());
document.addEventListener('visibilitychange', () => document.hidden && releaseAll());
window.addEventListener('blur', releaseAll);
const slot = () => h('div', { class: 'slot' });

export function renderAxis(stage, scope, node) {
  const tab = TABS.some(t => t.key === state.tab) ? state.tab : 'control';
  const body = { control: renderControl, tuning: renderTuning, settings: renderSettings, diagnostics: renderDiagnostics }[tab](node, scope);
  stage.replaceChildren(header(node, scope), faultBanner(node, scope), notices(node, scope), tabs(node, scope, tab), body);
}

// ---- Header -----------------------------------------------------------------------------
function header(node, scope) {
  const title = h('button', { class: 'rename', type: 'button', title: 'Rename this motor' }, store.nameOf(node));
  title.onclick = () => {
    const input = h('input', { class: 'rename-input', value: store.nameOf(node), 'aria-label': 'Motor name', maxlength: '40' });
    let finished = false;
    const done = commit => {
      if (finished) return;
      finished = true;
      if (commit) store.rename(node, input.value);
      else input.replaceWith(title);  // removing the input fires blur: ignored above
    };
    input.onkeydown = event => {
      if (event.key === 'Enter') done(true);
      if (event.key === 'Escape') {
        event.stopPropagation();
        done(false);
      }
    };
    input.onblur = () => done(true);
    title.replaceWith(input);
    input.select();
  };
  if (!node.info) title.disabled = true;  // names are stored against the chip serial number

  const chips = slot();
  scope.swap(chips, () => (node.t ? `${node.t.flags}|${node.t.warnings}|${isStale(node)}` : '-'), () => {
    const t = node.t;
    if (!t) return [];
    const list = [];
    if (!(t.flags & p.FLAGS.calibrated)) list.push(chip('Not calibrated', 'warn'));
    if (t.flags & p.FLAGS.homed) list.push(chip('Homed'));
    if (t.flags & p.FLAGS.movePending) list.push(chip('Move queued', 'accent'));
    if (t.flags & p.FLAGS.limitMin) list.push(chip('At IN1 limit', 'warn'));
    if (t.flags & p.FLAGS.limitMax) list.push(chip('At IN2 limit', 'warn'));
    for (const w of p.WARNINGS) if (t.warnings & w.bit) list.push(chip(w.name, 'warn'));
    return list;
  });

  const input = h('input', { type: 'checkbox', role: 'switch' });
  const label = h('span', { class: 'switch-label' });
  let busy = false;
  scope.each(() => {
    const on = has(node, p.FLAGS.enabled);
    if (!busy && input.checked !== on) input.checked = on;
    input.disabled = busy || !node.t;
    const text = on ? 'Motor on' : 'Motor off';
    if (label.textContent !== text) label.textContent = text;
  });
  input.onchange = async () => {
    busy = true;
    const ok = await store.enable(node.id, input.checked);
    busy = false;
    if (!ok) input.checked = !input.checked;
  };

  return h('div', { class: 'axis-head' },
    h('div', { class: 'axis-title' },
      h('div', { class: 'eyebrow' }, 'Tercio S1', node.info && ` · firmware ${node.info.firmware}`),
      h('h1', {}, title, h('span', { class: 'node' }, `node ${node.id}`)),
      h('div', { class: 'flags' }, statusChip(scope, node), chips)),
    h('div', { class: 'axis-actions' },
      h('label', { class: 'switch' }, input, h('span', { class: 'track' }), label),
      h('button', { class: 'btn btn-secondary', type: 'button', title: 'Decelerate to a stop and hold', onclick: () => store.stop(node.id) }, icon('stop'), 'Stop'),
      h('button', { class: 'btn btn-danger-outline', type: 'button', title: 'Switch the motor current off at once. The shaft turns freely.', onclick: () => store.emergencyStop(node.id) }, icon('power'), 'Cut power')));
}

function faultBanner(node, scope) {
  const el = slot();
  scope.swap(el, () => node.t?.faults ?? 0, () => {
    const faults = p.FAULTS.filter(f => (node.t?.faults ?? 0) & f.bit);
    if (!faults.length) return null;
    return h('div', { class: 'banner', role: 'alert' }, icon('alert'),
      h('div', { class: 'text' }, faults.map(f => h('div', {}, h('strong', {}, f.name), h('p', {}, f.help)))),
      h('button', { class: 'btn btn-sm', type: 'button', onclick: () => store.clearFaults(node.id) }, 'Clear fault'));
  });
  return el;
}

function noticeKey(node) {
  if (isStale(node)) return node.t ? 'stale' : 'waiting';
  if (node.t.state === p.AxisState.StepDir) return 'stepdir';
  if (!has(node, p.FLAGS.calibrated) && node.t.state !== p.AxisState.Calibrating) return 'calibrate';
  return '';
}

function notices(node, scope) {
  const el = slot();
  scope.swap(el, () => noticeKey(node), () => {
    switch (noticeKey(node)) {
      case 'waiting':
        return notice('info', 'Waiting for data from this motor', 'It answered on the bus but sends no telemetry. Set a telemetry rate under Settings → Communication.');
      case 'stale':
        return notice('warn', 'This motor stopped reporting', 'It may have lost power or its CAN connection. The values below are the last ones received.');
      case 'stepdir':
        return notice('info', 'Following the step/dir input', 'Moves from this page are refused while step/dir mode is on. Turn it off under Settings → Inputs.');
      case 'calibrate':
        return notice('info', 'Calibrate this motor before moving it',
          'Calibration turns the shaft a quarter turn each way to match the encoder to the motor. It takes about five seconds and the shaft must be free to turn.',
          h('button', { class: 'btn btn-sm', type: 'button', onclick: () => store.calibrate(node.id) }, 'Calibrate'));
      default:
        return null;
    }
  });
  return el;
}

const notice = (tone, title, text, action) =>
  h('div', { class: 'banner', dataset: { tone } }, icon(tone === 'warn' ? 'alert' : 'info'),
    h('div', { class: 'text' }, h('strong', {}, title), h('p', {}, text)), action);

function tabs(node, scope, current) {
  const count = h('span', { class: 'count' });
  scope.each(() => {
    const n = p.WARNINGS.filter(w => (node.t?.warnings ?? 0) & w.bit).length;
    count.hidden = !n;
    count.textContent = n;
  });
  return h('div', { class: 'tabs', role: 'tablist' }, TABS.map(t =>
    h('button', { type: 'button', role: 'tab', 'aria-selected': String(t.key === current), onclick: () => store.setTab(t.key) },
      t.label, t.key === 'diagnostics' && count)));
}

// ---- Control tab ----------------------------------------------------------------------------
function renderControl(node, scope) {
  if (!node.params && !node.loading) store.loadParams(node.id);  // limits for the speed slider
  installJogKeys(node, scope);
  return h('div', { class: 'grid cockpit' },
    dialPanel(node, scope),
    h('div', { class: 'stack stack-lg' }, movePanel(node), jogPanel(node), runPanel(node, scope), proceduresPanel(node, scope)));
}

function readout(scope, label, fn, unit, tone) {
  const value = h('span');
  scope.text(value, fn);
  const box = h('div', { class: 'value' }, value, unit && h('small', {}, unit));
  if (tone) scope.attr(box, 'data-tone', tone);
  return h('div', { class: 'readout' }, h('span', { class: 'eyebrow' }, label), box);
}

function dialPanel(node, scope) {
  const u = unitInfo();
  const canvas = h('canvas');
  const dial = new Dial(canvas);
  scope.frame(() => {
    const t = node.t;
    dial.draw({ turns: t?.position ?? 0, target: t?.target ?? 0, tone: status(node).tone, active: has(node, p.FLAGS.enabled), labels: dialLabels(state.unit) });
  });
  const value = h('div', { class: 'dial-value' });
  scope.text(value, () => (node.t ? pos(node.t.position, state.unit) : '—'));
  const revolutions = h('div', { class: 'dial-sub' });
  scope.text(revolutions, () => {
    if (!node.t) return '';
    const turns = Math.trunc(node.t.position);
    return state.unit === 'turn' || !turns ? '' : `${turns > 0 ? '+' : ''}${turns} rev`;
  });
  const limit = () => node.params?.followingErrorLimit ?? p.PARAM_DEFAULTS.followingErrorLimit;
  return h('section', { class: 'panel dial-panel' },
    h('div', { class: 'panel-body' },
      h('div', { class: 'dial-wrap' }, canvas,
        h('div', { class: 'dial-center' }, value, h('div', { class: 'dial-unit' }, u.pos), revolutions))),
    h('div', { class: 'readouts' },
      readout(scope, 'Target', () => (node.t ? pos(node.t.target, state.unit) : '—'), u.pos),
      readout(scope, 'Velocity', () => (node.t ? fixed(p.toDisplay(node.t.velocity, state.unit), 1) : '—'), u.vel),
      readout(scope, 'Following error', () => (node.t ? fixed(p.toDisplay(node.t.followingError, state.unit), u.digits + 1) : '—'), u.pos,
        () => (node.t && Math.abs(node.t.followingError) > limit() * 0.5 ? 'warn' : null)),
      readout(scope, 'Temperature', () => (node.t ? fixed(node.t.temperature, 1) : '—'), '°C',
        () => (node.t?.temperature > (node.params?.overTemperatureC ?? 95) - 10 ? 'bad' : node.t?.temperature > 70 ? 'warn' : null)),
      readout(scope, 'Supply', () => (node.t ? fixed(node.t.supply, 1) : '—'), 'V', () => (node.t && node.t.supply < 8 ? 'warn' : null)),
      readout(scope, 'Loop load', () => (node.t ? node.t.load : '—'), '%', () => (node.t?.load > 80 ? 'warn' : null))));
}

function numberField(value, suffix, attrs = {}) {
  const input = h('input', { type: 'number', step: 'any', inputmode: 'decimal', value, ...attrs });
  return { input, field: h('span', { class: 'field' }, input, suffix && h('span', { class: 'suffix' }, suffix)) };
}

function readNumber(input, { allowEmpty = false } = {}) {
  const field = input.closest('.field');
  if (allowEmpty && input.value.trim() === '') {
    field?.classList.remove('invalid');
    return 0;
  }
  const value = Number(input.value);
  const valid = input.value.trim() !== '' && Number.isFinite(value);
  field?.classList.toggle('invalid', !valid);
  return valid ? value : null;
}

function movePanel(node) {
  const u = unitInfo();
  const ui = node.ui;
  ui.moveMode ??= 'abs';
  const target = numberField(ui.moveValue ?? '', u.pos, { placeholder: ui.moveMode === 'abs' ? 'Target' : 'Distance', 'aria-label': 'Move target', dataset: { key: 'move-target' } });
  target.input.oninput = () => (ui.moveValue = target.input.value);
  const speed = numberField(ui.moveSpeed ?? '', u.vel, { placeholder: 'Max velocity', min: '0', 'aria-label': 'Velocity limit for this move', dataset: { key: 'move-speed' } });
  const accel = numberField(ui.moveAccel ?? '', u.acc, { placeholder: 'Max acceleration', min: '0', 'aria-label': 'Acceleration limit for this move', dataset: { key: 'move-accel' } });
  speed.input.oninput = () => (ui.moveSpeed = speed.input.value);
  accel.input.oninput = () => (ui.moveAccel = accel.input.value);

  const mode = h('div', { class: 'seg', role: 'radiogroup', 'aria-label': 'Move mode' });
  const drawMode = () => mode.replaceChildren(...[['abs', 'Absolute'], ['rel', 'Relative']].map(([key, label]) =>
    h('button', {
      type: 'button', role: 'radio', 'aria-checked': String(ui.moveMode === key),
      onclick: () => {
        ui.moveMode = key;
        target.input.placeholder = key === 'abs' ? 'Target' : 'Distance';
        drawMode();
      },
    }, label)));
  drawMode();

  const go = event => {
    event.preventDefault();
    const value = readNumber(target.input);
    const v = readNumber(speed.input, { allowEmpty: true });
    const a = readNumber(accel.input, { allowEmpty: true });
    if (value === null || v === null || a === null) return;
    const limits = { maxVelocity: Math.abs(p.fromDisplay(v, state.unit)), maxAcceleration: Math.abs(p.fromDisplay(a, state.unit)) };
    const turns = p.fromDisplay(value, state.unit);
    (ui.moveMode === 'abs' ? store.moveTo : store.moveBy)(node.id, turns, limits);
  };

  return panel('Move', h('form', { class: 'stack', onsubmit: go },
    h('div', { class: 'row between' }, mode,
      h('button', { class: 'btn btn-ghost btn-sm', type: 'button', onclick: () => store.moveTo(node.id, 0) }, icon('home'), 'Go to zero')),
    h('div', { class: 'move-form' }, target.field, h('button', { class: 'btn', type: 'submit' }, icon('play'), 'Move')),
    h('details', { class: 'more', open: Boolean(ui.moveSpeed || ui.moveAccel) },
      h('summary', {}, 'Limits for this move'),
      h('div', { class: 'grid cols-2 tight' }, speed.field, accel.field),
      h('p', { class: 'panel-note' }, 'Leave empty to use the motor’s settings. Lower values than the settings apply to this move only.'))));
}

function jogPanel(node) {
  const steps = jogSteps(state.unit);
  const ui = node.ui;
  if (!steps.includes(ui.jogStep)) ui.jogStep = steps[1];
  const jog = dir => store.moveBy(node.id, p.fromDisplay(dir * ui.jogStep, state.unit));
  const buttons = steps.map(step => h('button', {
    type: 'button', 'aria-pressed': String(step === ui.jogStep),
    onclick: () => {
      ui.jogStep = step;
      buttons.forEach((b, i) => b.setAttribute('aria-pressed', String(steps[i] === step)));
    },
  }, stepLabel(step, state.unit)));
  return panel('Jog', h('div', { class: 'jog' },
    h('button', { class: 'btn btn-secondary jog-btn', type: 'button', 'aria-label': 'Jog negative', onclick: () => jog(-1) }, icon('left')),
    h('div', { class: 'stack center' }, h('div', { class: 'steps', role: 'group', 'aria-label': 'Jog step' }, buttons),
      h('span', { class: 'muted small' }, 'Keyboard: ', h('kbd', {}, '←'), ' ', h('kbd', {}, '→'))),
    h('button', { class: 'btn btn-secondary jog-btn', type: 'button', 'aria-label': 'Jog positive', onclick: () => jog(1) }, icon('right'))));
}

function installJogKeys(node, scope) {
  scope.onKey(event => {
    if (event.key !== 'ArrowLeft' && event.key !== 'ArrowRight') return false;
    if (event.repeat) return true;  // one jog per press: repeats would pile up targets
    if (event.target.closest?.('input, select, textarea, [contenteditable], .seg, .steps, input[type=range]')) return false;
    const steps = jogSteps(state.unit);
    const step = steps.includes(node.ui.jogStep) ? node.ui.jogStep : steps[1];
    store.moveBy(node.id, p.fromDisplay((event.key === 'ArrowLeft' ? -1 : 1) * step, state.unit));
    return true;
  });
}

function runPanel(node, scope) {
  const u = unitInfo();
  const ui = node.ui;
  const max = () => node.params?.maxVelocity ?? p.PARAM_DEFAULTS.maxVelocity;
  ui.runSpeed ??= Math.min(0.5, max() / 2);
  ui.runSpeed = Math.min(ui.runSpeed, max());
  // Slider in thousandths of the max, on a square law so slow speeds get resolution.
  const toSlider = v => Math.round(Math.sqrt(v / max()) * 1000);
  const fromSlider = s => max() * (s / 1000) ** 2;
  const slider = h('input', { class: 'slider', type: 'range', min: '1', max: '1000', value: toSlider(ui.runSpeed), 'aria-label': 'Hold-to-run speed' });
  const readout = h('span', { class: 'num run-speed' });
  scope.text(readout, () => fmtVel(ui.runSpeed));
  slider.oninput = () => (ui.runSpeed = fromSlider(Number(slider.value)));

  const hold = (dir, text) => {
    const button = h('button', { class: 'btn btn-secondary hold-btn', type: 'button', 'aria-label': `Run ${dir < 0 ? 'negative' : 'positive'} while held` },
      dir < 0 && icon('left'), text, dir > 0 && icon('right'));
    let current = null;  // the press in progress: { timer }
    const end = () => {
      if (!current) return;
      clearInterval(current.timer);
      current = null;
      holds.delete(end);
      document.removeEventListener('pointerup', end);
      document.removeEventListener('pointercancel', end);
      button.classList.remove('active');
      store.release(node.id);
    };
    const start = async event => {
      if (current) return;
      event.preventDefault();
      const press = (current = { timer: null });
      holds.add(end);
      // Release is caught on the document, so it still counts when the pointer
      // left the button or the page re-rendered underneath it.
      document.addEventListener('pointerup', end);
      document.addEventListener('pointercancel', end);
      button.classList.add('active');
      const speed = dir * ui.runSpeed;
      const accepted = await store.runVelocity(node.id, speed);
      if (current !== press) return;  // released (and stopped) while waiting for the answer
      if (!accepted) return end();
      press.timer = setInterval(() => store.keepRunning(node.id, speed), 150);
    };
    button.addEventListener('pointerdown', start);
    button.addEventListener('keydown', event => (event.key === ' ' || event.key === 'Enter') && !event.repeat && start(event));
    button.addEventListener('keyup', event => (event.key === ' ' || event.key === 'Enter') && end());
    button.addEventListener('blur', end);
    scope.onClear(end);  // the page re-renders or the motor is left mid-hold
    return button;
  };

  return panel('Run while held', h('div', { class: 'stack' },
    h('div', { class: 'hold' }, hold(-1, 'Hold'), h('div', { class: 'stack center tight' }, slider, readout), hold(1, 'Hold'))),
  { note: `Turns at the chosen speed while you press, and stops when you let go. Top of the scale is the motor's max velocity (${fmtVel(max())}).` });
}

// ---- Procedures -------------------------------------------------------------------------------------
function proceduresPanel(node, scope) {
  const u = unitInfo();
  const ui = node.ui;
  const running = stateKey => node.t?.state === stateKey;

  const progress = (stateKey, describe) => {
    const el = slot();
    scope.swap(el, () => (running(stateKey) ? `run${node.t.procedureStep}` : 'idle'), () => {
      if (!running(stateKey)) return null;
      const { text, fraction } = describe(node.t.procedureStep);
      return h('div', { class: 'progress' },
        h('div', { class: 'row between small' }, h('span', {}, text), h('span', { class: 'num muted' }, `${Math.round(fraction * 100)}%`)),
        h('div', { class: 'progress-bar' }, h('span', { style: { width: `${Math.max(4, fraction * 100)}%` } })));
    });
    return el;
  };
  const action = (stateKey, label, run) => {
    const el = slot();
    scope.swap(el, () => running(stateKey), () => running(stateKey)
      ? h('button', { class: 'btn btn-secondary btn-sm', type: 'button', onclick: () => store.stop(node.id) }, icon('x'), 'Cancel')
      : h('button', { class: 'btn btn-secondary btn-sm', type: 'button', onclick: run }, label));
    return el;
  };

  const homingMode = node.params ? p.HOMING_MODES[node.params.homingMode] : null;
  const from = numberField(ui.tuneFrom ?? 0, u.pos, { 'aria-label': 'Auto-tune travel start', dataset: { key: 'tune-from' } });
  const to = numberField(ui.tuneTo ?? +pos(1, state.unit), u.pos, { 'aria-label': 'Auto-tune travel end', dataset: { key: 'tune-to' } });
  from.input.oninput = () => (ui.tuneFrom = from.input.value);
  to.input.oninput = () => (ui.tuneTo = to.input.value);
  const minimum = 25 / 30;  // turns: LimitTuner::minimumRange()
  const tune = () => {
    const a = readNumber(from.input);
    const b = readNumber(to.input);
    if (a === null || b === null) return;
    const lo = p.fromDisplay(Math.min(a, b), state.unit);
    const hi = p.fromDisplay(Math.max(a, b), state.unit);
    if (hi - lo < minimum) {
      to.input.closest('.field').classList.add('invalid');
      toast(`Auto-tune needs at least ${fmtPos(minimum)} of travel.`, 'warn');
      return;
    }
    store.autoTune(node.id, lo, hi);
  };

  const zeroValue = numberField(ui.zeroValue ?? '', u.pos, { placeholder: '0', 'aria-label': 'New position value', dataset: { key: 'zero-value' } });
  zeroValue.input.oninput = () => (ui.zeroValue = zeroValue.input.value);
  const setPosition = event => {
    event.preventDefault();
    const value = readNumber(zeroValue.input, { allowEmpty: true });
    if (value !== null) store.setZero(node.id, p.fromDisplay(value, state.unit));
  };

  return panel('Setup', h('div', { class: 'stack' },
    h('div', { class: 'procedure' },
      h('h4', {}, 'Set position'),
      h('form', { class: 'row tight', onsubmit: setPosition }, zeroValue.field, h('button', { class: 'btn btn-secondary btn-sm', type: 'submit' }, icon('zero'), 'Set')),
      h('p', {}, 'Make the current shaft position read this value (empty means zero). Only while stopped.')),
    h('div', { class: 'procedure' },
      h('h4', {}, 'Calibrate encoder'),
      action(p.AxisState.Calibrating, 'Calibrate', () => store.calibrate(node.id)),
      h('p', {}, 'Turns the shaft a quarter turn each way to match the encoder to the motor. Needed once, and again after changing the encoder, motor or gearing.'),
      progress(p.AxisState.Calibrating, step => ({ text: p.CALIBRATION_STEPS[step] ?? 'Working', fraction: (step + 0.5) / p.CALIBRATION_STEPS.length }))),
    h('div', { class: 'procedure' },
      h('h4', {}, 'Home'),
      action(p.AxisState.Homing, 'Home', () => store.home(node.id)),
      h('p', {}, homingMode ? `Finds the reference (${homingMode.toLowerCase()}) and makes it zero. Change the method under Settings → Homing.` : 'Finds the reference switch or hard stop and makes it zero.'),
      progress(p.AxisState.Homing, step => ({ text: p.HOMING_STEPS[step] ?? 'Working', fraction: (step + 0.5) / p.HOMING_STEPS.length }))),
    h('div', { class: 'procedure' },
      h('h4', {}, 'Find speed limits'),
      action(p.AxisState.Tuning, 'Start', tune),
      h('p', {}, 'Runs between two positions, faster each time, until the motor slips; then keeps the fastest level that passed three times as the new max velocity and acceleration.'),
      h('div', { class: 'grid cols-2 tight procedure-wide' }, from.field, to.field),
      progress(p.AxisState.Tuning, level => ({ text: `Level ${level + 1} of 19 · ${fmtVel(5 + 3 * level)}`, fraction: (level + 1) / 19 })))));
}
