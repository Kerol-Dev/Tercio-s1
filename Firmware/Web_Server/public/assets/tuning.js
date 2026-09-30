// -----------------------------------------------------------------------------
// Tuning tab: live strip chart, a step-response test that measures rise time,
// overshoot and settling, and the position-loop gains next to them.
// -----------------------------------------------------------------------------
import * as p from './protocol.js';
import * as store from './store.js';
import { state } from './store.js';
import { h, icon, fixed, toast } from './ui.js';
import { StripChart, themeColors } from './widgets.js';
import { fmtPos, panel, unitInfo } from './views.js';
import { applyDrafts, settingRow } from './settings.js';

const SERIES = {
  position: { label: 'Position', lines: c => [
    { key: 'target', color: c.accent, dash: [5, 4], width: 1.5, label: 'Target' },
    { key: 'position', color: c.ink, fill: true, label: 'Measured' }] },
  error: { label: 'Following error', lines: c => [{ key: 'error', color: c.warn, fill: true, label: 'Error' }] },
  velocity: { label: 'Velocity', lines: c => [{ key: 'velocity', color: c.good, fill: true, label: 'Velocity' }] },
};
const WINDOWS = [2, 5, 10, 30];
const GAINS = ['kp', 'ki', 'kd', 'positionDeadband', 'maxVelocity', 'maxAcceleration'];
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));

export function renderTuning(node, scope) {
  if (!node.params && !node.loading) store.loadParams(node.id);
  const ui = node.ui;
  ui.series ??= 'position';
  ui.window ??= 5;
  return h('div', { class: 'stack stack-lg' },
    chartPanel(node, scope, ui),
    h('div', { class: 'grid cols-2' }, stepPanel(node, scope, ui), gainsPanel(node, scope)));
}

function seg(options, current, onPick, label) {
  const el = h('div', { class: 'seg', role: 'radiogroup', 'aria-label': label });
  const draw = value => el.replaceChildren(...options.map(([key, text]) =>
    h('button', { type: 'button', role: 'radio', 'aria-checked': String(key === value), onclick: () => { onPick(key); draw(key); } }, text)));
  draw(current);
  return el;
}

function chartPanel(node, scope, ui) {
  const canvas = h('canvas');
  const chart = new StripChart(canvas);
  const legend = h('div', { class: 'legend' });
  let colors = null;
  let pausedAt = null;
  const lines = () => SERIES[ui.series].lines(colors);
  const drawLegend = () => legend.replaceChildren(...lines().map(line =>
    h('span', {}, h('i', { style: { background: line.color } }), line.label)));

  scope.frame(now => {
    if (!colors) {
      const c = themeColors(canvas);
      if (!c.ink) return;
      colors = c;
      drawLegend();
    }
    const u = unitInfo();
    chart.draw(node.history, pausedAt ?? now, {
      windowS: ui.window, lines: lines(), scale: u.scale,
      unit: ui.series === 'velocity' ? u.vel : u.pos,
      minSpan: ui.series === 'error' ? u.scale * 0.002 : u.scale * 0.01,
    });
  });

  const pause = h('button', { class: 'btn btn-ghost btn-sm', type: 'button' });
  const drawPause = () => pause.replaceChildren(icon(pausedAt ? 'play' : 'stop'), pausedAt ? 'Resume' : 'Freeze');
  pause.onclick = () => {
    pausedAt = pausedAt ? null : performance.now();
    drawPause();
  };
  drawPause();

  return h('section', { class: 'panel' },
    h('div', { class: 'panel-head wrap' },
      seg(Object.entries(SERIES).map(([key, s]) => [key, s.label]), ui.series, key => { ui.series = key; if (colors) drawLegend(); }, 'Chart'),
      h('div', { class: 'row' }, seg(WINDOWS.map(s => [s, `${s} s`]), ui.window, s => (ui.window = s), 'Time window'), pause)),
    h('div', { class: 'panel-body' }, h('div', { class: 'chart' }, canvas), legend));
}

// ---- Step response ------------------------------------------------------------------------------
function analyse(history, t0, start, target, deadband) {
  const span = target - start;
  const dir = Math.sign(span) || 1;
  const tolerance = Math.max(Math.abs(span) * 0.02, deadband * 2);
  let rise10 = null;
  let rise90 = null;
  let overshoot = 0;
  let settledAt = null;
  let peakError = 0;
  let last = null;
  for (const i of history.range(t0)) {
    const t = history.t[i] - t0;
    const position = history.series.position[i];
    const progress = (position - start) / span;
    if (rise10 === null && progress >= 0.1) rise10 = t;
    if (rise90 === null && progress >= 0.9) rise90 = t;
    overshoot = Math.max(overshoot, (position - target) * dir);
    peakError = Math.max(peakError, Math.abs(history.series.error[i]));
    if (Math.abs(position - target) > tolerance) settledAt = null;
    else settledAt ??= t;
    last = position;
  }
  return {
    rise: rise10 !== null && rise90 !== null ? rise90 - rise10 : null,
    overshoot, settle: settledAt, peakError, finalError: last === null ? null : last - target,
  };
}

function stepPanel(node, scope, ui) {
  const u = unitInfo();
  ui.stepSize ??= { deg: 10, rad: 0.2, turn: 0.03 }[state.unit];
  const input = h('input', { type: 'number', step: 'any', inputmode: 'decimal', value: ui.stepSize, 'aria-label': 'Step size', dataset: { key: 'step-size' } });
  input.oninput = () => (ui.stepSize = input.value);
  const results = h('div', { class: 'stat-grid two' });
  const show = r => {
    const ms = v => (v === null ? '—' : `${fixed(v, 0)} ms`);
    const stat = (label, value) => h('div', { class: 'stat' }, h('span', { class: 'eyebrow' }, label), h('div', { class: 'value' }, value));
    results.replaceChildren(
      stat('Rise 10–90 %', ms(r.rise)),
      stat('Overshoot', fmtPos(r.overshoot)),
      stat('Settles in', r.settle === null ? 'not settled' : ms(r.settle)),
      stat('Final error', r.finalError === null ? '—' : fmtPos(r.finalError)));
  };
  if (ui.stepResult) show(ui.stepResult);
  else results.append(h('p', { class: 'muted' }, 'Run a step to measure the response.'));

  const run = h('button', { class: 'btn', type: 'button' }, icon('activity'), 'Run step');
  run.onclick = async () => {
    const size = Number(input.value);
    if (!Number.isFinite(size) || size === 0 || !node.t) return;
    if (!(node.t.flags & p.FLAGS.settled)) return toast('Wait until the motor holds still.', 'warn');
    const amplitude = p.fromDisplay(size, state.unit);
    const start = node.t.target;
    run.disabled = true;
    const t0 = performance.now();
    const goal = start + amplitude;
    if (await store.moveTo(node.id, goal)) {
      await sleep(1500);
      ui.stepResult = analyse(node.history, t0, start, goal, node.params?.positionDeadband ?? 0.0005);
      show(ui.stepResult);
      // Go back only if nothing interrupted the step (Stop, power off, another move).
      const t = node.t;
      const untouched = t && t.flags & p.FLAGS.enabled && t.state === p.AxisState.Holding && Math.abs(t.target - goal) < 1e-9;
      if (untouched) await store.moveTo(node.id, start);
    }
    run.disabled = false;
  };

  return panel('Step response', h('div', { class: 'stack' },
    h('div', { class: 'move-form' }, h('span', { class: 'field' }, input, h('span', { class: 'suffix' }, u.pos)), run),
    results),
  { note: 'Moves by the step, measures for 1.5 s, then returns to where it started. Try it after each change to the gains.' });
}

function gainsPanel(node, scope) {
  if (!node.params) return panel('Position loop', h('p', { class: 'muted' }, 'Reading settings…'));
  const apply = h('button', { class: 'btn btn-sm', type: 'button', onclick: () => applyDrafts(node, GAINS.filter(key => key in node.drafts)) }, icon('check'), 'Apply');
  scope.each(() => (apply.disabled = !GAINS.some(key => key in node.drafts)));
  return panel('Position loop', h('div', { class: 'stack' },
    GAINS.map(key => settingRow(node, p.PARAM_BY_KEY.get(key), scope)),
    h('div', { class: 'row between' },
      h('span', { class: 'muted small' }, 'Applied at once. Save to the motor from the Settings tab.'),
      apply)),
  { note: 'Start with the proportional gain alone; add damping only if a step overshoots.' });
}
