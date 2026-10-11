// -----------------------------------------------------------------------------
// The Tercio IMU page: attitude (horizon, compass, live chart), calibration,
// motion readings, settings and diagnostics. Protocol v2, like the motors.
// -----------------------------------------------------------------------------
import * as p from './protocol.js';
import * as store from './store.js';
import { h, icon, confirmDialog, fixed, signed, toast, copyText } from './ui.js';
import { Dial, StripChart, themeColors } from './widgets.js';
import { chip, panel, pageHead, plural, STALE_MS } from './views.js';
import { settingRow, fromField, draftRaw } from './settings.js';

export const imuStale = imu => !imu.t || performance.now() - imu.seenAt > STALE_MS;

export function imuStatus(imu) {
  if (!imu.t) return { name: 'Waiting for data', tone: 'neutral' };
  if (imuStale(imu)) return { name: 'No signal', tone: 'warn' };
  if (imu.t.faults && imu.t.state === p.ImuState.Running) return { name: 'Fault cleared by itself', tone: 'warn' };
  return p.IMU_STATE_INFO[imu.t.state] ?? { name: 'Unknown', tone: 'neutral' };
}

const has = (imu, bit) => Boolean(imu.t && imu.t.flags & bit);
const angle = value => `${signed(value, 1)}°`;

function statusChip(scope, imu) {
  const el = h('span');
  scope.swap(el, () => { const s = imuStatus(imu); return s.name + s.tone; }, () => {
    const s = imuStatus(imu);
    return chip(h('span', { class: 'chip-label' }, s.name), s.tone);
  });
  return el;
}

export function renderImu(stage, scope, imu) {
  const ui = imu.ui;
  ui.window ??= 10;
  stage.replaceChildren(
    header(imu, scope),
    faultBanner(imu, scope),
    h('div', { class: 'grid cols-2' }, attitudePanel(imu, scope), calibrationPanel(imu, scope)),
    chartPanel(imu, scope, ui),
    h('div', { class: 'grid cols-2' }, motionPanel(imu, scope), settingsPanel(imu, scope)),
    diagnosticsPanel(imu, scope));
}

// ---- Header ----------------------------------------------------------------------------------
function header(imu, scope) {
  const chips = h('span', { class: 'flags' });
  scope.swap(chips, () => (imu.t ? `${imu.t.flags}|${imu.t.warnings}` : '-'), () => {
    const t = imu.t;
    if (!t) return [];
    const list = [];
    if (t.flags & p.IMU_FLAGS.zeroed) list.push(chip(t.flags & p.IMU_FLAGS.zeroHeadingOnly ? 'Heading zeroed' : 'Zeroed', 'accent'));
    list.push(chip(t.flags & p.IMU_FLAGS.magnetometer ? 'NDOF' : 'IMU mode'));
    for (const w of p.IMU_WARNINGS) if (t.warnings & w.bit) list.push(chip(w.name, 'warn'));
    return list;
  });
  const running = () => imu.t?.state === p.ImuState.Running && !imuStale(imu);
  const zero = h('button', { class: 'btn', type: 'button', title: 'Roll, pitch and yaw read 0 in this attitude', onclick: () => store.imuZero(imu.id, p.ZERO_ALL) }, icon('zero'), 'Zero');
  const heading = h('button', { class: 'btn btn-secondary', type: 'button', title: 'Yaw reads 0 here; roll and pitch stay level-referenced', onclick: () => store.imuZero(imu.id, p.ZERO_HEADING) }, icon('compass'), 'Zero heading');
  const clear = h('button', { class: 'btn btn-ghost', type: 'button', title: 'Back to the sensor’s own reference', onclick: () => store.imuClearZero(imu.id) }, 'Clear zero');
  scope.each(() => {
    zero.disabled = heading.disabled = !running();
    clear.hidden = !has(imu, p.IMU_FLAGS.zeroed);
  });
  return pageHead({
    eyebrow: ['Tercio IMU', imu.info && ` · firmware ${imu.info.firmware}`],
    title: store.nameOf(imu),
    suffix: `node ${imu.id}`,
    chips: [statusChip(scope, imu), chips],
    actions: [clear, heading, zero],
  });
}

function faultBanner(imu, scope) {
  const el = h('div');
  scope.swap(el, () => imu.t?.faults ?? 0, () => {
    const faults = p.IMU_FAULTS.filter(f => (imu.t?.faults ?? 0) & f.bit);
    if (!faults.length) return null;
    return h('div', { class: 'banner', role: 'alert' }, icon('alert'),
      h('div', { class: 'text' }, faults.map(f => h('div', {}, h('strong', {}, f.name), h('p', {}, f.help)))),
      h('button', { class: 'btn btn-sm', type: 'button', onclick: () => store.clearFaults(imu.id) }, 'Clear fault'));
  });
  return el;
}

// ---- Attitude -------------------------------------------------------------------------------------
function attitudePanel(imu, scope) {
  const sky = h('div', { class: 'horizon-sky' });
  scope.frame(() => {
    const t = imu.t;
    if (t) sky.style.transform = `rotate(${-t.roll}deg) translateY(${Math.max(-45, Math.min(45, t.pitch))}%)`;
  });
  const compass = h('canvas');
  const dial = new Dial(compass);
  scope.frame(() => {
    const yaw = imu.t?.yaw ?? 0;
    dial.draw({ turns: yaw / 360, target: yaw / 360, tone: imuStatus(imu).tone === 'good' ? 'accent' : 'neutral', labels: ['0°', '90°', '180°', '−90°'] });
  });
  const reading = (label, key) => {
    const value = h('span', { class: 'value' });
    scope.text(value, () => (imu.t ? angle(imu.t[key]) : '—'));
    return h('div', { class: 'readout' }, h('span', { class: 'eyebrow' }, label), value);
  };
  return h('section', { class: 'panel attitude' },
    h('div', { class: 'panel-head' }, h('h3', {}, 'Attitude')),
    h('div', { class: 'panel-body attitude-body' },
      h('div', { class: 'attitude-view' },
        h('div', { class: 'horizon', 'aria-label': 'Artificial horizon' }, sky, h('div', { class: 'horizon-mark' })),
        h('span', { class: 'caption' }, 'Roll and pitch')),
      h('div', { class: 'attitude-view' },
        h('div', { class: 'compass' }, compass),
        h('span', { class: 'caption' }, 'Yaw'))),
    h('div', { class: 'readouts' }, reading('Roll', 'roll'), reading('Pitch', 'pitch'), reading('Yaw', 'yaw')));
}

// ---- Calibration ------------------------------------------------------------------------------------
const CAL_PARTS = [
  { key: 'gyro', label: 'Gyroscope', how: 'Keep it still for a few seconds.' },
  { key: 'accel', label: 'Accelerometer', how: 'Hold it still in six orientations, each face pointing up in turn.' },
  { key: 'mag', label: 'Magnetometer', how: 'Move it slowly in a figure of eight. NDOF mode only.' },
  { key: 'sys', label: 'System', how: 'Follows the others.' },
];

function calibrationPanel(imu, scope) {
  const rows = CAL_PARTS.map(part => {
    const value = h('span', { class: 'num' });
    const bar = h('span');
    scope.each(() => {
      const c = imu.t?.calibration;
      const unused = part.key === 'mag' || part.key === 'sys' ? !has(imu, p.IMU_FLAGS.magnetometer) : false;
      const level = c ? c[part.key] : 0;
      const text = !c ? '—' : unused ? 'not used' : `${level} / 3`;
      if (value.textContent !== text) value.textContent = text;
      bar.style.width = `${unused ? 0 : (level / 3) * 100}%`;
      bar.dataset.tone = level === 3 ? 'good' : level ? 'warn' : 'bad';
    });
    return h('div', { class: 'meter', title: part.how }, h('div', { class: 'row between' }, h('span', {}, part.label), value), h('div', { class: 'bar' }, bar));
  });
  const save = h('button', { class: 'btn btn-secondary', type: 'button', onclick: () => store.imuSaveCalibration(imu.id) }, icon('save'), 'Save calibration');
  const stateText = h('p', { class: 'muted small' });
  scope.each(() => {
    const calibrated = has(imu, p.IMU_FLAGS.calibrated);
    save.disabled = !calibrated || imuStale(imu);
    const text = calibrated
      ? 'Fully calibrated. Save it, and the IMU starts calibrated after every power-up.'
      : has(imu, p.IMU_FLAGS.calibrationStored)
        ? 'A saved calibration was restored. The sensor fine-tunes it as it moves.'
        : 'Not calibrated yet. Follow the hints below, then save.';
    if (stateText.textContent !== text) stateText.textContent = text;
  });
  return panel('Calibration', h('div', { class: 'stack' },
    rows,
    stateText,
    h('ul', { class: 'hint-list small muted' }, CAL_PARTS.slice(0, 3).map(part => h('li', {}, h('b', {}, `${part.label}: `), part.how))),
    h('div', { class: 'row' }, save)));
}

// ---- Live chart ----------------------------------------------------------------------------------------
const WINDOWS = [5, 10, 30];

function seg(options, current, onPick, label) {
  const el = h('div', { class: 'seg', role: 'radiogroup', 'aria-label': label });
  const draw = value => el.replaceChildren(...options.map(([key, text]) =>
    h('button', { type: 'button', role: 'radio', 'aria-checked': String(key === value), onclick: () => { onPick(key); draw(key); } }, text)));
  draw(current);
  return el;
}

function chartPanel(imu, scope, ui) {
  const canvas = h('canvas');
  const chart = new StripChart(canvas);
  const legend = h('div', { class: 'legend' });
  let lines = null;
  let pausedAt = null;
  scope.frame(now => {
    if (!lines) {
      const c = themeColors(canvas);
      if (!c.ink) return;
      lines = [
        { key: 'roll', color: c.accent, label: 'Roll' },
        { key: 'pitch', color: c.good, label: 'Pitch' },
        { key: 'yaw', color: c.warn, label: 'Yaw' },
      ];
      legend.replaceChildren(...lines.map(line => h('span', {}, h('i', { style: { background: line.color } }), line.label)));
    }
    chart.draw(imu.history, pausedAt ?? now, { windowS: ui.window, lines, unit: 'deg', minSpan: 4 });
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
      h('h3', {}, 'Roll, pitch and yaw'),
      h('div', { class: 'row' }, seg(WINDOWS.map(s => [s, `${s} s`]), ui.window, s => (ui.window = s), 'Time window'), pause)),
    h('div', { class: 'panel-body' }, h('div', { class: 'chart' }, canvas), legend));
}

// ---- Motion -----------------------------------------------------------------------------------------------
function motionPanel(imu, scope) {
  const live = fn => scope.text(h('dd'), () => (imu.t ? fn(imu.t) : '—'));
  const vector = (v, digits) => v.map(x => signed(x, digits)).join('  ');
  return panel('Motion', h('dl', { class: 'kv' },
    h('dt', {}, 'Acceleration'), live(t => `${vector(t.acceleration, 2)} m/s²`),
    h('dt', {}, 'Magnitude'), live(t => `${fixed(Math.hypot(...t.acceleration) / 9.80665, 3)} g`),
    h('dt', {}, 'Angular rate'), live(t => `${vector(t.rate, 1)} °/s`),
    h('dt', {}, 'Quaternion'), live(t => t.quaternion.map(x => fixed(x, 4)).join('  ')),
    h('dt', {}, 'Board temperature'), live(t => `${fixed(t.temperature, 1)} °C`)),
  { note: 'Acceleration includes gravity and, like the angular rate, is in the sensor’s own axes. Angles and the quaternion follow the zero reference.' });
}

// ---- Settings ------------------------------------------------------------------------------------------------
function settingsPanel(imu, scope) {
  if (!imu.params) {
    if (!imu.loading) store.imuLoadParams(imu.id);
    return panel('Settings', h('div', { class: 'empty' }, h('span', { class: 'spinner', 'aria-hidden': 'true' }), h('p', {}, 'Reading settings from the IMU…')));
  }
  const drafts = () => Object.keys(imu.drafts).length;
  const status = h('span', { class: 'muted small' });
  const apply = h('button', { class: 'btn btn-sm', type: 'button', onclick: () => applyImuDrafts(imu) }, icon('check'), 'Apply');
  const save = h('button', { class: 'btn btn-secondary btn-sm', type: 'button', onclick: () => store.imuSave(imu.id) }, icon('save'), 'Save to IMU');
  scope.each(() => {
    const n = drafts();
    const text = n ? `${plural(n, 'change')} not applied` : imu.unsaved ? 'Applied, not saved' : 'Saved on the IMU';
    if (status.textContent !== text) status.textContent = text;
    apply.disabled = !n;
    save.disabled = !imu.unsaved || n > 0;
  });
  return panel('Settings', h('div', { class: 'stack' },
    p.IMU_PARAMS.map(def => settingRow(imu, def, scope)),
    h('div', { class: 'row between wrap' }, status, h('div', { class: 'row' },
      h('button', { class: 'btn btn-ghost btn-sm', type: 'button', onclick: () => restart(imu) }, 'Restart'),
      h('button', { class: 'btn btn-danger-outline btn-sm', type: 'button', onclick: () => factoryReset(imu) }, 'Factory reset'),
      apply, save))),
  { note: 'Zero and saved calibration are kept with the settings: Save keeps the zero after a power cycle.' });
}

async function applyImuDrafts(imu) {
  const defs = Object.keys(imu.drafts).map(key => p.IMU_PARAM_BY_KEY.get(key)).filter(Boolean)
    .sort((a, b) => (a.key === 'nodeId') - (b.key === 'nodeId'));
  const invalid = defs.filter(def => fromField(def, draftRaw(imu, def)) === null);
  if (invalid.length) return toast(`Check ${invalid.map(def => def.label.toLowerCase()).join(', ')}: out of range.`, 'warn');
  const wanted = 'nodeId' in imu.drafts ? fromField(p.IMU_PARAM_BY_KEY.get('nodeId'), draftRaw(imu, p.IMU_PARAM_BY_KEY.get('nodeId'))) : null;
  if (wanted !== null && wanted !== imu.id && (store.state.nodes.has(wanted) || store.state.imus.has(wanted))) {
    return toast(`Node ${wanted} is already on the bus. Pick a free ID.`, 'warn');
  }
  let applied = 0;
  for (const def of defs) {
    const id = imu.id;
    if ((await store.imuSetParam(id, def.key, fromField(def, draftRaw(imu, def)))) === null) break;
    delete imu.drafts[def.key];
    applied++;
  }
  if (applied) toast(`${plural(applied, 'setting')} applied. Save to keep ${applied === 1 ? 'it' : 'them'} after a restart.`, 'accent');
  store.changed('params', store.keyOf(imu));
}

async function restart(imu) {
  const ok = await confirmDialog({
    title: `Restart ${store.nameOf(imu)}?`,
    text: ['The IMU stops reporting for about a second. Settings and a zero that were not saved are lost.'],
    confirm: 'Restart',
  });
  if (!ok || !(await store.reboot(imu.id))) return;
  imu.drafts = {};
  imu.unsaved = false;
  setTimeout(() => {
    imu.params = null;
    store.imuLoadParams(imu.id);
  }, 1500);
  toast('Restarting…', 'accent', 2000);
}

async function factoryReset(imu) {
  const ok = await confirmDialog({
    title: 'Restore factory settings?',
    text: ['Fusion mode, mounting and telemetry rate return to their defaults; the zero and the saved calibration are cleared. The node ID stays.',
      'The new values apply at once; Save keeps them after a restart.'],
    confirm: 'Restore defaults',
  });
  if (ok) store.imuFactoryReset(imu.id);
}

// ---- Diagnostics -----------------------------------------------------------------------------------------------
function diagnosticsPanel(imu, scope) {
  const info = imu.info;
  const live = fn => scope.text(h('dd'), fn);
  const loss = () => (imu.frames ? (100 * imu.lost) / (imu.frames + imu.lost) : 0);
  return h('div', { class: 'grid cols-2' },
    panel('Identity', info ? h('dl', { class: 'kv' },
      h('dt', {}, 'Node ID'), h('dd', {}, imu.id),
      h('dt', {}, 'Firmware'), h('dd', {}, info.firmware),
      h('dt', {}, 'Protocol'), h('dd', {}, `v${info.protocol}`),
      h('dt', {}, 'Hardware'), h('dd', {}, `revision ${info.hardwareRevision}`),
      h('dt', {}, 'Sensor'), h('dd', {}, 'Bosch BNO055, 9-axis'),
      h('dt', {}, 'Clock'), h('dd', {}, info.crystalClock ? '170 MHz from the crystal' : '170 MHz from the internal oscillator'),
      h('dt', {}, 'Serial'), h('dd', {}, h('button', { class: 'copy', type: 'button', title: 'Copy', onclick: () => copyText(info.uid) }, info.uid, icon('copy')))) :
      h('p', { class: 'muted' }, 'The IMU has not identified itself yet.')),
    panel('Telemetry link', h('dl', { class: 'kv' },
      h('dt', {}, 'Rate'), live(() => `${imu.rateHz} frames/s`),
      h('dt', {}, 'Received'), live(() => imu.frames.toLocaleString('en-US')),
      h('dt', {}, 'Lost'), live(() => `${imu.lost.toLocaleString('en-US')} (${fixed(loss(), 2)} %)`),
      h('dt', {}, 'Last frame'), live(() => (imu.t ? (imuStale(imu) ? `${fixed((performance.now() - imu.seenAt) / 1000, 0)} s ago` : 'just now') : 'never'))),
    { note: 'Lost frames are counted from gaps in the sequence number.' }));
}
