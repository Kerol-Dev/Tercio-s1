// -----------------------------------------------------------------------------
// Everything outside a single motor: top bar, device rail, the connect screen,
// the bus overview, the adapter page and the IMU page.
// -----------------------------------------------------------------------------
import * as p from './protocol.js';
import { SerialTransport } from './link.js';
import * as store from './store.js';
import { state } from './store.js';
import { h, icon, confirmDialog, fixed, pos, vel, copyText, dialLabels, mount, toast } from './ui.js';
import { Dial } from './widgets.js';

export const STALE_MS = 1500;

// ---- Shared helpers ---------------------------------------------------------------------
export const unitInfo = () => p.UNITS[state.unit];
export const fmtPos = (turns, unit = state.unit) => `${pos(turns, unit)}${unit === 'deg' ? '°' : ` ${p.UNITS[unit].pos}`}`;
export const fmtVel = (turns, unit = state.unit) => `${vel(turns, unit)} ${p.UNITS[unit].vel}`;
export const hexId = id => `0x${id.toString(16).padStart(3, '0')}`;
export const plural = (n, word) => `${n} ${word}${n === 1 ? '' : 's'}`;

export function isStale(node) {
  return !node.t || performance.now() - node.seenAt > STALE_MS;
}

// What the motor is doing, as a name and a tone.
export function status(node) {
  if (!node.t) return { name: 'Waiting for data', tone: 'neutral' };
  if (isStale(node)) return { name: 'No signal', tone: 'warn' };
  if (node.t.faults && node.t.state !== p.AxisState.Fault) {
    return { name: p.FAULTS.find(f => node.t.faults & f.bit)?.name ?? 'Fault', tone: 'bad' };  // a soft fault while holding
  }
  return p.STATE_INFO[node.t.state] ?? { name: 'Unknown', tone: 'neutral' };
}

export const chip = (text, tone, attrs = {}) => h('span', { class: 'chip', dataset: tone ? { tone } : undefined, ...attrs }, text);
const dot = tone => h('span', { class: 'dot', dataset: { tone } });

export function liveDot(scope, fn, live = false) {
  const el = h('span', { class: live ? 'dot live' : 'dot' });
  scope.attr(el, 'data-tone', fn);
  return el;
}

export function statusChip(scope, node) {
  const el = h('span');
  scope.swap(el, () => { const s = status(node); return s.name + s.tone; }, () => {
    const s = status(node);
    return chip(h('span', { class: 'chip-label' }, s.name), s.tone);
  });
  return el;
}

export function panel(title, body, { note, actions, cls = '' } = {}) {
  return h('section', { class: `panel ${cls}` },
    title && h('div', { class: 'panel-head' }, h('h3', {}, title), actions),
    h('div', { class: 'panel-body' }, note && h('p', { class: 'panel-note' }, note), body));
}

export function pageHead({ eyebrow, title, suffix, chips, actions }) {
  return h('div', { class: 'axis-head' },
    h('div', { class: 'axis-title' },
      eyebrow && h('div', { class: 'eyebrow' }, eyebrow),
      h('h1', {}, title, suffix && h('span', { class: 'node' }, suffix)),
      chips && h('div', { class: 'flags' }, chips)),
    actions && h('div', { class: 'axis-actions' }, actions));
}

// ---- Top bar ----------------------------------------------------------------------------
const THEME_NEXT = { system: 'light', light: 'dark', dark: 'system' };
const THEME_NAME = { system: 'Theme: match system', light: 'Theme: light', dark: 'Theme: dark' };
const THEME_ICON = { system: 'monitor', light: 'sun', dark: 'moon' };
const UNIT_SHORT = { deg: 'deg', rad: 'rad', turn: 'rev' };

export function renderTopbar(scope) {
  scope.clear();
  document.getElementById('link-status').replaceChildren(...linkStatus(scope));

  document.getElementById('unit-switch').replaceChildren(...Object.values(p.UNITS).map(u =>
    h('button', { type: 'button', role: 'radio', 'aria-checked': String(state.unit === u.key), title: u.label, onclick: () => store.setUnit(u.key) },
      UNIT_SHORT[u.key])));

  const theme = document.getElementById('theme-toggle');
  theme.replaceChildren(icon(THEME_ICON[state.theme]));
  theme.setAttribute('aria-label', THEME_NAME[state.theme]);
  theme.title = `${THEME_NAME[state.theme]}. Click to change.`;
  theme.onclick = () => store.setTheme(THEME_NEXT[state.theme]);

  const stopAll = document.getElementById('stop-all');
  stopAll.disabled = !state.bus;
  stopAll.onclick = () => store.stopAll();
}

function busTone() {
  const status = state.adapter.status;
  if (!status || performance.now() - state.adapter.seenAt > 1500) return 'neutral';
  return p.BUS_STATES[status.busState]?.tone ?? 'neutral';
}

function linkStatus(scope) {
  if (state.mode === 'connecting') return [chip([dot('busy'), 'Connecting…'])];
  if (!state.bus) return [chip([dot('neutral'), 'Not connected'])];
  const demo = state.mode === 'demo';
  const count = h('span', { class: 'extra muted' });
  scope.text(count, () => `· ${plural(state.nodes.size, 'motor')}`);
  return [
    h('button', { class: 'pill', type: 'button', title: 'Adapter and bus health', onclick: () => store.select('adapter') },
      liveDot(scope, busTone, true), h('strong', {}, demo ? 'Demo bus' : 'Tercio FD'), count),
    h('button', { class: 'btn btn-ghost btn-sm disconnect', type: 'button', onclick: () => store.disconnect() }, 'Disconnect'),
  ];
}

// ---- Rail -------------------------------------------------------------------------------------
function railItem({ target, lead, name, sub, end, title }) {
  return h('button', {
    class: 'rail-item', type: 'button', title,
    'aria-current': String(state.selected === target),
    onclick: () => store.select(target),
  }, lead, h('span', { class: 'stack-tight' }, h('span', { class: 'name' }, name), sub), end ?? h('span'));
}

function railSection(title, action, items) {
  return h('div', { class: 'rail-section' },
    h('div', { class: 'rail-head' }, h('span', { class: 'eyebrow' }, title), action),
    items);
}

function motorItem(node, scope) {
  const sub = h('span', { class: 'sub' });
  scope.text(sub, () => (isStale(node) ? status(node).name : fmtPos(node.t.position)));
  const end = h('span', { class: 'end' }, liveDot(scope, () => status(node).tone));
  return railItem({ target: node.id, lead: h('span', { class: 'id' }, node.id), name: store.nameOf(node), sub, end, title: `Node ${node.id}` });
}

export function renderRail(scope) {
  scope.clear();
  const rail = document.getElementById('rail');
  const foot = h('div', { class: 'rail-foot muted small' }, 'Tercio Control · protocol v2');
  if (!state.bus) {
    mount(rail, railSection('Devices', null, h('div', { class: 'rail-empty' }, 'Connect an adapter to list the motors on its bus.')), foot);
    return;
  }
  const motors = [...state.nodes.values()].sort((a, b) => a.id - b.id);
  const scan = h('button', { class: 'icon-btn icon-btn-sm', type: 'button', title: 'Scan the bus', 'aria-label': 'Scan the bus', onclick: () => store.scan() }, icon('refresh'));
  const adapterSub = h('span', { class: 'sub' });
  scope.text(adapterSub, () => {
    const s = state.adapter.status;
    return s ? p.BUS_STATES[s.busState]?.name ?? '—' : 'Waiting for status';
  });
  mount(rail,
    h('div', { class: 'rail-section' }, railItem({ target: 'fleet', lead: h('span', { class: 'id' }, icon('grid')), name: 'Overview', sub: h('span', { class: 'sub' }, plural(motors.length, 'motor')) })),
    railSection('Motors', scan, motors.length ? motors.map(node => motorItem(node, scope)) : h('div', { class: 'rail-empty' }, 'No motors have answered yet.')),
    state.imus.size ? railSection('Sensors', null, [...state.imus.keys()].map(id =>
      railItem({ target: `imu:${id}`, lead: h('span', { class: 'id' }, icon('compass')), name: 'IMU', sub: h('span', { class: 'sub' }, hexId(id)) }))) : null,
    railSection('Adapter', null, railItem({
      target: 'adapter', lead: h('span', { class: 'id' }, icon('usb')), name: state.mode === 'demo' ? 'Simulated adapter' : 'Tercio FD',
      sub: adapterSub, end: h('span', { class: 'end' }, liveDot(scope, busTone)),
    })),
    foot);
}

// ---- Connect screen ------------------------------------------------------------------------------
const SPECS = [
  ['20 kHz', 'position loop'],
  ['0.5/2.5', 'Mbit/s CAN-FD'],
  ['1/256', 'microstep driver'],
  ['127', 'axes per bus'],
];

export function renderWelcome(stage) {
  const supported = SerialTransport.supported;
  const busy = state.mode === 'connecting';
  const known = state.knownPort;
  const actions = [];
  if (known) {
    actions.push(h('button', { class: 'btn btn-lg', type: 'button', disabled: busy, onclick: () => store.connectSerial({ port: known }) }, icon('usb'), 'Reconnect Tercio FD'));
    actions.push(h('button', { class: 'btn btn-lg btn-secondary', type: 'button', disabled: busy, onclick: () => store.connectSerial() }, 'Choose another'));
  } else {
    actions.push(h('button', { class: 'btn btn-lg', type: 'button', disabled: !supported || busy, onclick: () => store.connectSerial() }, icon('usb'), 'Connect Tercio FD'));
  }
  actions.push(h('button', { class: 'btn btn-lg btn-secondary', type: 'button', disabled: busy, onclick: () => store.connectDemo() }, icon('play'), 'Try the demo'));

  mount(stage, h('div', { class: 'welcome' },
    h('section', { class: 'panel hero' },
      h('div', { class: 'eyebrow' }, 'Tercio S1 · closed-loop stepper'),
      h('h1', {}, 'Move, tune and watch every axis on your bus.'),
      h('p', {}, 'Plug in the Tercio FD adapter and this page finds every motor on the CAN-FD bus. Calibrate, home and move them, tune the position loop against a live chart, and save settings to each motor.'),
      !supported && h('div', { class: 'banner', dataset: { tone: 'warn' } }, icon('alert'),
        h('div', { class: 'text' }, h('strong', {}, 'This browser cannot open serial ports.'),
          h('p', {}, 'Use Chrome or Edge on a desktop computer. The demo works in any browser.'))),
      h('div', { class: 'actions' }, actions),
      supported && h('button', { class: 'link-btn', type: 'button', disabled: busy, onclick: () => store.connectSerial({ anyDevice: true }) },
        'Adapter not listed? Show every serial port'),
      h('dl', { class: 'spec-strip' }, SPECS.map(([value, label]) => h('div', {}, h('dt', { class: 'num' }, value), h('dd', {}, label))))),
    h('section', { class: 'panel' },
      h('div', { class: 'panel-head' }, h('h3', {}, 'Before you connect')),
      h('div', { class: 'panel-body' }, h('ol', { class: 'steps-list' },
        step(1, 'Wire the bus', 'Daisy-chain CAN-H and CAN-L between the adapter and each motor, and terminate both ends with 120 Ω.'),
        step(2, 'Power the motors', 'The adapter runs from USB; the motors need their own supply.'),
        step(3, 'Plug in Tercio FD', 'Click Connect and pick “tercioFD” in the list the browser shows.'),
        step(4, 'Calibrate new motors', 'Each motor measures its encoder once before it can move. The motor page walks you through it.'))))));
}

const step = (n, title, text) => h('li', {}, h('span', { class: 'n' }, n), h('div', {}, h('b', {}, title), h('span', {}, text)));

// ---- Overview ------------------------------------------------------------------------------------------
export function renderFleet(stage, scope) {
  const motors = [...state.nodes.values()].sort((a, b) => a.id - b.id);
  const summary = h('span');
  scope.text(summary, () => {
    const moving = motors.filter(n => n.t && [2, 3, 4, 5, 6, 7].includes(n.t.state)).length;
    const faulted = motors.filter(n => n.t?.faults).length;
    return [plural(motors.length, 'motor'), moving && `${moving} moving`, faulted && `${faulted} faulted`].filter(Boolean).join(' · ');
  });
  mount(stage,
    pageHead({
      eyebrow: state.mode === 'demo' ? 'Demo bus' : 'CAN-FD bus',
      title: 'Overview',
      chips: [chip(summary)],
      actions: [
        h('button', { class: 'btn btn-secondary', type: 'button', onclick: () => store.scan() }, icon('scan'), 'Scan bus'),
        h('button', { class: 'btn btn-danger-outline', type: 'button', disabled: !motors.length, onclick: () => store.emergencyStopAll() }, icon('power'), 'Cut power to all'),
      ],
    }),
    duplicateBanner(),
    motors.length ? h('div', { class: 'fleet' }, motors.map(node => unitCard(node, scope))) : emptyBus(),
    motors.length > 1 ? syncPanel(motors, scope) : null,
  );
}

function duplicateBanner() {
  const clashes = store.duplicates();
  if (!clashes.length) return null;
  const used = new Set([...state.discovered.values()].map(info => info.nodeId));
  const free = () => {
    for (let id = 1; id <= p.MAX_NODE_ID; id++) if (!used.has(id)) return used.add(id), id;
    return null;
  };
  return h('div', { class: 'banner', dataset: { tone: 'warn' } }, icon('alert'),
    h('div', { class: 'text' },
      h('strong', {}, 'Several motors share a node ID'),
      h('p', {}, 'Their answers collide on the bus. Give each one its own ID; motors are told apart by their chip serial number.'),
      h('div', { class: 'stack' }, clashes.map(info => {
        const next = free();
        return h('div', { class: 'row' }, h('span', { class: 'mono' }, `#${info.nodeId}`), h('span', { class: 'mono muted' }, info.uid),
          next && h('button', { class: 'btn btn-secondary btn-sm', type: 'button', onclick: () => store.assignNodeId(info.uid, next) }, `Move to #${next}`));
      }))));
}

function emptyBus() {
  return panel(null, h('div', { class: 'empty' },
    icon('bus', 'empty-icon'),
    h('h2', {}, 'No motors have answered yet'),
    h('p', {}, 'Check that the motors are powered, that CAN-H and CAN-L are not swapped, and that the bus is terminated at both ends.'),
    h('button', { class: 'btn', type: 'button', onclick: () => store.scan() }, icon('scan'), 'Scan again')));
}

function unitCard(node, scope) {
  const canvas = h('canvas');
  const dial = new Dial(canvas);
  scope.frame(() => {
    const t = node.t;
    dial.draw({ turns: t?.position ?? 0, target: t?.target ?? 0, tone: status(node).tone, active: Boolean(t && t.flags & p.FLAGS.enabled), labels: dialLabels(state.unit) });
  });
  const big = h('div', { class: 'big' });
  scope.text(big, () => (node.t ? fmtPos(node.t.position) : '—'));
  const velocity = h('span', { class: 'num' });
  scope.text(velocity, () => (node.t ? fmtVel(node.t.velocity) : '—'));
  const temperature = h('span', { class: 'num' });
  scope.text(temperature, () => (node.t ? `${fixed(node.t.temperature, 1)} °C` : '—'));
  const open = () => store.select(node.id, 'control');
  return h('article', {
    class: 'panel unit-card', tabindex: '0', role: 'link', 'aria-label': `Open ${store.nameOf(node)}`,
    onclick: open, onkeydown: event => (event.key === 'Enter' || event.key === ' ') && (event.preventDefault(), open()),
  },
    h('div', { class: 'top' }, h('div', { class: 'stack-tight' }, h('strong', {}, store.nameOf(node)), h('span', { class: 'mono muted small' }, `node ${node.id}`)), statusChip(scope, node)),
    h('div', { class: 'mini' }, canvas),
    big,
    h('div', { class: 'row between small muted' }, velocity, temperature));
}

function syncPanel(motors, scope) {
  const rows = motors.map(node => {
    const ui = node.ui;
    const include = h('input', { type: 'checkbox', checked: Boolean(ui.syncInclude), 'aria-label': `Include ${store.nameOf(node)}`, dataset: { key: `sync-on-${node.id}` } });
    const target = h('input', {
      type: 'number', step: 'any', inputmode: 'decimal', value: ui.syncTarget ?? '', placeholder: 'Target',
      'aria-label': `Target for ${store.nameOf(node)}`, dataset: { key: `sync-to-${node.id}` },
    });
    include.onchange = () => (ui.syncInclude = include.checked);
    target.oninput = () => {
      ui.syncTarget = target.value;
      target.closest('.field').classList.remove('invalid');
      if (target.value.trim() !== '' && !include.checked) include.checked = ui.syncInclude = true;
    };
    const current = h('span', { class: 'num muted' });
    scope.text(current, () => (node.t ? fmtPos(node.t.position) : '—'));
    return { node, include, target, el: h('label', { class: 'sync-row' },
      include, h('span', { class: 'name' }, store.nameOf(node), h('span', { class: 'mono muted' }, ` #${node.id}`)), current,
      h('span', { class: 'field' }, target, h('span', { class: 'suffix' }, unitInfo().pos))) };
  });
  const start = () => {
    const chosen = rows.filter(r => r.include.checked);
    const empty = chosen.filter(r => r.target.value.trim() === '' || !Number.isFinite(Number(r.target.value)));
    empty.forEach(r => r.target.closest('.field').classList.add('invalid'));
    if (empty.length) return toast('Give every ticked motor a target.', 'warn');
    if (!chosen.length) return toast('Tick the motors that should move.', 'warn');
    store.syncMove(chosen.map(r => ({ id: r.node.id, turns: p.fromDisplay(Number(r.target.value), state.unit) })));
  };
  return panel('Move together', h('div', { class: 'stack' },
    h('div', { class: 'sync-list' }, rows.map(r => r.el)),
    h('div', { class: 'row between' },
      h('span', { class: 'muted small' }, 'Each motor gets its target first, then one broadcast starts them all in the same instant.'),
      h('button', { class: 'btn', type: 'button', onclick: start }, icon('play'), 'Start together'))));
}

// ---- Adapter ----------------------------------------------------------------------------------------------
const LEC = ['None', 'Stuff error', 'Form error', 'No acknowledge', 'Bit 1 error', 'Bit 0 error', 'CRC error', 'No change'];

function meter(scope, label, fn, limit) {
  const value = h('span', { class: 'num' });
  const bar = h('span');
  scope.text(value, () => fn() ?? '—');
  scope.each(() => {
    const v = fn() ?? 0;
    bar.style.width = `${Math.min(100, (v / 256) * 100)}%`;
    bar.dataset.tone = v >= limit ? 'bad' : v >= 96 ? 'warn' : 'good';
  });
  return h('div', { class: 'meter' }, h('div', { class: 'row between' }, h('span', {}, label), value), h('div', { class: 'bar' }, bar));
}

function stat(scope, label, fn, hint) {
  const value = h('div', { class: 'value' });
  scope.text(value, fn);
  return h('div', { class: 'stat', title: hint }, h('span', { class: 'eyebrow' }, label), value);
}

export function renderAdapter(stage, scope) {
  const a = state.adapter;
  const demo = state.mode === 'demo';
  const s = () => a.status;
  const busName = h('span');
  scope.swap(busName, () => (s() ? s().busState : -1), () => {
    const bus = p.BUS_STATES[s()?.busState];
    return bus ? chip([dot(bus.tone), bus.name], bus.tone) : chip('Waiting for status');
  });
  const count = n => (n === undefined ? '—' : n.toLocaleString('en-US'));
  const info = a.info;

  mount(stage,
    pageHead({
      eyebrow: 'USB to CAN-FD adapter',
      title: demo ? 'Simulated adapter' : 'Tercio FD',
      chips: [busName, info && chip(`firmware ${info.firmware}`)],
      actions: [
        h('button', { class: 'btn btn-secondary', type: 'button', onclick: () => store.resetAdapterCounters() }, icon('refresh'), 'Reset counters'),
        h('button', { class: 'btn btn-ghost', type: 'button', onclick: () => store.disconnect() }, icon('plug'), 'Disconnect'),
      ],
    }),
    h('div', { class: 'grid cols-2' },
      panel('Bus health', h('div', { class: 'stack' },
        meter(scope, 'Transmit errors', () => s()?.tec, 128),
        meter(scope, 'Receive errors', () => s()?.rec, 128),
        h('dl', { class: 'kv' },
          h('dt', {}, 'Last error'), scope.text(h('dd'), () => LEC[s()?.lastError ?? 0] ?? '—'),
          h('dt', {}, 'Bus-off events'), scope.text(h('dd'), () => count(s()?.busOffEvents)))),
      { note: 'Error counters rise on a bad bus and fall as frames get through. At 128 the adapter goes quiet; at 256 it leaves the bus. “No acknowledge” usually means nothing else is powered, or the bus is not terminated.' }),
      panel('Traffic', h('div', { class: 'stat-grid' },
        stat(scope, 'To the bus', () => `${fixed(a.rate.toCan, 0)}/s`),
        stat(scope, 'From the bus', () => `${fixed(a.rate.fromCan, 0)}/s`),
        stat(scope, 'Dropped out', () => count(s()?.droppedToCan), 'Frames the adapter could not send (bus busy or off)'),
        stat(scope, 'Dropped in', () => count(s()?.droppedFromCan), 'Frames lost because the USB side was too slow'),
        stat(scope, 'Bad USB frames', () => count(s()?.framingErrors), 'Damaged frames the adapter rejected'),
        stat(scope, 'Bad frames here', () => count(state.bus?.rejectedFrames), 'Damaged frames this page rejected'))),
      panel('Identity', info ? h('dl', { class: 'kv' },
        h('dt', {}, 'Firmware'), h('dd', {}, info.firmware),
        h('dt', {}, 'Adapter protocol'), h('dd', {}, `v${info.protocol}`),
        h('dt', {}, 'Hardware'), h('dd', {}, `revision ${info.hardwareRevision}`),
        h('dt', {}, 'Clock'), h('dd', {}, info.crystalClock ? '170 MHz from the crystal' : 'Internal oscillator (crystal not detected)'),
        h('dt', {}, 'Boot pin'), h('dd', {}, info.bootOptionsOk ? 'Ignored, as it should be' : 'Still active; fixed at the next power-up'),
        h('dt', {}, 'Serial'), h('dd', {}, h('button', { class: 'copy', type: 'button', title: 'Copy', onclick: () => copyText(info.uid) }, info.uid, icon('copy')))) :
        h('p', { class: 'muted' }, 'The adapter has not identified itself. Older adapter firmware does not support this; update it below.')),
      panel('Firmware update', h('div', { class: 'stack' },
        h('p', { class: 'muted' }, 'Restarts the adapter into the STM32 USB bootloader. Flash new firmware with STM32CubeProgrammer or dfu-util, then unplug and replug the adapter.'),
        h('div', {}, h('button', { class: 'btn btn-danger-outline', type: 'button', disabled: demo, onclick: bootloader }, icon('download'), 'Restart into update mode'))))),
  );
}

async function bootloader() {
  const ok = await confirmDialog({
    title: 'Restart the adapter into update mode?',
    text: ['The page disconnects and every motor loses its link until the adapter is back.', 'Motors that are moving keep going unless their command timeout is set.'],
    confirm: 'Restart into update mode',
  });
  if (ok) store.adapterBootloader();
}

// ---- IMU ---------------------------------------------------------------------------------------------------------
export function renderImu(stage, scope, id) {
  const entry = () => state.imus.get(id)?.imu;
  const angle = key => {
    const el = h('div', { class: 'value' });
    scope.text(el, () => (entry() ? `${fixed(entry()[key], 1)}°` : '—'));
    return el;
  };
  const horizon = h('div', { class: 'horizon-sky' });
  scope.frame(() => {
    const imu = entry();
    if (imu) horizon.style.transform = `rotate(${-imu.roll}deg) translateY(${Math.max(-45, Math.min(45, imu.pitch))}%)`;
  });
  const accel = h('dd');
  scope.text(accel, () => {
    const imu = entry();
    return imu ? `${fixed(imu.ax, 2)}, ${fixed(imu.ay, 2)}, ${fixed(imu.az, 2)} m/s²` : '—';
  });
  const temperature = h('dd');
  scope.text(temperature, () => (entry() ? `${fixed(entry().temperature, 1)} °C` : '—'));
  mount(stage,
    pageHead({
      eyebrow: 'Tercio IMU', title: 'Orientation', suffix: hexId(id),
      actions: [h('button', { class: 'btn btn-secondary', type: 'button', onclick: () => store.imuZero(id) }, icon('zero'), 'Zero orientation')],
    }),
    h('div', { class: 'grid cols-2' },
      panel(null, h('div', { class: 'horizon', 'aria-hidden': 'true' }, horizon, h('div', { class: 'horizon-mark' }))),
      panel('Readings', h('div', { class: 'stack' },
        h('div', { class: 'stat-grid' },
          h('div', { class: 'stat' }, h('span', { class: 'eyebrow' }, 'Roll'), angle('roll')),
          h('div', { class: 'stat' }, h('span', { class: 'eyebrow' }, 'Pitch'), angle('pitch')),
          h('div', { class: 'stat' }, h('span', { class: 'eyebrow' }, 'Yaw'), angle('yaw'))),
        h('dl', { class: 'kv' }, h('dt', {}, 'Acceleration'), accel, h('dt', {}, 'Temperature'), temperature)),
      { note: 'Zero orientation makes the current attitude the new reference.' })),
  );
}
