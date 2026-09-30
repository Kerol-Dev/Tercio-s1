// -----------------------------------------------------------------------------
// Settings and Diagnostics tabs. Edits are drafts (kept per motor across
// re-renders) until Apply writes them; Save stores the applied set in flash.
// -----------------------------------------------------------------------------
import * as p from './protocol.js';
import * as store from './store.js';
import { state } from './store.js';
import { h, icon, confirmDialog, fixed, toast, copyText } from './ui.js';
import { chip, isStale, panel, plural, unitInfo } from './views.js';

const SCALED = new Set(['pos', 'vel', 'acc']);

export const unitOf = def => (SCALED.has(def.kind) ? unitInfo()[def.kind] : def.unit ?? '');

// Wire value -> what the control shows.
export function toField(def, value) {
  if (def.type === 'bool') return Boolean(value);
  if (def.kind === 'select') return Number(value);
  if (value === undefined || value === null) return '';
  const shown = SCALED.has(def.kind) ? p.toDisplay(value, state.unit) : value;
  return def.type === 'f32' ? String(+shown.toPrecision(6)) : String(shown);
}

// Control value -> wire value, or null when it is not acceptable.
export function fromField(def, raw) {
  if (def.type === 'bool') return Boolean(raw);
  if (def.kind === 'select') return Number(raw);
  if (String(raw).trim() === '') return null;
  let value = Number(raw);
  if (!Number.isFinite(value)) return null;
  if (SCALED.has(def.kind)) value = p.fromDisplay(value, state.unit);
  if (def.type !== 'f32' && !Number.isInteger(value)) return null;
  const slack = Math.abs(def.max ?? 0) * 1e-6;
  if (def.min !== undefined && (value < def.min - slack || value > def.max + slack)) return null;
  return value;
}

// Drafts keep the unit they were typed in, so switching units never
// reinterprets a number.
export function draftRaw(node, def) {
  const draft = node.drafts[def.key];
  if (draft === undefined) return undefined;
  if (!SCALED.has(def.kind) || draft.unit === state.unit) return draft.raw;
  const value = Number(draft.raw);
  if (String(draft.raw).trim() === '' || !Number.isFinite(value)) return draft.raw;
  return String(+p.toDisplay(p.fromDisplay(value, draft.unit), state.unit).toPrecision(6));
}

const setDraft = (node, def, raw) => (node.drafts[def.key] = { raw, unit: state.unit });

function lockReason(node, def) {
  const t = node.t;
  if (!t) return '';
  const enabled = t.flags & p.FLAGS.enabled;
  if (def.access === 'disabled' && enabled) return 'Turn the motor off to change this.';
  if (def.access === 'still' && enabled && t.state !== p.AxisState.Holding) return 'Stop the motor to change this.';
  return '';
}

export function settingRow(node, def, scope) {
  const current = node.params?.[def.key];
  const id = `setting-${def.key}`;
  const draft = draftRaw(node, def);
  let field = null;
  let row;
  const mark = raw => {
    if (String(raw) === String(toField(def, current))) delete node.drafts[def.key];
    else setDraft(node, def, raw);
    const valid = fromField(def, raw) !== null;
    row.classList.toggle('changed', def.key in node.drafts);
    field?.classList.toggle('changed', def.key in node.drafts && valid);
    field?.classList.toggle('invalid', !valid);
  };

  let control;
  if (def.access === 'readonly') {
    control = chip(current ? 'Yes' : 'No', current ? 'good' : 'warn');
  } else if (def.type === 'bool') {
    const input = h('input', { type: 'checkbox', role: 'switch', id, checked: Boolean(draft ?? current) });
    input.onchange = () => mark(input.checked);
    control = h('label', { class: 'switch' }, input, h('span', { class: 'track' }));
  } else if (def.kind === 'select') {
    const value = draft ?? current;
    const select = h('select', { id }, def.options.map(o => h('option', { value: o.value, selected: o.value === value }, o.label)));
    select.onchange = () => mark(Number(select.value));
    control = field = h('span', { class: 'field' }, select);
  } else {
    const input = h('input', { id, type: 'number', step: 'any', inputmode: 'decimal', value: draft ?? toField(def, current) });
    input.oninput = () => mark(input.value);
    const suffix = unitOf(def);
    control = field = h('span', { class: 'field' }, input, suffix && h('span', { class: 'suffix' }, suffix));
  }

  const lock = h('span', { class: 'lock' });
  scope.text(lock, () => lockReason(node, def));
  row = h('div', { class: 'setting' },
    h('label', { for: id }, def.label),
    h('div', { class: 'control' }, control),
    h('span', { class: 'help' }, def.help),
    lock);
  if (draft !== undefined) mark(draft);
  return row;
}

// Writes the drafts one by one (the node ID last: the motor answers on its new ID after it).
export async function applyDrafts(node, keys = Object.keys(node.drafts)) {
  const defs = keys.map(key => p.PARAM_BY_KEY.get(key)).filter(Boolean)
    .sort((a, b) => (a.key === 'nodeId') - (b.key === 'nodeId'));
  const invalid = defs.filter(def => fromField(def, draftRaw(node, def)) === null);
  if (invalid.length) {
    toast(`Check ${invalid.map(def => def.label.toLowerCase()).join(', ')}: out of range.`, 'warn');
    return false;
  }
  if ('nodeId' in node.drafts) {
    const wanted = fromField(p.PARAM_BY_KEY.get('nodeId'), draftRaw(node, p.PARAM_BY_KEY.get('nodeId')));
    const holder = state.nodes.get(wanted);
    if (holder && holder !== node) {
      toast(`Node ${wanted} is taken by ${store.nameOf(holder)}. Pick a free ID.`, 'warn');
      return false;
    }
  }
  let applied = 0;
  for (const def of defs) {
    if ((await store.setParam(node.id, def.key, fromField(def, draftRaw(node, def)))) === null) break;
    delete node.drafts[def.key];
    applied++;
  }
  if (applied) toast(`${plural(applied, 'setting')} applied. Save to keep ${applied === 1 ? 'it' : 'them'} after a restart.`, 'accent');
  store.changed('params', node.id);
  return applied === defs.length;
}

// ---- Settings tab -------------------------------------------------------------------------------
export function renderSettings(node, scope) {
  if (!node.params) {
    if (!node.loading) store.loadParams(node.id);
    return panel(null, h('div', { class: 'empty' }, h('span', { class: 'spinner', 'aria-hidden': 'true' }), h('p', {}, 'Reading settings from the motor…')));
  }

  const drafts = () => Object.keys(node.drafts).length;
  const statusText = h('span');
  const statusDot = h('span', { class: 'dot' });
  scope.each(() => {
    const n = drafts();
    const text = n ? `${plural(n, 'change')} not applied` : node.unsaved ? 'Applied, not saved to the motor' : 'Saved on the motor';
    if (statusText.textContent !== text) statusText.textContent = text;
    statusDot.dataset.tone = n || node.unsaved ? 'warn' : 'good';
  });
  const apply = h('button', { class: 'btn', type: 'button', onclick: () => applyDrafts(node) }, icon('check'), 'Apply');
  const discard = h('button', { class: 'btn btn-ghost', type: 'button', onclick: () => { node.drafts = {}; store.changed('params', node.id); } }, 'Discard');
  const save = h('button', { class: 'btn btn-secondary', type: 'button', onclick: () => store.saveConfig(node.id) }, icon('save'), 'Save to motor');
  scope.each(() => {
    apply.disabled = !drafts();
    discard.hidden = !drafts();
    save.disabled = !node.unsaved || drafts() > 0;
  });

  const file = h('input', { type: 'file', accept: 'application/json,.json', hidden: true, onchange: () => importFile(node, file) });
  const toolbar = h('div', { class: 'toolbar' },
    h('span', { class: 'toolbar-status' }, statusDot, statusText),
    h('span', { class: 'topbar-spacer' }),
    discard, apply, save,
    h('span', { class: 'toolbar-sep' }),
    h('button', { class: 'icon-btn', type: 'button', title: 'Read again from the motor', 'aria-label': 'Reload settings', onclick: () => { node.params = null; store.loadParams(node.id); } }, icon('refresh')),
    h('button', { class: 'icon-btn', type: 'button', title: 'Export to a file', 'aria-label': 'Export settings', onclick: () => exportFile(node) }, icon('download')),
    h('button', { class: 'icon-btn', type: 'button', title: 'Import from a file', 'aria-label': 'Import settings', onclick: () => file.click() }, icon('upload')),
    file,
    h('button', { class: 'btn btn-ghost', type: 'button', onclick: () => reboot(node) }, 'Restart'),
    h('button', { class: 'btn btn-danger-outline', type: 'button', onclick: () => factoryReset(node) }, 'Factory reset'));

  return h('div', { class: 'stack stack-lg' },
    toolbar,
    h('div', { class: 'settings' }, p.PARAM_GROUPS.map(group => panel(group.title,
      p.PARAMS.filter(def => def.group === group.key).map(def => settingRow(node, def, scope)),
      { note: group.blurb }))));
}

function exportFile(node) {
  const data = {
    format: 'tercio-s1-settings', version: 1, exported: new Date().toISOString(),
    name: store.nameOf(node), node: node.id, uid: node.info?.uid, firmware: node.info?.firmware,
    units: 'turns, turns/s, turns/s²', params: node.params,
  };
  const url = URL.createObjectURL(new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' }));
  const slug = store.nameOf(node).toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '') || `node-${node.id}`;
  h('a', { href: url, download: `${slug}-settings.json` }).click();
  setTimeout(() => URL.revokeObjectURL(url), 2000);
}

async function importFile(node, input) {
  const [fileObj] = input.files;
  input.value = '';
  if (!fileObj) return;
  let data;
  try {
    data = JSON.parse(await fileObj.text());
  } catch {
    return toast('That file is not valid JSON.', 'bad');
  }
  if (data?.format !== 'tercio-s1-settings' || typeof data.params !== 'object') return toast('That is not a Tercio S1 settings file.', 'bad');
  // The encoder direction is measured per motor by calibration: only take it
  // back from this same motor's file.
  const sameMotor = Boolean(data.uid && data.uid === node.info?.uid);
  let count = 0;
  for (const def of p.PARAMS) {
    if (def.access === 'readonly' || def.key === 'nodeId' || !(def.key in data.params)) continue;
    if (def.key === 'encoderInvert' && !sameMotor) continue;
    const raw = toField(def, data.params[def.key]);
    if (String(raw) === String(toField(def, node.params[def.key]))) continue;
    setDraft(node, def, raw);
    count++;
  }
  toast(count ? `${plural(count, 'setting')} differ from the motor. Review, then Apply.` : 'The file matches the motor already.', count ? 'accent' : 'good');
  store.changed('params', node.id);
}

async function reboot(node) {
  const ok = await confirmDialog({
    title: `Restart ${store.nameOf(node)}?`,
    text: ['The motor stops holding for a moment while it restarts. Settings that were applied but not saved are lost.'],
    confirm: 'Restart',
  });
  if (!ok || !(await store.reboot(node.id))) return;
  node.drafts = {};
  node.unsaved = false;
  setTimeout(() => {
    node.params = null;
    store.loadParams(node.id);
  }, 1200);
  toast('Restarting…', 'accent', 2000);
}

async function factoryReset(node) {
  const ok = await confirmDialog({
    title: 'Restore factory settings?',
    text: ['Every setting returns to its default, including the calibration and homing setup. The node ID stays.',
      'The new values apply at once; Save keeps them after a restart. The motor must be off.'],
    confirm: 'Restore defaults',
  });
  if (ok) store.factoryReset(node.id);
}

// ---- Diagnostics tab ----------------------------------------------------------------------------------
export function renderDiagnostics(node, scope) {
  const info = node.info;
  const t = () => node.t;
  const live = (fn, cls) => scope.text(h('dd', { class: cls }), fn);
  const warningList = h('div', { class: 'stack tight' });
  scope.swap(warningList, () => t()?.warnings ?? -1, () => {
    const active = p.WARNINGS.filter(w => (t()?.warnings ?? 0) & w.bit);
    return active.length ? h('div', { class: 'flags' }, active.map(w => chip(w.name, 'warn'))) : h('p', { class: 'muted' }, 'No warnings.');
  });
  const loss = () => (node.frames ? (100 * node.lost) / (node.frames + node.lost) : 0);

  return h('div', { class: 'grid cols-2' },
    panel('Identity', info ? h('dl', { class: 'kv' },
      h('dt', {}, 'Node ID'), h('dd', {}, node.id),
      h('dt', {}, 'Firmware'), h('dd', {}, info.firmware),
      h('dt', {}, 'Protocol'), h('dd', {}, `v${info.protocol}`),
      h('dt', {}, 'Hardware'), h('dd', {}, `revision ${info.hardwareRevision}`),
      h('dt', {}, 'Encoder'), h('dd', {}, p.ENCODER_TYPES[info.encoderType] ?? `type ${info.encoderType}`),
      h('dt', {}, 'Clock'), h('dd', {}, info.crystalClock ? '170 MHz from the 16 MHz crystal' : 'Internal oscillator: crystal not detected'),
      h('dt', {}, 'Serial'), h('dd', {}, h('button', { class: 'copy', type: 'button', title: 'Copy', onclick: () => copyText(info.uid) }, info.uid, icon('copy')))) :
      h('p', { class: 'muted' }, 'The motor has not identified itself yet.')),
    panel('Telemetry link', h('dl', { class: 'kv' },
      h('dt', {}, 'Rate'), live(() => `${node.rateHz} frames/s`),
      h('dt', {}, 'Received'), live(() => node.frames.toLocaleString('en-US')),
      h('dt', {}, 'Lost'), live(() => `${node.lost.toLocaleString('en-US')} (${fixed(loss(), 2)} %)`),
      h('dt', {}, 'Last frame'), live(() => (node.t ? (isStale(node) ? `${fixed((performance.now() - node.seenAt) / 1000, 0)} s ago` : 'just now') : 'never'))),
    { note: 'Lost frames are counted from gaps in the sequence number. A few per thousand is normal on a busy bus.' }),
    panel('Health', h('div', { class: 'stack' },
      warningList,
      h('dl', { class: 'kv' },
        h('dt', {}, 'Board temperature'), live(() => (t() ? `${fixed(t().temperature, 1)} °C` : '—')),
        h('dt', {}, 'Supply'), live(() => (t() ? `${fixed(t().supply, 2)} V` : '—')),
        h('dt', {}, 'Control loop'), live(() => (t() ? `${t().load} % of the 50 µs budget at peak` : '—'))))),
    panel('Activity', h('div', { class: 'log' }, logLines()), {
      actions: h('button', { class: 'btn btn-ghost btn-sm', type: 'button', onclick: () => copyText(report(node)) }, icon('copy'), 'Copy report'),
    }));
}

function logLines() {
  if (!state.log.length) return h('p', { class: 'muted' }, 'Nothing yet.');
  return state.log.slice(0, 60).map(entry => h('div', {},
    h('time', {}, entry.time.toLocaleTimeString('en-GB')), h('span', { class: entry.level }, entry.text)));
}

// Plain-text summary to paste into an issue or a message.
function report(node) {
  const t = node.t;
  return JSON.stringify({
    name: store.nameOf(node), node: node.id, info: node.info,
    telemetry: t && { ...t, faults: p.FAULTS.filter(f => t.faults & f.bit).map(f => f.name), warnings: p.WARNINGS.filter(w => t.warnings & w.bit).map(w => w.name) },
    link: { rateHz: node.rateHz, frames: node.frames, lost: node.lost },
    params: node.params, userAgent: navigator.userAgent,
  }, null, 2);
}
