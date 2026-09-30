// -----------------------------------------------------------------------------
// Application state and every action the UI can take. Views read `state` and
// call these functions; they never talk to the bus directly.
// -----------------------------------------------------------------------------
import * as p from './protocol.js';
import { Bus, CommandError, SerialTransport } from './link.js';
import { DemoTransport } from './sim.js';
import { toast } from './ui.js';
import { History } from './widgets.js';

const load = (key, fallback) => {
  try {
    return JSON.parse(localStorage.getItem(`tercio.${key}`)) ?? fallback;
  } catch {
    return fallback;
  }
};
const save = (key, value) => {
  try {
    localStorage.setItem(`tercio.${key}`, JSON.stringify(value));
  } catch {}
};

export const state = {
  mode: 'offline',  // offline | connecting | serial | demo
  bus: null,
  adapter: { info: null, status: null, rate: { toCan: 0, fromCan: 0 }, seenAt: 0, previous: null },
  nodes: new Map(),     // node id -> motor
  imus: new Map(),      // CAN id -> { imu, seenAt }
  discovered: new Map(),  // uid -> info (latest scan)
  selected: 'fleet',    // 'fleet' | 'adapter' | node id | 'imu:<id>'
  tab: load('tab', 'control'),
  unit: load('unit', 'deg'),
  theme: load('theme', 'system'),
  names: load('names', {}),  // uid -> name given by the user
  log: [],
  knownPort: null,
};

// ---- Change notification ----------------------------------------------------------------
const listeners = new Set();
export const subscribe = fn => listeners.add(fn);
// 'structure': navigation, unit or connection changed (re-render everything)
// 'nodes': a device appeared or identified itself (detail: node id)
// 'params': a motor's settings were read or written (detail: node id)
// 'node': a motor's state, flags or faults changed (detail: node id)
export function changed(kind = 'structure', detail) {
  for (const fn of listeners) fn(kind, detail);
}

export function setUnit(unit) {
  state.unit = unit;
  save('unit', unit);
  // Move and jog fields hold numbers in the old unit; start them fresh.
  for (const node of state.nodes.values()) {
    const { series, window, runSpeed } = node.ui;
    node.ui = { series, window, runSpeed };
  }
  changed();
}

export function setTheme(theme) {
  state.theme = theme;
  save('theme', theme);
  if (theme === 'system') delete document.documentElement.dataset.theme;
  else document.documentElement.dataset.theme = theme;
  changed('theme');
}

export function select(target, tab) {
  state.selected = target;
  if (tab) {
    state.tab = tab;
    save('tab', tab);
  }
  changed();
}

export function setTab(tab) {
  state.tab = tab;
  save('tab', tab);
  changed();
}

export function log(text, level = 'info') {
  state.log.unshift({ time: new Date(), text, level });
  state.log.length = Math.min(state.log.length, 200);
}

export function nameOf(node) {
  return (node.info && state.names[node.info.uid]) || `Axis ${node.id}`;
}

export function rename(node, name) {
  if (!node.info) return;
  const trimmed = name.trim();
  if (trimmed) state.names[node.info.uid] = trimmed;
  else delete state.names[node.info.uid];
  save('names', state.names);
  changed();
}

// ---- Connection ---------------------------------------------------------------------------------
export async function refreshKnownPort() {
  state.knownPort = await SerialTransport.knownPort().catch(() => null);
  changed();
}

export async function connectSerial({ anyDevice = false, port = null } = {}) {
  return connect(new SerialTransport(port), 'serial', { anyDevice });
}

export async function connectDemo() {
  return connect(new DemoTransport(), 'demo');
}

async function connect(transport, mode, options) {
  if (state.bus) await disconnect();
  state.mode = 'connecting';
  changed();
  const bus = new Bus(transport);
  try {
    await bus.open(options);
  } catch (error) {
    state.mode = 'offline';
    changed();
    if (error?.name !== 'NotFoundError') toast(connectError(error), 'bad', 6000);  // NotFoundError: picker cancelled
    return false;
  }
  state.bus = bus;
  state.mode = mode;
  wire(bus);
  log(mode === 'demo' ? 'Demo started' : 'Adapter connected');
  changed();
  bus.adapter(p.AdapterOp.GetInfo).then(data => {
    state.adapter.info = p.parseAdapterInfo(data);
    changed();
  }).catch(() => {});
  scan();
  return true;
}

function connectError(error) {
  if (error?.name === 'NetworkError' || error?.name === 'InvalidStateError')
    return 'The port is busy. Close other apps or tabs that use the adapter, then try again.';
  if (error?.name === 'SecurityError') return 'The browser blocked access to the serial port.';
  return `Could not open the adapter: ${error?.message ?? error}`;
}

export async function disconnect() {
  const bus = state.bus;
  state.bus = null;
  state.mode = 'offline';
  state.nodes.clear();
  state.imus.clear();
  state.adapter = { info: null, status: null, rate: { toCan: 0, fromCan: 0 }, seenAt: 0, previous: null };
  state.selected = 'fleet';
  changed();
  await bus?.close().catch(() => {});
}

function wire(bus) {
  bus.addEventListener('telemetry', ({ detail }) => onTelemetry(detail.node, detail.telemetry));
  bus.addEventListener('info', ({ detail }) => onInfo(detail.node, detail.info));
  bus.addEventListener('fault', ({ detail }) => onFault(detail));
  bus.addEventListener('adapter-status', ({ detail }) => onAdapterStatus(detail));
  bus.addEventListener('imu', ({ detail }) => onImu(detail));
  bus.addEventListener('close', () => {
    if (state.bus !== bus) return;
    toast('The adapter was disconnected.', 'warn');
    log('Adapter disconnected', 'warn');
    disconnect();
  });
}

// ---- Incoming data ----------------------------------------------------------------------------------
function motor(id) {
  let node = state.nodes.get(id);
  if (!node) {
    node = { id, t: null, info: null, seenAt: 0, history: new History(), params: null, loading: false, unsaved: false,
      drafts: {}, ui: {}, lastSeq: null, frames: 0, lost: 0, rateHz: 0, rateWindow: [] };
    state.nodes.set(id, node);
    changed('nodes', id);
    state.bus?.request(id, p.Cmd.GetInfo).catch(() => {});  // the reply arrives as an 'info' event
  }
  return node;
}

const signature = t => `${t.state}|${t.flags}|${t.faults}|${t.warnings}|${t.procedureStep}`;

function onTelemetry(id, t) {
  const node = motor(id);
  const now = performance.now();
  const previous = node.t;
  node.t = t;
  node.seenAt = now;
  node.frames++;
  if (node.lastSeq !== null) node.lost += (t.sequence - node.lastSeq - 1 + 256) & 0xff;
  node.lastSeq = t.sequence;
  node.rateWindow.push(now);
  while (node.rateWindow[0] < now - 1000) node.rateWindow.shift();
  node.rateHz = node.rateWindow.length;
  node.history.push(now, { position: t.position, target: t.target, velocity: t.velocity, error: t.followingError });
  if (previous && PROCEDURES[previous.state] && t.state !== previous.state) procedureEnded(node, previous.state, t);
  if (!previous || signature(previous) !== signature(t)) changed('node', id);
}

// Calibration and auto-tune save to flash themselves; homing only moves the zero.
const PROCEDURES = {
  [p.AxisState.Calibrating]: { done: t => t.flags & p.FLAGS.calibrated, text: 'calibration finished and saved', saved: true },
  [p.AxisState.Homing]: { done: t => t.flags & p.FLAGS.homed, text: 'homed; this position is now zero', saved: false },
  [p.AxisState.Tuning]: { done: t => !t.faults, text: 'auto-tune finished; new speed limits saved', saved: true },
};
const inProcedure = node => Boolean(node?.t && PROCEDURES[node.t.state]);

// The host ended a running procedure (Stop, power off): no success message.
function markCancelled(node) {
  if (inProcedure(node)) node.cancelled = true;
}

function procedureEnded(node, state, t) {
  const cancelled = node.cancelled;
  node.cancelled = false;
  if (cancelled || !PROCEDURES[state].done(t)) return;  // failures arrive as fault events
  toast(`${nameOf(node)}: ${PROCEDURES[state].text}`, 'good', 5000);
  log(`${nameOf(node)}: ${PROCEDURES[state].text}`);
  if (!PROCEDURES[state].saved) {
    node.unsaved = true;
    changed('node', node.id);
  }
  if (node.params) loadParams(node.id);  // calibration and tuning change settings
}

function onInfo(id, info) {
  if (!info) return;
  state.discovered.set(info.uid, { ...info, nodeId: id });
  // The same board under another ID means it was re-addressed: drop the stale entry.
  let moved = false;
  for (const [otherId, other] of state.nodes) {
    if (otherId !== id && other.info?.uid === info.uid) {
      state.nodes.delete(otherId);
      if (state.selected === otherId) state.selected = id;
      moved = true;
    }
  }
  const node = motor(id);
  const first = !node.info;
  node.info = info;
  if (moved) changed();
  else if (first) changed('nodes', id);
}

function onFault({ node, faults }) {
  const names = p.FAULTS.filter(f => faults & f.bit).map(f => f.name);
  if (!names.length) return;
  const label = state.nodes.get(node) ? nameOf(state.nodes.get(node)) : `Axis ${node}`;
  toast(`${label}: ${names.join(', ')}`, 'bad', 6000);
  log(`${label}: ${names.join(', ')}`, 'bad');
}

function onAdapterStatus(status) {
  const a = state.adapter;
  const now = performance.now();
  if (a.previous) {
    const dt = (now - a.previous.at) / 1000;
    if (dt > 0.05) {
      a.rate.toCan = Math.max(0, (status.toCan - a.previous.toCan) / dt);
      a.rate.fromCan = Math.max(0, (status.fromCan - a.previous.fromCan) / dt);
    }
  }
  a.previous = { at: now, toCan: status.toCan, fromCan: status.fromCan };
  const firstOrChanged = !a.status || a.status.busState !== status.busState;
  a.status = status;
  a.seenAt = now;
  if (firstOrChanged) changed('adapter');
}

function onImu({ id, imu }) {
  const first = !state.imus.has(id);
  state.imus.set(id, { imu, seenAt: performance.now() });
  if (first) changed('nodes');
}

// ---- Commands -------------------------------------------------------------------------------------------
async function run(node, cmd, payload, { quiet = false, timeout, match } = {}) {
  if (!state.bus) return null;
  try {
    return await state.bus.request(node, cmd, payload, { timeout, match });
  } catch (error) {
    const label = state.nodes.get(node) ? nameOf(state.nodes.get(node)) : `Axis ${node}`;
    const message = error instanceof CommandError ? error.message : String(error);
    if (!quiet) toast(`${label}: ${message}`, error.status === 'timeout' ? 'warn' : 'bad', 5000);
    log(`${label}: ${message}`, 'warn');
    return null;
  }
}

const ok = data => data !== null;
export function enable(id, on) {
  if (!on) markCancelled(state.nodes.get(id));
  return run(id, p.Cmd.Enable, Uint8Array.of(on ? 1 : 0)).then(ok);
}
export function stop(id) {
  markCancelled(state.nodes.get(id));
  return run(id, p.Cmd.Stop).then(ok);
}
export function emergencyStop(id) {
  markCancelled(state.nodes.get(id));
  return run(id, p.Cmd.EmergencyStop).then(ok);
}
export const clearFaults = id => run(id, p.Cmd.ClearFaults).then(ok);
export const moveTo = (id, turns, limits) => run(id, p.Cmd.MoveTo, p.movePayload(turns, limits)).then(ok);
export const moveBy = (id, turns, limits) => run(id, p.Cmd.MoveBy, p.movePayload(turns, limits)).then(ok);
export async function setZero(id, turns = 0) {
  if (!ok(await run(id, p.Cmd.SetZero, p.f64Payload(turns)))) return false;
  const node = state.nodes.get(id);
  if (node) node.unsaved = true;  // the zero lives in the settings; Save keeps it
  changed('node', id);
  return true;
}
export const calibrate = id => run(id, p.Cmd.Calibrate).then(ok);
export const home = id => run(id, p.Cmd.Home).then(ok);
export const autoTune = (id, min, max) => run(id, p.Cmd.AutoTune, p.rangePayload(min, max)).then(ok);
export const reboot = id => run(id, p.Cmd.Reboot).then(ok);

// Hold-to-run: the first command waits for the answer (so a refusal is shown),
// the keep-alives that follow are fire-and-forget, and the release always asks.
export const runVelocity = (id, turnsPerSecond) => run(id, p.Cmd.SetVelocity, p.velocityPayload(turnsPerSecond)).then(ok);
export function keepRunning(id, turnsPerSecond) {
  state.bus?.command(id, p.Cmd.SetVelocity, p.velocityPayload(turnsPerSecond));
}
export async function release(id) {
  if (ok(await run(id, p.Cmd.SetVelocity, p.velocityPayload(0), { quiet: true }))) return true;
  return ok(await run(id, p.Cmd.Stop, undefined, { quiet: true }));  // no answer: make sure it stops
}

export async function saveConfig(id) {
  if (!ok(await run(id, p.Cmd.SaveConfig, undefined, { timeout: 1500 }))) return false;
  const node = state.nodes.get(id);
  if (node) node.unsaved = false;
  toast(`${nameOf(node)}: settings saved to the motor`);
  changed('node', id);
  return true;
}

export async function factoryReset(id) {
  const node = state.nodes.get(id);
  if (node?.t && node.t.flags & p.FLAGS.enabled) {
    toast(`${nameOf(node)}: turn the motor off first.`, 'warn');
    return false;
  }
  if (!ok(await run(id, p.Cmd.FactoryReset))) return false;
  if (node) node.drafts = {};
  await loadParams(id);
  if (node) node.unsaved = true;
  toast('Defaults restored. Save to keep them.', 'accent');
  return true;
}

// A late answer to an earlier, timed-out request must not be taken for this one.
const sameParam = def => ({ status, data }) => status !== p.Status.Ok || data[0] === def.id;

export async function loadParams(id) {
  const node = state.nodes.get(id);
  if (!node || node.loading) return;
  node.loading = true;
  changed('params', id);
  const params = {};
  for (const def of p.PARAMS) {
    const data = await run(id, p.Cmd.GetParam, Uint8Array.of(def.id), { quiet: true, match: sameParam(def) });
    if (data && data.length >= 5) params[def.key] = p.decodeParamValue(def, data, 1);
  }
  node.params = params;
  node.loading = false;
  changed('params', id);
}

// Writes one parameter; resolves with the value the motor applied, or null.
export async function setParam(id, key, value) {
  const def = p.PARAM_BY_KEY.get(key);
  const data = await run(id, p.Cmd.SetParam, Uint8Array.of(def.id, ...p.encodeParamValue(def, value)), { match: sameParam(def) });
  if (!data) return null;
  const applied = p.decodeParamValue(def, data, 1);
  const node = state.nodes.get(id);
  if (key === 'nodeId' && applied !== id) return moveNode(node, applied);
  if (node?.params) node.params[key] = applied;
  if (node) node.unsaved = true;
  changed('params', id);
  return applied;
}

function moveNode(node, newId) {
  const oldId = node.id;
  state.nodes.delete(oldId);
  node.id = newId;
  node.params.nodeId = newId;
  node.unsaved = true;
  state.nodes.set(newId, node);
  if (state.selected === oldId) state.selected = newId;
  toast(`Now node ${newId}. Save to keep it after a restart.`, 'accent');
  changed();
  return newId;
}

// ---- Bus-wide -------------------------------------------------------------------------------------------
export function stopAll() {
  state.nodes.forEach(markCancelled);
  state.bus?.broadcast(p.Cmd.Stop);
  toast('Stopping every motor', 'warn', 2000);
  log('Stop all', 'warn');
}

export function emergencyStopAll() {
  state.nodes.forEach(markCancelled);
  state.bus?.broadcast(p.Cmd.EmergencyStop);
  toast('Motor power cut on every axis', 'bad');
  log('Emergency stop, all axes', 'bad');
}

export async function scan() {
  if (!state.bus) return [];
  state.discovered.clear();
  state.bus.broadcast(p.Cmd.GetInfo);
  await new Promise(resolve => setTimeout(resolve, 180));
  changed('nodes');
  return [...state.discovered.values()];
}

// Node IDs claimed by more than one board (their frames collide on the bus).
export function duplicates() {
  const byId = new Map();
  for (const info of state.discovered.values()) byId.set(info.nodeId, [...(byId.get(info.nodeId) ?? []), info]);
  return [...byId.values()].filter(list => list.length > 1).flat();
}

export async function assignNodeId(uid, newId) {
  state.bus?.broadcast(p.Cmd.AssignNodeId, Uint8Array.of(...p.uidBytes(uid), newId));
  await new Promise(resolve => setTimeout(resolve, 100));
  await scan();
  toast(`Assigned node ${newId}. Open it and save to keep the ID.`, 'accent');
}

// Queue a move on every listed axis, then start them together with one broadcast.
export async function syncMove(targets) {
  const results = await Promise.all(targets.map(({ id, turns }) => run(id, p.Cmd.MoveTo, p.movePayload(turns, { deferred: true }))));
  if (!results.every(ok)) {
    // Someone refused: drop the moves already queued so a later Sync cannot start them.
    targets.forEach(({ id }, i) => ok(results[i]) && run(id, p.Cmd.Stop, undefined, { quiet: true }));
    return false;
  }
  state.bus?.broadcast(p.Cmd.Sync);
  toast(`Moving ${targets.length === 1 ? '1 axis' : `${targets.length} axes`} together`, 'accent', 2500);
  return true;
}

// ---- Adapter ------------------------------------------------------------------------------------------------
export async function resetAdapterCounters() {
  await state.bus?.adapter(p.AdapterOp.ResetCounters).catch(() => null);
  state.adapter.previous = null;
}

export async function adapterBootloader() {
  await state.bus?.adapter(p.AdapterOp.EnterBootloader).catch(() => null);
  toast('The adapter restarted into its USB update mode.', 'accent', 8000);
  await disconnect();
}

export function imuZero(id) {
  state.bus?.send(id, p.IMU_RESET_ORIENTATION);
}
