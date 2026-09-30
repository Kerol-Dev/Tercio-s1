// -----------------------------------------------------------------------------
// Demo mode: a simulated Tercio FD adapter with three Tercio S1 motors and a
// Tercio IMU on its bus. It speaks the real framed protocol through the same
// Bus as a USB connection, and follows the firmware's rules: the same state
// machine, the same refusal statuses, trapezoidal moves and the procedures'
// real step sequences. Nothing here is used when real hardware is connected.
// -----------------------------------------------------------------------------
import * as p from './protocol.js';

const TICK_MS = 10;
const TICK = TICK_MS / 1000;
const CPR = 4096;

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const u8 = (...values) => Uint8Array.from(values);

function f32Bytes(values) {
  const b = new DataView(new ArrayBuffer(values.length * 4));
  values.forEach((v, i) => b.setFloat32(i * 4, v, true));
  return new Uint8Array(b.buffer);
}

class Trajectory {
  constructor() {
    this.reset(0);
  }
  reset(pos) {
    Object.assign(this, { pos, vel: 0, target: pos, mode: 'pos', vTarget: 0, vmax: 1, amax: 1, done: true });
  }
  moveTo(target, vmax, amax) {
    Object.assign(this, { target, vmax, amax, mode: 'pos', done: false });
  }
  runAt(v, amax) {
    Object.assign(this, { vTarget: v, amax, mode: 'vel', done: false });
  }
  step(dt) {
    const dv = this.amax * dt;
    if (this.mode === 'vel') {
      this.vel += clamp(this.vTarget - this.vel, -dv, dv);
      this.pos += this.vel * dt;
      this.done = this.vel === this.vTarget;
      if (this.done && this.vTarget === 0) this.reset(this.pos);
      return;
    }
    if (this.done) return;
    const d = this.target - this.pos;
    if (Math.abs(d) <= Math.abs(this.vel) * dt + 1e-9 && Math.abs(this.vel) <= dv * 1.5) {
      this.reset(this.target);
      return;
    }
    const brake = Math.sqrt(2 * 0.98 * this.amax * Math.abs(d));
    const desired = Math.sign(d) * Math.min(this.vmax, brake);
    this.vel += clamp(desired - this.vel, -dv, dv);
    this.pos += this.vel * dt;
  }
}

class SimMotor {
  constructor(bus, { id, uid, calibrated, enabled, position = 0, temperature = 31, warnings = 0 }) {
    this.bus = bus;
    this.uid = uid;
    this.params = { ...p.PARAM_DEFAULTS, nodeId: id, calibrated };
    this.id = id;
    this.enabled = enabled && calibrated;
    this.homed = false;
    this.faults = 0;
    this.baseWarnings = warnings;
    this.procedure = null;
    this.trajectory = new Trajectory();
    this.trajectory.reset(position);
    this.actual = position;
    this.velocity = 0;
    this.temperature = temperature;
    this.sequence = 0;
    this.nextTelemetry = 0;
    this.pending = null;
  }

  get calibrated() {
    return this.params.calibrated;
  }

  state() {
    if (this.faults & 0x1f) return p.AxisState.Fault;
    if (!this.enabled) return p.AxisState.Disabled;
    if (this.procedure) return { calibrate: 5, home: 6, tune: 7 }[this.procedure.type];
    if (this.trajectory.mode === 'vel') return p.AxisState.Velocity;
    if (!this.trajectory.done) return p.AxisState.Moving;
    return p.AxisState.Holding;
  }

  motionAllowed() {
    if (this.faults & 0x1f) return p.Status.Faulted;
    if (!this.enabled) return p.Status.Disabled;
    if (!this.calibrated) return p.Status.NotCalibrated;
    if (this.procedure || this.params.stepDirMode) return p.Status.Busy;
    return p.Status.Ok;
  }

  still() {
    return !this.procedure && (!this.enabled || (this.trajectory.done && this.trajectory.mode === 'pos'));
  }

  limits(v, a) {
    return [v > 0 ? Math.min(v, this.params.maxVelocity) : this.params.maxVelocity,
      a > 0 ? Math.min(a, this.params.maxAcceleration) : this.params.maxAcceleration];
  }

  // ---- commands --------------------------------------------------------------------
  handle(cmd, payload) {
    const P = p.Cmd;
    const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
    const clearSoft = () => (this.faults &= ~((1 << 5) | (1 << 8)));
    switch (cmd) {
      case P.GetInfo:
        return [p.Status.Ok, this.info()];
      case P.SaveConfig:
        return [this.still() ? p.Status.Ok : p.Status.Busy];
      case P.FactoryReset:
        if (this.enabled) return [p.Status.Busy];
        this.params = { ...p.PARAM_DEFAULTS, nodeId: this.id };
        return [p.Status.Ok];
      case P.Reboot:
        setTimeout(() => {
          this.trajectory.reset(this.actual);
          this.enabled = this.params.enableOnBoot && this.calibrated;
          this.homed = false;
        }, 300);
        return [p.Status.Ok];
      case P.ClearFaults:
        this.faults = 0;
        return [p.Status.Ok];
      case P.GetParam: {
        const def = p.PARAM_BY_ID.get(payload[0]);
        if (!def) return [p.Status.UnknownParam];
        return [p.Status.Ok, u8(def.id, ...p.encodeParamValue(def, this.params[def.key]))];
      }
      case P.SetParam: {
        const def = p.PARAM_BY_ID.get(payload[0]);
        if (!def) return [p.Status.UnknownParam];
        if (payload.length < 5) return [p.Status.BadLength];
        if (def.access === 'readonly') return [p.Status.ReadOnly];
        if (def.access === 'disabled' && this.enabled) return [p.Status.Busy];
        if (def.access === 'still' && !this.still()) return [p.Status.Busy];
        const value = p.decodeParamValue(def, payload, 1);
        if (def.options && !def.options.some(o => o.value === value)) return [p.Status.BadValue];
        if (def.min !== undefined && (!Number.isFinite(value) || value < def.min || value > def.max)) return [p.Status.BadValue];
        this.params[def.key] = def.type === 'f32' ? value : value;
        if (def.key === 'encoderType') this.params.calibrated = false;
        if (def.key === 'nodeId') this.pending = () => (this.id = value);
        return [p.Status.Ok, u8(def.id, ...p.encodeParamValue(def, this.params[def.key]))];
      }
      case P.Enable: {
        if (payload.length < 1) return [p.Status.BadLength];
        if (!payload[0]) {
          this.enabled = false;
          this.procedure = null;
          this.deferred = null;
          this.trajectory.reset(this.actual);
          return [p.Status.Ok];
        }
        if (this.faults & 0x1f) return [p.Status.Faulted];
        if (!this.enabled) {
          this.enabled = true;
          this.trajectory.reset(this.actual);
        }
        return [p.Status.Ok];
      }
      case P.Stop:
        this.procedure = null;
        this.deferred = null;
        if (this.enabled) this.trajectory.runAt(0, this.params.maxAcceleration);
        return [p.Status.Ok];
      case P.EmergencyStop:
        this.enabled = false;
        this.procedure = null;
        this.deferred = null;
        this.trajectory.reset(this.actual);
        return [p.Status.Ok];
      case P.MoveTo:
      case P.MoveBy: {
        if (payload.length < 8) return [p.Status.BadLength];
        const status = this.motionAllowed();
        if (status) return [status];
        clearSoft();
        const value = view.getFloat64(0, true);
        const [v, a] = this.limits(payload.length >= 13 ? view.getFloat32(9, true) : 0, payload.length >= 17 ? view.getFloat32(13, true) : 0);
        const t = this.trajectory;
        const base = t.mode === 'pos' ? t.target : t.pos;
        let target = cmd === P.MoveTo ? value : base + value;
        const { softLimitMin: lo, softLimitMax: hi } = this.params;
        if (lo < hi) target = clamp(target, lo, hi);
        if (payload.length >= 9 && payload[8] & p.MOVE_DEFERRED) this.deferred = [target, v, a];
        else t.moveTo(target, v, a);
        return [p.Status.Ok];
      }
      case P.SetVelocity: {
        if (payload.length < 4) return [p.Status.BadLength];
        const status = this.motionAllowed();
        if (status) return [status];
        clearSoft();
        const vel = clamp(view.getFloat32(0, true), -this.params.maxVelocity, this.params.maxVelocity);
        const [, a] = this.limits(0, payload.length >= 8 ? view.getFloat32(4, true) : 0);
        this.deferred = null;
        this.trajectory.runAt(vel, a);
        return [p.Status.Ok];
      }
      case P.SetZero: {
        if (!this.still()) return [p.Status.Busy];
        const value = payload.length >= 8 ? view.getFloat64(0, true) : 0;
        const shift = value - this.actual;
        this.actual += shift;
        this.trajectory.reset(this.actual);
        return [p.Status.Ok];
      }
      case P.Sync:
        this.sync();
        return [p.Status.Ok];
      case P.Calibrate:
        if (this.faults & 0x1f) return [p.Status.Faulted];
        if (this.procedure) return [p.Status.Busy];
        this.enabled = true;
        this.startProcedure('calibrate');
        return [p.Status.Ok];
      case P.Home: {
        const status = this.motionAllowed();
        if (status) return [status];
        this.startProcedure('home');
        return [p.Status.Ok];
      }
      case P.AutoTune: {
        if (payload.length < 8) return [p.Status.BadLength];
        const min = view.getFloat32(0, true);
        const max = view.getFloat32(4, true);
        if (!(max - min >= 25 / 30)) return [p.Status.BadValue];
        const status = this.motionAllowed();
        if (status) return [status];
        this.startProcedure('tune', { min, max });
        return [p.Status.Ok];
      }
      default:
        return [p.Status.UnknownCommand];
    }
  }

  sync() {
    if (!this.deferred || this.motionAllowed()) return;
    this.trajectory.moveTo(...this.deferred);
    this.deferred = null;
  }

  info() {
    const uid = p.uidBytes(this.uid);
    return u8(2, 2, 0, 0, 1, this.params.encoderType, this.id, 1, ...uid);
  }

  // ---- procedures ------------------------------------------------------------------
  startProcedure(type, extra = {}) {
    this.faults &= ~((1 << 5) | (1 << 6) | (1 << 7) | (1 << 8));
    this.procedure = { type, step: 0, t: 0, ...extra };
    if (type === 'home') this.homed = false;
    if (type === 'tune') {
      Object.assign(this.procedure, { level: 0, verifying: false, passes: 0, phase: 'start' });
      this.trajectory.moveTo(extra.min, 5, 30);
    }
  }

  stepProcedure(dt) {
    const pr = this.procedure;
    const t = this.trajectory;
    pr.t += dt;
    if (pr.type === 'calibrate') {
      const phases = [0.3, 0.8, 0.3, 0.8, 0.3];
      if (pr.t < phases[pr.step]) return;
      pr.t = 0;
      if (pr.step === 0) t.moveTo(t.pos + 0.25, 0.5, 5);
      if (pr.step === 2) t.moveTo(t.pos - 0.25, 0.5, 5);
      if (++pr.step >= phases.length) {
        this.params.calibrated = true;
        this.shiftZero(this.actual);
        this.procedure = null;
      }
    } else if (pr.type === 'home') {
      const s = this.params;
      const dir = s.homingMode === 1 || s.homingMode === 3 ? 1 : -1;
      if (pr.step === 0) {
        if (pr.t === dt) t.runAt(dir * s.homingVelocity, s.maxAcceleration);
        if (pr.t > 1.4) {
          t.reset(this.actual);
          pr.step = 1;
          pr.t = 0;
        }
      } else if (pr.step === 1 && pr.t > 0.1) {
        t.moveTo(this.actual - dir * s.homingBackoff, s.homingVelocity, s.maxAcceleration);
        pr.step = 2;
        pr.t = 0;
      } else if (pr.step === 2 && t.done && pr.t > 0.2) {
        this.shiftZero(this.actual);
        this.homed = true;
        this.procedure = null;
      }
    } else if (pr.type === 'tune') {
      if (!t.done || pr.t < 0.05) return;
      pr.t = 0;
      const velocity = 5 + 3 * pr.level;
      const accel = 30 + 8 * pr.level;
      if (pr.phase === 'start' || pr.phase === 'back') {
        if (pr.phase === 'back') {
          if (pr.verifying && ++pr.passes >= 3) return this.finishTune(velocity, accel);
          if (!pr.verifying) {
            if (pr.level >= 9) {  // this motor slips above level 9
              pr.level--;
              pr.verifying = true;
              pr.passes = 0;
            } else pr.level++;
          }
        }
        t.moveTo(pr.max, 5 + 3 * pr.level, 30 + 8 * pr.level);
        pr.phase = 'forward';
      } else {
        t.moveTo(pr.min, velocity, accel);
        pr.phase = 'back';
      }
    }
  }

  finishTune(velocity, accel) {
    this.params.maxVelocity = velocity;
    this.params.maxAcceleration = accel;
    this.procedure = null;
  }

  shiftZero(position) {
    this.actual -= position;
    this.trajectory.reset(this.actual);
  }

  // ---- physics ---------------------------------------------------------------------
  step(now) {
    const dt = TICK;
    if (this.procedure) this.stepProcedure(dt);
    if (this.enabled) this.trajectory.step(dt);
    else this.trajectory.reset(this.actual);

    const before = this.actual;
    const lag = this.enabled ? 1 - Math.exp(-dt / 0.006) : 0;
    this.actual += (this.trajectory.pos - this.actual) * lag;
    this.velocity += ((this.actual - before) / dt - this.velocity) * 0.3;

    const load = this.enabled ? 0.35 + Math.abs(this.velocity) * 0.02 : 0.04;
    this.temperature += (28 + load * 22 - this.temperature) * 0.0006;

    if (now < this.nextTelemetry || !this.params.telemetryRateHz) return;
    this.nextTelemetry = now + 1000 / this.params.telemetryRateHz;
    this.sendTelemetry();
  }

  flags() {
    const f = p.FLAGS;
    const t = this.trajectory;
    const error = t.pos - this.actual;
    let flags = 0;
    if (this.enabled) flags |= f.enabled;
    if (this.calibrated) flags |= f.calibrated;
    if (this.homed) flags |= f.homed;
    if (t.done && t.mode === 'pos' && Math.abs(error) < this.params.positionDeadband * 2) flags |= f.settled;
    if (this.deferred) flags |= f.movePending;
    return flags;
  }

  sendTelemetry() {
    const quantized = Math.round(this.actual * CPR) / CPR;
    const t = this.trajectory;
    const noise = (Math.random() - 0.5) * 0.4;
    const b = new DataView(new ArrayBuffer(1 + p.TELEMETRY_SIZE));
    b.setUint8(0, p.FRAME_TELEMETRY);
    b.setUint8(1, this.state());
    b.setUint8(2, this.flags());
    b.setUint16(3, this.faults, true);
    b.setUint16(5, this.baseWarnings, true);
    b.setFloat64(7, quantized, true);
    b.setFloat64(15, t.mode === 'pos' ? t.target : t.pos, true);
    b.setFloat32(23, this.velocity, true);
    b.setFloat32(27, t.pos - quantized, true);
    b.setInt16(31, Math.round(this.temperature * 10), true);
    b.setUint16(33, Math.round((24.1 + noise * 0.05) * 1000), true);
    b.setUint8(35, this.procedure ? (this.procedure.type === 'tune' ? this.procedure.level : this.procedure.step) : 0);
    b.setUint8(36, this.sequence++ & 0xff);
    b.setUint8(37, 11 + Math.round(Math.random() * 3));
    const bytes = new Uint8Array(b.buffer);
    this.bus.transmit(p.FN_TELEMETRY + this.id, bytes[0], bytes.subarray(1));
  }
}

class SimImu {
  constructor(bus, id) {
    this.bus = bus;
    this.id = id;
    this.offset = [0, 0, 0];
    this.next = 0;
  }
  handle(opcode) {
    if (opcode === p.IMU_RESET_ORIENTATION) this.offset = this.angles(performance.now());
  }
  angles(now) {
    const s = now / 1000;
    return [12 * Math.sin(s * 0.7), 7 * Math.sin(s * 0.45 + 1), (s * 9) % 360];
  }
  step(now) {
    if (now < this.next) return;
    this.next = now + 40;
    const [r, pi, y] = this.angles(now).map((a, i) => a - this.offset[i]);
    this.bus.transmit(this.id, 0x02, f32Bytes([r, pi, y, 0.01, -0.02, 9.81, 29.5]));
  }
}

// ---- The simulated bus + adapter -------------------------------------------------------
class Simulator {
  constructor(deliver) {
    this.deliver = deliver;
    this.motors = [
      new SimMotor(this, { id: 1, uid: '3a0f1c2d48ab1e7700113355', calibrated: true, enabled: true, position: 0.125 }),
      new SimMotor(this, { id: 2, uid: '3a0f1c2d48ab1e7700224466', calibrated: true, enabled: false, position: -1.5, temperature: 38 }),
      new SimMotor(this, { id: 3, uid: '3a0f1c2d48ab1e7700335577', calibrated: false, enabled: false, warnings: 1 << 2 }),
    ];
    this.imu = new SimImu(this, 0x003);
    this.counters = { toCan: 0, fromCan: 0 };
    this.nextStatus = 0;
  }

  start() {
    this.timer = setInterval(() => this.step(), TICK_MS);
  }

  stop() {
    clearInterval(this.timer);
  }

  transmit(id, opcode, payload) {
    this.counters.fromCan++;
    this.deliver(p.encodeFrame(id, opcode, payload));
  }

  reply(node, cmd, status, data = new Uint8Array(0)) {
    this.transmit(p.FN_REPLY + node, cmd, u8(status, ...data));
  }

  step() {
    const now = performance.now();
    for (const motor of this.motors) motor.step(now);
    this.imu.step(now);
    if (now >= this.nextStatus) {
      this.nextStatus = now + 250;
      this.sendAdapterStatus(p.AdapterOp.GetStatus);
    }
  }

  sendAdapterStatus(op) {
    const b = new DataView(new ArrayBuffer(28));
    b.setUint32(4, this.counters.toCan, true);
    b.setUint32(8, this.counters.fromCan, true);
    this.deliver(p.encodeFrame(p.ADAPTER_ID, op, new Uint8Array(b.buffer)));
  }

  handle({ id, opcode, payload }) {
    if (id === p.ADAPTER_ID) return this.handleAdapter(opcode);
    this.counters.toCan++;
    const cmd = opcode & ~p.NO_REPLY;
    if (id === p.BROADCAST_ID) return this.handleBroadcast(cmd, payload);
    if (id === this.imu.id) return this.imu.handle(opcode);
    if ((id & 0x780) !== p.FN_COMMAND) return;
    const motor = this.motors.find(m => m.id === (id & 0x7f));
    if (!motor) return;  // nobody there: the host times out, as on a real bus
    const [status, data] = motor.handle(cmd, payload);
    if (status !== p.Status.Ok || !(opcode & p.NO_REPLY)) this.reply(motor.id, cmd, status, data);
    if (motor.pending) {
      motor.pending();
      motor.pending = null;
    }
  }

  handleBroadcast(cmd, payload) {
    for (const motor of this.motors) {
      if (cmd === p.Cmd.GetInfo) {
        const delay = parseInt(motor.uid.slice(-2), 16) % 64;
        setTimeout(() => this.reply(motor.id, p.Cmd.GetInfo, p.Status.Ok, motor.info()), delay);
      } else if (cmd === p.Cmd.AssignNodeId && payload.length >= 13) {
        const uid = Array.from(payload.subarray(0, 12), b => b.toString(16).padStart(2, '0')).join('');
        if (uid === motor.uid) {
          motor.id = motor.params.nodeId = payload[12];
          this.reply(motor.id, p.Cmd.AssignNodeId, p.Status.Ok);
        }
      } else if ([p.Cmd.Stop, p.Cmd.EmergencyStop, p.Cmd.Sync, p.Cmd.Enable].includes(cmd)) {
        motor.handle(cmd, payload);
      }
    }
  }

  handleAdapter(op) {
    if (op === p.AdapterOp.GetInfo) {
      this.deliver(p.encodeFrame(p.ADAPTER_ID, op, u8(1, 2, 0, 0, 1, 3, 0, 0, ...p.uidBytes('41444150544552444d4f3031'))));
    } else if (op === p.AdapterOp.GetStatus) {
      this.sendAdapterStatus(op);
    } else if (op === p.AdapterOp.ResetCounters) {
      this.counters = { toCan: 0, fromCan: 0 };
      this.sendAdapterStatus(op);
    } else if (op === p.AdapterOp.EnterBootloader) {
      this.deliver(p.encodeFrame(p.ADAPTER_ID, op));
    }
  }
}

export class DemoTransport {
  constructor() {
    this.onData = () => {};
    this.onClose = () => {};
    this.decoder = new p.FrameDecoder();
  }

  get label() {
    return 'Demo';
  }

  async open() {
    this.sim = new Simulator(bytes => setTimeout(() => this.onData(bytes), 1));
    this.sim.start();
  }

  write(bytes) {
    for (const frame of this.decoder.push(bytes)) setTimeout(() => this.sim?.handle(frame), 1);
  }

  async close() {
    this.sim?.stop();
    this.sim = null;
  }
}
