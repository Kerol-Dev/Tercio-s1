// -----------------------------------------------------------------------------
// Link layer: a transport (Web Serial, or the built-in demo) plus the Bus that
// speaks the Tercio protocol over it — framing, request/reply matching,
// telemetry and adapter events.
// -----------------------------------------------------------------------------
import * as p from './protocol.js';

export class CommandError extends Error {
  constructor(status, cmd, node) {
    super(p.STATUS_TEXT[status] ?? (status === 'timeout' ? 'No answer. Is the motor powered and on the bus?' : String(status)));
    this.status = status;
    this.cmd = cmd;
    this.node = node;
  }
}

// ---- Web Serial -------------------------------------------------------------------
export class SerialTransport {
  static get supported() {
    return typeof navigator !== 'undefined' && 'serial' in navigator;
  }

  // A Tercio FD port this site was already allowed to use, if one is plugged in.
  static async knownPort() {
    if (!SerialTransport.supported) return null;
    const ports = await navigator.serial.getPorts();
    return ports.find(port => {
      const info = port.getInfo();
      return info.usbVendorId === p.TERCIO_FD_USB.usbVendorId && info.usbProductId === p.TERCIO_FD_USB.usbProductId;
    }) ?? null;
  }

  constructor(port = null) {
    this.port = port;
    this.onData = () => {};
    this.onClose = () => {};
    this.queue = Promise.resolve();
  }

  get label() {
    return 'USB';
  }

  async open({ anyDevice = false } = {}) {
    this.port ??= await navigator.serial.requestPort(anyDevice ? {} : { filters: [p.TERCIO_FD_USB] });
    await this.port.open({ baudRate: 115200, bufferSize: 1 << 16 });
    // The adapter only talks to a host that raised DTR.
    await this.port.setSignals({ dataTerminalReady: true, requestToSend: true }).catch(() => {});
    this.writer = this.port.writable.getWriter();
    this.closing = false;
    this.reading = this.readLoop();
  }

  // Framing, parity, break and overrun errors end one stream and the port
  // offers a fresh one; only a lost device leaves `readable` null.
  async readLoop() {
    let lastError = null;
    while (this.port?.readable && !this.closing) {
      this.reader = this.port.readable.getReader();
      try {
        for (;;) {
          const { value, done } = await this.reader.read();
          if (done) break;
          if (value?.length) this.onData(value);
        }
      } catch (error) {
        lastError = error;
      } finally {
        this.reader.releaseLock();
      }
      if (this.closing) return;
    }
    if (!this.closing) this.onClose(lastError);
  }

  // One writer, one queue: writes never interleave or trip over a locked stream.
  write(bytes) {
    this.queue = this.queue.then(() => this.writer?.write(bytes)).catch(() => {});
  }

  async close() {
    this.closing = true;
    await this.reader?.cancel().catch(() => {});
    await this.queue;
    try {
      this.writer?.releaseLock();
    } catch {}
    await this.reading?.catch(() => {});
    await this.port?.close().catch(() => {});
  }
}

// ---- Protocol ----------------------------------------------------------------------------
const EMPTY = new Uint8Array(0);

export class Bus extends EventTarget {
  constructor(transport) {
    super();
    this.transport = transport;
    this.decoder = new p.FrameDecoder();
    this.pending = new Map();  // "node:cmd" -> { resolve, reject, timer }
    this.chains = new Map();   // "node:cmd" -> promise: one outstanding request per key
    this.frameListener = null; // optional raw frame tap (log view)
    this.counters = { tx: 0, rx: 0 };
    transport.onData = bytes => this.receive(bytes);
    transport.onClose = error => {
      this.rejectAll(new CommandError('disconnected'));
      this.dispatchEvent(new CustomEvent('close', { detail: error }));
    };
  }

  get rejectedFrames() {
    return this.decoder.rejected;
  }

  open(options) {
    return this.transport.open(options);
  }

  async close() {
    this.rejectAll(new CommandError('disconnected'));
    await this.transport.close();
  }

  rejectAll(error) {
    for (const entry of this.pending.values()) {
      clearTimeout(entry.timer);
      entry.reject(error);
    }
    this.pending.clear();
  }

  send(id, opcode, payload = EMPTY) {
    this.transport.write(p.encodeFrame(id, opcode, payload));
    this.counters.tx++;
    this.frameListener?.('tx', id, opcode, payload);
  }

  // Sends a command and resolves with the reply data; rejects with a CommandError.
  // `match(reply)` may reject a reply that belongs to an older request.
  request(node, cmd, payload = EMPTY, { timeout = 400, match } = {}) {
    const key = `${node}:${cmd}`;
    const run = () =>
      new Promise((resolve, reject) => {
        const timer = setTimeout(() => {
          this.pending.delete(key);
          reject(new CommandError('timeout', cmd, node));
        }, timeout);
        this.pending.set(key, {
          timer,
          reject,
          match,
          resolve: ({ status, data }) =>
            status === p.Status.Ok ? resolve(data) : reject(new CommandError(status, cmd, node)),
        });
        this.send(p.FN_COMMAND + node, cmd, payload);
      });
    const previous = this.chains.get(key) ?? Promise.resolve();
    const next = previous.catch(() => {}).then(run);
    this.chains.set(key, next);
    return next;
  }

  // Fire-and-forget: the node only answers if the command fails.
  command(node, cmd, payload = EMPTY) {
    this.send(p.FN_COMMAND + node, cmd | p.NO_REPLY, payload);
  }

  broadcast(cmd, payload = EMPTY) {
    this.send(p.BROADCAST_ID, cmd, payload);
  }

  adapter(op, { timeout = 400 } = {}) {
    const key = `adapter:${op}`;
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pending.delete(key);
        reject(new CommandError('timeout', op, 'adapter'));
      }, timeout);
      this.pending.set(key, { timer, reject, resolve: ({ data }) => resolve(data) });
      this.send(p.ADAPTER_ID, op);
    });
  }

  settle(key, reply) {
    const entry = this.pending.get(key);
    if (!entry || (entry.match && !entry.match(reply))) return;
    clearTimeout(entry.timer);
    this.pending.delete(key);
    entry.resolve(reply);
  }

  receive(bytes) {
    for (const frame of this.decoder.push(bytes)) {
      this.counters.rx++;
      this.frameListener?.('rx', frame.id, frame.opcode, frame.payload);
      this.dispatch(frame);
    }
  }

  emit(type, detail) {
    this.dispatchEvent(new CustomEvent(type, { detail }));
  }

  dispatch({ id, opcode, payload }) {
    if (id === p.ADAPTER_ID) {
      if (opcode === p.AdapterOp.GetStatus || opcode === p.AdapterOp.ResetCounters) {
        const status = p.parseAdapterStatus(payload);
        if (status) this.emit('adapter-status', status);
      }
      this.settle(`adapter:${opcode}`, { data: payload });
      return;
    }
    const fn = id & 0x780;
    const node = id & 0x7f;
    if (fn === p.FN_TELEMETRY && opcode === p.FRAME_TELEMETRY) {
      const telemetry = p.parseTelemetry(payload);
      if (telemetry) this.emit('telemetry', { node, telemetry });
    } else if (fn === p.FN_REPLY && payload.length >= 1) {
      const status = payload[0];
      const data = payload.subarray(1);
      if (opcode === p.Cmd.GetInfo && status === p.Status.Ok) {
        const info = p.parseInfo(data);
        if (info) this.emit('info', { node, info });
      }
      this.settle(`${node}:${opcode}`, { status, data });
    } else if (fn === p.FN_EVENT && opcode === p.FRAME_FAULT) {
      const fault = p.parseFault(payload);
      if (fault) this.emit('fault', { node, ...fault });
    } else if (id < p.FN_EVENT && opcode === 0x02) {
      const imu = p.parseImu(payload);
      if (imu) this.emit('imu', { id, imu });
    }
  }
}
