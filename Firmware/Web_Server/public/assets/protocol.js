// -----------------------------------------------------------------------------
// Tercio protocol for the browser: CAN protocol v2 (Tercio-s1
// Firmware/src/protocol/Protocol.h) and adapter serial framing v2 (Tercio-fdcan
// Firmware/src/bridge/{Framing,AdapterProtocol}.h). Pure module, no DOM — also
// imported by the Node tests. Keep in sync with those headers.
//
// Units: the wire carries turns (revolutions of the encoder shaft), turns/s
// and turns/s². Conversion to degrees or radians happens at the UI edge.
// -----------------------------------------------------------------------------

// ---- Identifiers ------------------------------------------------------------
export const BROADCAST_ID = 0x000;
export const FN_EVENT = 0x080;
export const FN_COMMAND = 0x100;
export const FN_REPLY = 0x180;
export const FN_TELEMETRY = 0x200;
export const ADAPTER_ID = 0x800;
export const MAX_NODE_ID = 127;
export const MAX_PAYLOAD = 63;
export const NO_REPLY = 0x80;
export const MOVE_DEFERRED = 0x01;
export const FRAME_TELEMETRY = 0x01;
export const FRAME_FAULT = 0x02;
export const TERCIO_FD_USB = { usbVendorId: 0x1209, usbProductId: 0x0011 };

export const Cmd = Object.freeze({
  GetInfo: 0x00, SaveConfig: 0x01, FactoryReset: 0x02, Reboot: 0x03, ClearFaults: 0x04,
  AssignNodeId: 0x05, GetParam: 0x08, SetParam: 0x09,
  Enable: 0x10, Stop: 0x11, EmergencyStop: 0x12, MoveTo: 0x13, MoveBy: 0x14,
  SetVelocity: 0x15, SetZero: 0x16, Sync: 0x17,
  Calibrate: 0x20, Home: 0x21, AutoTune: 0x22,
});

export const Status = Object.freeze({
  Ok: 0, UnknownCommand: 1, BadLength: 2, BadValue: 3, UnknownParam: 4, ReadOnly: 5, Busy: 6,
  NotCalibrated: 7, Faulted: 8, Disabled: 9, StorageError: 10,
});

// What a person should do about a refused command.
export const STATUS_TEXT = {
  [Status.UnknownCommand]: 'The motor does not know this command. Update its firmware.',
  [Status.BadLength]: 'The command was malformed.',
  [Status.BadValue]: 'That value is out of range.',
  [Status.UnknownParam]: 'The motor does not have this setting. Update its firmware.',
  [Status.ReadOnly]: 'This setting is read-only.',
  [Status.Busy]: 'The motor is busy. Wait until it stops, or stop it first.',
  [Status.NotCalibrated]: 'Calibrate the motor first.',
  [Status.Faulted]: 'Clear the fault first.',
  [Status.Disabled]: 'Enable the motor first.',
  [Status.StorageError]: 'Saving to flash failed. Try again; if it keeps failing, the flash may be worn.',
};

export const AxisState = Object.freeze({
  Disabled: 0, Holding: 1, Moving: 2, Velocity: 3, StepDir: 4, Calibrating: 5, Homing: 6, Tuning: 7, Fault: 8,
});
export const STATE_INFO = {
  0: { name: 'Disabled', tone: 'neutral' },
  1: { name: 'Holding', tone: 'good' },
  2: { name: 'Moving', tone: 'accent' },
  3: { name: 'Velocity', tone: 'accent' },
  4: { name: 'Step/Dir', tone: 'accent' },
  5: { name: 'Calibrating', tone: 'busy' },
  6: { name: 'Homing', tone: 'busy' },
  7: { name: 'Auto-tuning', tone: 'busy' },
  8: { name: 'Fault', tone: 'bad' },
};

export const FAULTS = [
  { bit: 1 << 0, name: 'Over-temperature', hard: true, help: 'The board is too hot. Let it cool, improve airflow or lower the run current.' },
  { bit: 1 << 1, name: 'Driver over-temperature', hard: true, help: 'The TMC2209 shut down on heat. Lower the current or add cooling.' },
  { bit: 1 << 2, name: 'Driver fault', hard: true, help: 'The driver switched its bridge off: short circuit or supply problem. Check the motor wiring.' },
  { bit: 1 << 3, name: 'Driver not responding', hard: true, help: 'The TMC2209 stopped answering over UART. Check the motor supply.' },
  { bit: 1 << 4, name: 'Encoder', hard: true, help: 'No valid encoder readings. Check the magnet and the encoder connection.' },
  { bit: 1 << 5, name: 'Stall', hard: false, help: 'The shaft fell behind the command. Reduce speed or acceleration, raise the current, or check for a blockage.' },
  { bit: 1 << 6, name: 'Calibration failed', hard: false, help: 'The encoder did not follow the motor. Check the magnet, steps per revolution and gear ratio.' },
  { bit: 1 << 7, name: 'Homing failed', hard: false, help: 'No switch or hard stop was found in time, or the axis was blocked.' },
  { bit: 1 << 8, name: 'Command timeout', hard: false, help: 'The host went quiet while the axis was moving, so it stopped.' },
];
export const WARNINGS = [
  { bit: 1 << 0, name: 'Running hot' },
  { bit: 1 << 1, name: 'Driver near thermal limit' },
  { bit: 1 << 2, name: 'Magnet too weak' },
  { bit: 1 << 3, name: 'Magnet too strong' },
  { bit: 1 << 4, name: 'Supply voltage low' },
  { bit: 1 << 5, name: 'CAN errors' },
  { bit: 1 << 6, name: 'Motor disconnected?' },
  { bit: 1 << 7, name: 'Saving settings failed' },
];
export const FLAGS = { enabled: 1, calibrated: 2, homed: 4, settled: 8, limitMin: 16, limitMax: 32, movePending: 64, extEnable: 128 };

export const CALIBRATION_STEPS = ['Settling', 'Turning forward', 'Measuring', 'Turning back', 'Checking'];
export const HOMING_STEPS = ['Searching', 'Settling', 'Backing off', 'Done'];

export const ENCODER_TYPES = ['On-board AS5600', 'External AS5600 (I²C)', 'External AS5048A (SPI)'];
export const HOMING_MODES = ['Switch IN1 (negative)', 'Switch IN2 (positive)', 'Hard stop, negative', 'Hard stop, positive'];

// ---- Parameters ---------------------------------------------------------------
// kind: how the value is shown. 'pos' / 'vel' / 'acc' are in turns on the wire
// and follow the chosen display unit; everything else is shown as is.
export const PARAM_GROUPS = [
  { key: 'motion', title: 'Motion', blurb: 'Speed and acceleration limits every move respects.' },
  { key: 'control', title: 'Position loop', blurb: 'How firmly the controller holds and follows the target.' },
  { key: 'motor', title: 'Motor & driver', blurb: 'Match these to your motor.' },
  { key: 'encoder', title: 'Encoder', blurb: 'Which sensor closes the loop.' },
  { key: 'homing', title: 'Homing', blurb: 'Used by the Home procedure.' },
  { key: 'io', title: 'Inputs', blurb: 'Limit switches and the external step/dir connector.' },
  { key: 'protection', title: 'Protection', blurb: 'Stall and temperature limits.' },
  { key: 'comm', title: 'Communication', blurb: 'CAN identity and streaming.' },
];

export const PARAMS = [
  { id: 0x20, key: 'maxVelocity', group: 'motion', label: 'Max velocity', type: 'f32', kind: 'vel', min: 0.001, max: 200, access: 'always', help: 'Upper speed limit for every move.' },
  { id: 0x21, key: 'maxAcceleration', group: 'motion', label: 'Max acceleration', type: 'f32', kind: 'acc', min: 0.01, max: 100000, access: 'always', help: 'Ramp steepness for every move.' },
  { id: 0x28, key: 'softLimitMin', group: 'motion', label: 'Soft limit, min', type: 'f32', kind: 'pos', min: -1e7, max: 1e7, access: 'always', help: 'Moves never go below this. Active when min is less than max.' },
  { id: 0x29, key: 'softLimitMax', group: 'motion', label: 'Soft limit, max', type: 'f32', kind: 'pos', min: -1e7, max: 1e7, access: 'always', help: 'Moves never go above this.' },
  { id: 0x2A, key: 'enableOnBoot', group: 'motion', label: 'Enable at power-up', type: 'bool', access: 'always', help: 'Energise and hold position as soon as the board starts.' },

  { id: 0x22, key: 'kp', group: 'control', label: 'Proportional gain', type: 'f32', kind: 'plain', unit: '1/s', min: 0, max: 1000, access: 'always', help: 'Correction speed per unit of error. Higher is stiffer; too high oscillates.' },
  { id: 0x23, key: 'ki', group: 'control', label: 'Integral gain', type: 'f32', kind: 'plain', unit: '1/s²', min: 0, max: 100000, access: 'always', help: 'Removes steady error under constant load. Usually 0 for steppers.' },
  { id: 0x24, key: 'kd', group: 'control', label: 'Damping gain', type: 'f32', kind: 'plain', unit: '', min: 0, max: 10, access: 'always', help: 'Acts on the velocity error. Usually 0.' },
  { id: 0x25, key: 'positionDeadband', group: 'control', label: 'Deadband', type: 'f32', kind: 'pos', min: 0, max: 0.1, access: 'always', help: 'At rest, errors smaller than this are ignored so the motor does not hunt.' },

  { id: 0x10, key: 'fullStepsPerRev', group: 'motor', label: 'Full steps per revolution', type: 'u16', kind: 'plain', unit: 'steps', min: 20, max: 1000, access: 'still', help: '200 for 1.8° motors, 400 for 0.9° motors.' },
  { id: 0x11, key: 'microsteps', group: 'motor', label: 'Microsteps', type: 'u16', kind: 'select', options: [1, 2, 4, 8, 16, 32, 64, 128, 256].map(v => ({ value: v, label: `1/${v}` })), access: 'still', help: 'The driver interpolates to 1/256 internally; 16 is a good default.' },
  { id: 0x12, key: 'runCurrentMa', group: 'motor', label: 'Run current', type: 'u16', kind: 'plain', unit: 'mA', min: 50, max: 2000, access: 'always', help: 'RMS coil current while moving. The board delivers up to about 1.77 A.' },
  { id: 0x13, key: 'holdCurrentPct', group: 'motor', label: 'Hold current', type: 'u8', kind: 'plain', unit: '%', min: 0, max: 100, access: 'always', help: 'Current at standstill, as a share of the run current.' },
  { id: 0x14, key: 'stealthChop', group: 'motor', label: 'Quiet mode (StealthChop)', type: 'bool', access: 'still', help: 'Near-silent at low speed. Above the switch speed the driver uses SpreadCycle.' },
  { id: 0x15, key: 'stealthChopMaxVel', group: 'motor', label: 'Quiet mode up to', type: 'f32', kind: 'plain', unit: 'motor rev/s', min: 0, max: 100, access: 'always', help: 'Motor speed where the driver switches to SpreadCycle. 0 keeps quiet mode at every speed.' },
  { id: 0x16, key: 'invertDirection', group: 'motor', label: 'Reverse direction', type: 'bool', access: 'still', help: 'Flip which way is positive. No recalibration needed.' },
  { id: 0x17, key: 'gearRatio', group: 'motor', label: 'Gear ratio', type: 'f32', kind: 'plain', unit: ': 1', min: 0.01, max: 1000, access: 'still', help: 'Motor turns per encoder turn. 1 with the on-board encoder.' },

  { id: 0x30, key: 'encoderType', group: 'encoder', label: 'Encoder', type: 'u8', kind: 'select', options: ENCODER_TYPES.map((label, value) => ({ value, label })), access: 'disabled', help: 'Changing the encoder clears the calibration. Disable the motor first.' },
  { id: 0x31, key: 'encoderInvert', group: 'encoder', label: 'Encoder reversed', type: 'bool', access: 'disabled', help: 'Set by calibration. Change by hand only if you know you need to.' },
  { id: 0x32, key: 'calibrated', group: 'encoder', label: 'Calibrated', type: 'bool', access: 'readonly', help: 'Run Calibrate to set this.' },

  { id: 0x50, key: 'homingMode', group: 'homing', label: 'Method', type: 'u8', kind: 'select', options: HOMING_MODES.map((label, value) => ({ value, label })), access: 'always', help: 'Seek a limit switch, or push gently into a hard stop.' },
  { id: 0x51, key: 'homingVelocity', group: 'homing', label: 'Search speed', type: 'f32', kind: 'vel', min: 0.001, max: 100, access: 'always', help: 'Keep it slow for repeatable results.' },
  { id: 0x52, key: 'homingCurrentMa', group: 'homing', label: 'Search current', type: 'u16', kind: 'plain', unit: 'mA', min: 50, max: 2000, access: 'always', help: 'Reduced current while searching, so a hard stop is gentle.' },
  { id: 0x53, key: 'homingBackoff', group: 'homing', label: 'Back-off distance', type: 'f32', kind: 'pos', min: 0, max: 1000, access: 'always', help: 'How far to move away from the trigger point before setting zero.' },
  { id: 0x54, key: 'homingStallError', group: 'homing', label: 'Hard-stop sensitivity', type: 'f32', kind: 'pos', min: 0.001, max: 100, access: 'always', help: 'Following error that counts as hitting the stop. Smaller is more sensitive.' },
  { id: 0x55, key: 'homingTimeoutS', group: 'homing', label: 'Give up after', type: 'u16', kind: 'plain', unit: 's', min: 1, max: 3600, access: 'always', help: 'Maximum search time.' },

  { id: 0x40, key: 'limitSwitchesEnabled', group: 'io', label: 'Stop at limit switches', type: 'bool', access: 'always', help: 'IN1 blocks negative motion, IN2 blocks positive motion.' },
  { id: 0x41, key: 'limitSwitchActiveLow', group: 'io', label: 'Switches pull to ground', type: 'bool', access: 'still', help: 'On for normally-open switches wired to GND.' },
  { id: 0x42, key: 'stepDirMode', group: 'io', label: 'Follow step/dir input', type: 'bool', access: 'still', help: 'Take position commands from the STEP/DIR/EN connector (3.3 V logic only).' },
  { id: 0x43, key: 'stepDirEnableActiveLow', group: 'io', label: 'EN input active low', type: 'bool', access: 'always', help: 'Polarity of the external enable input.' },

  { id: 0x26, key: 'followingErrorLimit', group: 'protection', label: 'Stall threshold', type: 'f32', kind: 'pos', min: 0.001, max: 1000, access: 'always', help: 'Following error that counts as a stall.' },
  { id: 0x27, key: 'stallTimeoutMs', group: 'protection', label: 'Stall confirm time', type: 'u16', kind: 'plain', unit: 'ms', min: 0, max: 10000, access: 'always', help: 'How long the error must persist. 0 turns stall detection off.' },
  { id: 0x60, key: 'overTemperatureC', group: 'protection', label: 'Shut down at', type: 'f32', kind: 'plain', unit: '°C', min: 40, max: 125, access: 'always', help: 'Board temperature that cuts the motor current.' },
  { id: 0x03, key: 'commandTimeoutMs', group: 'protection', label: 'Stop if host is silent for', type: 'u16', kind: 'plain', unit: 'ms', min: 0, max: 60000, access: 'always', help: 'Stops a moving axis when commands stop arriving. 0 turns it off.' },

  { id: 0x01, key: 'nodeId', group: 'comm', label: 'Node ID', type: 'u8', kind: 'plain', unit: '', min: 1, max: MAX_NODE_ID, access: 'still', help: 'Unique per motor on the bus. Takes effect immediately.' },
  { id: 0x02, key: 'telemetryRateHz', group: 'comm', label: 'Telemetry rate', type: 'u16', kind: 'plain', unit: 'Hz', min: 0, max: 1000, access: 'always', help: 'How often the motor reports its state. 100 Hz suits most setups.' },
];
// Factory defaults (Tercio-s1 Firmware/src/app/Settings.h).
export const PARAM_DEFAULTS = {
  maxVelocity: 10, maxAcceleration: 100, softLimitMin: 0, softLimitMax: 0, enableOnBoot: true,
  kp: 20, ki: 0, kd: 0, positionDeadband: 0.0005,
  fullStepsPerRev: 200, microsteps: 16, runCurrentMa: 1500, holdCurrentPct: 50, stealthChop: true,
  stealthChopMaxVel: 1, invertDirection: false, gearRatio: 1,
  encoderType: 0, encoderInvert: false, calibrated: false,
  homingMode: 0, homingVelocity: 1, homingCurrentMa: 800, homingBackoff: 0.05, homingStallError: 0.03, homingTimeoutS: 30,
  limitSwitchesEnabled: false, limitSwitchActiveLow: true, stepDirMode: false, stepDirEnableActiveLow: true,
  followingErrorLimit: 0.05, stallTimeoutMs: 50, overTemperatureC: 95, commandTimeoutMs: 0,
  nodeId: 1, telemetryRateHz: 100,
};

export const PARAM_BY_ID = new Map(PARAMS.map(p => [p.id, p]));
export const PARAM_BY_KEY = new Map(PARAMS.map(p => [p.key, p]));

export function encodeParamValue(param, value) {
  const b = new DataView(new ArrayBuffer(4));
  if (param.type === 'f32') b.setFloat32(0, Number(value), true);
  else b.setUint32(0, param.type === 'bool' ? (value ? 1 : 0) : Math.round(Number(value)) >>> 0, true);
  return new Uint8Array(b.buffer);
}

export function decodeParamValue(param, bytes, offset = 0) {
  const v = new DataView(bytes.buffer, bytes.byteOffset + offset, 4);
  if (param.type === 'f32') return v.getFloat32(0, true);
  const raw = v.getUint32(0, true);
  return param.type === 'bool' ? raw !== 0 : raw;
}

// ---- Units ----------------------------------------------------------------------
export const UNITS = {
  deg: { key: 'deg', scale: 360, pos: '°', vel: '°/s', acc: '°/s²', label: 'Degrees', digits: 2 },
  rad: { key: 'rad', scale: 2 * Math.PI, pos: 'rad', vel: 'rad/s', acc: 'rad/s²', label: 'Radians', digits: 4 },
  turn: { key: 'turn', scale: 1, pos: 'rev', vel: 'rev/s', acc: 'rev/s²', label: 'Revolutions', digits: 4 },
};
export const toDisplay = (turns, unit) => turns * UNITS[unit].scale;
export const fromDisplay = (value, unit) => value / UNITS[unit].scale;

// ---- Serial framing: COBS + CRC-16/CCITT-FALSE, 0x00-delimited ----------------------
export function crc16(bytes, length = bytes.length) {
  let crc = 0xffff;
  for (let i = 0; i < length; i++) {
    crc ^= bytes[i] << 8;
    for (let b = 0; b < 8; b++) crc = crc & 0x8000 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
  }
  return crc;
}

export function cobsEncode(data) {
  const out = [0];
  let codeIndex = 0;
  let code = 1;
  for (const byte of data) {
    if (byte === 0) {
      out[codeIndex] = code;
      codeIndex = out.length;
      out.push(0);
      code = 1;
    } else {
      out.push(byte);
      if (++code === 0xff) {
        out[codeIndex] = code;
        codeIndex = out.length;
        out.push(0);
        code = 1;
      }
    }
  }
  out[codeIndex] = code;
  return Uint8Array.from(out);
}

export function cobsDecode(data) {
  const out = [];
  let i = 0;
  while (i < data.length) {
    const code = data[i++];
    if (code === 0 || i + code - 1 > data.length) return null;
    for (let j = 1; j < code; j++) out.push(data[i++]);
    if (code !== 0xff && i < data.length) out.push(0);
  }
  return Uint8Array.from(out);
}

// One frame on the wire: COBS([id u16][opcode][payload][crc16]) + 0x00.
export function encodeFrame(id, opcode, payload = new Uint8Array(0)) {
  if (payload.length > MAX_PAYLOAD) throw new RangeError('payload longer than 63 bytes');
  const packet = new Uint8Array(5 + payload.length);
  packet[0] = id & 0xff;
  packet[1] = id >> 8;
  packet[2] = opcode & 0xff;
  packet.set(payload, 3);
  const crc = crc16(packet, 3 + payload.length);
  packet[3 + payload.length] = crc & 0xff;
  packet[4 + payload.length] = crc >> 8;
  const encoded = cobsEncode(packet);
  const frame = new Uint8Array(encoded.length + 1);
  frame.set(encoded);
  return frame;
}

export function decodePacket(chunk) {
  const packet = cobsDecode(chunk);
  if (!packet || packet.length < 5 || packet.length > 5 + MAX_PAYLOAD) return null;
  const n = packet.length;
  if ((packet[n - 2] | (packet[n - 1] << 8)) !== crc16(packet, n - 2)) return null;
  const id = packet[0] | (packet[1] << 8);
  if (id > 0x7ff && id !== ADAPTER_ID) return null;
  return { id, opcode: packet[2], payload: packet.subarray(3, n - 2) };
}

// Streaming decoder: feed raw bytes, get whole frames; damaged ones are counted.
export class FrameDecoder {
  constructor() {
    this.buffer = new Uint8Array(96);
    this.length = 0;
    this.overflow = false;
    this.rejected = 0;
  }
  push(bytes) {
    const frames = [];
    for (const byte of bytes) {
      if (byte !== 0) {
        if (this.length < this.buffer.length) this.buffer[this.length++] = byte;
        else this.overflow = true;
        continue;
      }
      if (this.length || this.overflow) {
        const frame = this.overflow ? null : decodePacket(this.buffer.slice(0, this.length));
        if (frame) frames.push(frame);
        else this.rejected++;
      }
      this.length = 0;
      this.overflow = false;
    }
    return frames;
  }
}

// ---- Payload codecs ---------------------------------------------------------------
const view = bytes => new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);

export const TELEMETRY_SIZE = 37;
export function parseTelemetry(payload) {
  if (payload.length < TELEMETRY_SIZE) return null;
  const v = view(payload);
  return {
    state: v.getUint8(0),
    flags: v.getUint8(1),
    faults: v.getUint16(2, true),
    warnings: v.getUint16(4, true),
    position: v.getFloat64(6, true),
    target: v.getFloat64(14, true),
    velocity: v.getFloat32(22, true),
    followingError: v.getFloat32(26, true),
    temperature: v.getInt16(30, true) / 10,
    supply: v.getUint16(32, true) / 1000,
    procedureStep: v.getUint8(34),
    sequence: v.getUint8(35),
    load: v.getUint8(36),
  };
}

const hex = bytes => Array.from(bytes, b => b.toString(16).padStart(2, '0')).join('');

export const INFO_SIZE = 20;
export function parseInfo(data) {
  if (data.length < INFO_SIZE) return null;
  return {
    protocol: data[0],
    firmware: `${data[1]}.${data[2]}.${data[3]}`,
    hardwareRevision: data[4],
    encoderType: data[5],
    nodeId: data[6],
    crystalClock: (data[7] & 1) !== 0,
    uid: hex(data.subarray(8, 20)),
  };
}

export function parseFault(payload) {
  if (payload.length < 5) return null;
  const v = view(payload);
  return { faults: v.getUint16(0, true), warnings: v.getUint16(2, true), state: v.getUint8(4) };
}

export const AdapterOp = Object.freeze({ GetInfo: 0x00, GetStatus: 0x01, ResetCounters: 0x02, EnterBootloader: 0x03 });
export const BUS_STATES = [
  { name: 'Healthy', tone: 'good' },
  { name: 'Errors seen', tone: 'warn' },
  { name: 'Error passive', tone: 'warn' },
  { name: 'Bus off', tone: 'bad' },
];

export function parseAdapterStatus(payload) {
  if (payload.length < 28) return null;
  const v = view(payload);
  return {
    busState: v.getUint8(0), tec: v.getUint8(1), rec: v.getUint8(2), lastError: v.getUint8(3),
    toCan: v.getUint32(4, true), fromCan: v.getUint32(8, true),
    droppedToCan: v.getUint32(12, true), droppedFromCan: v.getUint32(16, true),
    framingErrors: v.getUint32(20, true), busOffEvents: v.getUint16(24, true),
  };
}

export function parseAdapterInfo(payload) {
  if (payload.length < 20) return null;
  return {
    protocol: payload[0],
    firmware: `${payload[1]}.${payload[2]}.${payload[3]}`,
    hardwareRevision: payload[4],
    crystalClock: (payload[5] & 1) !== 0,
    bootOptionsOk: (payload[5] & 2) !== 0,
    uid: hex(payload.subarray(8, 20)),
  };
}

// Command payloads (positions and speeds in turns).
export function movePayload(turns, { deferred = false, maxVelocity = 0, maxAcceleration = 0 } = {}) {
  const b = new DataView(new ArrayBuffer(17));
  b.setFloat64(0, turns, true);
  b.setUint8(8, deferred ? MOVE_DEFERRED : 0);
  b.setFloat32(9, maxVelocity, true);
  b.setFloat32(13, maxAcceleration, true);
  return new Uint8Array(b.buffer);
}

export function velocityPayload(turnsPerSecond, maxAcceleration = 0) {
  const b = new DataView(new ArrayBuffer(8));
  b.setFloat32(0, turnsPerSecond, true);
  b.setFloat32(4, maxAcceleration, true);
  return new Uint8Array(b.buffer);
}

export function f64Payload(value) {
  const b = new DataView(new ArrayBuffer(8));
  b.setFloat64(0, value, true);
  return new Uint8Array(b.buffer);
}

export function rangePayload(min, max) {
  const b = new DataView(new ArrayBuffer(8));
  b.setFloat32(0, min, true);
  b.setFloat32(4, max, true);
  return new Uint8Array(b.buffer);
}

export function uidBytes(uidHex) {
  return Uint8Array.from(uidHex.match(/../g).map(h => parseInt(h, 16)));
}

// Tercio IMU module (its own, older protocol).
export function parseImu(payload) {
  if (payload.length < 28) return null;
  const v = view(payload);
  const f = i => v.getFloat32(i * 4, true);
  return { roll: f(0), pitch: f(1), yaw: f(2), ax: f(3), ay: f(4), az: f(5), temperature: f(6) };
}
export const IMU_SET_ID = 0xa1;
export const IMU_RESET_ORIENTATION = 0xa2;
