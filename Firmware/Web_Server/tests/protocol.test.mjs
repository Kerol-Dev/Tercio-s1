// Tests for assets/protocol.js. Run with: node --test tests/
//
// The FIRMWARE_* fixtures are wire bytes produced by the real firmware code:
// Tercio S1 frame encoders (Protocol.h) framed by the Tercio FD bridge
// (Framing.h). They pin the browser decoder to the firmware, byte for byte.
import assert from 'node:assert/strict';
import { test } from 'node:test';

import * as p from '../assets/protocol.js';

const bytes = hex => Uint8Array.from(hex.match(/../g).map(h => parseInt(h, 16)));

const FIRMWARE_TELEMETRY =
  '09050201020b20011109e47cfb84454a93c00101010104204540010e60c0cdcc4c3c85ff3b5e07c80d010101010101010101033e0f00';
const FIRMWARE_INFO_REPLY = '03850101030202011101020501a0a1a2a3a4a5a6a7a8a9aaab0103929f00';
const FIRMWARE_ADAPTER_STATUS = '0106080102820303e8030103d007010204010102050101020601010207010103608000';

test('crc16 check value', () => {
  assert.equal(p.crc16(new TextEncoder().encode('123456789')), 0x29b1);
});

test('cobs known vectors', () => {
  assert.deepEqual([...p.cobsEncode([0x00])], [0x01, 0x01]);
  assert.deepEqual([...p.cobsEncode([0x11, 0x22, 0x00, 0x33])], [0x03, 0x11, 0x22, 0x02, 0x33]);
  assert.deepEqual([...p.cobsEncode([0x11, 0x00, 0x00, 0x00])], [0x02, 0x11, 0x01, 0x01, 0x01]);
  assert.deepEqual([...p.cobsDecode(Uint8Array.from([0x02, 0x11, 0x01, 0x01, 0x01]))], [0x11, 0, 0, 0]);
  const long = Uint8Array.from({ length: 300 }, (_, i) => (i % 7) + 1);  // runs past 254
  assert.deepEqual([...p.cobsDecode(p.cobsEncode(long))], [...long]);
});

test('decodes firmware telemetry', () => {
  const decoder = new p.FrameDecoder();
  const [frame] = decoder.push(bytes(FIRMWARE_TELEMETRY));
  assert.equal(frame.id, 0x205);
  assert.equal(frame.opcode, p.FRAME_TELEMETRY);
  const t = p.parseTelemetry(frame.payload);
  assert.equal(t.state, p.AxisState.Moving);
  assert.equal(t.flags, 0x0b);
  assert.equal(t.faults, 0x0120);
  assert.equal(t.position, -1234.5678901);
  assert.equal(t.target, 42.25);
  assert.equal(t.velocity, -3.5);
  assert.ok(Math.abs(t.followingError - 0.0125) < 1e-7);
  assert.equal(t.temperature, -12.3);
  assert.equal(t.supply, 24.123);
  assert.deepEqual([t.procedureStep, t.sequence, t.load], [7, 200, 13]);
});

test('decodes firmware info reply and adapter status', () => {
  const decoder = new p.FrameDecoder();
  const [reply, status] = decoder.push(bytes(FIRMWARE_INFO_REPLY + FIRMWARE_ADAPTER_STATUS));
  assert.equal(reply.id, p.FN_REPLY + 5);
  assert.equal(reply.opcode, p.Cmd.GetInfo);
  assert.equal(reply.payload[0], p.Status.Ok);
  const info = p.parseInfo(reply.payload.subarray(1));
  assert.deepEqual(info, {
    protocol: 2, firmware: '2.0.0', hardwareRevision: 1, encoderType: 2, nodeId: 5, crystalClock: true,
    uid: 'a0a1a2a3a4a5a6a7a8a9aaab',
  });
  assert.equal(status.id, p.ADAPTER_ID);
  const s = p.parseAdapterStatus(status.payload);
  assert.deepEqual([s.busState, s.tec, s.rec, s.toCan, s.fromCan, s.busOffEvents], [2, 130, 3, 1000, 2000, 7]);
});

test('resynchronises after a torn frame and rejects corruption', () => {
  const decoder = new p.FrameDecoder();
  const good = bytes(FIRMWARE_TELEMETRY);
  const torn = good.slice(9);
  const corrupt = good.slice();
  corrupt[12] ^= 0x40;
  const frames = decoder.push(Uint8Array.from([...torn, ...corrupt, ...good]));
  assert.equal(frames.length, 1);
  assert.equal(decoder.rejected, 2);
});

test('round-trips every payload length, zeros included', () => {
  const decoder = new p.FrameDecoder();
  for (let n = 0; n <= p.MAX_PAYLOAD; n++) {
    const payload = Uint8Array.from({ length: n }, (_, i) => (i % 3 ? i : 0));
    const wire = p.encodeFrame(0x105, p.Cmd.MoveTo, payload);
    assert.equal(wire.at(-1), 0);
    assert.ok(wire.subarray(0, -1).every(b => b !== 0));
    const [frame] = decoder.push(wire);
    assert.equal(frame.id, 0x105);
    assert.deepEqual([...frame.payload], [...payload]);
  }
  assert.throws(() => p.encodeFrame(0x105, 1, new Uint8Array(64)));
});

test('command payloads match the firmware layout', () => {
  const move = p.movePayload(0.25, { deferred: true, maxVelocity: 2 });
  assert.equal(move.length, 17);
  const v = new DataView(move.buffer);
  assert.equal(v.getFloat64(0, true), 0.25);
  assert.equal(v.getUint8(8), p.MOVE_DEFERRED);
  assert.equal(v.getFloat32(9, true), 2);
  assert.equal(p.velocityPayload(-1.5).length, 8);
});

test('parameter values encode as 4 little-endian bytes', () => {
  const kp = p.PARAM_BY_KEY.get('kp');
  assert.equal(p.decodeParamValue(kp, p.encodeParamValue(kp, 12.5)), 12.5);
  const on = p.PARAM_BY_KEY.get('stealthChop');
  assert.deepEqual([...p.encodeParamValue(on, true)], [1, 0, 0, 0]);
  const micro = p.PARAM_BY_KEY.get('microsteps');
  assert.equal(p.decodeParamValue(micro, p.encodeParamValue(micro, 256)), 256);
  assert.equal(new Set(p.PARAMS.map(x => x.id)).size, p.PARAMS.length);  // unique ids
});

test('unit conversion', () => {
  assert.equal(p.toDisplay(0.25, 'deg'), 90);
  assert.equal(p.fromDisplay(Math.PI, 'rad'), 0.5);
  assert.equal(p.toDisplay(3, 'turn'), 3);
});
