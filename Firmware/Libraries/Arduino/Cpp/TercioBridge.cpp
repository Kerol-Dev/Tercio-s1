#include "TercioBridge.h"

#include <string.h>

namespace Tercio {
namespace {

constexpr uint16_t kBroadcast = 0x000;
constexpr uint16_t kAdapterId = 0x800;
constexpr uint8_t kAdapterGetStatus = 0x01, kAdapterResetCounters = 0x02;
constexpr uint8_t kAdapterStatusSize = 28;
constexpr uint16_t kFnEvent = 0x080, kFnCommand = 0x100, kFnReply = 0x180, kFnTelemetry = 0x200;
constexpr uint8_t kNoReply = 0x80;
constexpr uint8_t kMoveDeferred = 0x01;
constexpr uint8_t kFrameTelemetry = 0x01, kFrameFault = 0x02;
constexpr uint8_t kTelemetrySize = 37;
constexpr double kTwoPi = 6.283185307179586;

template <typename T>
T readLe(const uint8_t* p) {
  T value;
  memcpy(&value, p, sizeof(T));
  return value;
}

template <typename T>
uint8_t writeLe(uint8_t* p, T value) {
  memcpy(p, &value, sizeof(T));
  return sizeof(T);
}

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF).
uint16_t crc16(const uint8_t* data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  while (length--) {
    crc ^= uint16_t(*data++) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}

uint8_t cobsEncode(const uint8_t* in, uint8_t length, uint8_t* out) {
  uint8_t codeIndex = 0, outIndex = 1, code = 1;
  for (uint8_t i = 0; i < length; ++i) {
    if (in[i] == 0) {
      out[codeIndex] = code;
      codeIndex = outIndex++;
      code = 1;
    } else {
      out[outIndex++] = in[i];
      if (++code == 0xFF) {
        out[codeIndex] = code;
        codeIndex = outIndex++;
        code = 1;
      }
    }
  }
  out[codeIndex] = code;
  return outIndex;
}

int cobsDecode(const uint8_t* in, uint8_t length, uint8_t* out) {
  uint8_t i = 0;
  int o = 0;
  while (i < length) {
    const uint8_t code = in[i++];
    if (code == 0) return -1;
    for (uint8_t j = 1; j < code; ++j) {
      if (i >= length) return -1;
      out[o++] = in[i++];
    }
    if (code != 0xFF && i < length) out[o++] = 0;
  }
  return o;
}

}  // namespace

// ---------------------------------------------------------------- Bus --------

void Bus::send(uint16_t canId, uint8_t opcode, const uint8_t* payload, uint8_t length) {
  if ((canId > 0x7FF && canId != kAdapterId) || length > 63) return;
  uint8_t packet[2 + 1 + 63 + 2];
  packet[0] = uint8_t(canId);
  packet[1] = uint8_t(canId >> 8);
  packet[2] = opcode;
  if (length) memcpy(packet + 3, payload, length);
  const uint16_t crc = crc16(packet, uint8_t(3 + length));
  packet[3 + length] = uint8_t(crc);
  packet[4 + length] = uint8_t(crc >> 8);
  uint8_t encoded[72];
  const uint8_t size = cobsEncode(packet, uint8_t(5 + length), encoded);
  encoded[size] = 0;  // delimiter
  io_.write(encoded, size + 1);
}

void Bus::poll() {
  while (io_.available()) {
    const uint8_t byte = uint8_t(io_.read());
    if (byte != 0) {  // 0x00 only ever marks the end of a frame
      if (rxLength_ < sizeof rx_)
        rx_[rxLength_++] = byte;
      else
        rxOverflow_ = true;
      continue;
    }
    uint8_t packet[sizeof rx_];
    const int size = rxOverflow_ ? -1 : cobsDecode(rx_, rxLength_, packet);
    const bool empty = rxLength_ == 0 && !rxOverflow_;
    rxLength_ = 0;
    rxOverflow_ = false;
    if (empty) continue;
    if (size < 5 || size > 5 + 63 || readLe<uint16_t>(packet + size - 2) != crc16(packet, uint8_t(size - 2))) {
      ++framingErrors_;  // e.g. the tail of a frame from before the port was opened
      continue;
    }
    dispatch(readLe<uint16_t>(packet), packet[2], packet + 3, uint8_t(size - 5));
  }
}

bool Bus::adapterStatus(AdapterStatus& out) const {
  if (!adapterSeen_) return false;
  out = adapter_;
  return true;
}

void Bus::dispatch(uint16_t canId, uint8_t opcode, const uint8_t* payload, uint8_t length) {
  if (canId == kAdapterId) {
    if ((opcode == kAdapterGetStatus || opcode == kAdapterResetCounters) && length >= kAdapterStatusSize) {
      adapter_.busState = payload[0];
      adapter_.txErrorCount = payload[1];
      adapter_.rxErrorCount = payload[2];
      adapter_.lastError = payload[3];
      adapter_.toCan = readLe<uint32_t>(payload + 4);
      adapter_.fromCan = readLe<uint32_t>(payload + 8);
      adapter_.droppedToCan = readLe<uint32_t>(payload + 12);
      adapter_.droppedFromCan = readLe<uint32_t>(payload + 16);
      adapter_.framingErrors = readLe<uint32_t>(payload + 20);
      adapter_.busOffEvents = readLe<uint16_t>(payload + 24);
      adapter_.receivedMs = millis();
      adapterSeen_ = true;
    }
    return;
  }
  const uint16_t function = canId & 0x780;
  const uint8_t node = canId & 0x7F;

  if (function == kFnReply && length >= 1) {
    if (node == waitNode_ && opcode == waitCmd_) {
      replyStatus_ = Status(payload[0]);
      replyLength_ = uint8_t(length - 1 > sizeof replyData_ ? sizeof replyData_ : length - 1);
      memcpy(replyData_, payload + 1, replyLength_);
      replied_ = true;
    }
  } else if (function == kFnTelemetry && opcode == kFrameTelemetry && length >= kTelemetrySize) {
    Slot* slot = slotFor(node, true);
    if (!slot) return;
    Telemetry& t = slot->telemetry;
    t.state = AxisState(payload[0]);
    t.flags = payload[1];
    t.faults = readLe<uint16_t>(payload + 2);
    t.warnings = readLe<uint16_t>(payload + 4);
    t.position = readLe<double>(payload + 6);
    t.target = readLe<double>(payload + 14);
    t.velocity = readLe<float>(payload + 22);
    t.followingError = readLe<float>(payload + 26);
    t.temperatureC = readLe<int16_t>(payload + 30) / 10.0f;
    t.supplyV = readLe<uint16_t>(payload + 32) / 1000.0f;
    t.procedureStep = payload[34];
    t.sequence = payload[35];
    t.controlLoadPct = payload[36];
    t.receivedMs = millis();
  } else if (function == kFnEvent && opcode == kFrameFault && length >= 5 && onFault_) {
    onFault_(node, readLe<uint16_t>(payload), readLe<uint16_t>(payload + 2), AxisState(payload[4]));
  }
}

Bus::Slot* Bus::slotFor(uint8_t node, bool create) {
  Slot* free = nullptr;
  for (Slot& s : slots_) {
    if (s.node == node) return &s;
    if (!free && s.node == 0) free = &s;
  }
  if (create && free) free->node = node;
  return create ? free : nullptr;
}

bool Bus::telemetry(uint8_t node, Telemetry& out) const {
  for (const Slot& s : slots_) {
    if (s.node == node) {
      out = s.telemetry;
      return true;
    }
  }
  return false;
}

Status Bus::request(uint8_t node, Cmd cmd, const uint8_t* payload, uint8_t length, uint8_t* reply,
                    uint8_t* replyLength) {
  waitNode_ = node;
  waitCmd_ = uint8_t(cmd);
  replied_ = false;
  send(kFnCommand + node, uint8_t(cmd), payload, length);
  const uint32_t start = millis();
  while (!replied_ && millis() - start < timeoutMs_) poll();
  waitCmd_ = 0xFF;
  if (!replied_) return Status::Timeout;
  if (reply && replyLength) {
    memcpy(reply, replyData_, replyLength_);
    *replyLength = replyLength_;
  }
  return replyStatus_;
}

void Bus::command(uint8_t node, Cmd cmd, const uint8_t* payload, uint8_t length) {
  send(kFnCommand + node, uint8_t(cmd) | kNoReply, payload, length);
}

void Bus::stopAll() { send(kBroadcast, uint8_t(Cmd::Stop)); }
void Bus::emergencyStopAll() { send(kBroadcast, uint8_t(Cmd::EmergencyStop)); }
void Bus::sync() { send(kBroadcast, uint8_t(Cmd::Sync)); }

// ---------------------------------------------------------------- Stepper ----

double Stepper::scale() const {
  switch (unit_) {
    case Unit::Degrees: return 360.0;
    case Unit::Radians: return kTwoPi;
    case Unit::Turns: break;
  }
  return 1.0;
}

Status Stepper::enableBridge(bool on) {
  const uint8_t value = on ? 1 : 0;
  return bus_.request(id_, Cmd::Enable, &value, 1);
}

Status Stepper::move(Cmd cmd, double value, float velocity, float acceleration, bool deferred, bool ack) {
  uint8_t p[17];
  uint8_t n = 0;
  n += writeLe(p + n, value / scale());
  n += writeLe(p + n, uint8_t(deferred ? kMoveDeferred : 0));
  n += writeLe(p + n, float(velocity / scale()));
  n += writeLe(p + n, float(acceleration / scale()));
  if (!ack) {
    bus_.command(id_, cmd, p, n);
    return Status::Ok;
  }
  return bus_.request(id_, cmd, p, n);
}

Status Stepper::moveTo(double position, float velocity, float acceleration, bool deferred, bool ack) {
  return move(Cmd::MoveTo, position, velocity, acceleration, deferred, ack);
}

Status Stepper::moveBy(double delta, float velocity, float acceleration, bool deferred, bool ack) {
  return move(Cmd::MoveBy, delta, velocity, acceleration, deferred, ack);
}

Status Stepper::setVelocity(float velocity, float acceleration) {
  uint8_t p[8];
  writeLe(p, float(velocity / scale()));
  writeLe(p + 4, float(acceleration / scale()));
  return bus_.request(id_, Cmd::SetVelocity, p, sizeof p);
}

Status Stepper::setZero(double position) {
  uint8_t p[8];
  writeLe(p, position / scale());
  return bus_.request(id_, Cmd::SetZero, p, sizeof p);
}

Status Stepper::autoTune(double minimum, double maximum) {
  uint8_t p[8];
  writeLe(p, float(minimum / scale()));
  writeLe(p + 4, float(maximum / scale()));
  return bus_.request(id_, Cmd::AutoTune, p, sizeof p);
}

Status Stepper::getParam(Param param, uint32_t& value) {
  uint8_t id = uint8_t(param), reply[8], length = 0;
  const Status status = bus_.request(id_, Cmd::GetParam, &id, 1, reply, &length);
  if (status == Status::Ok && length >= 5) value = readLe<uint32_t>(reply + 1);
  return status;
}

Status Stepper::getParam(Param param, float& value) {
  uint32_t raw = 0;
  const Status status = getParam(param, raw);
  memcpy(&value, &raw, 4);
  return status;
}

Status Stepper::setParam(Param param, uint32_t value) {
  uint8_t p[5] = {uint8_t(param)};
  writeLe(p + 1, value);
  return bus_.request(id_, Cmd::SetParam, p, sizeof p);
}

Status Stepper::setParam(Param param, float value) {
  uint32_t raw;
  memcpy(&raw, &value, 4);
  return setParam(param, raw);
}

Status Stepper::setNodeId(uint8_t newId) {
  const Status status = setParam(Param::NodeId, uint32_t(newId));
  if (status == Status::Ok) id_ = newId;
  return status;
}

bool Stepper::position(double& out) const {
  Telemetry t;
  if (!telemetry(t)) return false;
  out = t.position * scale();
  return true;
}

}  // namespace Tercio
