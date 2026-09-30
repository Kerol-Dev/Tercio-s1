#include "app/Node.h"

#include <Arduino.h>

#include <cmath>
#include <cstring>

#include "app/Axis.h"
#include "app/Params.h"
#include "board/Board.h"
#include "board/Version.h"
#include "drivers/Analog.h"
#include "drivers/CanBus.h"
#include "drivers/ControlTimer.h"
#include "drivers/Crc32.h"
#include "drivers/FlashStore.h"

namespace {
using namespace proto;

constexpr uint32_t kCanIrqPriority = 4;
constexpr uint32_t kRebootDelayMs = 20;       // let the reply leave first
constexpr uint32_t kDiscoverySlots = 64;      // broadcast GetInfo replies spread over 64 ms

void readUid(uint8_t out[kUidSize]) {
  const uint32_t words[3] = {HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2()};
  std::memcpy(out, words, kUidSize);
}

}  // namespace

bool Node::begin(uint32_t nowMs) {
  nodeId_ = settings_.nodeId;
  nextTelemetryMs_ = nowMs;
  return can_bus::begin(canId(Function::Command, nodeId_), kBroadcastId, kCanIrqPriority);
}

void Node::service(uint32_t nowMs) {
  can_bus::service(nowMs);

  CanFrame frame;
  while (can_bus::receive(frame)) handle(frame, nowMs);

  if (infoReplyPending_ && static_cast<int32_t>(nowMs - infoReplyAtMs_) >= 0) {
    infoReplyPending_ = false;
    uint8_t info[kInfoSize];
    reply(static_cast<uint8_t>(Cmd::GetInfo), Status::Ok, info, writeInfo(info));
  }
  if (pendingNodeId_) {
    changeNodeId(pendingNodeId_);
    pendingNodeId_ = 0;
  }
  if (rebootPending_ && static_cast<int32_t>(nowMs - rebootAtMs_) >= 0) NVIC_SystemReset();

  // Results of calibration/tuning are saved as soon as the motor stands still.
  if (axis_.takeSaveRequest()) savePending_ = true;
  if (savePending_ && axis_.isStill()) {
    savePending_ = false;
    (void)save();
  }

  // Newly raised faults go out immediately, not at the next telemetry frame.
  const uint16_t faults = axis_.faults();
  if (faults & ~reportedFaults_) sendFaultEvent();
  reportedFaults_ = faults;

  if (settings_.telemetryRateHz && static_cast<int32_t>(nowMs - nextTelemetryMs_) >= 0) {
    nextTelemetryMs_ = nowMs + 1000u / settings_.telemetryRateHz;
    sendTelemetry();
  }
}

// ---------------------------------------------------------------- dispatch ---

void Node::handle(const CanFrame& frame, uint32_t nowMs) {
  if (frame.length == 0) return;
  const bool broadcast = frame.id == kBroadcastId;
  const uint8_t opcode = frame.data[0];
  const auto cmd = static_cast<Cmd>(opcode & ~kNoReply);
  Reader args(frame.data + 1, static_cast<uint8_t>(frame.length - 1));
  axis_.noteHostActivity(nowMs);

  if (broadcast) {
    handleBroadcast(cmd, args, nowMs);
    return;
  }

  uint8_t out[62];
  uint8_t outLength = 0;
  const Status status = execute(cmd, args, out, outLength, nowMs);
  if (status != Status::Ok || !(opcode & kNoReply)) reply(static_cast<uint8_t>(cmd), status, out, outLength);
}

void Node::handleBroadcast(Cmd cmd, Reader& args, uint32_t nowMs) {
  switch (cmd) {
    case Cmd::GetInfo: {
      // Stagger answers by UID so nodes sharing an ID can still be told apart.
      uint8_t uid[kUidSize];
      readUid(uid);
      infoReplyAtMs_ = nowMs + crc32::update(0, uid, kUidSize) % kDiscoverySlots;
      infoReplyPending_ = true;
      break;
    }
    case Cmd::AssignNodeId: {
      uint8_t uid[kUidSize];
      readUid(uid);
      uint8_t newId = 0;
      if (args.remaining() < kUidSize + 1 || std::memcmp(args.cursor(), uid, kUidSize) != 0) break;
      Reader rest(args.cursor() + kUidSize, 1);
      rest.get(newId);
      if (newId < 1 || newId > kMaxNodeId) break;
      settings_.nodeId = newId;
      changeNodeId(newId);
      reply(static_cast<uint8_t>(Cmd::AssignNodeId), Status::Ok, nullptr, 0);
      break;
    }
    case Cmd::Stop: axis_.stop(); break;
    case Cmd::EmergencyStop: axis_.emergencyStop(); break;
    case Cmd::Sync: axis_.sync(); break;
    case Cmd::Enable: {
      uint8_t on = 0;
      if (args.get(on)) axis_.enable(on != 0);
      break;
    }
    default: break;  // other commands are node-addressed only
  }
}

Status Node::execute(Cmd cmd, Reader& args, uint8_t* out, uint8_t& outLength, uint32_t nowMs) {
  switch (cmd) {
    // ---- System ----
    case Cmd::GetInfo:
      outLength = writeInfo(out);
      return Status::Ok;

    case Cmd::SaveConfig:
      return axis_.isStill() ? save() : Status::Busy;

    case Cmd::FactoryReset: {
      if (!axis_.isDisabled()) return Status::Busy;
      const uint8_t id = settings_.nodeId;  // keep the node reachable
      settings_ = Settings{};
      settings_.nodeId = id;
      axis_.applySettings();
      return Status::Ok;
    }

    case Cmd::Reboot:
      rebootPending_ = true;
      rebootAtMs_ = nowMs + kRebootDelayMs;
      return Status::Ok;

    case Cmd::ClearFaults:
      return axis_.clearFaults();

    case Cmd::GetParam: {
      uint8_t id = 0;
      if (!args.get(id)) return Status::BadLength;
      const params::Def* def = params::find(static_cast<Param>(id));
      if (!def) return Status::UnknownParam;
      outLength = Writer(out, 5).put(id).put(params::read(settings_, *def)).size();
      return Status::Ok;
    }

    case Cmd::SetParam: {
      uint8_t id = 0;
      uint32_t raw = 0;
      if (!args.get(id) || !args.get(raw)) return Status::BadLength;
      const Status status = setParam(static_cast<Param>(id), raw);
      if (status == Status::Ok) {
        const params::Def* def = params::find(static_cast<Param>(id));
        outLength = Writer(out, 5).put(id).put(params::read(settings_, *def)).size();
      }
      return status;
    }

    // ---- Motion ----
    case Cmd::Enable: {
      uint8_t on = 0;
      if (!args.get(on)) return Status::BadLength;
      return axis_.enable(on != 0);
    }

    case Cmd::Stop:
      return axis_.stop();

    case Cmd::EmergencyStop:
      return axis_.emergencyStop();

    case Cmd::MoveTo:
    case Cmd::MoveBy: {
      double position = 0.0;
      uint8_t flags = 0;
      float maxVelocity = 0.0f, maxAcceleration = 0.0f;
      if (!args.get(position) || !std::isfinite(position)) return Status::BadLength;
      args.getOr(flags);
      args.getOr(maxVelocity);
      args.getOr(maxAcceleration);
      const bool deferred = flags & kMoveDeferred;
      return cmd == Cmd::MoveTo ? axis_.moveTo(position, maxVelocity, maxAcceleration, deferred)
                                : axis_.moveBy(position, maxVelocity, maxAcceleration, deferred);
    }

    case Cmd::SetVelocity: {
      float velocity = 0.0f, maxAcceleration = 0.0f;
      if (!args.get(velocity) || !std::isfinite(velocity)) return Status::BadLength;
      args.getOr(maxAcceleration);
      return axis_.setVelocity(velocity, maxAcceleration);
    }

    case Cmd::SetZero: {
      double position = 0.0;
      args.getOr(position);
      if (!std::isfinite(position)) return Status::BadValue;
      return axis_.setZero(position);
    }

    case Cmd::Sync:
      axis_.sync();
      return Status::Ok;

    // ---- Procedures ----
    case Cmd::Calibrate:
      return axis_.calibrate();

    case Cmd::Home:
      return axis_.home();

    case Cmd::AutoTune: {
      float min = 0.0f, max = 0.0f;
      if (!args.get(min) || !args.get(max)) return Status::BadLength;
      return axis_.autoTune(min, max);
    }

    case Cmd::AssignNodeId:
      return Status::BadValue;  // broadcast only
  }
  return Status::UnknownCommand;
}

Status Node::setParam(Param id, uint32_t raw) {
  const params::Def* def = params::find(id);
  if (!def) return Status::UnknownParam;
  switch (def->access) {
    case params::Access::ReadOnly: return Status::ReadOnly;
    case params::Access::WhenDisabled:
      if (!axis_.isDisabled()) return Status::Busy;
      break;
    case params::Access::WhenStill:
      if (!axis_.isStill()) return Status::Busy;
      break;
    case params::Access::Always: break;
  }
  const Status status = params::write(settings_, *def, raw);
  if (status != Status::Ok) return status;
  if (id == Param::NodeId)
    pendingNodeId_ = settings_.nodeId;  // switch after the reply went out on the old ID
  else
    axis_.applySettings();
  return Status::Ok;
}

Status Node::save() {
  // Flash programming stalls the CPU (and the control interrupt) while TIM1
  // would keep stepping: hold the step output at zero for the duration.
  axis_.freezeOutput(true);
  const bool ok = store_.save(&settings_, sizeof settings_, Settings::kLayoutVersion);
  axis_.freezeOutput(false);
  storageWarning_ = !ok;
  return ok ? Status::Ok : Status::StorageError;
}

void Node::changeNodeId(uint8_t id) {
  nodeId_ = id;
  can_bus::setFilter(canId(Function::Command, id), kBroadcastId);
}

// ---------------------------------------------------------------- output -----

void Node::reply(uint8_t cmd, Status status, const uint8_t* data, uint8_t length) {
  uint8_t frame[64];
  Writer w(frame, sizeof frame);
  w.put(cmd).put(static_cast<uint8_t>(status));
  if (length) w.bytes(data, length);
  if (w.ok()) can_bus::send(canId(Function::Reply, nodeId_), frame, w.size());
}

uint8_t Node::writeInfo(uint8_t* out) const {
  InfoData info;
  info.fwMajor = version::kMajor;
  info.fwMinor = version::kMinor;
  info.fwPatch = version::kPatch;
  info.hardwareRevision = board::kHardwareRevision;
  info.encoderType = settings_.encoderType;
  info.nodeId = nodeId_;
  info.flags = board::g_runningOnCrystal ? kInfoCrystalClock : 0;
  readUid(info.uid);
  return encodeInfo(info, out);
}

void Node::sendTelemetry() {
  const MotionCore::Snapshot& s = axis_.snapshot();
  TelemetryData t;
  t.state = axis_.state();
  t.flags = axis_.flags();
  t.faults = axis_.faults();
  t.warnings = static_cast<uint16_t>(axis_.warnings() | (storageWarning_ ? warn::kStorage : 0));
  t.position = q32::toTurns(s.position);
  t.target = q32::toTurns(s.target);
  t.velocity = s.velocity;
  t.followingError = s.followingError;
  t.temperatureDeciC = static_cast<int16_t>(std::fmax(-32768.0f, std::fmin(32767.0f, std::round(analog::temperatureC() * 10.0f))));
  t.supplyMv = static_cast<uint16_t>(std::fmin(65535.0f, analog::supplyVolts() * 1000.0f));
  t.procedureStep = axis_.procedureStep();
  t.sequence = sequence_++;
  t.loadPct = static_cast<uint8_t>(std::fmin(control_timer::takePeakLoad() * 100.0f, 255.0f));

  uint8_t frame[1 + kTelemetrySize];
  can_bus::send(canId(Function::Telemetry, nodeId_), frame, encodeTelemetry(t, frame));
}

void Node::sendFaultEvent() {
  uint8_t frame[6];
  Writer w(frame, sizeof frame);
  w.put(kFrameFault).put(axis_.faults()).put(axis_.warnings()).put(static_cast<uint8_t>(axis_.state()));
  can_bus::send(canId(Function::Event, nodeId_), frame, w.size());
}
