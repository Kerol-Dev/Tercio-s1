#pragma once
// -----------------------------------------------------------------------------
// The CAN protocol endpoint (see protocol/Protocol.h): decodes commands,
// enforces parameter access rules, replies, streams telemetry, reports faults
// as events and owns persistence.
// -----------------------------------------------------------------------------
#include <cstdint>

#include "app/Settings.h"
#include "protocol/Protocol.h"

class Axis;
class FlashStore;
struct CanFrame;

class Node {
 public:
  Node(Settings& settings, Axis& axis, FlashStore& store) : settings_(settings), axis_(axis), store_(store) {}

  bool begin(uint32_t nowMs);
  // Main loop, every iteration.
  void service(uint32_t nowMs);

 private:
  void handle(const CanFrame& frame, uint32_t nowMs);
  proto::Status execute(proto::Cmd cmd, proto::Reader& args, uint8_t* out, uint8_t& outLength, uint32_t nowMs);
  void handleBroadcast(proto::Cmd cmd, proto::Reader& args, uint32_t nowMs);
  proto::Status setParam(proto::Param id, uint32_t raw);
  proto::Status save();
  void reply(uint8_t cmd, proto::Status status, const uint8_t* data, uint8_t length);
  uint8_t writeInfo(uint8_t* out) const;
  void sendTelemetry();
  void sendFaultEvent();
  void changeNodeId(uint8_t id);

  Settings& settings_;
  Axis& axis_;
  FlashStore& store_;

  uint8_t nodeId_ = 1;
  uint32_t nextTelemetryMs_ = 0;
  uint8_t sequence_ = 0;
  uint16_t reportedFaults_ = 0;
  bool storageWarning_ = false;
  bool savePending_ = false;
  uint32_t infoReplyAtMs_ = 0;
  bool infoReplyPending_ = false;
  uint32_t rebootAtMs_ = 0;
  bool rebootPending_ = false;
  uint8_t pendingNodeId_ = 0;  // applied after the reply to SetParam(NodeId) went out
};
