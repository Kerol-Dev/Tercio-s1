#pragma once
// -----------------------------------------------------------------------------
// Tercio host library for Arduino-class boards — CAN protocol v2, adapter
// serial framing v2.
//
// Talks to Tercio S1 drivers through the Tercio FD adapter over any Stream.
// Serial frames are [id u16][opcode u8][payload][crc16 u16] (little-endian),
// COBS-encoded and 0x00-terminated; id 0x800 is the adapter itself.
// No dynamic allocation. Call bus.poll() often (every loop iteration).
//
//   Tercio::Bus bus(Serial1);
//   Tercio::Stepper motor(bus, 1, Tercio::Unit::Degrees);
//   motor.enable();
//   motor.moveTo(90.0);
//
// Wire contracts: Tercio-s1/Firmware/src/protocol/Protocol.h (CAN) and
// Tercio-fdcan/Firmware/src/bridge/{Framing,AdapterProtocol}.h (serial).
// -----------------------------------------------------------------------------
#include <Arduino.h>

namespace Tercio {

enum class Cmd : uint8_t {
  GetInfo = 0x00, SaveConfig = 0x01, FactoryReset = 0x02, Reboot = 0x03, ClearFaults = 0x04,
  AssignNodeId = 0x05, GetParam = 0x08, SetParam = 0x09,
  Enable = 0x10, Stop = 0x11, EmergencyStop = 0x12, MoveTo = 0x13, MoveBy = 0x14,
  SetVelocity = 0x15, SetZero = 0x16, Sync = 0x17,
  Calibrate = 0x20, Home = 0x21, AutoTune = 0x22,
};

enum class Status : uint8_t {
  Ok = 0, UnknownCommand, BadLength, BadValue, UnknownParam, ReadOnly, Busy,
  NotCalibrated, Faulted, Disabled, StorageError,
  Timeout = 0xFF,  // host side: no reply
};

enum class AxisState : uint8_t {
  Disabled = 0, Holding, Moving, Velocity, StepDir, Calibrating, Homing, Tuning, Fault,
};

enum class Param : uint8_t {
  NodeId = 0x01, TelemetryRateHz = 0x02, CommandTimeoutMs = 0x03,
  FullStepsPerRev = 0x10, Microsteps = 0x11, RunCurrentMa = 0x12, HoldCurrentPct = 0x13,
  StealthChop = 0x14, StealthChopMaxVel = 0x15, InvertDirection = 0x16, GearRatio = 0x17,
  MaxVelocity = 0x20, MaxAcceleration = 0x21, Kp = 0x22, Ki = 0x23, Kd = 0x24,
  PositionDeadband = 0x25, FollowingErrorLimit = 0x26, StallTimeoutMs = 0x27,
  SoftLimitMin = 0x28, SoftLimitMax = 0x29, EnableOnBoot = 0x2A,
  EncoderType = 0x30, EncoderInvert = 0x31, Calibrated = 0x32,
  LimitSwitchesEnabled = 0x40, LimitSwitchActiveLow = 0x41, StepDirMode = 0x42, StepDirEnableActiveLow = 0x43,
  HomingMode = 0x50, HomingVelocity = 0x51, HomingCurrentMa = 0x52, HomingBackoff = 0x53,
  HomingStallError = 0x54, HomingTimeoutS = 0x55,
  OverTemperatureC = 0x60,
};

namespace fault {
constexpr uint16_t OverTemperature = 1u << 0, DriverOverTemp = 1u << 1, DriverFault = 1u << 2,
                   DriverComm = 1u << 3, Encoder = 1u << 4, Stall = 1u << 5, CalibrationFailed = 1u << 6,
                   HomingFailed = 1u << 7, CommandTimeout = 1u << 8;
}
namespace flag {
constexpr uint8_t Enabled = 1u << 0, Calibrated = 1u << 1, Homed = 1u << 2, Settled = 1u << 3,
                  LimitMin = 1u << 4, LimitMax = 1u << 5, MovePending = 1u << 6, ExtEnable = 1u << 7;
}

// Position unit for a Stepper (the wire carries turns).
enum class Unit : uint8_t { Turns, Degrees, Radians };

// Latest periodic status of one node, in turns.
struct Telemetry {
  AxisState state = AxisState::Disabled;
  uint8_t flags = 0;
  uint16_t faults = 0;
  uint16_t warnings = 0;
  double position = 0.0;
  double target = 0.0;
  float velocity = 0.0f;
  float followingError = 0.0f;
  float temperatureC = 0.0f;
  float supplyV = 0.0f;
  uint8_t procedureStep = 0;
  uint8_t sequence = 0;
  uint8_t controlLoadPct = 0;
  uint32_t receivedMs = 0;
};

// Bus health reported by the adapter every 250 ms.
struct AdapterStatus {
  uint8_t busState = 0;  // 0 error-active, 1 warning, 2 error-passive, 3 bus-off
  uint8_t txErrorCount = 0;
  uint8_t rxErrorCount = 0;
  uint8_t lastError = 0;
  uint32_t toCan = 0;
  uint32_t fromCan = 0;
  uint32_t droppedToCan = 0;
  uint32_t droppedFromCan = 0;
  uint32_t framingErrors = 0;
  uint16_t busOffEvents = 0;
  uint32_t receivedMs = 0;
};

using FaultCallback = void (*)(uint8_t node, uint16_t faults, uint16_t warnings, AxisState state);

class Bus {
 public:
  explicit Bus(Stream& io, uint32_t timeoutMs = 200) : io_(io), timeoutMs_(timeoutMs) {}

  // Parses everything received so far. Call every loop iteration.
  void poll();

  void send(uint16_t canId, uint8_t opcode, const uint8_t* payload = nullptr, uint8_t length = 0);
  // Sends a command and waits (polling) for its reply. `reply` receives the data after the status.
  Status request(uint8_t node, Cmd cmd, const uint8_t* payload = nullptr, uint8_t length = 0,
                 uint8_t* reply = nullptr, uint8_t* replyLength = nullptr);
  // Fire-and-forget: the node only answers if the command fails.
  void command(uint8_t node, Cmd cmd, const uint8_t* payload = nullptr, uint8_t length = 0);

  // Broadcast helpers.
  void stopAll();
  void emergencyStopAll();
  void sync();  // start moves queued with `deferred`

  bool telemetry(uint8_t node, Telemetry& out) const;
  void onFault(FaultCallback callback) { onFault_ = callback; }
  // False until the adapter has reported once.
  bool adapterStatus(AdapterStatus& out) const;
  uint32_t framingErrors() const { return framingErrors_; }

 private:
  static constexpr uint8_t kMaxNodes = 16;
  struct Slot {
    uint8_t node = 0;  // 0 = free
    Telemetry telemetry;
  };

  void dispatch(uint16_t canId, uint8_t opcode, const uint8_t* payload, uint8_t length);
  Slot* slotFor(uint8_t node, bool create);

  Stream& io_;
  uint32_t timeoutMs_;
  uint8_t rx_[72] = {};  // one COBS-encoded frame (max 69 bytes)
  uint8_t rxLength_ = 0;
  bool rxOverflow_ = false;
  uint32_t framingErrors_ = 0;
  AdapterStatus adapter_{};
  bool adapterSeen_ = false;
  Slot slots_[kMaxNodes];
  FaultCallback onFault_ = nullptr;

  // Reply being waited for.
  uint8_t waitNode_ = 0;
  uint8_t waitCmd_ = 0xFF;
  bool replied_ = false;
  Status replyStatus_ = Status::Ok;
  uint8_t replyData_[62] = {};
  uint8_t replyLength_ = 0;
};

class Stepper {
 public:
  Stepper(Bus& bus, uint8_t nodeId, Unit unit = Unit::Degrees) : bus_(bus), id_(nodeId), unit_(unit) {}

  uint8_t id() const { return id_; }

  // System
  Status save() { return bus_.request(id_, Cmd::SaveConfig); }
  Status clearFaults() { return bus_.request(id_, Cmd::ClearFaults); }
  Status reboot() { return bus_.request(id_, Cmd::Reboot); }
  Status getParam(Param param, float& value);
  Status getParam(Param param, uint32_t& value);
  Status setParam(Param param, float value);
  Status setParam(Param param, uint32_t value);
  Status setNodeId(uint8_t newId);  // applied immediately; call save() to keep it

  // Motion (positions in this stepper's unit; velocity/acceleration 0 = use the limits)
  Status enable() { return enableBridge(true); }
  Status disable() { return enableBridge(false); }
  Status stop() { return bus_.request(id_, Cmd::Stop); }
  Status emergencyStop() { return bus_.request(id_, Cmd::EmergencyStop); }
  Status moveTo(double position, float velocity = 0, float acceleration = 0, bool deferred = false, bool ack = true);
  Status moveBy(double delta, float velocity = 0, float acceleration = 0, bool deferred = false, bool ack = true);
  Status setVelocity(float velocity, float acceleration = 0);
  Status setZero(double position = 0.0);

  // Procedures (return once started; watch telemetry().state)
  Status calibrate() { return bus_.request(id_, Cmd::Calibrate); }
  Status home() { return bus_.request(id_, Cmd::Home); }
  Status autoTune(double minimum, double maximum);

  // Status
  bool telemetry(Telemetry& out) const { return bus_.telemetry(id_, out); }
  bool position(double& out) const;

 private:
  Status enableBridge(bool on);
  Status move(Cmd cmd, double value, float velocity, float acceleration, bool deferred, bool ack);
  double scale() const;

  Bus& bus_;
  uint8_t id_;
  Unit unit_;
};

}  // namespace Tercio
