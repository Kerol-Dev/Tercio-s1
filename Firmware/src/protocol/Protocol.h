#pragma once
// -----------------------------------------------------------------------------
// Tercio CAN protocol, version 2 — the wire contract between a driver and its
// hosts. The Python/C++ host libraries mirror this file; change them together.
//
// Bus: CAN-FD, 500 kbit/s arbitration, 2.5 Mbit/s data (BRS), 11-bit IDs.
//
// Identifier layout:  [10..7] function   [6..0] node id (1..127)
//
//   0x000         Broadcast   host -> all nodes
//   0x080 + node  Event       node -> host   (faults, asynchronous)
//   0x100 + node  Command     host -> node
//   0x180 + node  Reply       node -> host
//   0x200 + node  Telemetry   node -> host   (periodic)
//
// Every frame starts with an opcode byte. All multi-byte fields are
// little-endian; positions are f64 turns, velocities f32 turns/s,
// accelerations f32 turns/s². "Turns" are revolutions of the encoder shaft.
//
// Command:   [cmd][args...]          (cmd | kNoReply suppresses the Ok reply)
// Reply:     [cmd][status][data...]
// Telemetry: [kFrameTelemetry][TelemetryLayout]
// Event:     [kFrameFault][u16 faults][u16 warnings][u8 state]
// -----------------------------------------------------------------------------
#include <cstdint>
#include <cstring>

namespace proto {

inline constexpr uint8_t kProtocolVersion = 2;
inline constexpr uint8_t kMaxNodeId = 127;

// ---- Identifiers -------------------------------------------------------------
enum class Function : uint16_t {
  Broadcast = 0x000,
  Event = 0x080,
  Command = 0x100,
  Reply = 0x180,
  Telemetry = 0x200,
};

constexpr uint16_t canId(Function fn, uint8_t node) {
  return static_cast<uint16_t>(static_cast<uint16_t>(fn) | (node & 0x7F));
}
inline constexpr uint16_t kBroadcastId = 0x000;

// ---- Commands (host -> node) -------------------------------------------------
enum class Cmd : uint8_t {
  // System
  GetInfo = 0x00,       // -> InfoLayout. Broadcast: every node answers (staggered).
  SaveConfig = 0x01,    // persist parameters to flash (refused while moving)
  FactoryReset = 0x02,  // restore defaults in RAM (Save to persist)
  Reboot = 0x03,
  ClearFaults = 0x04,
  AssignNodeId = 0x05,  // broadcast only: [uid 12][new id]
  GetParam = 0x08,      // [param]            -> [param][value 4]
  SetParam = 0x09,      // [param][value 4]   -> [param][value 4] (as applied)

  // Motion
  Enable = 0x10,       // [u8 on]
  Stop = 0x11,         // decelerate to rest and hold
  EmergencyStop = 0x12,  // halt immediately and de-energise
  MoveTo = 0x13,       // [f64 pos][u8 flags][f32 vmax][f32 amax]  (trailing optional)
  MoveBy = 0x14,       // [f64 delta][u8 flags][f32 vmax][f32 amax]
  SetVelocity = 0x15,  // [f32 vel][f32 amax]                     (amax optional)
  SetZero = 0x16,      // [f64 pos] (optional, default 0): current position := pos
  Sync = 0x17,         // start moves queued with kMoveDeferred (usually broadcast)

  // Procedures
  Calibrate = 0x20,  // encoder direction/scale check, sets zero
  Home = 0x21,       // homing using the Homing* parameters
  AutoTune = 0x22,   // [f32 min][f32 max] find max velocity/acceleration
};
inline constexpr uint8_t kNoReply = 0x80;
inline constexpr uint8_t kMoveDeferred = 0x01;  // MoveTo/MoveBy flag: wait for Sync

// ---- Reply status --------------------------------------------------------------
enum class Status : uint8_t {
  Ok = 0,
  UnknownCommand = 1,
  BadLength = 2,
  BadValue = 3,
  UnknownParam = 4,
  ReadOnly = 5,
  Busy = 6,           // not allowed in the current state (moving, procedure running)
  NotCalibrated = 7,
  Faulted = 8,
  Disabled = 9,
  StorageError = 10,
};

// ---- Node-originated frame types -------------------------------------------------
inline constexpr uint8_t kFrameTelemetry = 0x01;
inline constexpr uint8_t kFrameFault = 0x02;

// ---- Axis state -------------------------------------------------------------------
enum class AxisState : uint8_t {
  Disabled = 0,     // bridge off
  Holding = 1,      // closed loop, at rest
  Moving = 2,       // executing a position move
  Velocity = 3,     // velocity mode
  StepDir = 4,      // following the external step/dir inputs
  Calibrating = 5,
  Homing = 6,
  Tuning = 7,
  Fault = 8,        // hard fault latched, bridge off
};

// ---- Bit sets ------------------------------------------------------------------
namespace fault {  // latched until ClearFaults (Stall also clears on a new move)
inline constexpr uint16_t kOverTemperature = 1u << 0;
inline constexpr uint16_t kDriverOverTemp = 1u << 1;
inline constexpr uint16_t kDriverFault = 1u << 2;     // bridge shut down: short circuit or supply fault (DIAG)
inline constexpr uint16_t kDriverComm = 1u << 3;
inline constexpr uint16_t kEncoder = 1u << 4;
inline constexpr uint16_t kStall = 1u << 5;
inline constexpr uint16_t kCalibrationFailed = 1u << 6;
inline constexpr uint16_t kHomingFailed = 1u << 7;
inline constexpr uint16_t kCommandTimeout = 1u << 8;
// Faults that de-energise the motor. The rest stop motion and hold position.
inline constexpr uint16_t kHardMask =
    kOverTemperature | kDriverOverTemp | kDriverFault | kDriverComm | kEncoder;
}  // namespace fault

namespace warn {  // live conditions, not latched
inline constexpr uint16_t kTemperatureHigh = 1u << 0;
inline constexpr uint16_t kDriverPreWarning = 1u << 1;
inline constexpr uint16_t kMagnetWeak = 1u << 2;
inline constexpr uint16_t kMagnetStrong = 1u << 3;
inline constexpr uint16_t kSupplyLow = 1u << 4;
inline constexpr uint16_t kCanErrorPassive = 1u << 5;
inline constexpr uint16_t kOpenLoad = 1u << 6;
inline constexpr uint16_t kStorage = 1u << 7;
}  // namespace warn

namespace flag {  // status flags in telemetry
inline constexpr uint8_t kEnabled = 1u << 0;
inline constexpr uint8_t kCalibrated = 1u << 1;
inline constexpr uint8_t kHomed = 1u << 2;
inline constexpr uint8_t kSettled = 1u << 3;  // at target within the deadband
inline constexpr uint8_t kLimitMin = 1u << 4;
inline constexpr uint8_t kLimitMax = 1u << 5;
inline constexpr uint8_t kMovePending = 1u << 6;  // deferred move waiting for Sync
inline constexpr uint8_t kExtEnable = 1u << 7;
}  // namespace flag

// ---- Parameters -------------------------------------------------------------------
// Values travel as 4 little-endian bytes: integers zero-extended, bools 0/1, f32 raw.
enum class Param : uint8_t {
  // Communication
  NodeId = 0x01,              // u8   1..127
  TelemetryRateHz = 0x02,     // u16  0 (off) .. 1000
  CommandTimeoutMs = 0x03,    // u16  0 (off) .. 60000; stops motion if the host goes quiet
  // Motor & driver
  FullStepsPerRev = 0x10,     // u16  motor full steps per revolution (200, 400)
  Microsteps = 0x11,          // u16  1..256, power of two
  RunCurrentMa = 0x12,        // u16  RMS run current
  HoldCurrentPct = 0x13,      // u8   standstill current, % of run current
  StealthChop = 0x14,         // bool
  StealthChopMaxVel = 0x15,   // f32  motor turns/s; above it the driver uses SpreadCycle
  InvertDirection = 0x16,     // bool flip the positive direction
  GearRatio = 0x17,           // f32  motor turns per encoder turn
  // Motion & control
  MaxVelocity = 0x20,         // f32  turns/s
  MaxAcceleration = 0x21,     // f32  turns/s²
  Kp = 0x22,                  // f32  1/s     (turns/s per turn of error)
  Ki = 0x23,                  // f32  1/s²
  Kd = 0x24,                  // f32  -       (on velocity error)
  PositionDeadband = 0x25,    // f32  turns
  FollowingErrorLimit = 0x26, // f32  turns; stall when exceeded for StallTimeoutMs
  StallTimeoutMs = 0x27,      // u16  0 disables stall detection
  SoftLimitMin = 0x28,        // f32  turns (active when min < max)
  SoftLimitMax = 0x29,        // f32  turns
  EnableOnBoot = 0x2A,        // bool
  // Encoder
  EncoderType = 0x30,         // u8   0 AS5600 on-board, 1 AS5600 external, 2 AS5048A SPI
  EncoderInvert = 0x31,       // bool (set by Calibrate)
  Calibrated = 0x32,          // bool read-only
  // Inputs
  LimitSwitchesEnabled = 0x40,  // bool stop at IN1 (min) / IN2 (max)
  LimitSwitchActiveLow = 0x41,  // bool
  StepDirMode = 0x42,           // bool follow external STEP/DIR/EN
  StepDirEnableActiveLow = 0x43,// bool
  // Homing
  HomingMode = 0x50,          // u8   0 switch min, 1 switch max, 2 sensorless -, 3 sensorless +
  HomingVelocity = 0x51,      // f32  turns/s
  HomingCurrentMa = 0x52,     // u16
  HomingBackoff = 0x53,       // f32  turns moved away from the trigger point before zeroing
  HomingStallError = 0x54,    // f32  turns of following error that count as a hard stop
  HomingTimeoutS = 0x55,      // u16
  // Protection
  OverTemperatureC = 0x60,    // f32  board temperature fault threshold
};

enum class EncoderType : uint8_t { As5600Internal = 0, As5600External = 1, As5048a = 2 };

enum class HomingMode : uint8_t { SwitchMin = 0, SwitchMax = 1, SensorlessNeg = 2, SensorlessPos = 3 };

// ---- Payload layouts ------------------------------------------------------------------
// Telemetry (after the frame type byte), 37 bytes:
//   u8  state            u8  flags          u16 faults        u16 warnings
//   f64 position         f64 target         f32 velocity      f32 following error
//   i16 temperature (0.1 °C)                u16 supply (mV)
//   u8  procedure step   u8  sequence       u8  control-loop peak load (% of the 50 µs tick)
inline constexpr uint8_t kTelemetrySize = 1 + 1 + 2 + 2 + 8 + 8 + 4 + 4 + 2 + 2 + 1 + 1 + 1;
static_assert(kTelemetrySize == 37, "update the host libraries when the telemetry layout changes");

// GetInfo reply data, 20 bytes:
//   u8 protocol  u8 fw major  u8 fw minor  u8 fw patch  u8 hw revision
//   u8 encoder type  u8 node id  u8 info flags  u8 uid[12]
inline constexpr uint8_t kInfoSize = 20;
inline constexpr uint8_t kUidSize = 12;
inline constexpr uint8_t kInfoCrystalClock = 1u << 0;  // info flags: core runs from the crystal

// ---- Little-endian codec ----------------------------------------------------------------
class Writer {
 public:
  Writer(uint8_t* buf, uint8_t capacity) : buf_(buf), cap_(capacity) {}
  template <typename T>
  Writer& put(T value) {
    if (len_ + sizeof(T) <= cap_) std::memcpy(buf_ + len_, &value, sizeof(T));
    len_ = static_cast<uint8_t>(len_ + sizeof(T));
    return *this;
  }
  Writer& bytes(const uint8_t* data, uint8_t n) {
    if (len_ + n <= cap_) std::memcpy(buf_ + len_, data, n);
    len_ = static_cast<uint8_t>(len_ + n);
    return *this;
  }
  uint8_t size() const { return len_; }
  bool ok() const { return len_ <= cap_; }

 private:
  uint8_t* buf_;
  uint8_t cap_;
  uint8_t len_ = 0;
};

class Reader {
 public:
  Reader(const uint8_t* buf, uint8_t len) : buf_(buf), len_(len) {}
  template <typename T>
  bool get(T& out) {
    if (pos_ + sizeof(T) > len_) return false;
    std::memcpy(&out, buf_ + pos_, sizeof(T));
    pos_ = static_cast<uint8_t>(pos_ + sizeof(T));
    return true;
  }
  // Optional trailing field: keeps `out` unchanged when absent.
  template <typename T>
  void getOr(T& out) { (void)get(out); }
  uint8_t remaining() const { return static_cast<uint8_t>(len_ - pos_); }
  const uint8_t* cursor() const { return buf_ + pos_; }

 private:
  const uint8_t* buf_;
  uint8_t len_;
  uint8_t pos_ = 0;
};


// ---- Frame builders ------------------------------------------------------------------------
struct TelemetryData {
  AxisState state = AxisState::Disabled;
  uint8_t flags = 0;
  uint16_t faults = 0;
  uint16_t warnings = 0;
  double position = 0.0;
  double target = 0.0;
  float velocity = 0.0f;
  float followingError = 0.0f;
  int16_t temperatureDeciC = 0;
  uint16_t supplyMv = 0;
  uint8_t procedureStep = 0;
  uint8_t sequence = 0;
  uint8_t loadPct = 0;
};

// Writes [kFrameTelemetry][layout]; `out` holds at least 1 + kTelemetrySize bytes.
inline uint8_t encodeTelemetry(const TelemetryData& t, uint8_t* out) {
  Writer w(out, 1 + kTelemetrySize);
  w.put(kFrameTelemetry).put(static_cast<uint8_t>(t.state)).put(t.flags).put(t.faults).put(t.warnings);
  w.put(t.position).put(t.target).put(t.velocity).put(t.followingError);
  w.put(t.temperatureDeciC).put(t.supplyMv).put(t.procedureStep).put(t.sequence).put(t.loadPct);
  return w.size();
}

struct InfoData {
  uint8_t fwMajor = 0, fwMinor = 0, fwPatch = 0;
  uint8_t hardwareRevision = 0;
  uint8_t encoderType = 0;
  uint8_t nodeId = 0;
  uint8_t flags = 0;
  uint8_t uid[kUidSize] = {};
};

// Writes the GetInfo reply data; `out` holds at least kInfoSize bytes.
inline uint8_t encodeInfo(const InfoData& info, uint8_t* out) {
  Writer w(out, kInfoSize);
  w.put(kProtocolVersion).put(info.fwMajor).put(info.fwMinor).put(info.fwPatch);
  w.put(info.hardwareRevision).put(info.encoderType).put(info.nodeId).put(info.flags);
  w.bytes(info.uid, kUidSize);
  return w.size();
}

}  // namespace proto
