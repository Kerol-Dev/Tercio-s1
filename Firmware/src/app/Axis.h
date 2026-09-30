#pragma once
// -----------------------------------------------------------------------------
// The axis as the outside world sees it: state machine, command validation,
// procedures and fault management. Runs in the main loop; the real-time work
// is delegated to MotionCore.
//
// Fault policy
//   hard faults (over-temperature, driver, encoder): de-energise, state Fault;
//     ClearFaults once the cause is gone, then Enable again.
//   soft faults (stall, failed procedure, command timeout): stop and hold;
//     cleared by ClearFaults or, for stall/timeout, by the next motion command.
// -----------------------------------------------------------------------------
#include <cstdint>

#include "app/MotionCore.h"
#include "app/Procedures.h"
#include "app/Settings.h"
#include "protocol/Protocol.h"

class Encoder;
class Tmc2209;

class Axis {
 public:
  struct Hardware {
    MotionCore& core;
    Tmc2209& driver;
    Encoder* const* encoders;  // indexed by proto::EncoderType
  };

  Axis(Settings& settings, const Hardware& hardware) : settings_(settings), hw_(hardware) {}

  void begin(uint32_t nowMs);
  // Main loop, every millisecond.
  void service(uint32_t nowMs);
  // Main loop, every 10 ms: encoder diagnostics and bus recovery.
  void serviceSlow();
  // Re-derives every hardware setting from `settings`; call after a parameter write.
  void applySettings();
  // Host traffic for this node, for the command timeout.
  void noteHostActivity(uint32_t nowMs) { lastHostMs_ = nowMs; }

  // ---- Commands ----
  proto::Status enable(bool on);
  proto::Status stop();
  proto::Status emergencyStop();
  proto::Status moveTo(double position, float maxVelocity, float maxAcceleration, bool deferred);
  proto::Status moveBy(double delta, float maxVelocity, float maxAcceleration, bool deferred);
  proto::Status setVelocity(float velocity, float maxAcceleration);
  proto::Status setZero(double position);
  void sync();
  proto::Status calibrate();
  proto::Status home();
  proto::Status autoTune(float min, float max);
  proto::Status clearFaults();

  // ---- Status ----
  proto::AxisState state() const;
  uint16_t faults() const { return faults_; }
  uint16_t warnings() const { return warnings_; }
  uint8_t flags() const;
  uint8_t procedureStep() const;
  const MotionCore::Snapshot& snapshot() const { return snapshot_; }
  // Not moving and no procedure: parameters that affect motion may change, flash may be written.
  bool isStill() const;
  bool isDisabled() const { return !enabled_; }
  // Set when a procedure produced results that should be persisted.
  bool takeSaveRequest();
  // Holds the step output at zero (around flash writes, which stall the CPU).
  void freezeOutput(bool frozen) { hw_.core.freezeOutput(frozen); }

 private:
  enum class Procedure : uint8_t { None, Calibration, Homing, Tuning };

  proto::Status checkMotionAllowed() const;
  proto::Status startMove(q32::Pos target, float maxVelocity, float maxAcceleration, bool deferred);
  void resumeNormalMode();
  void startProcedure(Procedure procedure);
  void finishProcedure(ProcedureResult result);
  void raise(uint16_t fault);
  void updateDriver();
  void superviseFaults(uint32_t nowMs, uint8_t events);
  bool selectEncoder();
  Encoder* encoder() const;

  Settings& settings_;
  Hardware hw_;
  MotionCore::Snapshot snapshot_{};

  bool enabled_ = false;
  bool homed_ = false;
  bool extGateOpen_ = true;
  bool appliedStepDirMode_ = false;  // step/dir setting last acted on
  int8_t appliedSign_ = 0;           // direction · encoder sign last configured
  bool saveRequested_ = false;
  uint16_t faults_ = 0;
  uint16_t warnings_ = 0;
  uint32_t lastHostMs_ = 0;
  uint32_t nowMs_ = 0;
  uint16_t diagMs_ = 0;  // consecutive milliseconds with DIAG high

  int8_t activeEncoder_ = -1;  // encoder currently begun, -1 none

  Procedure procedure_ = Procedure::None;
  Calibration calibration_{};
  Homing homing_{};
  LimitTuner tuner_{};
  q32::Pos tunerMin_ = 0, tunerMax_ = 0;

  struct PendingMove {
    bool valid = false;
    q32::Pos target = 0;
    float maxVelocity = 0.0f;
    float maxAcceleration = 0.0f;
  } pending_{};
};
