#pragma once
// -----------------------------------------------------------------------------
// Multi-step procedures, each a non-blocking state machine stepped from the
// main loop (~1 kHz). They command MotionCore and watch its snapshot/events;
// nothing here waits, so CAN stays responsive and any procedure can be aborted.
// -----------------------------------------------------------------------------
#include <cstdint>

#include "app/MotionCore.h"
#include "app/Settings.h"

enum class ProcedureResult : uint8_t { Running, Succeeded, Failed };

struct ProcedureContext {
  MotionCore& core;
  const Settings& settings;
  const MotionCore::Snapshot& snapshot;
  uint8_t events;  // MotionCore events collected this cycle
  uint32_t nowMs;
};

// Open-loop sweep that finds the encoder direction and checks its scale.
class Calibration {
 public:
  void start(ProcedureContext& ctx, uint8_t encoderBits);
  ProcedureResult service(ProcedureContext& ctx);
  uint8_t step() const { return static_cast<uint8_t>(phase_); }
  bool encoderInverted() const { return inverted_; }

 private:
  enum class Phase : uint8_t { SettleStart, Forward, SettleForward, Backward, SettleBack };
  void enter(Phase phase, uint32_t nowMs) {
    phase_ = phase;
    phaseStartMs_ = nowMs;
  }

  Phase phase_ = Phase::SettleStart;
  uint32_t phaseStartMs_ = 0;
  int64_t startCounts_ = 0, forwardCounts_ = 0;
  float expectedCounts_ = 0.0f;
  bool inverted_ = false;
};

// Seek a switch or a hard stop, back off, and declare that point zero.
class Homing {
 public:
  void start(ProcedureContext& ctx);
  ProcedureResult service(ProcedureContext& ctx);
  uint8_t step() const { return static_cast<uint8_t>(phase_); }
  // True while the reduced homing current should be applied.
  bool reducedCurrent() const { return phase_ != Phase::Done; }

 private:
  enum class Phase : uint8_t { Seek, Settle, Backoff, Done };

  Phase phase_ = Phase::Done;
  uint32_t phaseStartMs_ = 0;
  uint32_t doneSinceMs_ = 0;
  float direction_ = 1.0f;
};

// Raises velocity/acceleration in steps between two positions until the motor
// stalls, then confirms the last good level three times. Result replaces the
// MaxVelocity/MaxAcceleration settings.
class LimitTuner {
 public:
  static constexpr float kStartVelocity = 5.0f, kVelocityStep = 3.0f;           // turns/s
  static constexpr float kStartAcceleration = 30.0f, kAccelerationStep = 8.0f;  // turns/s²
  static constexpr uint8_t kMaxLevel = 18;                                      // 59 turns/s
  static constexpr float kMaxAcceleration = kStartAcceleration + kAccelerationStep * kMaxLevel;
  // Travel needed to reach level 0's velocity and stop again.
  static constexpr float minimumRange() { return kStartVelocity * kStartVelocity / kStartAcceleration; }

  void start(ProcedureContext& ctx, q32::Pos min, q32::Pos max);
  ProcedureResult service(ProcedureContext& ctx);
  uint8_t step() const { return level_; }
  float velocity() const { return velocityAt(level_); }
  float acceleration() const { return accelerationAt(level_); }

 private:
  enum class Phase : uint8_t { ToStart, Forward, Backward };
  static constexpr float velocityAt(uint8_t level) { return kStartVelocity + kVelocityStep * level; }
  static constexpr float accelerationAt(uint8_t level) { return kStartAcceleration + kAccelerationStep * level; }
  bool reachable(uint8_t level) const;
  void move(ProcedureContext& ctx, Phase phase);

  Phase phase_ = Phase::ToStart;
  q32::Pos min_ = 0, max_ = 0;
  float range_ = 0.0f;
  uint8_t level_ = 0;
  uint8_t passes_ = 0;
  uint8_t stallsAtFloor_ = 0;
  bool verifying_ = false;
  uint32_t phaseStartMs_ = 0;
  uint32_t doneSinceMs_ = 0;
};
