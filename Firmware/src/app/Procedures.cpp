#include "app/Procedures.h"

#include <cmath>

#include "protocol/Protocol.h"

// ================================================================ Calibration

namespace {
constexpr float kSweepTurns = 0.25f;        // motor turns each way
constexpr float kSweepVelocity = 0.5f;      // motor turns/s
constexpr float kSweepAcceleration = 5.0f;  // motor turns/s²
constexpr uint32_t kSettleMs = 300;
constexpr uint32_t kMoveTimeoutMs = 5000;
constexpr float kScaleTolerance = 0.5f;     // accept 50 %..150 % of the expected travel
constexpr float kReturnTolerance = 0.25f;   // must come back within 25 % of the sweep
}  // namespace

void Calibration::start(ProcedureContext& ctx, uint8_t encoderBits) {
  ctx.core.setIdle();  // energised, no steps, no feedback
  expectedCounts_ = kSweepTurns / ctx.settings.gearRatio * static_cast<float>(1u << encoderBits);
  enter(Phase::SettleStart, ctx.nowMs);
}

ProcedureResult Calibration::service(ProcedureContext& ctx) {
  const uint32_t elapsed = ctx.nowMs - phaseStartMs_;
  const bool moveDone = ctx.snapshot.openLoopDone && ctx.snapshot.mode == MotionCore::Mode::OpenLoop;

  switch (phase_) {
    case Phase::SettleStart:
      if (elapsed < kSettleMs) break;
      startCounts_ = ctx.snapshot.counts;
      ctx.core.openLoopMove(kSweepTurns, kSweepVelocity, kSweepAcceleration);
      enter(Phase::Forward, ctx.nowMs);
      break;

    case Phase::Forward:
      if (moveDone) enter(Phase::SettleForward, ctx.nowMs);
      else if (elapsed > kMoveTimeoutMs) return ProcedureResult::Failed;
      break;

    case Phase::SettleForward: {
      if (elapsed < kSettleMs) break;
      forwardCounts_ = ctx.snapshot.counts;
      const float moved = static_cast<float>(forwardCounts_ - startCounts_);
      const float ratio = std::fabs(moved) / expectedCounts_;
      // Too little: magnet missing or motor not turning. Too much: wrong steps/rev or gear ratio.
      if (ratio < 1.0f - kScaleTolerance || ratio > 1.0f + kScaleTolerance) return ProcedureResult::Failed;
      inverted_ = moved < 0.0f;
      ctx.core.openLoopMove(-kSweepTurns, kSweepVelocity, kSweepAcceleration);
      enter(Phase::Backward, ctx.nowMs);
      break;
    }

    case Phase::Backward:
      if (moveDone) enter(Phase::SettleBack, ctx.nowMs);
      else if (elapsed > kMoveTimeoutMs) return ProcedureResult::Failed;
      break;

    case Phase::SettleBack: {
      if (elapsed < kSettleMs) break;
      const float offset = std::fabs(static_cast<float>(ctx.snapshot.counts - startCounts_));
      if (offset > kReturnTolerance * expectedCounts_) return ProcedureResult::Failed;  // lost steps
      return ProcedureResult::Succeeded;
    }
  }
  return ProcedureResult::Running;
}

// ================================================================ Homing

namespace {
constexpr uint32_t kHomingSettleMs = 100;
constexpr uint32_t kBackoffTimeoutMs = 10000;
constexpr uint32_t kBackoffDwellMs = 100;  // at the backoff point before declaring zero
}  // namespace

void Homing::start(ProcedureContext& ctx) {
  const Settings& s = ctx.settings;
  const auto mode = static_cast<proto::HomingMode>(s.homingMode);
  const bool negative = mode == proto::HomingMode::SwitchMin || mode == proto::HomingMode::SensorlessNeg;
  direction_ = negative ? -1.0f : 1.0f;

  uint8_t triggers = MotionCore::kTriggerFollowingError;
  if (mode == proto::HomingMode::SwitchMin) triggers = MotionCore::kTriggerLimitMin;
  if (mode == proto::HomingMode::SwitchMax) triggers = MotionCore::kTriggerLimitMax;

  ctx.core.seek(direction_ * s.homingVelocity, s.maxAcceleration, triggers, s.homingStallError);
  phase_ = Phase::Seek;
  phaseStartMs_ = ctx.nowMs;
}

ProcedureResult Homing::service(ProcedureContext& ctx) {
  const Settings& s = ctx.settings;
  const uint32_t elapsed = ctx.nowMs - phaseStartMs_;
  const bool stalled = ctx.events & MotionCore::kEventStall;

  switch (phase_) {
    case Phase::Seek:
      if (ctx.events & MotionCore::kEventSeekTriggered) {
        phase_ = Phase::Settle;
        phaseStartMs_ = ctx.nowMs;
      } else if (stalled || elapsed > s.homingTimeoutS * 1000u) {
        phase_ = Phase::Done;
        return ProcedureResult::Failed;  // blocked before the switch, or nothing found
      }
      break;

    case Phase::Settle:
      if (elapsed < kHomingSettleMs) break;
      // Unbounded: the soft limits refer to the old frame, which homing replaces.
      ctx.core.moveTo(ctx.snapshot.position - q32::fromDelta(direction_ * s.homingBackoff), s.homingVelocity,
                      s.maxAcceleration, false);
      phase_ = Phase::Backoff;
      phaseStartMs_ = ctx.nowMs;
      doneSinceMs_ = 0;
      break;

    case Phase::Backoff:
      if (stalled || elapsed > kBackoffTimeoutMs) {
        phase_ = Phase::Done;
        return ProcedureResult::Failed;
      }
      if (!ctx.snapshot.trajectoryDone) break;
      if (doneSinceMs_ == 0) doneSinceMs_ = ctx.nowMs | 1u;
      if (ctx.nowMs - doneSinceMs_ >= kBackoffDwellMs) {
        ctx.core.setPosition(0);
        phase_ = Phase::Done;
        return ProcedureResult::Succeeded;
      }
      break;

    case Phase::Done:
      break;
  }
  return ProcedureResult::Running;
}

// ================================================================ LimitTuner

namespace {
constexpr uint8_t kConfirmPasses = 3;
constexpr uint8_t kMaxFloorStalls = 3;
constexpr uint32_t kTunerMoveTimeoutMs = 30000;
constexpr uint32_t kDwellMs = 50;
}  // namespace

// A level is only meaningful if the range lets the profile reach its velocity.
bool LimitTuner::reachable(uint8_t level) const {
  const float v = velocityAt(level);
  return range_ >= v * v / accelerationAt(level);
}

void LimitTuner::start(ProcedureContext& ctx, q32::Pos min, q32::Pos max) {
  min_ = min;
  max_ = max;
  range_ = q32::diff(max, min);
  level_ = 0;
  passes_ = 0;
  stallsAtFloor_ = 0;
  verifying_ = false;
  move(ctx, Phase::ToStart);
}

void LimitTuner::move(ProcedureContext& ctx, Phase phase) {
  phase_ = phase;
  phaseStartMs_ = ctx.nowMs;
  doneSinceMs_ = 0;
  const bool gentle = phase == Phase::ToStart;  // reposition at the lowest level
  ctx.core.moveTo(phase == Phase::Forward ? max_ : min_, velocityAt(gentle ? 0 : level_),
                  accelerationAt(gentle ? 0 : level_));
}

ProcedureResult LimitTuner::service(ProcedureContext& ctx) {
  if (ctx.events & MotionCore::kEventStall) {
    // The control tick already stopped and holds. Drop a level and confirm it.
    if (level_ == 0) {
      if (++stallsAtFloor_ >= kMaxFloorStalls) return ProcedureResult::Failed;
    } else {
      --level_;
    }
    verifying_ = true;
    passes_ = 0;
    move(ctx, Phase::ToStart);
    return ProcedureResult::Running;
  }
  if (ctx.nowMs - phaseStartMs_ > kTunerMoveTimeoutMs) return ProcedureResult::Failed;

  if (!ctx.snapshot.trajectoryDone) {
    doneSinceMs_ = 0;
    return ProcedureResult::Running;
  }
  if (doneSinceMs_ == 0) doneSinceMs_ = ctx.nowMs | 1u;
  if (ctx.nowMs - doneSinceMs_ < kDwellMs) return ProcedureResult::Running;

  switch (phase_) {
    case Phase::ToStart:
      move(ctx, Phase::Forward);
      break;
    case Phase::Forward:
      move(ctx, Phase::Backward);
      break;
    case Phase::Backward:
      if (verifying_) {
        if (++passes_ >= kConfirmPasses) return ProcedureResult::Succeeded;
      } else {
        // Stop at the top level, or where the range no longer allows testing more.
        if (level_ >= kMaxLevel || !reachable(static_cast<uint8_t>(level_ + 1))) return ProcedureResult::Succeeded;
        ++level_;
      }
      move(ctx, Phase::Forward);
      break;
  }
  return ProcedureResult::Running;
}
