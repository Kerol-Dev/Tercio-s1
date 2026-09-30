#include "control/Trajectory.h"

#include <cmath>

namespace {
// Within this distance and one tick's worth of velocity change, snap onto the target.
constexpr float kSnapDistance = 1e-6f;
// Braking is planned at 98 % of the acceleration limit. Riding exactly on the
// limit leaves no authority to absorb float rounding (each v − a·dt rounds a
// little high), which would accumulate into a small overshoot.
constexpr float kBrakingShare = 0.98f;
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

void Trajectory::reset(q32::Pos position) {
  position_ = target_ = position;
  velocity_ = targetVelocity_ = 0.0f;
  mode_ = Mode::Position;
  holdWhenStopped_ = false;
  done_ = true;
}

void Trajectory::moveTo(q32::Pos target, float maxVelocity, float maxAcceleration) {
  target_ = clampToBounds(target);
  maxVelocity_ = std::fabs(maxVelocity);
  maxAcceleration_ = std::fabs(maxAcceleration);
  mode_ = Mode::Position;
  holdWhenStopped_ = false;
  done_ = false;
}

void Trajectory::runAt(float velocity, float maxAcceleration) {
  targetVelocity_ = velocity;
  maxAcceleration_ = std::fabs(maxAcceleration);
  mode_ = Mode::Velocity;
  holdWhenStopped_ = false;
  done_ = false;
}

void Trajectory::stop(float maxAcceleration) {
  runAt(0.0f, maxAcceleration);
  holdWhenStopped_ = true;
}

void Trajectory::setBounds(bool enabled, q32::Pos min, q32::Pos max) {
  bounded_ = enabled && min < max;
  min_ = min;
  max_ = max;
  if (bounded_ && mode_ == Mode::Position) target_ = clampToBounds(target_);
}

q32::Pos Trajectory::clampToBounds(q32::Pos p) const {
  if (!bounded_) return p;
  return p < min_ ? min_ : (p > max_ ? max_ : p);
}

void Trajectory::step(float dt) {
  if (mode_ == Mode::Velocity)
    stepVelocity(dt);
  else
    stepPosition(dt);
}

void Trajectory::stepVelocity(float dt) {
  const float dv = maxAcceleration_ * dt;
  const float next = velocity_ + clampf(targetVelocity_ - velocity_, -dv, dv);
  const float travel = 0.5f * (velocity_ + next) * dt;

  // Heading for a bound: if taking this step would leave less room than the
  // stopping distance, hand over to a position move that stops exactly on it.
  if (bounded_ && next != 0.0f) {
    const q32::Pos bound = next > 0.0f ? max_ : min_;
    const float room = std::fabs(q32::diff(bound, position_)) - std::fabs(travel);
    if (next * next > 2.0f * kBrakingShare * maxAcceleration_ * room) {
      moveTo(bound, std::fmax(std::fabs(velocity_), dv), maxAcceleration_);
      stepPosition(dt);
      return;
    }
  }

  position_ += q32::fromDelta(travel);
  velocity_ = next;
  done_ = velocity_ == targetVelocity_;
  if (done_ && holdWhenStopped_ && velocity_ == 0.0f) reset(position_);
}

void Trajectory::stepPosition(float dt) {
  if (done_) return;
  const float distance = q32::diff(target_, position_);
  const float dv = maxAcceleration_ * dt;

  if (std::fabs(distance) <= kSnapDistance && std::fabs(velocity_) <= dv) {
    position_ = target_;
    velocity_ = 0.0f;
    done_ = true;
    return;
  }

  // Fastest next velocity from which the target is still reached exactly. With
  // trapezoidal integration the stopping distance from v is exactly v²/2a, and
  // this step covers (v + v')·dt/2, so v' must satisfy
  //   v'²/2a + v'·dt/2 <= d − v·dt/2   =>   v' = sqrt((a·dt/2)² + 2a·(d − v·dt/2)) − a·dt/2
  const float toward = distance > 0.0f ? velocity_ : -velocity_;  // speed toward the target
  const float reach = std::fabs(distance) - 0.5f * toward * dt;
  const float brakeAcceleration = kBrakingShare * maxAcceleration_;
  const float dvBrake = brakeAcceleration * dt;
  const float brake =
      reach > 0.0f ? std::sqrt(0.25f * dvBrake * dvBrake + 2.0f * brakeAcceleration * reach) - 0.5f * dvBrake : 0.0f;
  const float desired = std::copysign(std::fmin(maxVelocity_, brake), distance);
  const float next = velocity_ + clampf(desired - velocity_, -dv, dv);

  const float travel = 0.5f * (velocity_ + next) * dt;
  // Final approach: never step past the target.
  if (std::fabs(travel) >= std::fabs(distance) && (travel > 0.0f) == (distance > 0.0f) && std::fabs(next) <= dv) {
    position_ = target_;
    velocity_ = 0.0f;
    done_ = true;
    return;
  }
  position_ += q32::fromDelta(travel);
  velocity_ = next;
}
