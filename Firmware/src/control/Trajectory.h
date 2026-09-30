#pragma once
// -----------------------------------------------------------------------------
// Online time-optimal trapezoidal motion profile.
//
// Recomputed every tick from the current state, so targets, speed and
// acceleration limits can change at any moment and the profile adapts
// smoothly — no precomputed move to invalidate. Position mode brakes along
// the exact discrete-time stopping curve, so it lands on the target without
// overshoot or end-of-move chatter. Optional bounds turn velocity mode into a
// stop exactly at the boundary.
// -----------------------------------------------------------------------------
#include "control/Fixed.h"

class Trajectory {
 public:
  enum class Mode : uint8_t { Position, Velocity };

  // At rest at `position`.
  void reset(q32::Pos position);
  void moveTo(q32::Pos target, float maxVelocity, float maxAcceleration);
  void runAt(float velocity, float maxAcceleration);
  // Decelerate to rest, then hold where it stopped.
  void stop(float maxAcceleration);
  void setBounds(bool enabled, q32::Pos min, q32::Pos max);

  void step(float dt);

  q32::Pos position() const { return position_; }
  float velocity() const { return velocity_; }
  q32::Pos target() const { return target_; }
  Mode mode() const { return mode_; }
  // At rest at the target (position mode) or at the commanded speed (velocity mode).
  bool done() const { return done_; }

 private:
  void stepPosition(float dt);
  void stepVelocity(float dt);
  q32::Pos clampToBounds(q32::Pos p) const;

  q32::Pos position_ = 0;
  q32::Pos target_ = 0;
  float velocity_ = 0.0f;
  float targetVelocity_ = 0.0f;
  float maxVelocity_ = 1.0f;
  float maxAcceleration_ = 1.0f;
  Mode mode_ = Mode::Position;
  bool done_ = true;
  bool holdWhenStopped_ = false;
  bool bounded_ = false;
  q32::Pos min_ = 0, max_ = 0;
};
