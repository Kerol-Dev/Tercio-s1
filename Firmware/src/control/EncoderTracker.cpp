#include "control/EncoderTracker.h"

namespace {
constexpr uint8_t kGlitchConfirmations = 3;
}

void EncoderTracker::configure(uint8_t resolutionBits, float tickPeriod, float bandwidth) {
  bits_ = resolutionBits;
  cpr_ = int32_t{1} << resolutionBits;
  tickPeriod_ = tickPeriod;
  // Critically damped second-order loop: kp = 2ω, ki = ω².
  kp_ = 2.0f * bandwidth;
  ki_ = bandwidth * bandwidth;
}

void EncoderTracker::reset(int64_t counts, uint16_t raw) {
  counts_ = counts;
  raw_ = raw;
  ticksSinceSample_ = 0;
  glitchStreak_ = 0;
  estimate_ = 0.0f;
  velocity_ = 0.0f;
}

void EncoderTracker::update(bool fresh, uint16_t raw) {
  ++ticksSinceSample_;
  if (!fresh) return;

  // Shortest signed distance on the circle.
  int32_t delta = (static_cast<int32_t>(raw) - static_cast<int32_t>(raw_)) & (cpr_ - 1);
  if (delta >= cpr_ / 2) delta -= cpr_;

  if (delta > cpr_ / 4 || delta < -cpr_ / 4) {
    if (++glitchStreak_ < kGlitchConfirmations) {
      ++rejected_;
      return;
    }
  }
  glitchStreak_ = 0;

  const float dt = static_cast<float>(ticksSinceSample_) * tickPeriod_;
  ticksSinceSample_ = 0;
  raw_ = raw;
  counts_ += delta;

  // After a long gap (bus errors, CPU stalled by a flash write) one PLL step
  // would overshoot (kp·dt > 1): resynchronise on the measurement instead.
  if (kp_ * dt >= 1.0f) {
    estimate_ = 0.0f;
    velocity_ = static_cast<float>(delta) / dt;
    return;
  }

  // PLL: predict, then correct with the new sample (estimate_ is relative to counts_).
  estimate_ += velocity_ * dt - static_cast<float>(delta);
  const float error = -estimate_;
  estimate_ += kp_ * dt * error;
  velocity_ += ki_ * dt * error;
}
