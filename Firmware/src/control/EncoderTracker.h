#pragma once
// -----------------------------------------------------------------------------
// Turns single-turn encoder samples into a multi-turn count and a velocity.
//
//  * Unwrapping: each sample moves the count by the shortest signed distance.
//  * Glitch rejection: a jump of more than a quarter turn between consecutive
//    samples (physically impossible at control rates) is dropped, unless it
//    repeats — then it is real and accepted.
//  * Velocity: a phase-locked loop tracks the samples; its velocity state is a
//    low-noise, low-lag estimate even with coarse 12-bit counts. Between
//    samples the position is extrapolated with it.
// -----------------------------------------------------------------------------
#include <cstdint>

class EncoderTracker {
 public:
  void configure(uint8_t resolutionBits, float tickPeriod, float bandwidth);
  // Start tracking at `counts` (multi-turn), with `raw` the current reading.
  void reset(int64_t counts, uint16_t raw);

  // Once per control tick; `fresh` when `raw` is a new sample.
  void update(bool fresh, uint16_t raw);

  int64_t counts() const { return counts_; }
  uint16_t raw() const { return raw_; }
  float velocity() const { return velocity_; }  // counts/s
  // Counts travelled since the last sample, predicted from the velocity.
  float extrapolation() const { return velocity_ * static_cast<float>(ticksSinceSample_) * tickPeriod_; }
  uint32_t ticksSinceSample() const { return ticksSinceSample_; }
  uint32_t rejectedSamples() const { return rejected_; }

 private:
  uint8_t bits_ = 12;
  int32_t cpr_ = 4096;
  float tickPeriod_ = 50e-6f;
  float kp_ = 0.0f, ki_ = 0.0f;

  int64_t counts_ = 0;  // 64-bit: continuous rotation never wraps
  uint16_t raw_ = 0;
  uint32_t ticksSinceSample_ = 0;
  uint8_t glitchStreak_ = 0;
  uint32_t rejected_ = 0;

  // PLL state, relative to counts_ to keep the float small and precise.
  float estimate_ = 0.0f;  // estimated position minus counts_
  float velocity_ = 0.0f;
};
