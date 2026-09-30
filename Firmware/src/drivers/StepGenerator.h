#pragma once
// -----------------------------------------------------------------------------
// Hardware step pulse generator on TIM1_CH2N (PB14).
//
// The step rate is the timer's PWM frequency, so pulses cost no CPU time. Each
// period starts with a 2 µs pulse. New rates are written to the preload
// registers and take effect at the next update event, which keeps every period
// whole: no runt or doubled pulses.
// The prescaler is re-chosen per rate so the period resolution is always
// better than 1/32768, from 0.05 Hz to 500 kHz.
//
// When speeding up from a slow rate, waiting out the old (long) period would
// add latency; if the next step is already overdue at the new rate, the
// period is restarted immediately instead.
// -----------------------------------------------------------------------------
#include <cstdint>

#include "drivers/StepTiming.h"

class StepGenerator {
 public:
  static constexpr float kMaxRate = 500000.0f;  // steps/s
  static constexpr float kMinRate = 0.05f;      // below this: stopped

  void begin();

  // Signed step rate in steps/s; positive drives DIR high. Interrupt-safe.
  void setRate(float stepsPerSecond);
  void stop() { setRate(0.0f); }

  void enableDriver(bool on);
  bool driverEnabled() const { return enabled_; }

 private:
  using Timing = step_timing::Timing;

  void apply(const Timing& next);

  uint32_t timerClock_ = 0;
  uint32_t idleArr_ = 0;
  uint32_t pulseClocks_ = 0;  // STEP high time in timer clocks
  Timing active_{};   // registers the counter is running with
  Timing pending_{};  // registers waiting in preload
  bool dirPositive_ = true;
  bool enabled_ = false;
};
