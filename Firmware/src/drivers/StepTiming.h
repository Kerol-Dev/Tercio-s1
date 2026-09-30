#pragma once
// -----------------------------------------------------------------------------
// Pure timing decisions of the step generator (unit tested on the host):
// the timer registers for a step rate, and whether a running period may be
// cut short. See StepGenerator for how they are applied to TIM1.
// -----------------------------------------------------------------------------
#include <cstdint>

namespace step_timing {

struct Timing {
  uint32_t psc = 0;  // prescaler register (divides by psc + 1)
  uint32_t arr = 0;  // period register   (period = arr + 1 ticks)
  uint32_t ccr = 0;  // pulse length in ticks; 0 = no pulse
  bool operator==(const Timing& o) const { return psc == o.psc && arr == o.arr && ccr == o.ccr; }
};

// Registers for `rate` steps/s from a timer clocked at `clock` Hz, with a pulse
// of `pulseClocks` (at most half the period). `rate` must be > 0.
inline Timing forRate(float rate, uint32_t clock, uint32_t pulseClocks) {
  Timing t;
  const auto period = static_cast<uint32_t>(static_cast<float>(clock) / rate);  // timer clocks
  t.psc = (period - 1u) >> 16;  // smallest prescaler that fits ARR in 16 bits
  t.arr = period / (t.psc + 1u) - 1u;
  const uint32_t pulse = (pulseClocks + t.psc) / (t.psc + 1u);  // ceil, in prescaled ticks
  const uint32_t half = (t.arr + 1u) / 2u;
  t.ccr = pulse < half ? pulse : half;
  if (t.ccr == 0) t.ccr = 1;
  return t;
}

// Speeding up from a slow rate: restart the period (emitting the next pulse
// now) when that pulse is already overdue at the new rate, the current pulse
// is complete, and the period is not about to end anyway (`margin` clocks).
inline bool shouldRestart(const Timing& active, const Timing& next, uint32_t count, uint64_t margin) {
  if (next.ccr == 0 || active.ccr == 0 || count < active.ccr) return false;
  const uint64_t elapsed = static_cast<uint64_t>(count) * (active.psc + 1u);
  const uint64_t activePeriod = static_cast<uint64_t>(active.arr + 1u) * (active.psc + 1u);
  const uint64_t nextPeriod = static_cast<uint64_t>(next.arr + 1u) * (next.psc + 1u);
  return elapsed >= nextPeriod && activePeriod - elapsed > margin;
}

}  // namespace step_timing
