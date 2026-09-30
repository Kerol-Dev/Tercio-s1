#pragma once
// -----------------------------------------------------------------------------
// Digital inputs: limit switches (IN1/IN2), the external step/dir/enable
// interface and the driver's DIAG line. Reads are single register accesses, safe from the control ISR.
// External step pulses are counted in a top-priority EXTI interrupt that also
// samples DIR at the edge, so direction changes between control ticks are exact.
// -----------------------------------------------------------------------------
#include <cstdint>

namespace inputs {

void begin();
void setLimitPolarity(bool activeLow);
bool limitMin();  // true = switch active (polarity applied)
bool limitMax();

void setStepDirEnabled(bool enabled);
int32_t stepDirCount();  // cumulative signed pulse count (wraps)
bool stepDirEnable(bool activeLow);

// TMC2209 DIAG output: high while the driver has shut its bridge down.
bool driverDiag();

}  // namespace inputs
