#pragma once
// -----------------------------------------------------------------------------
// Fixed-rate control interrupt on TIM7 (basic timer). The callback runs at
// kRateHz with a constant period, independent of anything the main loop does.
// Also measures how long each tick takes (DWT cycle counter) for load reporting.
// -----------------------------------------------------------------------------
#include <cstdint>

namespace control_timer {

inline constexpr uint32_t kRateHz = 20000;
inline constexpr float kPeriod = 1.0f / static_cast<float>(kRateHz);

using Callback = void (*)();

void begin(Callback callback, uint32_t priority);

// Longest tick since the last call, as a fraction of the period (0..1+), then reset.
float takePeakLoad();

}  // namespace control_timer
