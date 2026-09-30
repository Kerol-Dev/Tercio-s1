#pragma once
// -----------------------------------------------------------------------------
// Absolute positions as signed Q32.32 fixed-point turns.
//
// Range ±2^31 turns at 2.3e-10 turn resolution; additions are exact, so a
// position integrated at 20 kHz for days never drifts, and the Cortex-M4 does
// the arithmetic in a couple of instructions (double would be software-emulated).
// Differences between nearby positions are handed to the float math as turns.
// -----------------------------------------------------------------------------
#include <cstdint>

namespace q32 {

using Pos = int64_t;

inline constexpr double kOne = 4294967296.0;  // 2^32
inline constexpr float kOneF = 4294967296.0f;
inline constexpr float kInvF = 1.0f / 4294967296.0f;
inline constexpr double kLimitTurns = 2.0e9;

// Protocol boundary only (double is slow on this core).
inline Pos fromTurns(double turns) {
  if (turns > kLimitTurns) turns = kLimitTurns;
  if (turns < -kLimitTurns) turns = -kLimitTurns;
  return static_cast<Pos>(turns * kOne);
}
inline double toTurns(Pos p) { return static_cast<double>(p) / kOne; }

// a - b in turns, and a small offset in turns as a Pos delta.
inline float diff(Pos a, Pos b) { return static_cast<float>(a - b) * kInvF; }
inline Pos fromDelta(float turns) { return static_cast<Pos>(turns * kOneF); }

// Encoder counts at 2^bits counts/turn -> Pos (exact).
inline Pos fromCounts(int64_t counts, uint8_t bits) { return counts * (Pos{1} << (32 - bits)); }

}  // namespace q32
