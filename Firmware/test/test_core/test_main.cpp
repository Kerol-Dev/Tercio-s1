// Host-side tests for the hardware-independent firmware modules.
// Run with: pio test -e native
#include <unity.h>

#include <cmath>
#include <cstring>
#include <algorithm>
#include <cstdint>
#include <vector>

#include "app/Params.h"
#include "app/Settings.h"
#include "control/EncoderTracker.h"
#include "control/Fixed.h"
#include "control/PositionController.h"
#include "control/Trajectory.h"
#include "drivers/Crc32.h"
#include "drivers/StepTiming.h"
#include "protocol/Protocol.h"

namespace {

constexpr float kDt = 50e-6f;  // 20 kHz control tick

struct ProfileStats {
  float peakVelocity = 0.0f;
  float peakAcceleration = 0.0f;
  float overshoot = 0.0f;  // beyond the target, in the direction of travel
  float seconds = 0.0f;
  bool finished = false;
};

// Runs a position move to completion and records the profile's extremes.
ProfileStats runMove(Trajectory& t, q32::Pos target, float vmax, float amax, float timeoutS = 30.0f) {
  ProfileStats stats;
  t.moveTo(target, vmax, amax);
  const float direction = q32::diff(target, t.position()) >= 0.0f ? 1.0f : -1.0f;
  float previous = t.velocity();
  for (int i = 0; i < static_cast<int>(timeoutS / kDt); ++i) {
    t.step(kDt);
    stats.seconds += kDt;
    stats.peakVelocity = std::fmax(stats.peakVelocity, std::fabs(t.velocity()));
    stats.peakAcceleration = std::fmax(stats.peakAcceleration, std::fabs(t.velocity() - previous) / kDt);
    stats.overshoot = std::fmax(stats.overshoot, direction * q32::diff(t.position(), target));
    previous = t.velocity();
    if (t.done()) {
      stats.finished = true;
      break;
    }
  }
  return stats;
}

}  // namespace

// ---------------------------------------------------------------- Trajectory

void test_trapezoid_respects_limits_and_lands_exactly() {
  Trajectory t;
  t.reset(0);
  const ProfileStats s = runMove(t, q32::fromTurns(10.0), 5.0f, 20.0f);
  TEST_ASSERT_TRUE(s.finished);
  TEST_ASSERT_TRUE(t.position() == q32::fromTurns(10.0));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, t.velocity());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, s.peakVelocity);
  TEST_ASSERT_TRUE(s.peakAcceleration <= 20.0f * 1.001f);  // Δv/dt from floats near 5 carries ~0.05 % rounding
  TEST_ASSERT_TRUE(s.overshoot <= 0.0f);
  // Time-optimal: d/v + v/a = 2.25 s.
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 2.25f, s.seconds);
}

void test_short_move_is_triangular() {
  Trajectory t;
  t.reset(0);
  const ProfileStats s = runMove(t, q32::fromTurns(0.1), 5.0f, 20.0f);
  TEST_ASSERT_TRUE(s.finished);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, std::sqrt(20.0f * 0.1f), s.peakVelocity);  // v = sqrt(a d)
  TEST_ASSERT_FLOAT_WITHIN(0.003f, 2.0f * std::sqrt(0.1f / 20.0f), s.seconds);
  TEST_ASSERT_TRUE(s.overshoot <= 0.0f);
}

void test_retarget_reversal_mid_move() {
  Trajectory t;
  t.reset(0);
  t.moveTo(q32::fromTurns(10.0), 5.0f, 20.0f);
  for (int i = 0; i < 20000; ++i) t.step(kDt);  // 1 s in, cruising
  const ProfileStats s = runMove(t, q32::fromTurns(-2.0), 5.0f, 20.0f);
  TEST_ASSERT_TRUE(s.finished);
  TEST_ASSERT_TRUE(t.position() == q32::fromTurns(-2.0));
  TEST_ASSERT_TRUE(s.peakAcceleration <= 20.0f * 1.001f);
}

void test_tiny_moves_far_from_origin_are_exact() {
  Trajectory t;
  const q32::Pos start = q32::fromTurns(1.0e6);
  t.reset(start);
  const q32::Pos target = start + q32::fromTurns(0.0001);
  const ProfileStats s = runMove(t, target, 5.0f, 20.0f);
  TEST_ASSERT_TRUE(s.finished);
  TEST_ASSERT_TRUE(t.position() == target);
  TEST_ASSERT_TRUE(s.overshoot <= 0.0f);
}

void test_velocity_mode_ramps_and_stops_holding() {
  Trajectory t;
  t.reset(0);
  t.runAt(-3.0f, 30.0f);
  for (int i = 0; i < 4000; ++i) t.step(kDt);  // 0.2 s > 3/30 s ramp
  TEST_ASSERT_EQUAL_FLOAT(-3.0f, t.velocity());
  TEST_ASSERT_TRUE(t.done());
  t.stop(30.0f);
  for (int i = 0; i < 4000; ++i) t.step(kDt);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, t.velocity());
  TEST_ASSERT_TRUE(t.mode() == Trajectory::Mode::Position && t.done());
}

void test_bounds_stop_velocity_mode_at_the_limit() {
  Trajectory t;
  t.reset(0);
  const q32::Pos max = q32::fromTurns(1.0);
  t.setBounds(true, q32::fromTurns(-1.0), max);
  t.runAt(8.0f, 40.0f);
  bool crossed = false;
  for (int i = 0; i < 40000; ++i) {
    t.step(kDt);
    crossed |= t.position() > max;
  }
  TEST_ASSERT_FALSE(crossed);
  TEST_ASSERT_TRUE(t.position() == max);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, t.velocity());
  // Position targets are clamped too.
  t.moveTo(q32::fromTurns(5.0), 5.0f, 40.0f);
  TEST_ASSERT_TRUE(t.target() == max);
}

// ------------------------------------------------------------ Controller

void test_controller_feedforward_deadband_and_slew() {
  PositionController c;
  PositionController::Gains g;
  g.kp = 20.0f;
  g.deadband = 0.001f;
  g.maxVelocity = 10.0f;
  g.maxAcceleration = 100.0f;
  c.setGains(g);

  // Inside the deadband at rest: silent.
  for (int i = 0; i < 10; ++i) c.update(0.0005f, 0.0f, 0.0f, false, kDt);
  TEST_ASSERT_TRUE(c.settled());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.output());

  // Hysteresis: 1.5x deadband keeps it settled, beyond 2x releases it.
  c.update(0.0015f, 0.0f, 0.0f, false, kDt);
  TEST_ASSERT_TRUE(c.settled());
  c.update(0.0025f, 0.0f, 0.0f, false, kDt);
  TEST_ASSERT_FALSE(c.settled());

  // Output slew: a step demand rises at maxAcceleration.
  c.reset();
  const float first = c.update(0.0f, 5.0f, 0.0f, true, kDt);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 100.0f * kDt, first);
  for (int i = 0; i < 2000; ++i) c.update(0.0f, 5.0f, 0.0f, true, kDt);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 5.0f, c.output());  // pure feed-forward once ramped
}

// --------------------------------------------------------- EncoderTracker

void test_tracker_unwraps_and_estimates_velocity() {
  EncoderTracker tracker;
  tracker.configure(12, kDt, 600.0f);
  tracker.reset(0, 4000);
  // 2 turns/s = 8192 counts/s, one sample every 3 ticks (150 µs), crossing 4095 -> 0.
  double exact = 4000.0;
  for (int tick = 1; tick <= 20000; ++tick) {
    exact += 8192.0 * static_cast<double>(kDt);
    const bool fresh = tick % 3 == 0;
    const auto raw = static_cast<uint16_t>(static_cast<int64_t>(std::floor(exact)) & 4095);
    tracker.update(fresh, raw);
  }
  TEST_ASSERT_INT32_WITHIN(2, static_cast<int32_t>(std::floor(exact)) - 4000, static_cast<int32_t>(tracker.counts()));
  TEST_ASSERT_FLOAT_WITHIN(8192.0f * 0.01f, 8192.0f, tracker.velocity());
}

void test_tracker_rejects_single_glitch_but_accepts_real_jump() {
  EncoderTracker tracker;
  tracker.configure(12, kDt, 600.0f);
  tracker.reset(0, 100);
  tracker.update(true, 100);
  tracker.update(true, 2600);  // single wild sample
  TEST_ASSERT_TRUE(tracker.counts() == 0);
  tracker.update(true, 101);
  TEST_ASSERT_TRUE(tracker.counts() == 1);
  // A persistent jump is real.
  for (int i = 0; i < 3; ++i) tracker.update(true, 1301);
  TEST_ASSERT_TRUE(tracker.counts() == 1201);
}

// --------------------------------------------------- closed-loop simulation

// Stepper + 12-bit encoder model: the rotor follows the commanded steps
// exactly; the encoder quantises and lags one sample. The loop must settle
// on target within the deadband without hunting.
void test_closed_loop_move_settles_without_hunting() {
  Trajectory trajectory;
  PositionController controller;
  EncoderTracker tracker;
  PositionController::Gains g;
  g.kp = 20.0f;
  g.deadband = 0.0005f;
  g.maxVelocity = 20.0f;
  g.maxAcceleration = 400.0f;
  controller.setGains(g);
  tracker.configure(12, kDt, 600.0f);
  tracker.reset(0, 0);
  trajectory.reset(0);
  trajectory.moveTo(q32::fromTurns(3.25), 10.0f, 100.0f);

  const double stepsPerTurn = 3200.0;
  double rotorSteps = 0.0;
  int outputSignChanges = 0;
  float lastOutput = 0.0f;
  float worstFollowing = 0.0f;
  for (int tick = 0; tick < 40000; ++tick) {  // 2 s
    const auto counts = static_cast<int64_t>(std::floor(rotorSteps / stepsPerTurn * 4096.0));
    tracker.update(tick % 3 == 0, static_cast<uint16_t>(counts & 4095));
    const q32::Pos measured = q32::fromCounts(tracker.counts(), 12) + q32::fromDelta(tracker.extrapolation() / 4096.0f);
    trajectory.step(kDt);
    const float error = q32::diff(trajectory.position(), measured);
    worstFollowing = std::fmax(worstFollowing, std::fabs(error));
    const float v = controller.update(error, trajectory.velocity(), tracker.velocity() / 4096.0f,
                                      !trajectory.done(), kDt);
    if (trajectory.done() && tick > 0 && (v > 0.0f) != (lastOutput > 0.0f) && v != 0.0f) ++outputSignChanges;
    lastOutput = v;
    rotorSteps += static_cast<double>(v * kDt) * stepsPerTurn;
  }
  const float final = static_cast<float>(rotorSteps / stepsPerTurn) - 3.25f;
  TEST_ASSERT_TRUE(trajectory.done());
  TEST_ASSERT_FLOAT_WITHIN(2.0f / 4096.0f, 0.0f, final);  // within two encoder counts
  TEST_ASSERT_TRUE(worstFollowing < 0.01f);               // < 3.6° during a 10 turns/s move
  TEST_ASSERT_TRUE(outputSignChanges <= 2);                // no hunting at rest
  TEST_ASSERT_TRUE(controller.settled());
}


// ------------------------------------------------ step generator simulation

// Clock-level model of TIM1 as StepGenerator drives it: preloaded registers go
// live at the update event, each period starts with a pulse, and a restart
// begins a new period (and pulse) immediately.
struct TimerModel {
  static constexpr uint32_t kClock = 170000000;
  static constexpr uint32_t kPulseClocks = 340;  // 2 µs
  static constexpr uint64_t kMargin = 512;
  step_timing::Timing active{0, 3399, 0}, pending{0, 3399, 0};
  uint64_t periodStart = 0;
  std::vector<uint64_t> pulses;

  uint64_t periodLength() const { return uint64_t(active.arr + 1) * (active.psc + 1); }
  void advance(uint64_t now) {
    while (periodStart + periodLength() <= now) {
      periodStart += periodLength();
      active = pending;
      if (active.ccr) pulses.push_back(periodStart);
    }
  }
  void setRate(float rate, uint64_t now) {
    const step_timing::Timing next =
        rate >= 0.05f ? step_timing::forRate(rate, kClock, kPulseClocks) : step_timing::Timing{0, 3399, 0};
    if (next == pending) return;
    advance(now);
    const auto count = static_cast<uint32_t>((now - periodStart) / (active.psc + 1));
    const bool restart = step_timing::shouldRestart(active, next, count, kMargin);
    pending = next;
    if (restart) {
      periodStart = now;
      active = next;
      pulses.push_back(now);
    }
  }
};

void test_step_timing_registers() {
  // 500 kHz: 340 clocks, 50 % duty caps the 2 µs pulse.
  step_timing::Timing t = step_timing::forRate(500000.0f, 170000000, 340);
  TEST_ASSERT_EQUAL_UINT32(0, t.psc);
  TEST_ASSERT_EQUAL_UINT32(339, t.arr);
  TEST_ASSERT_EQUAL_UINT32(170, t.ccr);
  // 1 step/s: prescaled, period exact to a tick, pulse still ~2 µs.
  t = step_timing::forRate(1.0f, 170000000, 340);
  TEST_ASSERT_EQUAL_UINT32(170000000u, (t.arr + 1u) * (t.psc + 1u) + (170000000u % (t.psc + 1u)));
  TEST_ASSERT_TRUE(t.ccr * (t.psc + 1u) >= 340u && t.ccr * (t.psc + 1u) < 340u + t.psc + 1u);
}

// Firmware v2's first step generator used 50 % duty; a slow first period then
// blocked acceleration for half its length and the calibration sweep emitted a
// single step. The sweep must produce the commanded steps, never too fast.
void test_open_loop_sweep_emits_commanded_steps() {
  const float stepsPerTurn = 200.0f * 16.0f;
  TimerModel timer;
  Trajectory t;
  t.reset(0);
  t.moveTo(q32::fromTurns(0.25), 0.5f, 5.0f);
  uint64_t now = 0;
  for (int tick = 0; tick < 40000; ++tick) {  // 2 s
    now += TimerModel::kClock / 20000;
    t.step(kDt);
    timer.setRate(t.velocity() * stepsPerTurn, now);
  }
  timer.advance(now);
  // Each period's pulse fires at its start, so open-loop output leads the ideal
  // position by a step or so (the closed loop removes it); allow 0.5 %.
  TEST_ASSERT_INT32_WITHIN(4, 800, static_cast<int32_t>(timer.pulses.size()));
  // Never faster than the peak rate (1600 steps/s = 106250 clocks).
  uint64_t shortest = UINT64_MAX;
  for (size_t i = 1; i < timer.pulses.size(); ++i) shortest = std::min(shortest, timer.pulses[i] - timer.pulses[i - 1]);
  TEST_ASSERT_TRUE(shortest >= 106250u * 99u / 100u);
}

// Closed loop through the timer model: the rotor moves one microstep per pulse.
void test_closed_loop_through_step_generator() {
  const double stepsPerTurn = 3200.0;
  TimerModel timer;
  Trajectory trajectory;
  PositionController controller;
  EncoderTracker tracker;
  PositionController::Gains g;
  g.maxVelocity = 20.0f;
  g.maxAcceleration = 400.0f;
  controller.setGains(g);
  tracker.configure(12, kDt, 600.0f);
  tracker.reset(0, 0);
  trajectory.reset(0);
  trajectory.moveTo(q32::fromTurns(-2.5), 10.0f, 100.0f);

  int64_t rotorSteps = 0;
  size_t counted = 0;
  bool direction = true;
  float worst = 0.0f;
  uint64_t now = 0;
  for (int tick = 0; tick < 40000; ++tick) {
    now += TimerModel::kClock / 20000;
    timer.advance(now);
    for (; counted < timer.pulses.size(); ++counted) rotorSteps += direction ? 1 : -1;
    const auto counts = static_cast<int64_t>(std::floor(static_cast<double>(rotorSteps) / stepsPerTurn * 4096.0));
    tracker.update(tick % 3 == 0, static_cast<uint16_t>(counts & 4095));
    const q32::Pos measured = q32::fromCounts(tracker.counts(), 12) + q32::fromDelta(tracker.extrapolation() / 4096.0f);
    trajectory.step(kDt);
    const float error = q32::diff(trajectory.position(), measured);
    worst = std::fmax(worst, std::fabs(error));
    const float v = controller.update(error, trajectory.velocity(), tracker.velocity() / 4096.0f, !trajectory.done(), kDt);
    const float rate = v * static_cast<float>(stepsPerTurn);
    if (std::fabs(rate) >= 0.05f) direction = rate > 0.0f;  // DIR follows the sign of a live rate
    timer.setRate(std::fabs(rate), now);
  }
  const double final = static_cast<double>(rotorSteps) / stepsPerTurn + 2.5;
  TEST_ASSERT_TRUE(trajectory.done());
  TEST_ASSERT_DOUBLE_WITHIN(2.0 / 4096.0, 0.0, final);
  TEST_ASSERT_TRUE(worst < 0.01f);
}

// ---------------------------------------------------------------- Params

void test_params_validate_and_round_trip() {
  Settings s;
  const params::Def* micro = params::find(proto::Param::Microsteps);
  TEST_ASSERT_NOT_NULL(micro);
  TEST_ASSERT_EQUAL(static_cast<int>(proto::Status::Ok), static_cast<int>(params::write(s, *micro, 64)));
  TEST_ASSERT_EQUAL_UINT16(64, s.microsteps);
  TEST_ASSERT_EQUAL(static_cast<int>(proto::Status::BadValue), static_cast<int>(params::write(s, *micro, 48)));
  TEST_ASSERT_EQUAL(static_cast<int>(proto::Status::BadValue), static_cast<int>(params::write(s, *micro, 512)));

  const params::Def* kp = params::find(proto::Param::Kp);
  float value = 12.5f;
  uint32_t raw;
  std::memcpy(&raw, &value, 4);
  TEST_ASSERT_EQUAL(static_cast<int>(proto::Status::Ok), static_cast<int>(params::write(s, *kp, raw)));
  TEST_ASSERT_EQUAL_UINT32(raw, params::read(s, *kp));
  value = NAN;
  std::memcpy(&raw, &value, 4);
  TEST_ASSERT_EQUAL(static_cast<int>(proto::Status::BadValue), static_cast<int>(params::write(s, *kp, raw)));

  const params::Def* calibrated = params::find(proto::Param::Calibrated);
  TEST_ASSERT_EQUAL(static_cast<int>(proto::Status::ReadOnly), static_cast<int>(params::write(s, *calibrated, 1)));
  TEST_ASSERT_NULL(params::find(static_cast<proto::Param>(0xEE)));
}

void test_sanitize_repairs_out_of_range_fields() {
  Settings s;
  s.microsteps = 48;
  s.nodeId = 0;
  s.maxVelocity = NAN;
  s.runCurrentMa = 9000;
  params::sanitize(s);
  const Settings d;
  TEST_ASSERT_EQUAL_UINT16(d.microsteps, s.microsteps);
  TEST_ASSERT_EQUAL_UINT8(d.nodeId, s.nodeId);
  TEST_ASSERT_EQUAL_FLOAT(d.maxVelocity, s.maxVelocity);
  TEST_ASSERT_EQUAL_UINT16(d.runCurrentMa, s.runCurrentMa);
}

// ------------------------------------------------------ Protocol & CRC

void test_codec_round_trip_and_ids() {
  uint8_t buffer[32];
  proto::Writer w(buffer, sizeof buffer);
  w.put(uint8_t{0x13}).put(1234.5678).put(uint8_t{1}).put(2.5f);
  TEST_ASSERT_EQUAL_UINT8(1 + 8 + 1 + 4, w.size());
  proto::Reader r(buffer, w.size());
  uint8_t cmd = 0, flags = 0;
  double position = 0.0;
  float velocity = 0.0f, missing = 7.0f;
  TEST_ASSERT_TRUE(r.get(cmd) && r.get(position) && r.get(flags) && r.get(velocity));
  r.getOr(missing);
  TEST_ASSERT_EQUAL_UINT8(0x13, cmd);
  TEST_ASSERT_EQUAL_DOUBLE(1234.5678, position);
  TEST_ASSERT_EQUAL_FLOAT(2.5f, velocity);
  TEST_ASSERT_EQUAL_FLOAT(7.0f, missing);  // absent optional field keeps its default

  TEST_ASSERT_EQUAL_HEX16(0x105, proto::canId(proto::Function::Command, 5));
  TEST_ASSERT_EQUAL_HEX16(0x27F, proto::canId(proto::Function::Telemetry, 127));
}

void test_crc32_check_value() {
  const char* check = "123456789";
  TEST_ASSERT_EQUAL_HEX32(0xCBF43926u, crc32::update(0, check, 9));
}

void test_fixed_point_conversions() {
  TEST_ASSERT_TRUE(q32::fromCounts(-1, 12) == -(q32::Pos{1} << 20));
  TEST_ASSERT_EQUAL_DOUBLE(-1.25, q32::toTurns(q32::fromTurns(-1.25)));
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.5f, q32::diff(q32::fromTurns(100.5), q32::fromTurns(100.0)));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_trapezoid_respects_limits_and_lands_exactly);
  RUN_TEST(test_short_move_is_triangular);
  RUN_TEST(test_retarget_reversal_mid_move);
  RUN_TEST(test_tiny_moves_far_from_origin_are_exact);
  RUN_TEST(test_velocity_mode_ramps_and_stops_holding);
  RUN_TEST(test_bounds_stop_velocity_mode_at_the_limit);
  RUN_TEST(test_controller_feedforward_deadband_and_slew);
  RUN_TEST(test_tracker_unwraps_and_estimates_velocity);
  RUN_TEST(test_tracker_rejects_single_glitch_but_accepts_real_jump);
  RUN_TEST(test_closed_loop_move_settles_without_hunting);
  RUN_TEST(test_step_timing_registers);
  RUN_TEST(test_open_loop_sweep_emits_commanded_steps);
  RUN_TEST(test_closed_loop_through_step_generator);
  RUN_TEST(test_params_validate_and_round_trip);
  RUN_TEST(test_sanitize_repairs_out_of_range_fields);
  RUN_TEST(test_codec_round_trip_and_ids);
  RUN_TEST(test_crc32_check_value);
  RUN_TEST(test_fixed_point_conversions);
  return UNITY_END();
}
