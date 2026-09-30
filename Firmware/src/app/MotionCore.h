#pragma once
// -----------------------------------------------------------------------------
// The real-time part of the axis: everything that runs in the 20 kHz control
// interrupt.
//
//   encoder -> tracker -> [trajectory | step/dir input] -> controller -> steps
//
// Frames: the *motor* frame is the step direction (DIR high = +). Encoder
// counts map onto it with `encoderSign` (found by calibration); the *user*
// frame is the motor frame times `direction`, with a zero offset. All public
// positions are user-frame Q32.32 turns of the encoder shaft.
//
// The main loop commands it through the methods below; each takes the
// interrupt lock for a few hundred nanoseconds. Asynchronous outcomes (stall,
// seek trigger, limit stop, lost encoder) are latched as events.
// -----------------------------------------------------------------------------
#include "control/EncoderTracker.h"
#include "control/Fixed.h"
#include "control/PositionController.h"
#include "control/Trajectory.h"

class Encoder;
class StepGenerator;

class MotionCore {
 public:
  enum class Mode : uint8_t {
    Off,       // bridge off; the reference follows the shaft
    Idle,      // bridge on, no steps (e.g. not calibrated yet)
    Closed,    // closed loop on the trajectory
    StepDir,   // closed loop on the external step/dir input
    Seek,      // closed-loop velocity until a trigger fires (homing)
    OpenLoop,  // motor-frame move without feedback (calibration)
  };

  // Seek triggers.
  static constexpr uint8_t kTriggerLimitMin = 1u << 0;
  static constexpr uint8_t kTriggerLimitMax = 1u << 1;
  static constexpr uint8_t kTriggerFollowingError = 1u << 2;

  // Latched events.
  static constexpr uint8_t kEventStall = 1u << 0;
  static constexpr uint8_t kEventSeekTriggered = 1u << 1;
  static constexpr uint8_t kEventLimitStop = 1u << 2;
  static constexpr uint8_t kEventEncoderLost = 1u << 3;

  struct Config {
    float stepsPerTurn = 3200.0f;       // motor microsteps per encoder turn (gear ratio included)
    float motorStepsPerTurn = 3200.0f;  // motor microsteps per motor turn
    int8_t direction = 1;               // user -> motor frame
    int8_t encoderSign = 1;             // encoder counts -> motor frame
    PositionController::Gains gains{};
    float followingErrorLimit = 0.05f;  // turns
    uint32_t stallTicks = 1000;         // 0: stall detection off
    bool softLimits = false;
    q32::Pos softMin = 0, softMax = 0;
    bool limitSwitches = false;
    q32::Pos stepDirIncrement = 0;      // user turns per external step pulse
  };

  struct Snapshot {
    Mode mode = Mode::Off;
    bool tracking = false;       // encoder delivering samples
    q32::Pos position = 0;       // measured
    q32::Pos reference = 0;      // instantaneous setpoint
    q32::Pos target = 0;         // end point of the current move
    float velocity = 0.0f;       // measured, turns/s
    float referenceVelocity = 0.0f;
    float followingError = 0.0f; // reference - position
    float output = 0.0f;         // controller velocity command, turns/s
    bool trajectoryDone = true;
    bool velocityMode = false;   // closed loop, trajectory running at a commanded velocity
    bool settled = false;
    bool openLoopDone = true;
    bool limitMin = false;
    bool limitMax = false;
    int64_t counts = 0;          // raw multi-turn encoder counts
    uint8_t seekCause = 0;       // trigger that ended the last seek
  };

  void attach(Encoder* encoder, StepGenerator* stepper);
  void configure(const Config& config);
  // Switches the position sensor. Tracking restarts on its first sample, at the
  // user position implied by `zeroRaw` (the raw reading at user position 0).
  void setEncoder(Encoder* encoder, uint16_t zeroRaw);

  // Control interrupt only.
  void tick();

  // ---- Commands (main loop) ----
  void setOff();
  void setIdle();
  void hold();
  // `bounded`: clamp the target to the soft limits (homing moves in a frame the
  // limits do not describe yet).
  void moveTo(q32::Pos target, float maxVelocity, float maxAcceleration, bool bounded = true);
  void runAt(float velocity, float maxAcceleration);
  void stop(float maxAcceleration);
  void seek(float velocity, float maxAcceleration, uint8_t triggers, float errorThreshold);
  void followStepDir();
  void openLoopMove(float motorTurns, float velocity, float acceleration);
  // Declares the current position to be `position`.
  void setPosition(q32::Pos position);
  // While frozen the tick keeps sensing but emits no steps and holds the
  // reference still — used around flash writes, which stall the CPU.
  void freezeOutput(bool frozen);

  Snapshot snapshot() const;
  uint8_t takeEvents();
  // Raw single-turn reading at user position 0, for persisting the zero.
  uint16_t zeroRaw() const;

 private:
  void enter(Mode mode);
  void holdHere();  // interrupt context: stop dead at the measured position
  void applyBounds();
  q32::Pos countsToUser(q32::Pos countsQ) const { return (sign_ > 0 ? countsQ : -countsQ) - zero_; }

  Encoder* encoder_ = nullptr;
  StepGenerator* stepper_ = nullptr;
  Config config_{};
  int8_t sign_ = 1;  // direction * encoderSign: encoder counts -> user frame
  uint8_t bits_ = 12;
  float countsToTurns_ = 1.0f / 4096.0f;

  EncoderTracker tracker_{};
  Trajectory trajectory_{};
  Trajectory openLoop_{};  // motor-frame turns
  PositionController controller_{};

  Mode mode_ = Mode::Off;
  bool tracking_ = false;
  bool frozen_ = false;
  uint16_t pendingZeroRaw_ = 0;
  q32::Pos zero_ = 0;
  q32::Pos measured_ = 0;
  q32::Pos reference_ = 0;
  float measuredVelocity_ = 0.0f;
  float referenceVelocity_ = 0.0f;

  uint8_t limitMinCount_ = 0, limitMaxCount_ = 0;
  bool limitMin_ = false, limitMax_ = false;

  int32_t lastStepCount_ = 0;
  float stepDirVelocity_ = 0.0f;

  uint8_t seekTriggers_ = 0;
  float seekErrorThreshold_ = 0.0f;
  uint32_t seekArmTicks_ = 0;
  uint32_t seekTicks_ = 0;
  uint8_t seekCause_ = 0;

  uint32_t stallCount_ = 0;
  volatile uint8_t events_ = 0;
};
