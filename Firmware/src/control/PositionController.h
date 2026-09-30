#pragma once
// -----------------------------------------------------------------------------
// Position loop: velocity feed-forward from the trajectory plus PI on the
// position error and D on the velocity error. Output is a velocity command
// (turns/s) for the step generator.
//
//   v = v_ref + kp·e + ki·∫e + kd·(v_ref − v_meas)
//
// At rest a deadband with hysteresis zeroes the output and freezes the
// integrator, so encoder noise never turns into hunting. The output is
// clamped in magnitude and in rate of change (acceleration), which protects
// the stepper from pull-out on sudden corrections.
// -----------------------------------------------------------------------------

class PositionController {
 public:
  struct Gains {
    float kp = 20.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float deadband = 0.0005f;      // turns
    float integralLimit = 1.0f;    // turns/s
    float maxVelocity = 10.0f;     // turns/s, output clamp
    float maxAcceleration = 400.0f;  // turns/s², output slew limit
  };

  void setGains(const Gains& gains) { gains_ = gains; }
  void reset();

  // `moving`: the reference is in motion (deadband and settling only apply at rest).
  float update(float error, float referenceVelocity, float measuredVelocity, bool moving, float dt);

  bool settled() const { return settled_; }
  float output() const { return output_; }

 private:
  Gains gains_{};
  float integral_ = 0.0f;
  float output_ = 0.0f;
  bool settled_ = false;
};
