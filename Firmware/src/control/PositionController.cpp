#include "control/PositionController.h"

#include <cmath>

namespace {
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

void PositionController::reset() {
  integral_ = 0.0f;
  output_ = 0.0f;
  settled_ = false;
}

float PositionController::update(float error, float referenceVelocity, float measuredVelocity, bool moving,
                                 float dt) {
  const float magnitude = std::fabs(error);
  if (moving)
    settled_ = false;
  else if (magnitude <= gains_.deadband)
    settled_ = true;
  else if (magnitude > 2.0f * gains_.deadband)
    settled_ = false;

  float command = 0.0f;
  if (!settled_) {
    integral_ = clampf(integral_ + gains_.ki * error * dt, -gains_.integralLimit, gains_.integralLimit);
    command = referenceVelocity + gains_.kp * error + integral_ + gains_.kd * (referenceVelocity - measuredVelocity);
  }

  command = clampf(command, -gains_.maxVelocity, gains_.maxVelocity);
  const float step = gains_.maxAcceleration * dt;
  output_ = clampf(command, output_ - step, output_ + step);
  return output_;
}
