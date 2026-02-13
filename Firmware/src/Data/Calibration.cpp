#include "Calibration.h"
#include "Encoder.h"
#include "StepperControl.h"
#include "AxisController.h"
#include "ConfigStore.h"

// --------------------------- Local Helpers -----------------------------------

static void jog(StepperControl& stepper, Encoder& encoder, double rps, uint32_t durationMs)
{
  stepper.setSpeedRPS(rps);

  const uint32_t startTimeMs = millis();
  uint32_t lastTimeUs = micros();

  while ((millis() - startTimeMs) < durationMs)
  {
    const uint32_t nowUs = micros();
    const double dtSec = (nowUs - lastTimeUs) * 1.0e-6;
    lastTimeUs = nowUs;
    
    encoder.update(dtSec);
  }

  stepper.stop();
}


// ---------------------------- Public API -------------------------------------

bool Calibrate_EncoderDirection(Encoder& encoder,
                                StepperControl& stepper,
                                AxisController& axis,
                                AxisConfig& config,
                                double testRPS,
                                uint32_t jogDurationMs)
{
  config.calibratedOnce = true;
  
  // Reset inversion state to known default before test
  encoder.setInvert(false);
  stepper.enable();

  axis.setMicrosteps(config.microsteps);

  // Perform positive (CW) motion test
  const double startAngleDeg = encoder.angle(Encoder::Radians) * RAD_TO_DEG;
  jog(stepper, encoder, +testRPS, jogDurationMs);
  const double endAngleDeg = encoder.angle(Encoder::Radians) * RAD_TO_DEG;
  
  const double deltaAngleDeg = endAngleDeg - startAngleDeg;

  // If positive motion resulted in negative angle change, encoder is inverted
  const bool isInverted = (deltaAngleDeg < 0.0);
  config.encInvert = isInverted;
  encoder.setInvert(isInverted);

  // Return to approximate start position
  jog(stepper, encoder, -testRPS, jogDurationMs);
  encoder.calibrateZero();

  return true;
}