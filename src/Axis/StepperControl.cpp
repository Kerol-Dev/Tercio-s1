#include "StepperControl.h"
#include <HardwareTimer.h>
#include <algorithm>


namespace
{
  constexpr uint8_t  PWM_CHANNEL        = 2;
  constexpr double   MIN_ACTIVE_SPS     = 0.05;
  constexpr uint32_t MIN_ARR            = 2;
  constexpr uint32_t MAX_ARR            = 65535;
  constexpr uint32_t MIN_PRESCALER      = 1;
  constexpr uint32_t MAX_PRESCALER      = 65536;
  constexpr uint8_t  DUTY_PERCENT       = 50;
}


// -------------------------- StepperControl -----------------------------------

StepperControl::StepperControl(uint8_t stepPin,
                               uint8_t dirPin,
                               uint8_t enPin,
                               AxisConfig &cfg)
  : _stepPin(stepPin),
    _dirPin(dirPin),
    _enPin(enPin),
    _cfg(cfg)
{
}

void StepperControl::begin()
{
  pinMode(_dirPin, OUTPUT);
  pinMode(_enPin, OUTPUT);
  disable();

  _tim = new HardwareTimer(TIM1);
  _tim->setMode(PWM_CHANNEL, TIMER_OUTPUT_COMPARE_PWM1, _stepPin);

  _tim->pause();
  _tim->setPrescaleFactor(MIN_PRESCALER);
  _tim->setOverflow(MAX_ARR, TICK_FORMAT);
  _tim->setCaptureCompare(PWM_CHANNEL, 0, PERCENT_COMPARE_FORMAT);
  _tim->resume();
}

void StepperControl::enable()
{
  digitalWrite(_enPin, LOW);
}

void StepperControl::disable()
{
  digitalWrite(_enPin, HIGH);
}

bool StepperControl::getEnabled()
{
  return digitalRead(_enPin) == LOW;
}

void StepperControl::setDir(bool clockwise)
{
  digitalWrite(_dirPin, clockwise ? HIGH : LOW);
}

void StepperControl::setStepRate(double stepsPerSecond)
{
  if (!_tim)
    return;

  if (std::abs(stepsPerSecond) < MIN_ACTIVE_SPS)
  {
    _tim->pause();
    _tim->setCaptureCompare(PWM_CHANNEL, 0, PERCENT_COMPARE_FORMAT);
    _tim->resume();
    return;
  }

  const double targetFrequency = std::max(0.0, stepsPerSecond);
  const uint32_t timerClockFreq = _tim->getTimerClkFreq();
  const double totalTicks = static_cast<double>(timerClockFreq) / targetFrequency;

  uint32_t prescaler = (totalTicks > static_cast<double>(MAX_ARR))
                         ? static_cast<uint32_t>((totalTicks + MAX_ARR - 1.0) / MAX_ARR)
                         : 1u;

  prescaler = std::clamp(prescaler, MIN_PRESCALER, MAX_PRESCALER);

  const double arrFloat = totalTicks / static_cast<double>(prescaler);
  uint32_t arr = static_cast<uint32_t>(
      std::clamp(arrFloat, static_cast<double>(MIN_ARR), static_cast<double>(MAX_ARR))
  );

  _tim->pause();
  _tim->setPrescaleFactor(prescaler);
  _tim->setOverflow(arr, TICK_FORMAT);
  _tim->setCaptureCompare(PWM_CHANNEL, DUTY_PERCENT, PERCENT_COMPARE_FORMAT);
  _tim->resume();
}

void StepperControl::stop()
{
  setStepRate(0.0);
}

void StepperControl::setSpeedRPS(double revolutionsPerSecond)
{
  if (_cfg.dirInvert)
    revolutionsPerSecond = -revolutionsPerSecond;

  const double stepsPerSecond = std::abs(revolutionsPerSecond) * getStepsPerRevolution();
  setStepRate(stepsPerSecond);
  setDir(revolutionsPerSecond >= 0.0);
}

double StepperControl::getStepsPerRevolution()
{
  return static_cast<double>(_cfg.stepsPerRev) * static_cast<double>(_cfg.microsteps);
}