#include "StepperControl.h"
#include <HardwareTimer.h>
#include <algorithm>

namespace
{
  constexpr uint8_t PWM_CHANNEL = 2;
  constexpr double MIN_ACTIVE_SPS = 0.05;
  constexpr uint32_t MIN_ARR = 2;
  constexpr uint32_t MAX_ARR = 65535;
  constexpr uint32_t MIN_PRESCALER = 1;
  constexpr uint32_t MAX_PRESCALER = 65536;
  constexpr uint8_t DUTY_PERCENT = 50;
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

  uint32_t psc = (_tim->getTimerClkFreq() / 1000000) - 1;
  _tim->setPrescaleFactor(psc);

  _tim->setOverflow(65535, TICK_FORMAT);
  _tim->setCaptureCompare(PWM_CHANNEL, 0, TICK_FORMAT);
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
    _tim->setCaptureCompare(PWM_CHANNEL, 0, TICK_FORMAT);
    return;
  }

  static const uint32_t timerClockFreq = _tim->getTimerClkFreq();
  static const uint32_t FIXED_PSC = (timerClockFreq / 1000000) - 1;

  if (_tim->getPrescaleFactor() != FIXED_PSC)
  {
    _tim->setPrescaleFactor(FIXED_PSC);
  }

  uint32_t arr = static_cast<uint32_t>(1000000.0 / std::abs(stepsPerSecond));

  if (arr < MIN_ARR)
    arr = MIN_ARR;
  if (arr > MAX_ARR)
    arr = MAX_ARR;

  _tim->setOverflow(arr, TICK_FORMAT);

  _tim->setCaptureCompare(PWM_CHANNEL, arr / 2, TICK_FORMAT);
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