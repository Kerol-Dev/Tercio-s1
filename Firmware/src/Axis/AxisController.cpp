#include "AxisController.h"
#include <algorithm>
#include <cmath>


// ---------------------------- SimplePID --------------------------------------

void SimplePID::setTunings(double Kp, double Ki, double Kd)
{
  kp = Kp;
  ki = Ki;
  kd = Kd;
}

double SimplePID::compute(double err, double dt)
{
  integ += err * ki * dt;

  const double deriv = (dt > 0.0) ? (err - ePrev) / dt : 0.0;
  ePrev = err;

  return (kp * err) + integ + (kd * deriv);
}


// -------------------------- AxisController -----------------------------------

AxisController *AxisController::s_owner = nullptr;

AxisController::AxisController(Encoder &enc,
                               StepperControl &stepgen,
                               TMC2209Stepper &tmc,
                               AxisConfig &cfg)
  : _enc(enc),
    _stepgen(stepgen),
    _tmc(tmc),
    _cfg(cfg)
{
}

void AxisController::begin()
{
  _enc.calibrateZero();
  _stepgen.begin();
  _stepgen.enable();
}

void AxisController::configureDriver(const TmcConfig &c)
{
  _tmc.begin();
  _tmc.shaft(false);

  _tmc.intpol(true);
  _tmc.toff(4);
  _tmc.blank_time(1);
  _tmc.rms_current(c.mA);
  _tmc.pwm_autoscale(true);
  _tmc.pwm_freq(1);
  _tmc.pwm_grad(1);

  _tmc.vsense(false);

  if (c.stealth)
  {
    _tmc.en_spreadCycle(false);
    _tmc.pwm_autograd(true);
  }
  else
  {
    _tmc.en_spreadCycle(true);
  }

  setMicrosteps(_cfg.microsteps);
}

void AxisController::setFullSteps(uint16_t fullSteps)
{
  _cfg.stepsPerRev = fullSteps ? fullSteps : 200;
  _ustepAngleRad = (2.0 * PI) / static_cast<double>(_cfg.stepsPerRev * _cfg.microsteps);
  setSpreadSwitchRPS(_spreadSwitchRPS);
}

void AxisController::setMicrosteps(uint16_t micro)
{
  _cfg.microsteps = micro ? micro : 1;
  _ustepAngleRad = (2.0 * PI) / static_cast<double>(_cfg.stepsPerRev * _cfg.microsteps);

  _tmc.microsteps(_cfg.microsteps);
  _tmc.intpol(true);

  setSpreadSwitchRPS(_spreadSwitchRPS);
}

uint16_t AxisController::microsteps() const
{
  return _cfg.microsteps;
}

void AxisController::setPID(double Kp, double Ki, double Kd)
{
  _pid.setTunings(Kp, Ki, Kd);
}

void AxisController::setTargetAngleRad(double r)
{
  _targetRad = r;
}

double AxisController::targetAngleRad() const
{
  return _targetRad;
}

void AxisController::setExternalMode(bool enabled)
{
  _externalMode = enabled;
}

bool AxisController::externalMode() const
{
  return _externalMode;
}

void AxisController::attachExternal(const ExtPins &p)
{
  _ext = p;

  if (_ext.en >= 0)
    pinMode(_ext.en, INPUT_PULLUP);

  if (_ext.dir >= 0)
    pinMode(_ext.dir, INPUT_PULLDOWN);

  if (_ext.step >= 0)
  {
    pinMode(_ext.step, INPUT_PULLDOWN);
    s_owner = this;
    attachInterrupt(digitalPinToInterrupt(_ext.step), AxisController::onStepISR, RISING);
  }
}

void AxisController::update(double dt)
{
  if (_externalMode && _ext.step >= 0)
  {
    int32_t stepPulses = 0;
    noInterrupts();
    stepPulses = _extStepPulses;
    _extStepPulses = 0;
    interrupts();

    if (stepPulses != 0)
    {
      const int dirSign = (_ext.dir >= 0) ? (digitalRead(_ext.dir) ? +1 : -1) : +1;
      _targetRad += static_cast<double>(stepPulses * dirSign) * _ustepAngleRad;
    }

    if (_ext.en >= 0)
    {
      const bool enActive = _ext.enActiveLow ? (digitalRead(_ext.en) == LOW)
                                            : (digitalRead(_ext.en) == HIGH);

      if (!enActive)
      {
        _stepgen.stop();
        _stepgen.disable();
        _cmdRPS = 0.0;
        return;
      }

      _stepgen.enable();
    }
  }

  _enc.update(dt);

  const double posRad = _enc.angle(Encoder::Radians);
  const double errRad = _targetRad - posRad;

  const double velTargetRPS = _pid.compute(errRad, dt);

  const double dtEff = std::clamp(dt, 0.001, 0.020);
  const double dvMax = _cfg.maxRPS2 * dtEff;

  double cmdRPS = 0.0;
  if (fabs(velTargetRPS) > fabs(_cmdRPS) && (fabs(velTargetRPS - _cmdRPS) > dvMax))
  {
    const double sign = (velTargetRPS > _cmdRPS) ? +1.0 : -1.0;
    cmdRPS = _cmdRPS + sign * dvMax;
  }
  else
  {
    cmdRPS = velTargetRPS;
  }

  _cmdRPS = std::clamp(cmdRPS,
                       -static_cast<double>(_cfg.maxRPS),
                       static_cast<double>(_cfg.maxRPS));

  if (fabs(_cmdRPS) <= 0.001)
    _stepgen.setSpeedRPS(0);
  else
    _stepgen.setSpeedRPS(_cmdRPS);
}

void AxisController::setSpreadSwitchRPS(double rps)
{
  _spreadSwitchRPS = (rps > 0.0) ? rps : 0.1;

  static constexpr uint32_t kTmcClkHz = 12000000UL;

  const double ustepsPerRev =
      static_cast<double>(_cfg.stepsPerRev) * static_cast<double>(_cfg.microsteps);

  const double stepHz = _spreadSwitchRPS * ustepsPerRev;

  uint32_t tpwm = 0xFFFFF;
  if (stepHz > 1.0)
  {
    const double calc = static_cast<double>(kTmcClkHz) / stepHz;
    tpwm = static_cast<uint32_t>(std::lround(calc));
  }

  _tmc.TPWMTHRS(tpwm);
}

void AxisController::onStepISR()
{
  if (s_owner)
    s_owner->_extStepPulses++;
}