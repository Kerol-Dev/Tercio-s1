#include "StepperHoming.h"
#include <Arduino.h>
#include <cmath>

bool StepperHoming::begin(const HomingConfig& cfg)
{
  _cfg = cfg;

  if (!_cfg.sensorlessHoming)
  {
    pinMode(cfg.inMinPin, cfg.minActiveLow ? INPUT_PULLUP : INPUT_PULLDOWN);
    pinMode(cfg.inMaxPin, cfg.maxActiveLow ? INPUT_PULLUP : INPUT_PULLDOWN);
  }

  return true;
}

bool StepperHoming::readPin(uint8_t pin, bool activeLow) const
{
  const bool rawState = (digitalRead(pin) == HIGH);
  return activeLow ? !rawState : rawState;
}

void StepperHoming::update()
{
  if (_cfg.sensorlessHoming)
  {
    _minTrig = false;
    _maxTrig = false;
    return;
  }

  _minTrig = readPin(_cfg.inMinPin, _cfg.minActiveLow);
  _maxTrig = readPin(_cfg.inMaxPin, _cfg.maxActiveLow);
}

bool StepperHoming::home(SetVelFn setVel,
                         StopFn stop,
                        BroadcastFn broadcast,
                         Encoder& enc,
                         AxisController& con,
                         AxisConfig& axisCfg,
                         SetFn setZero,
                         bool seekToMin)
{
  const uint32_t startTime = millis();
  auto isTimeout = [&]() { return (millis() - startTime) > _cfg.timeoutMs; };

  const float seekSpeedMag = std::fabs(_cfg.seekSpeed);
  const float seekVelocity = seekToMin ? -seekSpeedMag : seekSpeedMag;
  
  // Start seeking
  setVel(seekVelocity);

  // Stall detection state
  const float stallSpeedThreshold = seekSpeedMag * 0.5f;
  const uint32_t stallDurationMs = 200;
  uint32_t stallStartTime = 0;
  bool stallTimerActive = false;

  // Wait briefly for acceleration to ramp up before checking stall
  delay(250); 

  while (!isTimeout())
  {
    update();
    broadcast();
    enc.update(0.01);

    if (_cfg.sensorlessHoming)
    {
      float currentVel = std::fabs(enc.velocity(Encoder::Rotations));

      if (currentVel < stallSpeedThreshold)
      {
        if (!stallTimerActive)
        {
          stallStartTime = millis();
          stallTimerActive = true;
        }
        else if ((millis() - stallStartTime) >= stallDurationMs)
        {
          break; // Confirmed stall
        }
      }
      else
      {
        stallTimerActive = false;
      }
    }
    else
    {
      if ((seekToMin && _minTrig) || (!seekToMin && _maxTrig))
        break;
    }
    
    delay(1);
  }

  stop();

  if (isTimeout())
    return false;

  delay(500); 
  
  float currentPos = enc.angle(Encoder::Radians);
  float backoffDir = seekToMin ? 1.0f : -1.0f;
  float backoffTarget = currentPos + (backoffDir * std::fabs(_cfg.backoffOffset));

  // Use position control for backoff
  float originalMaxRPS = axisCfg.maxRPS;
  axisCfg.maxRPS = seekSpeedMag; 

  con.setTargetAngleRad(backoffTarget);

  const uint32_t backoffStart = millis();
  while ((millis() - backoffStart) < 5000)
  {
    broadcast();
    enc.update(0.01);
    con.update(0.01);

    float error = std::fabs(enc.angle(Encoder::Radians) - backoffTarget);
    if (error < 0.01f)
      break;

    delay(1);
  }

  stop();
  axisCfg.maxRPS = originalMaxRPS;

  setZero(0.0f);
  con.setTargetAngleRad(0.0f);
  
  return true;
}