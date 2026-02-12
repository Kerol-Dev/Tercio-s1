#include "StepperHoming.h"
#include <Arduino.h>
#include <cmath>

// -----------------------------------------------------------------------------
// Initialization
// -----------------------------------------------------------------------------
bool StepperHoming::begin(const HomingConfig& cfg)
{
  _cfg = cfg;

  // Only configure pins if physical switches are used
  if (!_cfg.sensorlessHoming)
  {
    pinMode(cfg.inMinPin, cfg.minActiveLow ? INPUT_PULLUP : INPUT_PULLDOWN);
    pinMode(cfg.inMaxPin, cfg.maxActiveLow ? INPUT_PULLUP : INPUT_PULLDOWN);
  }

  return true;
}

// -----------------------------------------------------------------------------
// Pin Reading Helper
// -----------------------------------------------------------------------------
bool StepperHoming::readPin(uint8_t pin, bool activeLow) const
{
  const bool rawState = (digitalRead(pin) == HIGH);
  return activeLow ? !rawState : rawState;
}

// -----------------------------------------------------------------------------
// Sensor Update
// -----------------------------------------------------------------------------
void StepperHoming::update()
{
  if (_cfg.sensorlessHoming)
  {
    // Physical pins ignored in sensorless mode
    _minTrig = false;
    _maxTrig = false;
    return;
  }

  _minTrig = readPin(_cfg.inMinPin, _cfg.minActiveLow);
  _maxTrig = readPin(_cfg.inMaxPin, _cfg.maxActiveLow);
}

// -----------------------------------------------------------------------------
// Homing Routine
// -----------------------------------------------------------------------------
bool StepperHoming::home(SetVelFn setVel,
                         StopFn stop,
                         StallFn stallDetected,
                         Encoder& enc,             // Passed by reference
                         AxisController& con,      // Passed by reference
                         AxisConfig& axisCfg,
                         SetFn setZero,
                         bool seekToMin)
{
  const uint32_t startTime = millis();
  auto isTimeout = [&]() { return (millis() - startTime) > _cfg.timeoutMs; };

  // Determine velocity direction
  // seekToMin -> Negative Velocity
  // !seekToMin -> Positive Velocity
  const float seekVelocity = seekToMin ? -std::fabs(_cfg.seekSpeed) : std::fabs(_cfg.seekSpeed);
  setVel(seekVelocity);

  // Stall detection debounce
  constexpr uint8_t STALL_DEBOUNCE_THRESHOLD = 5;
  uint8_t stallConsecutiveCount = 0;

  // --- Phase 1: Seek Limit ---
  while (!isTimeout())
  {
    update();
    enc.update(0.01); // Simulated dt for encoder update

    if (_cfg.sensorlessHoming)
    {
      if (!stallDetected)
      {
        stop();
        return false; // Callback required
      }

      if (stallDetected())
      {
        if (++stallConsecutiveCount >= STALL_DEBOUNCE_THRESHOLD)
          break; // Confirmed stall
      }
      else
      {
        stallConsecutiveCount = 0;
      }
    }
    else
    {
      // Physical Switch Check
      if ((seekToMin && _minTrig) || (!seekToMin && _maxTrig))
        break;
    }

    delay(1);
  }

  stop();

  if (isTimeout())
    return false;

  delay(1000); // Settle time

  // --- Phase 2: Backoff ---
  enc.update(0.01); // Refresh encoder state

  // Backoff moves opposite to seek direction
  const double backoffDir = (seekVelocity < 0.0f) ? 1.0 : -1.0;
  const double backoffTarget = enc.angle() + (backoffDir * std::fabs(_cfg.backoffOffset));

  // Temporarily limit speed for precise backoff
  const double originalMaxRPS = axisCfg.maxRPS;
  axisCfg.maxRPS = std::fabs(seekVelocity);

  con.setTargetAngleRad(backoffTarget);

  // Closed-loop move to backoff position
  while (true)
  {
    enc.update(0.01);
    con.update(0.01);

    const double error = std::fabs(enc.angle() - backoffTarget);
    if (error < 0.01) // Convergence threshold (radians)
      break;

    delay(1);
  }

  stop();
  axisCfg.maxRPS = originalMaxRPS; // Restore config

  // --- Phase 3: Zeroing ---
  setZero(0.0f);
  con.setTargetAngleRad(0.0);

  return true;
}