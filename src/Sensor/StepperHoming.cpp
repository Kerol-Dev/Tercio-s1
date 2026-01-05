#include "StepperHoming.h"

bool StepperHoming::begin(const HomingConfig& cfg) {
  _cfg = cfg;

  // If using sensorless homing, limit switch pins may be unused.
  if (!_cfg.sensorlessHoming) {
    pinMode(cfg.inMinPin, cfg.minActiveLow ? INPUT_PULLUP : INPUT_PULLDOWN);
    pinMode(cfg.inMaxPin, cfg.maxActiveLow ? INPUT_PULLUP : INPUT_PULLDOWN);
  }

  return true;
}

bool StepperHoming::readPin(uint8_t pin, bool activeLow) const {
  bool raw = digitalRead(pin);
  return activeLow ? !raw : raw;
}

void StepperHoming::update() {
  if (_cfg.sensorlessHoming) {
    // No physical limit switches to read
    _minTrig = false;
    _maxTrig = false;
    return;
  }

  _minTrig = readPin(_cfg.inMinPin, _cfg.minActiveLow);
  _maxTrig = readPin(_cfg.inMaxPin, _cfg.maxActiveLow);
}

/**
 * Notes:
 * - For sensorless homing, provide a StallFn callback that returns true when stall is detected.
 * - When _cfg.sensorlessHoming == true, min/max pins are ignored and stall detection is used instead.
 */
bool StepperHoming::home(SetVelFn setVel,
                         StopFn stop,
                         StallFn stallDetected,   // <-- added
                         Encoder enc,
                         AxisController con,
                         AxisConfig &axisCfg,
                         SetFn setZero,
                         bool seekToMin) {
  const uint32_t t0 = millis();
  auto timeout = [&]() { return (millis() - t0) > _cfg.timeoutMs; };

  // Pick which direction to seek
  const bool useMin = seekToMin;

  // Start moving toward the chosen limit (or stall direction for sensorless)
  float vel = useMin ? -fabsf(_cfg.seekSpeed) : fabsf(_cfg.seekSpeed);
  setVel(vel);

  // Basic debounce for stall detection to reduce false positives
  const uint8_t STALL_DEBOUNCE_COUNT = 5;
  uint8_t stallCount = 0;

  while (!timeout()) {
    // Update sensors
    update();
    enc.update(0.01);

    if (_cfg.sensorlessHoming) {
      // Require a valid callback
      if (!stallDetected) {
        stop();
        return false;
      }

      if (stallDetected()) {
        if (++stallCount >= STALL_DEBOUNCE_COUNT) break;
      } else {
        stallCount = 0;
      }
    } else {
      // Physical limit switch homing
      if ((useMin && _minTrig) || (!useMin && _maxTrig)) break;
    }

    delay(1);
  }

  stop();
  if (timeout()) return false;

  delay(1000);

  // Back off
  enc.update(0.01); // update once to avoid large jump

  // Backoff direction:
  // If we were seeking negative (min), backing off should be positive.
  // If we were seeking positive (max), backing off should be negative.
  const double backoffDir = (vel < 0.0f) ? +1.0 : -1.0;
  const double goal = enc.angle() + backoffDir * fabs(_cfg.backoffOffset);

  const double previousSpeed = axisCfg.maxRPS;
  axisCfg.maxRPS = fabs(vel);

  con.setTargetAngleRad(goal);

  while (true) {
    enc.update(0.01);
    con.update(0.01);

    const double err = fabs(enc.angle() - goal);
    if (err < 0.01) break;

    delay(1);
  }

  stop();
  axisCfg.maxRPS = previousSpeed;

  // Zero encoder
  setZero(0.0f);
  con.setTargetAngleRad(0);

  return true;
}