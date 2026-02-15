#pragma once
#include <Arduino.h>
#include <functional>
#include <Encoder.h>
#include "AxisController.h"

struct HomingConfig {
  uint8_t inMinPin;       // IN1 pin
  uint8_t inMaxPin;       // IN2 pin
  bool sensorlessHoming;
  float currentLimit;
  bool minActiveLow;
  bool maxActiveLow;
  float seekSpeed;
  uint32_t timeoutMs;
  float backoffOffset;
};

class StepperHoming {
public:
  using SetVelFn = std::function<void(float)>;  // set velocity
  using StopFn   = std::function<void(void)>;   // stop motor
  using BroadcastFn   = std::function<void(void)>;   // broadcast can telemetry
  using StallFn  = std::function<bool()>;
  using ReadFn   = std::function<double(void)>;  // read position
  using SetFn    = std::function<void(float)>;  // set position

  StepperHoming() = default;
  bool begin(const HomingConfig& cfg);

  bool home(SetVelFn setVel,
                         StopFn stop,
                         BroadcastFn broadcast,
                         Encoder& enc,            
                         AxisController& con,   
                         AxisConfig& axisCfg,
                         SetFn setZero,
                         bool seekToMin);

  // Check raw state
  bool minTriggered() const { return _minTrig; }
  bool maxTriggered() const { return _maxTrig; }

  void update();
private:
  HomingConfig _cfg{};
  bool _minTrig = false;
  bool _maxTrig = false;

  bool readPin(uint8_t pin, bool activeLow) const;
};