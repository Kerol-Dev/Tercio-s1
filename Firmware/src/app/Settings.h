#pragma once
// -----------------------------------------------------------------------------
// Persistent configuration. Stored verbatim in flash (see FlashStore), so bump
// kLayoutVersion whenever fields are added, removed or reordered — a stored
// record with a different layout is ignored and defaults are used instead.
// -----------------------------------------------------------------------------
#include <cstdint>

struct Settings {
  static constexpr uint16_t kLayoutVersion = 2;

  // Communication
  uint8_t nodeId = 1;
  uint16_t telemetryRateHz = 100;
  uint16_t commandTimeoutMs = 0;

  // Motor & driver
  uint16_t fullStepsPerRev = 200;
  uint16_t microsteps = 16;
  uint16_t runCurrentMa = 1500;
  uint8_t holdCurrentPct = 50;
  bool stealthChop = true;
  float stealthChopMaxVel = 1.0f;  // motor turns/s
  bool invertDirection = false;
  float gearRatio = 1.0f;          // motor turns per encoder turn

  // Motion & control (encoder turns)
  float maxVelocity = 10.0f;
  float maxAcceleration = 100.0f;
  float kp = 20.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float positionDeadband = 0.0005f;
  float followingErrorLimit = 0.05f;
  uint16_t stallTimeoutMs = 50;
  float softLimitMin = 0.0f;
  float softLimitMax = 0.0f;
  bool enableOnBoot = true;

  // Encoder
  uint8_t encoderType = 0;
  bool encoderInvert = false;
  bool calibrated = false;
  uint16_t encoderZeroRaw = 0;  // raw single-turn reading at user position 0

  // Inputs
  bool limitSwitchesEnabled = false;
  bool limitSwitchActiveLow = true;
  bool stepDirMode = false;
  bool stepDirEnableActiveLow = true;

  // Homing
  uint8_t homingMode = 0;
  float homingVelocity = 1.0f;
  uint16_t homingCurrentMa = 800;
  float homingBackoff = 0.05f;
  float homingStallError = 0.03f;
  uint16_t homingTimeoutS = 30;

  // Protection
  float overTemperatureC = 95.0f;

  bool softLimitsActive() const { return softLimitMin < softLimitMax; }
};
