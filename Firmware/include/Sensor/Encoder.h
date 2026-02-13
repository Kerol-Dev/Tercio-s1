#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <AS5600.h>
#include <AS5048A.h>

class Encoder
{
public:
  enum Units : uint8_t
  {
    Degrees,
    Radians,
    Rotations
  };

  // Constructor
  explicit Encoder(uint16_t countsPerRevolution = 4096);

  bool begin(uint8_t sdaPin, uint8_t sclPin, bool useAS5600, uint8_t csPin);

  // Resets internal position tracking to zero based on current angle
  void calibrateZero();

  // Configuration
  void setInvert(bool invert);
  void setVelAlpha(float alpha); // Set velocity smoothing (0.0 - 1.0)

  // Main update loop - call this as frequently as possible
  void update(double dt);

  // Getters
  double angle(Units unit) const;
  double velocity(Units unit) const;

  // Diagnostics (AS5600 specific mostly)
  bool magnetPresent();
  bool magnetTooWeak();
  bool magnetTooStrong();
  bool magnetWrong(); // Checks presence and strength

  uint16_t rawCounts();
  uint16_t cpr() const;

private:
  // Sensor Objects
  AS5600 _as5600;
  AS5048A _as5048a;

  // Configuration
  uint16_t _cpr;
  bool _useAS5600;
  bool _invert;
  float _velocityAlpha;

  // Pin configurations
  uint8_t _csPin;
  uint8_t _sdaPin;
  uint8_t _sclPin;

  // State
  uint16_t _lastRawCount;
  int32_t _continuousPosition;
  double _velocityCountsPerSec;

  // Error handling
  uint8_t _i2cErrorCount;
};