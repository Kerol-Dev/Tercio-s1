#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <AS5600.h>

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

  bool begin(uint8_t sdaPin, uint8_t sclPin, uint8_t csPin);

  void calibrateZero();

  void setInvert(bool invert);
  void setVelAlpha(float alpha);

  void update(double dt);

  double angle(Units unit) const;
  double velocity(Units unit) const;

  bool magnetPresent();
  bool magnetTooWeak();
  bool magnetTooStrong();
  bool magnetWrong();

  uint16_t rawCounts();
  uint16_t cpr() const;

private:
  // Sensor Objects
  AS5600 _as5600;

  // Configuration
  uint16_t _cpr;
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