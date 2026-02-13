#include "Encoder.h"
#include <Wire.h>
#include <SPI.h>
#include <cmath>

// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------
namespace
{
  constexpr uint32_t I2C_CLOCK_SPEED   = 400000;
  constexpr uint32_t I2C_TIMEOUT_MS    = 10;
  constexpr uint8_t  I2C_MAX_RETRIES   = 3;
  constexpr uint16_t AS5600_MAX_RAW    = 4095;
  
  // Default SPI pins for STM32G431 (adjust if different board variant)
  constexpr uint8_t  SPI_MISO_PIN      = PA6;
  constexpr uint8_t  SPI_MOSI_PIN      = PA7;
  constexpr uint8_t  SPI_SCLK_PIN      = PA5;
}


// -----------------------------------------------------------------------------
// Helper Functions
// -----------------------------------------------------------------------------

// Manually bit-bangs I2C clock to release stuck SDA lines
static void recoverI2CBus(uint8_t sdaPin, uint8_t sclPin)
{
  // Configure as GPIO to manually toggle
  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, OUTPUT_OPEN_DRAIN);
  
  digitalWrite(sclPin, HIGH);
  delayMicroseconds(5);

  // Clock out up to 9 pulses to clear any slave holding SDA low
  for (int i = 0; i < 9; ++i)
  {
    digitalWrite(sclPin, LOW);
    delayMicroseconds(5);
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(5);
  }

  // Generate STOP condition
  pinMode(sdaPin, OUTPUT_OPEN_DRAIN);
  digitalWrite(sdaPin, LOW);
  delayMicroseconds(5);
  digitalWrite(sclPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(sdaPin, HIGH);
  delayMicroseconds(5);

  // Restore to Input/Pullup before handing back to peripheral
  pinMode(sdaPin, INPUT_PULLUP);
}


// -----------------------------------------------------------------------------
// Encoder Implementation
// -----------------------------------------------------------------------------

Encoder::Encoder(uint16_t countsPerRevolution)
  : _as5600(),
    _as5048a(PB2, false), // Default CS, can be overridden in begin()
    _cpr(countsPerRevolution),
    _useAS5600(true),
    _csPin(PB2),
    _sdaPin(PB7),
    _sclPin(PA15),
    _invert(false),
    _velocityAlpha(1.0f),
    _lastRawCount(0),
    _continuousPosition(0),
    _velocityCountsPerSec(0.0),
    _i2cErrorCount(0)
{
}

bool Encoder::begin(uint8_t sdaPin, uint8_t sclPin, bool useAS5600, uint8_t csPin)
{
  _useAS5600 = useAS5600;
  _csPin = csPin;
  _sdaPin = sdaPin;
  _sclPin = sclPin;

  // Initialize I2C (Shared bus for AS5600)
  Wire.setSDA(_sdaPin);
  Wire.setSCL(_sclPin);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);
  Wire.setTimeout(I2C_TIMEOUT_MS);

  // Initialize SPI (Shared bus for AS5048A)
  SPI.setMISO(SPI_MISO_PIN);
  SPI.setMOSI(SPI_MOSI_PIN);
  SPI.setSCLK(SPI_SCLK_PIN);

  // Update AS5048A instance with correct CS pin
  _as5048a = AS5048A(_csPin, false);

  if (_useAS5600)
  {
    // AS5600 begin() checks for device presence
    if (!_as5600.begin())
      return false;
  }
  else
  {
    _as5048a.begin();
  }

  // Initialize state
  calibrateZero();
  return true;
}

void Encoder::calibrateZero()
{
  if (_useAS5600)
  {
     // Ensure we get a valid reading or fallback to 0
    _lastRawCount = _as5600.readAngle();
    if (_lastRawCount > AS5600_MAX_RAW) 
      _lastRawCount = 0;
  }
  else
  {
    _lastRawCount = _as5048a.getRawRotation();
  }
  
  _continuousPosition = 0;
  _velocityCountsPerSec = 0.0;
  _i2cErrorCount = 0;
}

void Encoder::setInvert(bool invert)
{
  _invert = invert;
}

void Encoder::setVelAlpha(float alpha)
{
  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha > 1.0f) alpha = 1.0f;
  _velocityAlpha = alpha;
}

void Encoder::update(double dt)
{
  if (dt <= 0.0)
    return;

  uint16_t currentRaw = 0;

  if (_useAS5600)
  {
    // AS5600 I2C Read with Error Handling
    currentRaw = _as5600.readAngle();

    // Check for I2C errors (AS5600 returns > 4095 on some libs or we infer from timeout)
    if (currentRaw > AS5600_MAX_RAW)
    {
      _i2cErrorCount++;
      
      if (_i2cErrorCount >= I2C_MAX_RETRIES)
      {
        // Try to recover the bus if we have persistent failures
        recoverI2CBus(_sdaPin, _sclPin);
        
        // Restart Wire peripheral
        Wire.end();
        Wire.begin();
        Wire.setClock(I2C_CLOCK_SPEED);
        Wire.setTimeout(I2C_TIMEOUT_MS);
        
        _i2cErrorCount = 0;
      }
      return; // Skip update this cycle
    }
    else
    {
      _i2cErrorCount = 0;
    }
  }
  else
  {
    // AS5048A SPI Read
    currentRaw = _as5048a.getRawRotation();
  }

  // Calculate delta with wrap-around handling
  int16_t delta = static_cast<int16_t>(currentRaw - _lastRawCount);
  const int16_t halfRange = static_cast<int16_t>(_cpr / 2);

  if (delta > halfRange)
    delta -= _cpr;
  else if (delta < -halfRange)
    delta += _cpr;

  _lastRawCount = currentRaw;
  _continuousPosition += delta;

  // Calculate velocity (exponential moving average)
  const double instantaneousVelocity = static_cast<double>(delta) / dt;
  _velocityCountsPerSec = ((1.0 - _velocityAlpha) * _velocityCountsPerSec) + (_velocityAlpha * instantaneousVelocity);
}

double Encoder::angle(Units unit) const
{
  const double direction = _invert ? -1.0 : 1.0;
  const double position = static_cast<double>(_continuousPosition);

  switch (unit)
  {
    case Degrees:
      return direction * position * (360.0 / _cpr);
    case Radians:
      return direction * position * (2.0 * M_PI / _cpr);
    case Rotations:
      return direction * position / _cpr;
    default:
      return 0.0;
  }
}

double Encoder::velocity(Units unit) const
{
  const double direction = _invert ? -1.0 : 1.0;

  switch (unit)
  {
    case Degrees:
      return direction * _velocityCountsPerSec * (360.0 / _cpr);
    case Radians:
      return direction * _velocityCountsPerSec * (2.0 * M_PI / _cpr);
    case Rotations:
      return direction * _velocityCountsPerSec / _cpr;
    default:
      return 0.0;
  }
}

// -----------------------------------------------------------------------------
// Diagnostics
// -----------------------------------------------------------------------------

bool Encoder::magnetPresent()
{
  return _useAS5600 ? _as5600.detectMagnet() : true; // AS5048A usually doesn't expose this simply
}

bool Encoder::magnetTooWeak()
{
  return _useAS5600 ? _as5600.magnetTooWeak() : false;
}

bool Encoder::magnetTooStrong()
{
  return _useAS5600 ? _as5600.magnetTooStrong() : false;
}

bool Encoder::magnetWrong()
{
  if (!_useAS5600)
    return false;
    
  return (!magnetPresent() || magnetTooWeak() || magnetTooStrong());
}

uint16_t Encoder::rawCounts()
{
  if (_useAS5600)
  {
     uint16_t val = _as5600.readAngle();
     return (val > AS5600_MAX_RAW) ? 0 : val;
  }
  return _as5048a.getRawRotation();
}

uint16_t Encoder::cpr() const
{
  return _cpr;
}