#pragma once
// -----------------------------------------------------------------------------
// ams AS5600, 12-bit, I2C. Continuously sampled in the background: with the
// register pointer parked on RAW ANGLE (the chip does not auto-increment past
// it), each sample is a bare 2-byte read, ~75 µs at 400 kHz — about twice the
// chip's own 150 µs sampling rate. A status read is slotted in every ~100 ms.
// -----------------------------------------------------------------------------
#include <Wire.h>

#include "drivers/I2cPoller.h"
#include "encoder/Encoder.h"

class As5600 final : public Encoder {
 public:
  As5600(uint32_t sdaPin, uint32_t sclPin, I2C_TypeDef* i2c) : wire_(sdaPin, sclPin), bus_(i2c) {}

  bool begin() override;
  void end() override;
  bool poll(uint16_t& raw) override;
  void service() override;
  uint8_t resolutionBits() const override { return 12; }
  MagnetStatus magnet() const override;

 private:
  enum class Step : uint8_t { Angle, Status, RestorePointer };

  bool readRegisters(uint8_t reg, uint8_t* data, uint8_t length);
  bool writeRegisters(uint8_t reg, const uint8_t* data, uint8_t length);

  TwoWire wire_;
  I2cPoller bus_;
  bool started_ = false;            // begin() requested by the application
  volatile bool running_ = false;   // background sampling active
  uint8_t retryCount_ = 0;
  volatile bool statusRequested_ = false;
  volatile uint16_t errorStreak_ = 0;
  volatile uint8_t status_ = 0;
  Step step_ = Step::Angle;
  bool pointerLost_ = false;  // register pointer must be restored before the next angle read
  uint8_t serviceCount_ = 0;
};
