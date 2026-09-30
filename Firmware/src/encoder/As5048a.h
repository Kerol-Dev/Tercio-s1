#pragma once
// -----------------------------------------------------------------------------
// ams AS5048A, 14-bit, SPI (mode 1). Read every control tick with a single
// pipelined 16-bit frame: each frame carries the next command and returns the
// answer to the previous one, so the angle is one tick (50 µs) old and a read
// costs ~4 µs. SPI1 is driven directly through its registers.
// -----------------------------------------------------------------------------
#include "encoder/Encoder.h"

class As5048a final : public Encoder {
 public:
  bool begin() override;
  void end() override;
  bool poll(uint16_t& raw) override;
  void service() override;
  uint8_t resolutionBits() const override { return 14; }
  MagnetStatus magnet() const override;

 private:
  uint16_t transfer(uint16_t command);

  volatile bool running_ = false;
  volatile bool diagnosticsRequested_ = false;
  volatile uint16_t diagnostics_ = 0;
  uint16_t lastCommand_ = 0;
  uint16_t nextCommand_ = 0;
  uint8_t serviceCount_ = 0;
};
