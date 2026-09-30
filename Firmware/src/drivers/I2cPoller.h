#pragma once
// -----------------------------------------------------------------------------
// Non-blocking I2C master for the STM32G4 I2C peripheral, driven by polling.
// start() launches a transaction ([write n bytes][restart][read m bytes]); each
// poll() advances it using only register accesses and returns immediately, so
// it can be stepped from the 20 kHz control interrupt. Clock/pin setup is done
// beforehand by the core's TwoWire, which also owns bus recovery.
// -----------------------------------------------------------------------------
#include <Arduino.h>

class I2cPoller {
 public:
  enum class Result : uint8_t { Idle, Busy, Done, Error };

  explicit I2cPoller(I2C_TypeDef* i2c) : i2c_(i2c) {}

  // Takes the peripheral over from the HAL: masks its interrupts.
  void attach();
  // True when a new transaction may start: nothing in flight and the bus free
  // (after an error the STOP condition may still be going out).
  bool ready();
  void start(uint8_t address, const uint8_t* tx, uint8_t txLength, uint8_t rxLength);
  Result poll();
  const uint8_t* rx() const { return rx_; }

 private:
  enum class Phase : uint8_t { Idle, Write, Read, Recover };
  void fail();
  void resetPeripheral();

  I2C_TypeDef* i2c_;
  Phase phase_ = Phase::Idle;
  uint8_t address_ = 0;
  uint8_t tx_[4] = {};
  uint8_t txLength_ = 0, txIndex_ = 0;
  uint8_t rx_[4] = {};
  uint8_t rxLength_ = 0, rxIndex_ = 0;
  uint16_t polls_ = 0;
};
