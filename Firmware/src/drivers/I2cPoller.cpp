#include "drivers/I2cPoller.h"

namespace {
constexpr uint32_t kErrorFlags = I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR | I2C_ISR_TIMEOUT;
constexpr uint32_t kClearAll = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF |
                               I2C_ICR_TIMOUTCF;
constexpr uint32_t kInterruptEnables = I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_ADDRIE | I2C_CR1_NACKIE |
                                       I2C_CR1_STOPIE | I2C_CR1_TCIE | I2C_CR1_ERRIE;
// A 4-byte transaction at 400 kHz takes ~120 µs; give up after ~2 ms of polls at 20 kHz.
constexpr uint16_t kMaxPolls = 40;

uint32_t transferBits(uint8_t address, uint8_t length) {
  return (static_cast<uint32_t>(address) << 1) | (static_cast<uint32_t>(length) << I2C_CR2_NBYTES_Pos);
}
}  // namespace

void I2cPoller::attach() {
  i2c_->CR1 &= ~kInterruptEnables;
  i2c_->ICR = kClearAll;
  phase_ = Phase::Idle;
}

void I2cPoller::start(uint8_t address, const uint8_t* tx, uint8_t txLength, uint8_t rxLength) {
  address_ = address;
  txLength_ = txLength > sizeof tx_ ? sizeof tx_ : txLength;
  rxLength_ = rxLength > sizeof rx_ ? sizeof rx_ : rxLength;
  for (uint8_t i = 0; i < txLength_; ++i) tx_[i] = tx[i];
  txIndex_ = rxIndex_ = 0;
  polls_ = 0;
  i2c_->ICR = kClearAll;
  if (txLength_ > 0) {
    // Write phase; AUTOEND only when nothing is read afterwards (else restart on TC).
    i2c_->CR2 = transferBits(address_, txLength_) | (rxLength_ ? 0u : I2C_CR2_AUTOEND) | I2C_CR2_START;
    phase_ = Phase::Write;
  } else {
    i2c_->CR2 = transferBits(address_, rxLength_) | I2C_CR2_RD_WRN | I2C_CR2_AUTOEND | I2C_CR2_START;
    phase_ = Phase::Read;
  }
}

void I2cPoller::resetPeripheral() {
  // PE low for >= 3 APB cycles releases the lines and clears the state machine.
  i2c_->CR1 &= ~I2C_CR1_PE;
  (void)i2c_->CR1;
  (void)i2c_->CR1;
  (void)i2c_->CR1;
  i2c_->CR1 |= I2C_CR1_PE;
  i2c_->ICR = kClearAll;
}

void I2cPoller::fail() {
  const uint32_t isr = i2c_->ISR;
  if ((isr & (I2C_ISR_BERR | I2C_ISR_ARLO)) || polls_ > kMaxPolls) {
    resetPeripheral();
    phase_ = Phase::Idle;
    return;
  }
  // NACK and friends: let the STOP condition finish before the next start().
  if ((isr & I2C_ISR_BUSY) && !(isr & I2C_ISR_STOPF) && !(i2c_->CR2 & I2C_CR2_AUTOEND)) i2c_->CR2 |= I2C_CR2_STOP;
  i2c_->ICR = kClearAll & ~I2C_ICR_STOPCF;
  phase_ = Phase::Recover;
  polls_ = 0;
}

bool I2cPoller::ready() {
  if (phase_ == Phase::Recover) {
    if (i2c_->ISR & I2C_ISR_BUSY) {
      if (++polls_ > kMaxPolls) {
        resetPeripheral();
        phase_ = Phase::Idle;
      }
      return false;
    }
    i2c_->ICR = kClearAll;
    phase_ = Phase::Idle;
  }
  return phase_ == Phase::Idle && !(i2c_->ISR & I2C_ISR_BUSY);
}

I2cPoller::Result I2cPoller::poll() {
  if (phase_ == Phase::Idle || phase_ == Phase::Recover) return Result::Idle;

  uint32_t isr = i2c_->ISR;
  if ((isr & kErrorFlags) || ++polls_ > kMaxPolls) {
    fail();
    return Result::Error;
  }

  if (phase_ == Phase::Write) {
    while ((isr & I2C_ISR_TXIS) && txIndex_ < txLength_) {
      i2c_->TXDR = tx_[txIndex_++];
      isr = i2c_->ISR;
    }
    if (txIndex_ < txLength_) return Result::Busy;
    if (rxLength_ == 0) {
      if (!(isr & I2C_ISR_STOPF)) return Result::Busy;
      i2c_->ICR = I2C_ICR_STOPCF;
      phase_ = Phase::Idle;
      return Result::Done;
    }
    if (!(isr & I2C_ISR_TC)) return Result::Busy;
    i2c_->CR2 = transferBits(address_, rxLength_) | I2C_CR2_RD_WRN | I2C_CR2_AUTOEND | I2C_CR2_START;  // restart
    phase_ = Phase::Read;
    return Result::Busy;
  }

  // Read phase
  while ((isr & I2C_ISR_RXNE) && rxIndex_ < rxLength_) {
    rx_[rxIndex_++] = static_cast<uint8_t>(i2c_->RXDR);
    isr = i2c_->ISR;
  }
  if (!(isr & I2C_ISR_STOPF)) return Result::Busy;
  i2c_->ICR = I2C_ICR_STOPCF;
  phase_ = Phase::Idle;
  return rxIndex_ == rxLength_ ? Result::Done : Result::Error;
}
