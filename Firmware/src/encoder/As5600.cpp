#include "encoder/As5600.h"

namespace {
constexpr uint8_t kAddress = 0x36;
constexpr uint8_t kRegConf = 0x07;      // CONF high byte; 0x08 low byte
constexpr uint8_t kRegStatus = 0x0B;
constexpr uint8_t kRegRawAngle = 0x0C;  // RAW ANGLE high byte
constexpr uint8_t kStatusMagnetDetected = 1u << 5;
constexpr uint8_t kStatusTooWeak = 1u << 4;
constexpr uint8_t kStatusTooStrong = 1u << 3;
constexpr uint32_t kClockHz = 400000;
constexpr uint16_t kRecoverAfterErrors = 200;  // ~10 ms of consecutive failures

// CONF: normal power mode, no hysteresis, watchdog off; slow filter 16x (lowest
// noise at rest) with the fast filter taking over beyond 6 LSB, so the output
// follows motion with ~0.3 ms instead of 2.2 ms of lag.
constexpr uint8_t kConfHighMask = 0x3F;  // WD, FTH[2:0], SF[1:0]
constexpr uint8_t kConfHigh = (0b001u << 2) | 0b00u;
constexpr uint8_t kConfLowMask = 0x0F;   // HYST[1:0], PM[1:0]
}  // namespace

bool As5600::readRegisters(uint8_t reg, uint8_t* data, uint8_t length) {
  wire_.beginTransmission(kAddress);
  wire_.write(reg);
  if (wire_.endTransmission(false) != 0) return false;
  if (wire_.requestFrom(kAddress, length) != length) return false;
  for (uint8_t i = 0; i < length; ++i) data[i] = static_cast<uint8_t>(wire_.read());
  return true;
}

bool As5600::writeRegisters(uint8_t reg, const uint8_t* data, uint8_t length) {
  wire_.beginTransmission(kAddress);
  wire_.write(reg);
  if (length) wire_.write(data, length);
  return wire_.endTransmission() == 0;
}

bool As5600::begin() {
  started_ = true;
  running_ = false;
  wire_.begin();  // also clocks out a slave holding SDA low
  wire_.setClock(kClockHz);

  uint8_t status = 0;
  if (!readRegisters(kRegStatus, &status, 1)) return false;
  status_ = status;

  // Read-modify-write: unused CONF bits hold factory settings.
  uint8_t conf[2];
  if (!readRegisters(kRegConf, conf, 2)) return false;
  conf[0] = static_cast<uint8_t>((conf[0] & ~kConfHighMask) | kConfHigh);
  conf[1] = static_cast<uint8_t>(conf[1] & ~kConfLowMask);
  uint8_t check[2];
  if (!writeRegisters(kRegConf, conf, 2) || !readRegisters(kRegConf, check, 2)) return false;
  if ((check[0] & kConfHighMask) != kConfHigh || (check[1] & kConfLowMask) != 0) return false;

  // Park the pointer on RAW ANGLE, then hand the peripheral to the poller.
  if (!writeRegisters(kRegRawAngle, nullptr, 0)) return false;
  bus_.attach();
  step_ = Step::Angle;
  errorStreak_ = 0;
  running_ = true;
  return true;
}

void As5600::end() {
  started_ = false;
  running_ = false;
  wire_.end();
}

bool As5600::poll(uint16_t& raw) {
  if (!running_) return false;

  const I2cPoller::Result result = bus_.poll();
  if (result == I2cPoller::Result::Busy) return false;

  bool fresh = false;
  if (result == I2cPoller::Result::Done) {
    const uint8_t* rx = bus_.rx();
    if (step_ == Step::Angle) {
      raw = static_cast<uint16_t>(((rx[0] & 0x0Fu) << 8) | rx[1]);
      fresh = true;
    } else if (step_ == Step::Status) {
      status_ = rx[0];
    }
    errorStreak_ = 0;
  } else if (result == I2cPoller::Result::Error) {
    errorStreak_ = static_cast<uint16_t>(errorStreak_ + 1);
    pointerLost_ = true;  // the pointer may have moved; put it back before trusting reads
  }

  // Launch the next transaction right away to keep the bus busy. After an
  // error the bus may still be finishing its STOP: retry on a later tick.
  if (!bus_.ready()) return fresh;
  if (pointerLost_ || step_ == Step::Status) {
    pointerLost_ = false;
    step_ = Step::RestorePointer;
    static constexpr uint8_t kPointer = kRegRawAngle;
    bus_.start(kAddress, &kPointer, 1, 0);
  } else if (statusRequested_) {
    statusRequested_ = false;
    step_ = Step::Status;
    static constexpr uint8_t kStatus = kRegStatus;
    bus_.start(kAddress, &kStatus, 1, 1);
  } else {
    step_ = Step::Angle;
    bus_.start(kAddress, nullptr, 0, 2);
  }
  return fresh;
}

void As5600::service() {
  if (!started_) return;
  // Wedged bus: re-initialise now. Sensor that failed to start: retry every ~1 s.
  const bool wedged = running_ && errorStreak_ > kRecoverAfterErrors;
  const bool retryDue = !running_ && ++retryCount_ >= 100;
  if (wedged || retryDue) {
    retryCount_ = 0;
    running_ = false;
    wire_.end();
    (void)begin();  // TwoWire::begin() includes bus recovery
    return;
  }
  if (!running_) return;
  if (++serviceCount_ >= 10) {  // ~10 Hz at a 100 Hz service rate
    serviceCount_ = 0;
    statusRequested_ = true;
  }
}

MagnetStatus As5600::magnet() const {
  const uint8_t s = status_;
  return {(s & kStatusMagnetDetected) != 0, (s & kStatusTooWeak) != 0, (s & kStatusTooStrong) != 0};
}
