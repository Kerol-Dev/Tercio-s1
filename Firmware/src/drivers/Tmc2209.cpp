#include "drivers/Tmc2209.h"

#include <Arduino.h>

#include <cmath>

#include "board/Board.h"

namespace {

constexpr uint8_t kSync = 0x05;
constexpr uint8_t kMasterAddress = 0xFF;
constexpr uint32_t kReplyTimeoutMs = 5;  // request + reply take ~1.1 ms at 115200 baud
constexpr uint32_t kPollPeriodMs = 10;
constexpr uint8_t kFailuresBeforeCommLoss = 3;
constexpr uint8_t kVersion = 0x21;

// GCONF
constexpr uint32_t kIScaleAnalog = 1u << 0;
constexpr uint32_t kEnSpreadCycle = 1u << 2;
constexpr uint32_t kPdnDisable = 1u << 6;
constexpr uint32_t kMstepRegSelect = 1u << 7;
constexpr uint32_t kMultistepFilt = 1u << 8;
// CHOPCONF: TOFF=4, HSTRT=5, HEND=0, TBL=16 clocks, interpolation to 256 µsteps —
// the chopper setup firmware v1 was validated with on this board.
constexpr uint32_t kChopconfBase = (1u << 28) | (5u << 4) | 4u;
constexpr uint32_t kVsense = 1u << 17;
constexpr uint32_t kMresShift = 24;
// PWMCONF: datasheet reset value with PWM_GRAD = 1 (autoscale + autograd on).
constexpr uint32_t kPwmconf = 0xC10D0124u;
constexpr uint32_t kIholdDelay = 1;
constexpr uint32_t kTpowerdown = 20;  // ~0.44 s at standstill before dropping to hold current
// GSTAT
constexpr uint32_t kGstatReset = 1u << 0;
constexpr uint32_t kGstatDrvErr = 1u << 1;
// DRV_STATUS
constexpr uint32_t kOtpw = 1u << 0, kOt = 1u << 1, kShorts = 0x3Cu, kOpenLoad = 0xC0u;

constexpr float kRsense = board::kTmcSenseOhms + 0.02f;  // datasheet adds 20 mΩ of internal resistance
constexpr float kSqrt2 = 1.41421356f;

uint8_t crc8(const uint8_t* data, uint8_t length) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < length; ++i) {
    uint8_t byte = data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = ((crc >> 7) ^ (byte & 1u)) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
      byte >>= 1;
    }
  }
  return crc;
}

uint8_t microstepsToMres(uint16_t microsteps) {
  uint8_t log2 = 0;
  while ((1u << (log2 + 1)) <= microsteps && log2 < 8) ++log2;
  return static_cast<uint8_t>(8 - log2);  // 256 -> 0 ... 1 (full step) -> 8
}

}  // namespace

void Tmc2209::currentToRegisters(uint16_t mA, uint8_t& cs, bool& vsense) {
  const float amps = static_cast<float>(mA) * 0.001f;
  float scale = 32.0f * kSqrt2 * amps * kRsense / 0.325f - 1.0f;
  vsense = scale < 15.5f;  // low currents: use the sensitive range for resolution
  if (vsense) scale = 32.0f * kSqrt2 * amps * kRsense / 0.180f - 1.0f;
  const float rounded = std::round(scale);
  cs = static_cast<uint8_t>(rounded < 0.0f ? 0.0f : (rounded > 31.0f ? 31.0f : rounded));
}

uint16_t Tmc2209::registersToCurrent(uint8_t cs, bool vsense) {
  const float vfs = vsense ? 0.180f : 0.325f;
  return static_cast<uint16_t>((static_cast<float>(cs) + 1.0f) / 32.0f * vfs / kRsense / kSqrt2 * 1000.0f + 0.5f);
}

void Tmc2209::computeShadows(const Config& c) {
  uint8_t irun;
  bool vsense;
  currentToRegisters(c.runCurrentMa, irun, vsense);
  const uint32_t ihold = static_cast<uint32_t>(std::lround((irun + 1) * c.holdCurrentPct / 100.0f)) ;
  status_.appliedCurrentMa = registersToCurrent(irun, vsense);

  uint32_t tpwmthrs = 0;  // 0: StealthChop at every speed
  if (c.stealthChopMaxFullStepsPerSec > 0.0f) {
    // TSTEP counts 12 MHz clocks per 1/256 microstep, whatever MRES is.
    const float tstep = 12e6f / (c.stealthChopMaxFullStepsPerSec * 256.0f);
    tpwmthrs = tstep >= 0xFFFFF ? 0xFFFFFu : static_cast<uint32_t>(tstep);
  }

  const uint32_t values[kShadowCount] = {
      (board::kTmcCurrentRefFromVref ? kIScaleAnalog : 0u) | kPdnDisable | kMstepRegSelect | kMultistepFilt |
          (c.stealthChop ? 0u : kEnSpreadCycle),
      kChopconfBase | (vsense ? kVsense : 0u) | (static_cast<uint32_t>(microstepsToMres(c.microsteps)) << kMresShift),
      kPwmconf,
      (kIholdDelay << 16) | (static_cast<uint32_t>(irun) << 8) | (ihold > 0 ? ihold - 1 : 0),
      kTpowerdown,
      tpwmthrs,
  };
  for (uint8_t i = 0; i < kShadowCount; ++i) {
    if (shadow_[i] != values[i]) dirty_ |= static_cast<uint8_t>(1u << i);
    shadow_[i] = values[i];
  }
}

bool Tmc2209::begin(const Config& config) {
  uart_.begin(board::kTmcBaud);
  delay(2);

  uint32_t ioin = 0;
  status_.commOk = readBlocking(kIoin, ioin) && (ioin >> 24) == kVersion;
  if (!status_.commOk) return false;

  sendWrite(kGstat, 0x7);  // clear reset / drv_err / uv_cp
  computeShadows(config);
  dirty_ = (1u << kShadowCount) - 1u;  // write everything once
  static constexpr uint8_t kRegs[kShadowCount] = {kGconf, kChopconf, kPwmconf, kIholdIrun, kTpowerdown, kTpwmthrs};
  for (uint8_t i = 0; i < kShadowCount; ++i) sendWrite(kRegs[i], shadow_[i]);
  dirty_ = 0;
  uart_.flush();

  uint32_t chopconf = 0;
  status_.commOk = readBlocking(kChopconf, chopconf) && chopconf == shadow_[kShChopconf];
  return status_.commOk;
}

void Tmc2209::apply(const Config& config) { computeShadows(config); }

void Tmc2209::sendWrite(uint8_t reg, uint32_t value) {
  uint8_t d[8] = {kSync, board::kTmcAddress, static_cast<uint8_t>(reg | 0x80), static_cast<uint8_t>(value >> 24),
                  static_cast<uint8_t>(value >> 16), static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value), 0};
  d[7] = crc8(d, 7);
  uart_.write(d, sizeof d);
}

void Tmc2209::sendReadRequest(uint8_t reg) {
  flushRx();
  uint8_t d[4] = {kSync, board::kTmcAddress, reg, 0};
  d[3] = crc8(d, 3);
  uart_.write(d, sizeof d);
}

void Tmc2209::flushRx() {
  while (uart_.available()) (void)uart_.read();
  rxLen_ = 0;
}

// Replies look like [05 FF reg d3 d2 d1 d0 crc]. Scanning for that header skips
// our own echoed request, whose second byte is the slave address instead.
bool Tmc2209::pollReply(uint8_t reg, uint32_t& value) {
  while (uart_.available() && rxLen_ < sizeof rx_) rx_[rxLen_++] = static_cast<uint8_t>(uart_.read());
  for (uint8_t i = 0; i + 8 <= rxLen_; ++i) {
    const uint8_t* r = rx_ + i;
    if (r[0] != kSync || r[1] != kMasterAddress || r[2] != reg) continue;
    if (crc8(r, 7) != r[7]) return false;
    value = (static_cast<uint32_t>(r[3]) << 24) | (static_cast<uint32_t>(r[4]) << 16) |
            (static_cast<uint32_t>(r[5]) << 8) | r[6];
    rxLen_ = 0;
    return true;
  }
  return false;
}

bool Tmc2209::readBlocking(uint8_t reg, uint32_t& value) {
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    uart_.flush();
    sendReadRequest(reg);
    const uint32_t start = millis();
    while (millis() - start < kReplyTimeoutMs)
      if (pollReply(reg, value)) return true;
  }
  return false;
}

void Tmc2209::handleRead(uint8_t reg, uint32_t value) {
  switch (reg) {
    case kDrvStatus:
      status_.overTemp = value & kOt;
      status_.preWarning = value & kOtpw;
      status_.shortCircuit = value & kShorts;
      status_.openLoad = value & kOpenLoad;
      break;
    case kGstat:
      if (value & (kGstatReset | kGstatDrvErr)) {
        // Chip reset (supply dip) or a latched shutdown: clear and rewrite everything.
        sendWrite(kGstat, value);
        dirty_ = (1u << kShadowCount) - 1u;
      }
      break;
    case kChopconf:
      if (value != shadow_[kShChopconf]) dirty_ = (1u << kShadowCount) - 1u;
      break;
    default:
      break;
  }
}

void Tmc2209::service(uint32_t nowMs) {
  // A read is in flight: collect the reply or give up after the timeout.
  if (pendingReg_ != 0xFF) {
    uint32_t value;
    if (pollReply(pendingReg_, value)) {
      failures_ = 0;
      status_.commOk = true;
      handleRead(pendingReg_, value);
      pendingReg_ = 0xFF;
    } else if (nowMs - requestMs_ > kReplyTimeoutMs) {
      if (++failures_ >= kFailuresBeforeCommLoss) status_.commOk = false;
      pendingReg_ = 0xFF;
    } else {
      return;  // bus busy until the reply arrives
    }
  }

  // Bus idle: send queued register writes first, then leave the line alone
  // until they have been shifted out (10 bits per byte).
  if (dirty_) {
    static constexpr uint8_t kRegs[kShadowCount] = {kGconf, kChopconf, kPwmconf, kIholdIrun, kTpowerdown, kTpwmthrs};
    uint32_t bytes = 0;
    for (uint8_t i = 0; i < kShadowCount; ++i) {
      if (dirty_ & (1u << i)) {
        sendWrite(kRegs[i], shadow_[i]);
        bytes += 8;
      }
    }
    dirty_ = 0;
    nextPollMs_ = nowMs + 1 + bytes * 10000u / board::kTmcBaud;
    return;
  }

  if (static_cast<int32_t>(nowMs - nextPollMs_) < 0) return;
  nextPollMs_ = nowMs + kPollPeriodMs;
  static constexpr uint8_t kRotation[] = {kDrvStatus, kGstat, kDrvStatus, kChopconf};
  pendingReg_ = kRotation[pollIndex_];
  pollIndex_ = static_cast<uint8_t>((pollIndex_ + 1) % sizeof kRotation);
  sendReadRequest(pendingReg_);
  requestMs_ = nowMs;
}
