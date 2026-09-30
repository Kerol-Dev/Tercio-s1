#pragma once
// -----------------------------------------------------------------------------
// TMC2209 over its single-wire UART.
//
// After begin() nothing blocks: register writes are queued and sent from
// service() whenever the bus is idle, and status registers are read in the
// background. The driver heals itself — if the chip reports a reset (e.g. a
// motor-supply dip) or a read-back register differs, the whole configuration
// is written again.
// -----------------------------------------------------------------------------
#include <cstdint>

class Uart;

class Tmc2209 {
 public:
  struct Config {
    uint16_t microsteps = 16;
    uint16_t runCurrentMa = 1000;
    uint8_t holdCurrentPct = 50;
    bool stealthChop = true;
    float stealthChopMaxFullStepsPerSec = 200.0f;  // above: SpreadCycle; <= 0: StealthChop always
  };

  struct Status {
    bool commOk = false;
    bool overTemp = false;      // ot: bridge shut down
    bool preWarning = false;    // otpw
    bool shortCircuit = false;  // s2g / s2vs on either coil
    bool openLoad = false;
    uint16_t appliedCurrentMa = 0;
  };

  explicit Tmc2209(Uart& uart) : uart_(uart) {}

  // Blocking: checks communication and writes the configuration. False if the chip does not answer.
  bool begin(const Config& config);
  // Queues the registers that changed; sent on the next service() calls.
  void apply(const Config& config);
  void service(uint32_t nowMs);
  const Status& status() const { return status_; }

  // CS register value (0..31) and vsense bit that best approximate `mA` RMS.
  static void currentToRegisters(uint16_t mA, uint8_t& cs, bool& vsense);
  static uint16_t registersToCurrent(uint8_t cs, bool vsense);

 private:
  enum Reg : uint8_t {
    kGconf = 0x00, kGstat = 0x01, kIoin = 0x06, kIholdIrun = 0x10, kTpowerdown = 0x11,
    kTpwmthrs = 0x13, kChopconf = 0x6C, kDrvStatus = 0x6F, kPwmconf = 0x70,
  };
  enum Shadow : uint8_t { kShGconf, kShChopconf, kShPwmconf, kShIholdIrun, kShTpowerdown, kShTpwmthrs, kShadowCount };

  void computeShadows(const Config& config);
  void sendWrite(uint8_t reg, uint32_t value);
  void sendReadRequest(uint8_t reg);
  bool readBlocking(uint8_t reg, uint32_t& value);
  bool pollReply(uint8_t reg, uint32_t& value);  // parses a complete reply if one arrived
  void handleRead(uint8_t reg, uint32_t value);
  void flushRx();

  Uart& uart_;
  uint32_t shadow_[kShadowCount] = {};
  uint8_t dirty_ = 0;  // bit per Shadow
  Status status_{};

  // Background read state.
  uint8_t rx_[24] = {};
  uint8_t rxLen_ = 0;
  uint8_t pendingReg_ = 0xFF;  // 0xFF: no read in flight
  uint32_t requestMs_ = 0;
  uint32_t nextPollMs_ = 0;
  uint8_t pollIndex_ = 0;
  uint8_t failures_ = 0;
};
