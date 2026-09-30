#include "encoder/As5048a.h"

#include <Arduino.h>

#include "board/Board.h"

static_assert(board::kSpiSckPin == PA5 && board::kSpiMisoPin == PA6 && board::kSpiMosiPin == PA7 &&
                  board::kSpiCsPin == PB2,
              "As5048a drives SPI1 on PA5/PA6/PA7 with CS on PB2");

namespace {
// Command words (bit 15 even parity, bit 14 read).
constexpr uint16_t kReadAngle = 0xFFFF;        // 0x3FFF
constexpr uint16_t kReadDiagnostics = 0x7FFD;  // 0x3FFD: AGC + flags
constexpr uint16_t kClearError = 0x4001;       // 0x0001
constexpr uint16_t kNop = 0x0000;
constexpr uint16_t kErrorFlag = 1u << 14;
constexpr uint16_t kDiagOcf = 1u << 8;        // offset compensation finished (normal)
constexpr uint16_t kDiagCof = 1u << 9;        // CORDIC overflow: angle invalid
constexpr uint16_t kDiagCompLow = 1u << 10;   // field too strong
constexpr uint16_t kDiagCompHigh = 1u << 11;  // field too weak
constexpr uint32_t kCsMask = GPIO_PIN_2;

bool parityOk(uint16_t word) { return (__builtin_popcount(word) & 1) == 0; }

void delayCycles(uint32_t cycles) {
  const uint32_t start = DWT->CYCCNT;
  while (DWT->CYCCNT - start < cycles) {
  }
}
}  // namespace

bool As5048a::begin() {
  running_ = false;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef pin{};
  pin.Pin = kCsMask;
  pin.Mode = GPIO_MODE_OUTPUT_PP;
  pin.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &pin);
  GPIOB->BSRR = kCsMask;  // deselect

  pin.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  pin.Mode = GPIO_MODE_AF_PP;
  pin.Pull = GPIO_NOPULL;
  pin.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &pin);

  // Master, mode 1 (CPOL 0, CPHA 1), PCLK2/32 = 5.3 MHz (the chip allows 10 MHz,
  // PCLK2/16 would be 10.6), software NSS, 8-bit frames.
  __HAL_RCC_SPI1_CLK_ENABLE();
  SPI1->CR1 = 0;
  SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_CPHA | (0b100u << SPI_CR1_BR_Pos) | SPI_CR1_SSM | SPI_CR1_SSI;
  SPI1->CR2 = (7u << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH;
  SPI1->CR1 |= SPI_CR1_SPE;

  // Presence check: diagnostics must parse and report a finished offset compensation.
  transfer(kClearError);
  transfer(kReadDiagnostics);
  const uint16_t diagnostics = transfer(kNop);
  if (!parityOk(diagnostics) || (diagnostics & kErrorFlag) || !(diagnostics & kDiagOcf)) return false;
  diagnostics_ = diagnostics;

  lastCommand_ = kNop;
  nextCommand_ = kReadAngle;
  running_ = true;
  return true;
}

void As5048a::end() {
  running_ = false;
  SPI1->CR1 &= ~SPI_CR1_SPE;
}

uint16_t As5048a::transfer(uint16_t command) {
  GPIOB->BSRR = kCsMask << 16;  // select
  delayCycles(60);              // tL: >= 350 ns from CSn low to the first clock
  volatile uint8_t* dr = reinterpret_cast<volatile uint8_t*>(&SPI1->DR);
  *dr = static_cast<uint8_t>(command >> 8);
  *dr = static_cast<uint8_t>(command);
  while (!(SPI1->SR & SPI_SR_RXNE)) {
  }
  const uint8_t high = *dr;
  while (!(SPI1->SR & SPI_SR_RXNE)) {
  }
  const uint8_t low = *dr;
  while (SPI1->SR & SPI_SR_BSY) {
  }
  GPIOB->BSRR = kCsMask;  // deselect
  delayCycles(60);        // tCSn: >= 350 ns high before the next frame
  return static_cast<uint16_t>((high << 8) | low);
}

bool As5048a::poll(uint16_t& raw) {
  if (!running_) return false;

  const uint16_t answeredCommand = lastCommand_;
  const uint16_t response = transfer(nextCommand_);
  lastCommand_ = nextCommand_;

  const bool valid = parityOk(response) && !(response & kErrorFlag);
  if (!valid) {
    nextCommand_ = kClearError;  // the error flag stays set until read
    return false;
  }
  nextCommand_ = diagnosticsRequested_ ? kReadDiagnostics : kReadAngle;
  diagnosticsRequested_ = false;

  if (answeredCommand == kReadDiagnostics) diagnostics_ = response;
  if (answeredCommand != kReadAngle || (diagnostics_ & kDiagCof)) return false;
  raw = response & 0x3FFFu;
  return true;
}

void As5048a::service() {
  if (running_ && ++serviceCount_ >= 10) {  // ~10 Hz at a 100 Hz service rate
    serviceCount_ = 0;
    diagnosticsRequested_ = true;
  }
}

MagnetStatus As5048a::magnet() const {
  const uint16_t d = diagnostics_;
  return {(d & kDiagOcf) && !(d & kDiagCof), (d & kDiagCompHigh) != 0, (d & kDiagCompLow) != 0};
}
