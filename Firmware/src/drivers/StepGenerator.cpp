#include "drivers/StepGenerator.h"

#include <Arduino.h>

#include <cmath>

#include "board/Board.h"
#include "drivers/StepTiming.h"

static_assert(board::kStepPin == PB14, "StepGenerator is hard-wired to TIM1_CH2N on PB14");
static_assert(board::kDirPin == PB15, "StepGenerator drives DIR on PB15 through GPIOB->BSRR");

namespace {
constexpr uint32_t kDirMask = GPIO_PIN_15;
// While stopped the timer keeps running with this short, pulse-less period so a
// new rate takes effect within a few microseconds.
constexpr uint32_t kIdlePeriodUs = 20;
// Minimum time before the current period ends for a restart to be safe, in
// timer clocks (~3 µs): leaves room for the few instructions in between.
constexpr uint64_t kRestartMargin = 512;
// STEP high time. The TMC2209 needs ~100 ns; a short fixed pulse (instead of
// 50 % duty) lets a slow period be cut short right after its pulse.
constexpr uint32_t kPulseNs = 2000;
}  // namespace

void StepGenerator::begin() {
  pinMode(board::kDriverEnablePin, OUTPUT);
  digitalWrite(board::kDriverEnablePin, HIGH);  // bridge off
  enabled_ = false;

  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef pin{};
  pin.Pin = kDirMask;
  pin.Mode = GPIO_MODE_OUTPUT_PP;
  pin.Pull = GPIO_NOPULL;
  pin.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &pin);
  GPIOB->BSRR = kDirMask;
  dirPositive_ = true;

  pin.Pin = GPIO_PIN_14;
  pin.Mode = GPIO_MODE_AF_PP;
  pin.Alternate = GPIO_AF6_TIM1;
  HAL_GPIO_Init(GPIOB, &pin);

  // TIM1 runs from the APB2 timer clock (x2 when APB2 is divided).
  __HAL_RCC_TIM1_CLK_ENABLE();
  timerClock_ = HAL_RCC_GetPCLK2Freq() * ((RCC->CFGR & RCC_CFGR_PPRE2_2) ? 2u : 1u);
  idleArr_ = timerClock_ / 1000000u * kIdlePeriodUs - 1u;
  pulseClocks_ = static_cast<uint32_t>(static_cast<uint64_t>(timerClock_) * kPulseNs / 1000000000u);

  active_ = {0, idleArr_, 0};
  pending_ = active_;

  TIM1->CR1 = TIM_CR1_ARPE;                                   // up-counting, buffered ARR
  TIM1->CR2 = 0;
  TIM1->SMCR = 0;
  TIM1->DIER = 0;
  TIM1->CCMR1 = (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;  // PWM1: active while CNT < CCR2
  TIM1->CCER = TIM_CCER_CC2NE;                                 // N output only, not inverted
  TIM1->BDTR = TIM_BDTR_MOE;
  TIM1->PSC = active_.psc;
  TIM1->ARR = active_.arr;
  TIM1->CCR2 = 0;
  TIM1->EGR = TIM_EGR_UG;  // load the shadow registers
  TIM1->SR = 0;
  TIM1->CR1 |= TIM_CR1_CEN;
}

void StepGenerator::enableDriver(bool on) {
  if (!on) stop();
  enabled_ = on;
  digitalWrite(board::kDriverEnablePin, on ? LOW : HIGH);
}

void StepGenerator::setRate(float stepsPerSecond) {
  const bool positive = stepsPerSecond >= 0.0f;
  float rate = std::fabs(stepsPerSecond);

  Timing next{0, idleArr_, 0};
  if (enabled_ && rate >= kMinRate) {
    next = step_timing::forRate(rate > kMaxRate ? kMaxRate : rate, timerClock_, pulseClocks_);

    // DIR is sampled on the rising STEP edge; the next edge belongs to the new rate.
    if (positive != dirPositive_) {
      dirPositive_ = positive;
      GPIOB->BSRR = positive ? kDirMask : (kDirMask << 16);
    }
  }

  if (next == pending_) return;
  apply(next);
}

void StepGenerator::apply(const Timing& next) {
  // Called from the control interrupt; the critical section also makes it safe
  // from thread context (enableDriver/stop).
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  // Freeze update events first, so the bookkeeping below cannot go stale:
  // any update event before this point means the preload values went live.
  TIM1->CR1 |= TIM_CR1_UDIS;
  if (TIM1->SR & TIM_SR_UIF) {
    active_ = pending_;
    TIM1->SR = ~TIM_SR_UIF;
  }

  // Speeding up from a slow rate: start the next period now if its pulse is overdue.
  const bool restart = step_timing::shouldRestart(active_, next, TIM1->CNT, kRestartMargin);

  // UDIS (still set) keeps the three registers from going live half-written.
  TIM1->PSC = next.psc;
  TIM1->ARR = next.arr;
  TIM1->CCR2 = next.ccr;
  TIM1->CR1 &= ~TIM_CR1_UDIS;
  pending_ = next;

  if (restart) {
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = ~TIM_SR_UIF;
    active_ = next;
  }

  __set_PRIMASK(primask);
}
