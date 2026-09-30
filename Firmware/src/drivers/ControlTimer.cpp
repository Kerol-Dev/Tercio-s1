#include "drivers/ControlTimer.h"

#include <Arduino.h>

namespace control_timer {
namespace {
Callback g_callback = nullptr;
uint32_t g_periodCycles = 1;
volatile uint32_t g_peakCycles = 0;
}  // namespace

void begin(Callback callback, uint32_t priority) {
  g_callback = callback;
  g_periodCycles = SystemCoreClock / kRateHz;

  // Cycle counter for load measurement.
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // TIM7 runs from the APB1 timer clock (x2 when APB1 is divided).
  __HAL_RCC_TIM7_CLK_ENABLE();
  const uint32_t clock = HAL_RCC_GetPCLK1Freq() * ((RCC->CFGR & RCC_CFGR_PPRE1_2) ? 2u : 1u);
  TIM7->CR1 = TIM_CR1_ARPE | TIM_CR1_URS;  // only overflow raises the interrupt
  TIM7->PSC = 0;
  TIM7->ARR = clock / kRateHz - 1u;
  TIM7->EGR = TIM_EGR_UG;
  TIM7->SR = 0;
  TIM7->DIER = TIM_DIER_UIE;

  HAL_NVIC_SetPriority(TIM7_IRQn, priority, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
  TIM7->CR1 |= TIM_CR1_CEN;
}

float takePeakLoad() {
  const uint32_t peak = g_peakCycles;
  g_peakCycles = 0;
  return static_cast<float>(peak) / static_cast<float>(g_periodCycles);
}

}  // namespace control_timer

extern "C" void TIM7_IRQHandler(void) {
  TIM7->SR = ~TIM_SR_UIF;
  const uint32_t start = DWT->CYCCNT;
  control_timer::g_callback();
  const uint32_t cycles = DWT->CYCCNT - start;
  if (cycles > control_timer::g_peakCycles) control_timer::g_peakCycles = cycles;
}
