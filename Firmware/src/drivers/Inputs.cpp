#include "drivers/Inputs.h"

#include <Arduino.h>

#include "board/Board.h"

static_assert(board::kLimitMinPin == PB12 && board::kLimitMaxPin == PB13, "limit switch masks below assume PB12/PB13");
static_assert(board::kExtStepPin == PA1 && board::kExtDirPin == PA2 && board::kExtEnablePin == PA0,
              "step/dir masks below assume PA1/PA2/PA0");
static_assert(board::kDriverDiagPin == PA4, "DIAG mask below assumes PA4");

namespace inputs {
namespace {

volatile int32_t g_stepCount = 0;
bool g_limitActiveLow = true;
bool g_stepDirAttached = false;

void onStepEdge() {
  // DIR high = positive, sampled at the same instant as the step edge.
  g_stepCount = g_stepCount + ((GPIOA->IDR & GPIO_PIN_2) ? 1 : -1);
}

bool level(GPIO_TypeDef* port, uint32_t mask) { return (port->IDR & mask) != 0; }

}  // namespace

void begin() {
  pinMode(board::kExtDirPin, INPUT_PULLDOWN);
  pinMode(board::kExtEnablePin, INPUT_PULLUP);
  pinMode(board::kDriverDiagPin, INPUT_PULLDOWN);
  setLimitPolarity(true);
}

void setLimitPolarity(bool activeLow) {
  g_limitActiveLow = activeLow;
  const auto mode = activeLow ? INPUT_PULLUP : INPUT_PULLDOWN;
  pinMode(board::kLimitMinPin, mode);
  pinMode(board::kLimitMaxPin, mode);
}

bool limitMin() { return level(GPIOB, GPIO_PIN_12) != g_limitActiveLow; }
bool limitMax() { return level(GPIOB, GPIO_PIN_13) != g_limitActiveLow; }

void setStepDirEnabled(bool enabled) {
  if (enabled == g_stepDirAttached) return;
  g_stepDirAttached = enabled;
  if (enabled) {
    pinMode(board::kExtStepPin, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(board::kExtStepPin), onStepEdge, RISING);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);  // above the control tick: never miss an edge
  } else {
    detachInterrupt(digitalPinToInterrupt(board::kExtStepPin));
  }
}

int32_t stepDirCount() { return g_stepCount; }

bool stepDirEnable(bool activeLow) { return level(GPIOA, GPIO_PIN_0) != activeLow; }

bool driverDiag() { return level(GPIOA, GPIO_PIN_4); }

}  // namespace inputs
