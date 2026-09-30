// -----------------------------------------------------------------------------
// Tercio S1 closed-loop stepper driver — firmware entry point.
//
// Two execution contexts:
//   * TIM7 interrupt, 20 kHz: MotionCore::tick() — encoder, trajectory,
//     control law, step rate. Fixed period, independent of the main loop.
//   * main loop: CAN protocol, procedures and fault supervision (1 kHz),
//     driver UART, analog sensing, persistence. Nothing here blocks for long.
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <IWatchdog.h>

#include "app/Axis.h"
#include "app/MotionCore.h"
#include "app/Node.h"
#include "app/Params.h"
#include "app/Settings.h"
#include "board/Board.h"
#include "drivers/Analog.h"
#include "drivers/ControlTimer.h"
#include "drivers/FlashStore.h"
#include "drivers/Inputs.h"
#include "drivers/StepGenerator.h"
#include "drivers/Tmc2209.h"
#include "encoder/As5048a.h"
#include "encoder/As5600.h"

namespace {

constexpr uint32_t kControlIrqPriority = 1;   // only the external step input preempts it
constexpr uint32_t kWatchdogTimeoutUs = 500000;
constexpr uint32_t kSlowTaskPeriodMs = 10;

Settings g_settings;
FlashStore g_store;
StepGenerator g_stepper;
Uart g_tmcUart(board::kTmcRxPin, board::kTmcTxPin);
Tmc2209 g_driver(g_tmcUart);

As5600 g_as5600Internal(board::kI2cSdaPin, board::kI2cSclPin, I2C1);
As5600 g_as5600External(board::kExtI2cSdaPin, board::kExtI2cSclPin, I2C2);
As5048a g_as5048a;
Encoder* const g_encoders[] = {&g_as5600Internal, &g_as5600External, &g_as5048a};  // by proto::EncoderType

MotionCore g_core;
Axis g_axis(g_settings, {g_core, g_driver, g_encoders});
Node g_node(g_settings, g_axis, g_store);

uint32_t g_lastMs = 0;
uint32_t g_lastSlowMs = 0;

void controlTick() { g_core.tick(); }

}  // namespace

void setup() {
  // Cycle counter: used for sub-microsecond delays and control-loop load.
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  g_stepper.begin();  // first: holds the bridge off while everything else starts

  if (!g_store.load(&g_settings, sizeof g_settings, Settings::kLayoutVersion)) g_settings = Settings{};
  params::sanitize(g_settings);

  analog::begin();
  analog::update();
  inputs::begin();
  (void)g_driver.begin(Tmc2209::Config{});  // Axis applies the real configuration

  g_core.attach(nullptr, &g_stepper);
  control_timer::begin(controlTick, kControlIrqPriority);

  const uint32_t now = millis();
  g_axis.begin(now);
  (void)g_node.begin(now);

  IWatchdog.begin(kWatchdogTimeoutUs);
  g_lastMs = g_lastSlowMs = now;
}

void loop() {
  const uint32_t now = millis();
  g_node.service(now);

  if (now != g_lastMs) {
    g_lastMs = now;
    g_axis.service(now);
    g_driver.service(now);
  }

  if (now - g_lastSlowMs >= kSlowTaskPeriodMs) {
    g_lastSlowMs = now;
    analog::update();
    g_axis.serviceSlow();
  }

  IWatchdog.reload();
}
