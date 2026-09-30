#include "drivers/Analog.h"

#include <Arduino.h>

#include <cmath>

#include "board/Board.h"

namespace analog {
namespace {

constexpr float kFilterAlpha = 0.1f;  // ~100 ms time constant at 100 Hz
constexpr float kAdcFullScale = 4095.0f;
constexpr float kKelvin = 273.15f;

ADC_HandleTypeDef g_adc{};
bool g_ready = false;
bool g_primed = false;
float g_temperature = 25.0f;
float g_supply = 0.0f;

bool convert(uint32_t channel, uint32_t& value) {
  ADC_ChannelConfTypeDef config{};
  config.Channel = channel;
  config.Rank = ADC_REGULAR_RANK_1;
  config.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;  // high-impedance dividers
  config.SingleDiff = ADC_SINGLE_ENDED;
  config.OffsetNumber = ADC_OFFSET_NONE;
  if (HAL_ADC_ConfigChannel(&g_adc, &config) != HAL_OK || HAL_ADC_Start(&g_adc) != HAL_OK) return false;
  const bool ok = HAL_ADC_PollForConversion(&g_adc, 1) == HAL_OK;
  value = HAL_ADC_GetValue(&g_adc);
  return ok;
}

float ntcCelsius(float volts) {
  // Divider: 3.3 V -> pull-up -> node -> NTC -> GND.
  const float headroom = board::kAdcRefVolts - volts;
  if (headroom <= 1e-3f || volts <= 1e-3f) return volts <= 1e-3f ? 200.0f : -60.0f;  // shorted / open
  const float resistance = board::kNtcPullupOhms * volts / headroom;
  const float invT = 1.0f / (25.0f + kKelvin) + std::log(resistance / board::kNtcR25) / board::kNtcBeta;
  return 1.0f / invT - kKelvin;
}

}  // namespace

bool begin() {
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef pins{};
  pins.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  pins.Mode = GPIO_MODE_ANALOG;
  pins.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &pins);

  __HAL_RCC_ADC12_CLK_ENABLE();
  g_adc.Instance = ADC1;
  ADC_InitTypeDef& init = g_adc.Init;
  init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;  // 42.5 MHz
  init.Resolution = ADC_RESOLUTION_12B;
  init.DataAlign = ADC_DATAALIGN_RIGHT;
  init.GainCompensation = 0;
  init.ScanConvMode = ADC_SCAN_DISABLE;
  init.EOCSelection = ADC_EOC_SINGLE_CONV;
  init.LowPowerAutoWait = DISABLE;
  init.ContinuousConvMode = DISABLE;
  init.NbrOfConversion = 1;
  init.DiscontinuousConvMode = DISABLE;
  init.ExternalTrigConv = ADC_SOFTWARE_START;
  init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  init.DMAContinuousRequests = DISABLE;
  init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  init.OversamplingMode = DISABLE;
  g_ready = HAL_ADC_Init(&g_adc) == HAL_OK && HAL_ADCEx_Calibration_Start(&g_adc, ADC_SINGLE_ENDED) == HAL_OK;
  return g_ready;
}

void update() {
  if (!g_ready) return;
  uint32_t ntcRaw = 0, supplyRaw = 0;
  if (!convert(board::kThermistorAdcChannel, ntcRaw) || !convert(board::kVbusAdcChannel, supplyRaw)) return;

  const float scale = board::kAdcRefVolts / kAdcFullScale;
  const float temperature = ntcCelsius(static_cast<float>(ntcRaw) * scale);
  const float supply = static_cast<float>(supplyRaw) * scale * board::kVbusDividerRatio;
  if (!g_primed) {  // start the filters at the first reading, not at a guess
    g_temperature = temperature;
    g_supply = supply;
    g_primed = true;
  }
  g_temperature += kFilterAlpha * (temperature - g_temperature);
  g_supply += kFilterAlpha * (supply - g_supply);
}

float temperatureC() { return g_temperature; }
float supplyVolts() { return g_supply; }

}  // namespace analog
