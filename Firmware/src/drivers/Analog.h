#pragma once
// -----------------------------------------------------------------------------
// Board temperature (NTC on PB0) and motor supply voltage (divider on PB1).
// ADC1 is initialised and calibrated once; a reading is two ~2 µs conversions,
// unlike analogRead(), which re-initialises and re-calibrates the ADC each call.
// -----------------------------------------------------------------------------
#include <cstdint>

namespace analog {

bool begin();
// Samples both channels and updates the filtered values. Call at ~100 Hz.
void update();

float temperatureC();
float supplyVolts();

}  // namespace analog
