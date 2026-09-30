#pragma once
// -----------------------------------------------------------------------------
// Tercio S1 hardware description (STM32G431CBT6, LQFP48, 170 MHz).
//
// Everything that depends on the PCB lives here: pin assignments, sense
// resistor and divider values, clock source. Source: schematic Rev V1.0.
// -----------------------------------------------------------------------------
#include <Arduino.h>

namespace board {

// ---- Step/dir interface to the TMC2209 ---------------------------------------
// STEP is driven by TIM1 in hardware (PB14 = TIM1_CH2N, AF6); no CPU per step.
inline constexpr uint32_t kStepPin = PB14;         // STEP_DRV
inline constexpr uint32_t kDirPin = PB15;          // DIR_DRV
inline constexpr uint32_t kDriverEnablePin = PB5;  // EN_DRV -> ENN, active low (R24 pulls it low: enabled)
inline constexpr uint32_t kDriverDiagPin = PA4;    // DIAG: high on driver error (short, overtemperature)
inline constexpr uint32_t kDriverIndexPin = PA3;   // INDEX: unused

// ---- TMC2209 UART -------------------------------------------------------------
// USART1, TX and RX joined on PDN_UART through a resistor: transmitted bytes echo.
inline constexpr uint32_t kTmcRxPin = PA10;
inline constexpr uint32_t kTmcTxPin = PB6;
inline constexpr uint32_t kTmcBaud = 115200;
inline constexpr uint8_t kTmcAddress = 0;  // MS1 = MS2 = 0
inline constexpr float kTmcSenseOhms = 0.11f;      // R26/R27
// GCONF.I_scale_analog. VREF is tied to 3.3 V (R30), and any VREF of 2.5-5 V
// equals the internal reference, so the internal one is used: same current as
// firmware v1, but independent of the 3.3 V rail.
inline constexpr bool kTmcCurrentRefFromVref = false;
inline constexpr uint16_t kMaxRunCurrentMa = 2000;

// ---- Encoders -----------------------------------------------------------------
inline constexpr uint32_t kI2cSdaPin = PB7;      // on-board AS5600 (I2C1)
inline constexpr uint32_t kI2cSclPin = PA15;
inline constexpr uint32_t kExtI2cSdaPin = PA8;   // external header (I2C2)
inline constexpr uint32_t kExtI2cSclPin = PA9;
inline constexpr uint32_t kSpiSckPin = PA5;      // external AS5048A (SPI1)
inline constexpr uint32_t kSpiMisoPin = PA6;
inline constexpr uint32_t kSpiMosiPin = PA7;
inline constexpr uint32_t kSpiCsPin = PB2;

// ---- CAN-FD (FDCAN1, AF9) -----------------------------------------------------
inline constexpr uint32_t kCanRxPin = PA11;
inline constexpr uint32_t kCanTxPin = PA12;

// ---- Digital inputs -----------------------------------------------------------
inline constexpr uint32_t kLimitMinPin = PB12;  // J7 IN1
inline constexpr uint32_t kLimitMaxPin = PB13;  // J7 IN2
inline constexpr uint32_t kExtEnablePin = PA0;  // J5 EN
inline constexpr uint32_t kExtStepPin = PA1;    // J5 STEP
inline constexpr uint32_t kExtDirPin = PA2;     // J5 DIR

// ---- Analog -------------------------------------------------------------------
inline constexpr uint32_t kThermistorAdcChannel = ADC_CHANNEL_15;  // PB0 = ADC1_IN15
inline constexpr uint32_t kVbusAdcChannel = ADC_CHANNEL_12;        // PB1 = ADC1_IN12
inline constexpr float kAdcRefVolts = 3.3f;
inline constexpr float kVbusDividerRatio = (91000.0f + 10000.0f) / 10000.0f;
inline constexpr float kNtcR25 = 10000.0f;     // NTC nominal resistance at 25 °C
inline constexpr float kNtcBeta = 3435.0f;
inline constexpr float kNtcPullupOhms = 10000.0f;

// ---- Clock ----------------------------------------------------------------------
// Crystal Y1: 16 MHz on PF0/PF1 (HSE_VALUE in platformio.ini). Its frequency is
// checked against HSI at boot; if it does not start or does not match, the
// core runs from HSI16 instead (CAN-FD then relies on HSI accuracy, about
// ±1 % over temperature).
inline constexpr uint32_t kHseHz = HSE_VALUE;
static_assert(kHseHz == 16000000, "Tercio S1 uses a 16 MHz crystal");
extern bool g_runningOnCrystal;  // set by SystemClock_Config()

// ---- Hardware revision reported over CAN ----------------------------------
inline constexpr uint8_t kHardwareRevision = 1;

}  // namespace board
