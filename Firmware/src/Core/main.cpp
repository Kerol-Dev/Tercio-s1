// =============================================================================
// Main Motor Controller Firmware
// =============================================================================
#include <Arduino.h>
#include <AS5600.h>
#include <TMCStepper.h>
#include <EEPROM.h>
#include <ACANFD_STM32_Settings.h>
#include "Encoder.h"
#include "StepperControl.h"
#include "AxisController.h"
#include "ConfigStore.h"
#include "Calibration.h"
#include "SensorsADC.h"
#include "CanCmdBus.h"
#include "StepperHoming.h"

// =============================================================================
// CAN Command Definitions
// =============================================================================
enum CommandID : uint8_t
{
  CMD_TARGET_ANGLE = 0x01,
  CMD_SET_CURRENT_MA = 0x02,
  CMD_SET_SPEED_LIMIT = 0x03,
  CMD_SET_PID = 0x04,
  CMD_SET_ID = 0x05,
  CMD_SET_MICROSTEPS = 0x06,
  CMD_SET_STEALTHCHOP = 0x07,
  CMD_SET_EXT_MODE = 0x08,
  CMD_SET_UNITS = 0x09,
  CMD_SET_ENC_INVERT = 0x0A,
  CMD_SET_ENABLED = 0x0B,
  CMD_SET_STEPS_PER_REV = 0x0C,
  CMD_DO_CALIBRATE = 0x0D,
  CMD_DO_HOMING = 0x0E,
  CMD_SET_ENDSTOP = 0x0F,
  CMD_SET_EXT_ENCODER = 0x10,
  CMD_SET_ACCEL_LIMIT = 0x11,
  CMD_SET_DIR_INVERT = 0x12,
  CMD_AUTO_TUNE = 0x13,
  CMD_SET_LIMITSWITCH_ACTIVELOW = 0x14,
};

// =============================================================================
// Hardware Pin Definitions
// =============================================================================
namespace HardwarePins
{
  constexpr uint8_t STEP = PB14;
  constexpr uint8_t DIR = PB15;
  constexpr uint8_t ENABLE = PB3;
  constexpr uint8_t I2C_SDA = PB7;
  constexpr uint8_t I2C_SCL = PA15;
  constexpr uint8_t EXT_I2C_SDA = PA8;
  constexpr uint8_t EXT_I2C_SCL = PA9;
  constexpr uint8_t LIMIT_MIN = PB12;
  constexpr uint8_t LIMIT_MAX = PB13;
  constexpr uint8_t EXT_STEP = PA1;
  constexpr uint8_t EXT_DIR = PB5;
  constexpr uint8_t EXT_ENABLE = PA0;
  constexpr uint8_t SPI_CS = PB2;
}

// =============================================================================
// Auto-Tuning State Machine
// =============================================================================
enum TuningState : uint8_t
{
  TUNE_IDLE = 0,
  TUNE_PREP,
  TUNE_FWD,
  TUNE_BWD,
  TUNE_INCREASE,
  TUNE_DONE
};

struct AutoTuneContext
{
  TuningState state = TUNE_IDLE;
  float minAngleRad = 0.0f;
  float maxAngleRad = 0.0f;
  float currentRPS = 5.0f;
  float currentRPS2 = 30.0f;
  int verifyCount = 0;
  bool isVerifying = false;
  unsigned long doneTimestamp = 0;

  static constexpr float START_RPS = 5.0f;
  static constexpr float START_RPS2 = 30.0f;
  static constexpr float INCREMENT_RPS = 3.0f;
  static constexpr float INCREMENT_RPS2 = 8.0f;
  static constexpr float MAX_RPS = 60.0f;
};

// =============================================================================
// Stall Detection Context
// =============================================================================
struct StallDetection
{
  bool detected = false;
  unsigned long lastMoveTime = 0;
  float lastCheckAngle = 0.0f;
  bool motorRecentlyEnabled = false;
  unsigned long enableTime = 0;

  static constexpr float THRESHOLD_RAD = 2.0f * DEG_TO_RAD;
  static constexpr unsigned long TIMEOUT_MS = 500;
  static constexpr unsigned long SETTLE_MS = 500;
};

// =============================================================================
// Homing Wire Protocol
// =============================================================================
#pragma pack(push, 1)
struct HomingWirePayload
{
  uint8_t useMINTrigger;
  uint8_t sensorlessHoming;
  uint16_t homingCurrent;
  float offset;
  uint8_t activeLow;
  float speed;
  uint8_t direction;
};
#pragma pack(pop)
static_assert(sizeof(HomingWirePayload) == 14, "HomingWirePayload size mismatch");

// =============================================================================
// Global System Objects
// =============================================================================
ConfigStore configStore;
AxisConfig axisConfig;
SensorsADC sensors;
Encoder encoder;
StepperHoming homing;
StepperControl stepperControl(HardwarePins::STEP, HardwarePins::DIR, HardwarePins::ENABLE, axisConfig);
HardwareSerial tmcSerial(PA10, PB6);
TMC2209Stepper tmcDriver(&tmcSerial, 0.11f, 0b00);
AxisController axisController(encoder, stepperControl, tmcDriver, axisConfig);

AxisController::ExtPins externalPins{
    HardwarePins::EXT_STEP,
    HardwarePins::EXT_DIR,
    HardwarePins::EXT_ENABLE,
    true};

AutoTuneContext autoTune;
StallDetection stallContext;
float limitAngleCovered = 0.0f;
float loopDeltaTime = 0.0f;
unsigned long lastLoopTime = 0;
uint16_t preHomingCurrent = 0;

void broadcastTelemetry();

// =============================================================================
// Utility Functions
// =============================================================================
inline bool readBoolPayload(const uint8_t *payload, uint8_t len, uint8_t offset, bool &out)
{
  uint8_t value;
  if (!CanCmdBus::readU8(payload, len, offset, value))
    return false;
  out = (value != 0);
  return true;
}

bool isOverTemperature()
{
  return sensors.temperatureC() >= 95.0f;
}

void resetStallDetection()
{
  stallContext.detected = false;
  stallContext.lastMoveTime = millis();
  stallContext.lastCheckAngle = encoder.angle(Encoder::Radians);
}

void updateStallDetection()
{
  const unsigned long now = millis();

  if (stallContext.motorRecentlyEnabled)
  {
    if (now - stallContext.enableTime > StallDetection::SETTLE_MS)
      stallContext.motorRecentlyEnabled = false;
  }

  if (!stallContext.detected && axisConfig.calibratedOnce &&
      stepperControl.getEnabled() && !stallContext.motorRecentlyEnabled)
  {
    float currentAngle = encoder.angle(Encoder::Radians);
    float targetAngle = axisController.targetAngleRad();
    float error = abs(targetAngle - currentAngle);

    if (error < StallDetection::THRESHOLD_RAD)
    {
      stallContext.lastMoveTime = now;
      stallContext.lastCheckAngle = currentAngle;
    }
    else
    {
      float deltaAngle = abs(currentAngle - stallContext.lastCheckAngle);

      if (deltaAngle > StallDetection::THRESHOLD_RAD)
      {
        stallContext.lastMoveTime = now;
        stallContext.lastCheckAngle = currentAngle;
      }
      else if ((now - stallContext.lastMoveTime) > StallDetection::TIMEOUT_MS)
      {
        stallContext.detected = true;
      }
    }
  }
}

// =============================================================================
// Auto-Tuning State Machine
// =============================================================================
void processAutoTuning()
{
  if (autoTune.state == TUNE_IDLE)
  {
    autoTune.isVerifying = false;
    autoTune.verifyCount = 0;
    return;
  }

  if (autoTune.state == TUNE_DONE)
  {
    if (millis() - autoTune.doneTimestamp > 3000)
    {
      autoTune.state = TUNE_IDLE;
      resetStallDetection();
      autoTune.isVerifying = false;
      autoTune.verifyCount = 0;
    }
    return;
  }

  if (stallContext.detected)
  {
    stepperControl.stop();
    axisController.setTargetAngleRad(encoder.angle(Encoder::Radians));
    resetStallDetection();

    if (autoTune.currentRPS > AutoTuneContext::START_RPS)
    {
      autoTune.currentRPS -= AutoTuneContext::INCREMENT_RPS;
      autoTune.currentRPS2 -= AutoTuneContext::INCREMENT_RPS2;
    }
    else
    {
      autoTune.currentRPS = AutoTuneContext::START_RPS;
      autoTune.currentRPS2 = AutoTuneContext::START_RPS2;
    }

    autoTune.isVerifying = true;
    autoTune.verifyCount = 0;
    autoTune.state = TUNE_PREP;
    return;
  }

  float currentAngle = encoder.angle(Encoder::Radians);
  float distance;

  switch (autoTune.state)
  {
  case TUNE_PREP:
    axisController.setTargetAngleRad(autoTune.minAngleRad);
    stallContext.lastMoveTime = millis();
    stallContext.lastCheckAngle = currentAngle;

    distance = abs(currentAngle - autoTune.minAngleRad);
    if (distance < 0.05f)
      autoTune.state = TUNE_FWD;
    break;

  case TUNE_FWD:
    axisConfig.maxRPS = autoTune.currentRPS;
    axisConfig.maxRPS2 = autoTune.currentRPS2;
    axisController.setTargetAngleRad(autoTune.maxAngleRad);

    distance = abs(currentAngle - autoTune.maxAngleRad);
    if (distance < 0.05f)
    {
      stallContext.lastMoveTime = millis();
      stallContext.lastCheckAngle = currentAngle;
      autoTune.state = TUNE_BWD;
    }
    break;

  case TUNE_BWD:
    axisController.setTargetAngleRad(autoTune.minAngleRad);

    distance = abs(currentAngle - autoTune.minAngleRad);
    if (distance < 0.05f)
      autoTune.state = TUNE_INCREASE;
    break;

  case TUNE_INCREASE:
    if (autoTune.isVerifying)
    {
      autoTune.verifyCount++;

      if (autoTune.verifyCount >= 3)
      {
        axisConfig.maxRPS = autoTune.currentRPS;
        axisConfig.maxRPS2 = autoTune.currentRPS2;
        configStore.save(axisConfig);
        autoTune.state = TUNE_DONE;
        autoTune.doneTimestamp = millis();
      }
      else
      {
        autoTune.state = TUNE_FWD;
      }
    }
    else
    {
      autoTune.currentRPS += AutoTuneContext::INCREMENT_RPS;
      autoTune.currentRPS2 += AutoTuneContext::INCREMENT_RPS2;

      if (autoTune.currentRPS > AutoTuneContext::MAX_RPS)
      {
        autoTune.currentRPS = AutoTuneContext::MAX_RPS;
        axisConfig.maxRPS = autoTune.currentRPS;
        axisConfig.maxRPS2 = autoTune.currentRPS2;
        configStore.save(axisConfig);
        autoTune.state = TUNE_DONE;
        autoTune.doneTimestamp = millis();
      }
      else
      {
        stallContext.lastMoveTime = millis();
        stallContext.lastCheckAngle = currentAngle;
        autoTune.state = TUNE_FWD;
      }
    }
    break;

  default:
    break;
  }
}

// =============================================================================
// CAN Command Handlers
// =============================================================================
void handleTargetAngle(const CanCmdBus::CmdFrame &frame)
{
  if (autoTune.state != TUNE_IDLE)
    autoTune.state = TUNE_IDLE;

  float angle;
  if (!CanCmdBus::readF32(frame.payload, frame.len, 0, angle))
    return;

  const float angleRad = (axisConfig.units == 1) ? angle * DEG_TO_RAD : angle;
  resetStallDetection();
  axisController.setTargetAngleRad(angleRad);
}

void handleSetCurrentMA(const CanCmdBus::CmdFrame &frame)
{
  uint16_t milliamps;
  if (!CanCmdBus::readU16(frame.payload, frame.len, 0, milliamps))
    return;

  milliamps = constrain(milliamps, 50, 2000);
  axisConfig.driver_mA = milliamps;
  tmcDriver.rms_current(milliamps);
  configStore.save(axisConfig);
}

void handleSetSpeedLimit(const CanCmdBus::CmdFrame &frame)
{
  float rps;
  if (!CanCmdBus::readF32(frame.payload, frame.len, 0, rps))
    return;

  axisConfig.maxRPS = rps;
  configStore.save(axisConfig);
}

void handleSetAccelLimit(const CanCmdBus::CmdFrame &frame)
{
  float rps2;
  if (!CanCmdBus::readF32(frame.payload, frame.len, 0, rps2))
    return;

  axisConfig.maxRPS2 = rps2;
  configStore.save(axisConfig);
}

void handleSetPID(const CanCmdBus::CmdFrame &frame)
{
  float kp, ki, kd;
  if (!CanCmdBus::readF32(frame.payload, frame.len, 0, kp))
    return;
  if (!CanCmdBus::readF32(frame.payload, frame.len, 4, ki))
    return;
  if (!CanCmdBus::readF32(frame.payload, frame.len, 8, kd))
    return;

  axisConfig.Kp = kp;
  axisConfig.Ki = ki;
  axisConfig.Kd = kd;
  axisController.setPID(kp, ki, kd);
  configStore.save(axisConfig);
}

void handleSetID(const CanCmdBus::CmdFrame &frame)
{
  uint16_t newID;
  if (!CanCmdBus::readU16(frame.payload, frame.len, 0, newID))
    return;

  newID &= 0x7FF;
  axisConfig.canArbId = newID;
  CanCmdBus::setIdFilter(axisConfig.canArbId, 0x7FF);
  configStore.save(axisConfig);
}

void handleSetMicrosteps(const CanCmdBus::CmdFrame &frame)
{
  uint16_t microsteps;
  if (!CanCmdBus::readU16(frame.payload, frame.len, 0, microsteps))
    return;

  if (microsteps == 0)
    microsteps = 1;

  axisConfig.microsteps = microsteps;
  axisController.setMicrosteps(microsteps);
  configStore.save(axisConfig);
}

void handleSetStealthChop(const CanCmdBus::CmdFrame &frame)
{
  bool enabled;
  if (!readBoolPayload(frame.payload, frame.len, 0, enabled))
    return;

  axisConfig.stealthChop = enabled;

  AxisController::TmcConfig tmcConfig{};
  tmcConfig.mA = axisConfig.driver_mA;
  tmcConfig.toff = 5;
  tmcConfig.blank = 24;
  tmcConfig.stealth = axisConfig.stealthChop;
  tmcConfig.spreadSwitchRPS = 5.0f;
  axisController.configureDriver(tmcConfig);

  configStore.save(axisConfig);
}

void handleSetExternalMode(const CanCmdBus::CmdFrame &frame)
{
  bool enabled;
  if (!readBoolPayload(frame.payload, frame.len, 0, enabled))
    return;

  axisConfig.externalMode = enabled;
  axisController.setExternalMode(axisConfig.externalMode);

  if (!axisConfig.externalMode)
  {
    stepperControl.enable();
    resetStallDetection();
  }

  configStore.save(axisConfig);
}

void handleSetUnits(const CanCmdBus::CmdFrame &frame)
{
  uint8_t units;
  if (!CanCmdBus::readU8(frame.payload, frame.len, 0, units))
    return;

  axisConfig.units = (units != 0) ? 1 : 0;
  configStore.save(axisConfig);
}

void handleSetEncoderInvert(const CanCmdBus::CmdFrame &frame)
{
  bool inverted;
  if (!readBoolPayload(frame.payload, frame.len, 0, inverted))
    return;

  axisConfig.encInvert = inverted;
  encoder.setInvert(axisConfig.encInvert);
  configStore.save(axisConfig);
}

void handleSetDirectionInvert(const CanCmdBus::CmdFrame &frame)
{
  bool inverted;
  if (!readBoolPayload(frame.payload, frame.len, 0, inverted))
    return;

  axisConfig.dirInvert = inverted;
  axisConfig.calibratedOnce = false;
  configStore.save(axisConfig);
}

void handleSetEnabled(const CanCmdBus::CmdFrame &frame)
{
  autoTune.state = TUNE_IDLE;

  bool enabled;
  if (!readBoolPayload(frame.payload, frame.len, 0, enabled))
    return;

  if (enabled)
  {
    stepperControl.enable();
    resetStallDetection();
    stallContext.motorRecentlyEnabled = true;
    stallContext.enableTime = millis();
  }
  else
  {
    stepperControl.stop();
    stepperControl.disable();
  }
}

void handleSetStepsPerRev(const CanCmdBus::CmdFrame &frame)
{
  uint16_t stepsPerRev;
  if (!CanCmdBus::readU16(frame.payload, frame.len, 0, stepsPerRev))
    return;

  axisConfig.stepsPerRev = stepsPerRev;
  axisController.setFullSteps(stepsPerRev);
  configStore.save(axisConfig);
}

void handleCalibrate(const CanCmdBus::CmdFrame &frame)
{
  (void)frame;
  autoTune.state = TUNE_IDLE;
  resetStallDetection();
  Calibrate_EncoderDirection(encoder, stepperControl, axisController, axisConfig, 1.0f, 1000);
  configStore.save(axisConfig);
}

void handleHoming(const CanCmdBus::CmdFrame &frame)
{
  if (!axisConfig.calibratedOnce)
    return;

  autoTune.state = TUNE_IDLE;
  stepperControl.enable();
  resetStallDetection();

  if (frame.len != sizeof(HomingWirePayload))
    return;

  HomingWirePayload payload{};
  memcpy(&payload, frame.payload, sizeof(HomingWirePayload));

  HomingConfig homingConfig{
      static_cast<uint8_t>(HardwarePins::LIMIT_MIN),
      static_cast<uint8_t>(HardwarePins::LIMIT_MAX),
      (payload.sensorlessHoming != 0),
      static_cast<float>(payload.homingCurrent),
      (payload.activeLow != 0),
      (payload.activeLow != 0),
      payload.speed,
      30000u,
      payload.offset};

  homing.begin(homingConfig);

  preHomingCurrent = axisConfig.driver_mA;
  float current = constrain(payload.homingCurrent, 50, 2000);
  axisConfig.driver_mA = current;
  tmcDriver.rms_current(current);

  const bool useMinTrigger = (payload.useMINTrigger != 0);
  const bool directionPositive = (payload.direction != 0);

  homing.home(
      [&](float velocity)
      {
        if (!directionPositive)
          velocity = -velocity;
        stepperControl.setSpeedRPS(velocity);
      },
      [&]()
      { stepperControl.stop(); },
      [&]()
      { broadcastTelemetry(); },
      encoder,
      axisController,
      axisConfig,
      [&](float)
      { encoder.calibrateZero(); },
      useMinTrigger);
}

void handleSetEndstop(const CanCmdBus::CmdFrame &frame)
{
  bool enabled;
  if (!readBoolPayload(frame.payload, frame.len, 0, enabled))
    return;

  axisConfig.enableEndStop = enabled;
  configStore.save(axisConfig);
}

void handleSetExternalEncoder(const CanCmdBus::CmdFrame &frame)
{
  bool external;
  if (!readBoolPayload(frame.payload, frame.len, 0, external))
    return;

  axisConfig.externalEncoder = external;
  encoder.begin(
      external ? HardwarePins::EXT_I2C_SDA : HardwarePins::I2C_SDA,
      external ? HardwarePins::EXT_I2C_SCL : HardwarePins::I2C_SCL,
      HardwarePins::SPI_CS);
  configStore.save(axisConfig);
}

void handleAutoTune(const CanCmdBus::CmdFrame &frame)
{
  float minDeg, maxDeg;
  if (!CanCmdBus::readF32(frame.payload, frame.len, 0, minDeg))
    return;
  if (!CanCmdBus::readF32(frame.payload, frame.len, 4, maxDeg))
    return;

  if (!axisConfig.calibratedOnce)
    return;

  stepperControl.enable();
  resetStallDetection();

  autoTune.minAngleRad = minDeg * DEG_TO_RAD;
  autoTune.maxAngleRad = maxDeg * DEG_TO_RAD;
  autoTune.currentRPS = AutoTuneContext::START_RPS;
  autoTune.currentRPS2 = AutoTuneContext::START_RPS2;
  axisConfig.maxRPS = autoTune.currentRPS;
  axisConfig.maxRPS2 = autoTune.currentRPS2;
  autoTune.state = TUNE_PREP;
}

void handleSetLimitActiveLow(const CanCmdBus::CmdFrame &frame)
{
  bool activeLow;
  if (!readBoolPayload(frame.payload, frame.len, 0, activeLow))
    return;

  axisConfig.limitSwitchActiveLow = activeLow;

  HomingConfig homingConfig{
      static_cast<uint8_t>(HardwarePins::LIMIT_MIN),
      static_cast<uint8_t>(HardwarePins::LIMIT_MAX),
      false,
      1000.0f,
      axisConfig.limitSwitchActiveLow,
      axisConfig.limitSwitchActiveLow,
      0.0f,
      30000u,
      0.0f};

  homing.begin(homingConfig);
  configStore.save(axisConfig);
}

// =============================================================================
// Telemetry Broadcast
// =============================================================================
void broadcastTelemetry()
{
  struct TelemetryPacket
  {
    AxisConfigWire config;
    float currentSpeed;
    float currentAngle;
    float targetAngle;
    float temperature;
    uint8_t stalled;
    uint8_t tuneState;
    uint8_t minTriggered;
    uint8_t maxTriggered;
  };

  TelemetryPacket packet;
  const bool useDegrees = (axisConfig.units == 1);

  // packet.currentSpeed = encoder.velocity(useDegrees ? Encoder::Degrees : Encoder::Radians);
  packet.currentAngle = HAL_RCC_GetSysClockFreq();
  packet.currentAngle = encoder.angle(useDegrees ? Encoder::Degrees : Encoder::Radians);
  packet.targetAngle = axisController.targetAngleRad() * (useDegrees ? RAD_TO_DEG : 1.0f);
  packet.temperature = sensors.temperatureC();
  packet.stalled = stallContext.detected;
  packet.tuneState = static_cast<uint8_t>(autoTune.state);
  packet.minTriggered = homing.minTriggered();
  packet.maxTriggered = homing.maxTriggered();
  packet.config = toWire(axisConfig);

  CanCmdBus::sendStruct(0x000, 0x01, packet);
}

// =============================================================================
// System Initialization
// =============================================================================
void setup()
{
  EEPROM.begin();
  sensors.begin();
  delay(100);

  if (!configStore.load(axisConfig))
    configStore.save(axisConfig);

  delay(100);

  CanCmdBus::begin(500000, 5, PA11, PA12, true);
  CanCmdBus::setIdFilter(axisConfig.canArbId, 0x7FF);

  CanCmdBus::registerHandler(CMD_TARGET_ANGLE, handleTargetAngle);
  CanCmdBus::registerHandler(CMD_SET_CURRENT_MA, handleSetCurrentMA);
  CanCmdBus::registerHandler(CMD_SET_SPEED_LIMIT, handleSetSpeedLimit);
  CanCmdBus::registerHandler(CMD_SET_ACCEL_LIMIT, handleSetAccelLimit);
  CanCmdBus::registerHandler(CMD_SET_PID, handleSetPID);
  CanCmdBus::registerHandler(CMD_SET_ID, handleSetID);
  CanCmdBus::registerHandler(CMD_SET_MICROSTEPS, handleSetMicrosteps);
  CanCmdBus::registerHandler(CMD_SET_STEALTHCHOP, handleSetStealthChop);
  CanCmdBus::registerHandler(CMD_SET_EXT_MODE, handleSetExternalMode);
  CanCmdBus::registerHandler(CMD_SET_UNITS, handleSetUnits);
  CanCmdBus::registerHandler(CMD_SET_ENC_INVERT, handleSetEncoderInvert);
  CanCmdBus::registerHandler(CMD_SET_ENABLED, handleSetEnabled);
  CanCmdBus::registerHandler(CMD_SET_STEPS_PER_REV, handleSetStepsPerRev);
  CanCmdBus::registerHandler(CMD_DO_CALIBRATE, handleCalibrate);
  CanCmdBus::registerHandler(CMD_DO_HOMING, handleHoming);
  CanCmdBus::registerHandler(CMD_SET_ENDSTOP, handleSetEndstop);
  CanCmdBus::registerHandler(CMD_SET_EXT_ENCODER, handleSetExternalEncoder);
  CanCmdBus::registerHandler(CMD_SET_DIR_INVERT, handleSetDirectionInvert);
  CanCmdBus::registerHandler(CMD_AUTO_TUNE, handleAutoTune);
  CanCmdBus::registerHandler(CMD_SET_LIMITSWITCH_ACTIVELOW, handleSetLimitActiveLow);

  HomingConfig initialHomingConfig{
      static_cast<uint8_t>(HardwarePins::LIMIT_MIN),
      static_cast<uint8_t>(HardwarePins::LIMIT_MAX),
      false,
      1000.0f,
      axisConfig.limitSwitchActiveLow,
      axisConfig.limitSwitchActiveLow,
      0.0f,
      30000u,
      0.0f};

  homing.begin(initialHomingConfig);

  encoder.begin(
      axisConfig.externalEncoder ? HardwarePins::EXT_I2C_SDA : HardwarePins::I2C_SDA,
      axisConfig.externalEncoder ? HardwarePins::EXT_I2C_SCL : HardwarePins::I2C_SCL,
      HardwarePins::SPI_CS);
  encoder.setVelAlpha(1);
  encoder.setInvert(axisConfig.encInvert);

  tmcSerial.begin(115200);
  axisController.begin();

  AxisController::TmcConfig tmcConfig{};
  tmcConfig.mA = axisConfig.driver_mA;
  tmcConfig.toff = 5;
  tmcConfig.blank = 24;
  tmcConfig.stealth = axisConfig.stealthChop;
  tmcConfig.spreadSwitchRPS = 5.0f;
  axisController.configureDriver(tmcConfig);

  axisController.setFullSteps(axisConfig.stepsPerRev);
  axisController.setPID(axisConfig.Kp, axisConfig.Ki, axisConfig.Kd);
  axisController.setMicrosteps(axisConfig.microsteps);
  axisController.attachExternal(externalPins);
  axisController.setExternalMode(axisConfig.externalMode);
  axisController.setTargetAngleRad(0.0f);

  resetStallDetection();
  delay(100);
}

// =============================================================================
// Main Loop
// =============================================================================
void loop()
{
  const unsigned long now = millis();
  loopDeltaTime = (now - lastLoopTime) * 0.001f;
  lastLoopTime = now;

  processAutoTuning();
  updateStallDetection();

  if (stallContext.detected)
  {
    axisController.setTargetAngleRad(encoder.angle(Encoder::Radians));
    stepperControl.stop();
  }

  if (axisConfig.calibratedOnce && !isOverTemperature() && !stallContext.detected)
    axisController.update(loopDeltaTime);

  sensors.update();
  homing.update();

  if (axisConfig.enableEndStop)
  {
    if (homing.minTriggered())
    {
      if (limitAngleCovered == 0.0f)
        limitAngleCovered = encoder.angle(Encoder::Radians);

      if (axisController.targetAngleRad() < limitAngleCovered)
        axisController.setTargetAngleRad(limitAngleCovered);
    }
    else if (homing.maxTriggered())
    {
      if (limitAngleCovered == 0.0f)
        limitAngleCovered = encoder.angle(Encoder::Radians);

      if (axisController.targetAngleRad() > limitAngleCovered)
        axisController.setTargetAngleRad(limitAngleCovered);
    }
    else
    {
      limitAngleCovered = 0.0f;
    }
  }

  if (isOverTemperature() || !axisConfig.calibratedOnce)
  {
    axisController.setTargetAngleRad(0.0f);
    stepperControl.stop();
  }

  static unsigned long lastTelemetryTime = 0;
  if (now - lastTelemetryTime >= 10)
  {
    lastTelemetryTime = now;
    broadcastTelemetry();
  }

  CanCmdBus::poll();
  delay(5);
}