#pragma once
#include <Arduino.h>

namespace Tercio {

static const uint8_t  TELEMETRY_CMD       = 0x01;
static const uint8_t  GET_CONFIG_CMD      = 0x20;
static const uint8_t  IMU_TELEMETRY_CMD   = 0x02;

static const uint16_t MAX_CAN_ID          = 0x7FF;
static const size_t   MAX_PAYLOAD_SIZE    = 64;
static const size_t   HDR_SIZE            = 4;   // uint16_t id, uint8_t cmd, uint8_t len

enum class Cmd : uint8_t {
  TARGET_ANGLE              = 0x01,
  SET_CURRENT_MA            = 0x02,
  SET_SPEED_LIMIT           = 0x03,
  SET_PID                   = 0x04,
  SET_ID                    = 0x05,
  SET_MICROSTEPS            = 0x06,
  SET_STEALTHCHOP           = 0x07,
  SET_EXT_MODE              = 0x08,
  SET_UNITS                 = 0x09,
  SET_ENC_INVERT            = 0x0A,
  SET_ENABLED               = 0x0B,
  SET_STEPS_PER_REV         = 0x0C,
  DO_CALIBRATE              = 0x0D,
  DO_HOMING                 = 0x0E,
  SET_ENDSTOP               = 0x0F,
  SET_EXT_ENCODER           = 0x10,
  SET_ACCEL_LIMIT           = 0x11,
  SET_DIR_INVERT            = 0x12,
  DO_AUTO_TUNE              = 0x13,
  SET_LIMITSWITCH_ACTIVELOW = 0x14,
  GET_CONFIG                = 0x20,
  SET_IMU_ID                = 0xA1,
  RESET_ORIENT              = 0xA2
};

enum class TuningState : uint8_t {
  IDLE      = 0,
  PREP      = 1,
  FWD       = 2,
  BWD       = 3,
  INCREASE  = 4,
  DONE      = 5
};

struct AxisFlags {
  bool encInvert;
  bool dirInvert;
  bool stealthChop;
  bool externalMode;
  bool enableEndStop;
  bool externalEncoder;
  bool calibratedOnce;
  bool limitSwitchActiveLow; // moved from externalSPI slot
};

struct AxisConfig {
  uint32_t crc32;
  uint16_t microsteps;
  uint16_t stepsPerRev;
  uint8_t  units;
  AxisFlags flags;
  uint16_t encZeroCounts;
  uint16_t driver_mA;
  float    maxRPS;
  float    maxRPS2;
  float    Kp;
  float    Ki;
  float    Kd;
  uint16_t canArbId;
};

struct AxisState {
  AxisConfig   config;
  float        currentSpeed;
  float        currentAngle;
  float        targetAngle;
  float        temperature;
  bool         stalled;
  TuningState  tuneState;
  bool         minTriggered;
  bool         maxTriggered;
  uint32_t     timestampMs;
};

struct ImuState {
  float    roll;
  float    pitch;
  float    yaw;
  float    ax;
  float    ay;
  float    az;
  float    temp;
  uint32_t timestampMs;
};

struct HomingParams {
  bool  useIN1Trigger = true;
  bool  sensorlessHoming = false;
  uint16_t homingCurrent = 1200;
  float offset = 0.0f;
  bool  activeLow = true;
  float speed = 1.0f;
  bool  direction = true;
};

class Bridge {
public:
  explicit Bridge(Stream& io);

  void poll();

  bool getAxisState(uint16_t canId, AxisState& out) const;
  bool getImuState(uint16_t canId, ImuState& out) const;

  void requestConfig(uint16_t canId);

  void setTargetAngle(uint16_t canId, float angle);
  void setCurrentMA(uint16_t canId, uint16_t mA);
  void setSpeedLimitRps(uint16_t canId, float rps);
  void setAccelLimitRps2(uint16_t canId, float rps2);
  void setPid(uint16_t canId, float kp, float ki, float kd);
  void setCanId(uint16_t canId, uint16_t newId);
  void setMicrosteps(uint16_t canId, uint16_t microsteps);
  void setStealthChop(uint16_t canId, bool enable);
  void setExternalMode(uint16_t canId, bool enable);
  void setUnitsDegrees(uint16_t canId, bool useDegrees);
  void setEncoderInvert(uint16_t canId, bool enable);
  void setDirectionInvert(uint16_t canId, bool invert);
  void enableMotor(uint16_t canId, bool enable);
  void setStepsPerRev(uint16_t canId, uint16_t stepsPerRev);
  void setExternalEncoder(uint16_t canId, bool enable);
  void setEndstop(uint16_t canId, bool enable);
  void setLimitSwitchActiveLow(uint16_t canId, bool activeLow);
  void doCalibrate(uint16_t canId);
  void doHoming(uint16_t canId, const HomingParams& p);
  void doAutoTune(uint16_t canId, float minAngle, float maxAngle);
  void setImuId(uint16_t currentId, uint16_t newId);
  void resetOrientation(uint16_t canId);

private:
  Stream& _io;
  static const size_t RX_BUF_SIZE = 256;
  uint8_t _rxBuf[RX_BUF_SIZE];
  size_t  _rxLen;

  static uint16_t clamp11(uint16_t id);
  void sendFrame(uint16_t canId, Cmd cmd, const uint8_t* payload, uint8_t len);
  void sendU16(uint16_t canId, Cmd cmd, uint16_t value);
  void sendF32(uint16_t canId, Cmd cmd, float value);
  void sendBool(uint16_t canId, Cmd cmd, bool value);

  void processBuf();
  void handleTelemetry(uint16_t id, const uint8_t* payload, uint8_t len);
  void handleConfig(uint16_t id, const uint8_t* payload, uint8_t len);
  void handleImuTelemetry(uint16_t id, const uint8_t* payload, uint8_t len);

  bool parseAxisConfig(const uint8_t* b, size_t len, AxisConfig& out) const;
  bool parseAxisTelemetry(const uint8_t* b, size_t len, AxisState& out) const;
  bool parseImuTelemetry(const uint8_t* b, size_t len, ImuState& out) const;

  static const size_t MAX_AXES = 16;
  AxisState _axes[MAX_AXES];
  ImuState  _imus[MAX_AXES];
  bool      _axisUsed[MAX_AXES];
  bool      _imuUsed[MAX_AXES];

  int findAxisIndex(uint16_t canId) const;
  int findImuIndex(uint16_t canId) const;
  int allocAxisIndex(uint16_t canId);
  int allocImuIndex(uint16_t canId);
};

class Stepper {
public:
  Stepper(Bridge& b, uint16_t canId);

  uint16_t id() const { return _id; }
  void setId(uint16_t newId);

  void requestConfig();
  void enableMotor(bool en);
  void setTargetAngle(float ang);
  void setCurrentMA(uint16_t mA);
  void setSpeedLimitRps(float rps);
  void setAccelLimitRps2(float rps2);
  void setPid(float kp, float ki, float kd);
  void setMicrosteps(uint16_t m);
  void setStealthChop(bool en);
  void setExternalMode(bool en);
  void setUnitsDegrees(bool on);
  void setEncoderInvert(bool on);
  void setDirectionInvert(bool on);
  void setExternalEncoder(bool on);
  void setEndstop(bool on);
  void setLimitSwitchActiveLow(bool activeLow);
  void doCalibrate();
  void doHoming(const HomingParams& p);
  void doAutoTune(float minAngle, float maxAngle);

  bool getAxisState(AxisState& out) const;

private:
  Bridge&  _bridge;
  uint16_t _id;
};

class IMU {
public:
  IMU(Bridge& b, uint16_t controlId = 0x003);

  uint16_t id() const { return _id; }
  void setId(uint16_t newId);

  void resetOrientation();
  bool getState(ImuState& out) const;

  bool getRoll(float& out) const;
  bool getPitch(float& out) const;
  bool getYaw(float& out) const;
  bool getAccelX(float& out) const;
  bool getAccelY(float& out) const;
  bool getAccelZ(float& out) const;
  bool getTemperature(float& out) const;

private:
  Bridge&  _bridge;
  uint16_t _id;
};

} // namespace Tercio