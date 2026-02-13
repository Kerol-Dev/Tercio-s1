#pragma once
#include <Arduino.h>

namespace tercio
{

  // ================= Protocol constants =================
  static constexpr uint8_t TELEMETRY_CMD = 0x01;
  static constexpr uint8_t IMU_TELEMETRY_CMD = 0x02;
  static constexpr uint8_t GET_CONFIG_CMD = 0x20;

  static constexpr size_t MAX_PAYLOAD = 64;
  static constexpr size_t HDR_SIZE = 4; // <HBB : can_id(u16) + cmd(u8) + len(u8)

  // ================= Commands =================
  enum class Cmd : uint8_t
  {
    TARGET_ANGLE = 0x01,
    SET_CURRENT_MA = 0x02,
    SET_SPEED_LIMIT = 0x03,
    SET_PID = 0x04,
    SET_ID = 0x05,
    SET_MICROSTEPS = 0x06,
    SET_STEALTHCHOP = 0x07,
    SET_EXT_MODE = 0x08,
    SET_UNITS = 0x09, // 0=rad, 1=deg
    SET_ENC_INVERT = 0x0A,
    SET_ENABLED = 0x0B,
    SET_STEPS_PER_REV = 0x0C,
    DO_CALIBRATE = 0x0D,
    DO_HOMING = 0x0E,
    SET_ENDSTOP = 0x0F,
    SET_EXT_ENCODER = 0x10,
    SET_ACCEL_LIMIT = 0x11,
    SET_DIR_INVERT = 0x12,
    SET_EXT_SPI = 0x13,
    DO_AUTO_TUNE = 0x14,
    SET_LIMITSWITCH_ACTIVELOW = 0x15, // New!
    GET_CONFIG = 0x20,
    SET_IMU_ID = 0xA1,
    RESET_ORIENT = 0xA2,
  };

  enum class TuningState : uint8_t
  {
    IDLE = 0,
    PREP = 1,
    FWD = 2,
    BWD = 3,
    INCREASE = 4,
    DONE = 5
  };

// ================= Wire structs (packed, little-endian) =================
// Python: AXIS_CONFIG_FMT = "<I H H B B H H f f f f f H"
#pragma pack(push, 1)
  struct AxisConfigWire
  {
    uint32_t crc32;
    uint16_t microsteps;
    uint16_t stepsPerRev;
    uint8_t units;
    uint16_t flags;

    uint16_t encZeroCounts;
    uint16_t driver_mA;
    float maxRPS;
    float maxRPS2;
    float Kp;
    float Ki;
    float Kd;
    uint16_t canArbId;
  };
#pragma pack(pop)
  static constexpr size_t AXIS_CONFIG_SIZE = sizeof(AxisConfigWire);

// Python: TELEM_TAIL_FMT = "<f f f f B B B B"
#pragma pack(push, 1)
  struct TelemTailWire
  {
    float curSpd;
    float curAng;
    float tgtAng;
    float temp;
    uint8_t stalled_u8;
    uint8_t tuneState_u8;
    uint8_t minT_u8;
    uint8_t maxT_u8;
  };
#pragma pack(pop)
  static constexpr size_t TELEM_TAIL_SIZE = sizeof(TelemTailWire);

// Python: IMU_PAYLOAD_FMT = "<fffffff"
#pragma pack(push, 1)
  struct ImuWire
  {
    float roll, pitch, yaw;
    float ax, ay, az;
    float temp;
  };
#pragma pack(pop)
  static constexpr size_t IMU_PAYLOAD_SIZE = sizeof(ImuWire);

  // ================= Decoded state =================
  struct AxisFlags
  {
    bool encInvert = false;            // bit0
    bool dirInvert = false;            // bit1
    bool stealthChop = false;          // bit2
    bool externalMode = false;         // bit3
    bool enableEndstop = false;        // bit4
    bool externalEncoder = false;      // bit5
    bool calibratedOnce = false;       // bit6
    bool externalSPI = false;          // bit7
    bool limitSwitchActiveLow = false; // bit8
  };

  struct AxisConfig
  {
    uint32_t crc32 = 0;
    uint16_t microsteps = 0;
    uint16_t stepsPerRev = 0;
    uint8_t units = 0;
    AxisFlags flags;
    uint16_t encZeroCounts = 0;
    uint16_t driver_mA = 0;
    float maxRPS = 0;
    float maxRPS2 = 0;
    float Kp = 0, Ki = 0, Kd = 0;
    uint16_t canArbId = 0;
  };

  struct AxisState
  {
    AxisConfig config;
    float currentSpeed = 0;
    float currentAngle = 0;
    float targetAngle = 0;
    float temperature = 0;
    bool stalled = false;
    TuningState tuneState = TuningState::IDLE;
    bool minTriggered = false;
    bool maxTriggered = false;
    uint32_t timestampMs = 0;
  };

  struct ImuState
  {
    float roll = 0, pitch = 0, yaw = 0;
    float ax = 0, ay = 0, az = 0;
    float temp = 0;
    uint32_t timestampMs = 0;
  };

// Homing wire in Python: "<BBHfBfB" (8+? => 1+1+2+4+1+4+1 = 14 bytes)
#pragma pack(push, 1)
  struct HomingWire
  {
    uint8_t use_in1;        // 0/1
    uint8_t sensorless;     // 0/1
    uint16_t homingCurrent; // mA
    float offset;           // units
    uint8_t active_low;     // 0/1
    float speed;            // rps
    uint8_t direction;      // 0/1
  };
#pragma pack(pop)
  static constexpr size_t HOMING_WIRE_SIZE = sizeof(HomingWire);

  struct HomingParams
  {
    bool useMinTrigger = true;
    bool sensorlessHoming = false;
    uint16_t homingCurrent = 1200;
    float offset = 0.0f;
    bool activeLow = true;
    float speed = 1.0f;
    bool direction = true;
  };

  // ================= Bridge =================
  class Bridge
  {
  public:
    Bridge();

    bool begin(Stream &s);
    void end();

    void update();

    // ---- Raw send
    bool send(uint16_t can_id, uint8_t cmd, const uint8_t *payload, uint8_t len);

    // ---- High-level wrappers
    bool request_config(uint16_t can_id);

    bool set_target_angle(uint16_t can_id, float angle);
    bool set_current_ma(uint16_t can_id, uint16_t ma);
    bool set_speed_limit_rps(uint16_t can_id, float rps);
    bool set_accel_limit_rps2(uint16_t can_id, float rps2);
    bool set_pid(uint16_t can_id, float kp, float ki, float kd);
    bool set_can_id(uint16_t can_id, uint16_t new_id);
    bool set_microsteps(uint16_t can_id, uint16_t microsteps);
    bool set_stealthchop(uint16_t can_id, bool enable);
    bool set_external_mode(uint16_t can_id, bool enable);
    bool set_units_degrees(uint16_t can_id, bool use_degrees);
    bool set_encoder_invert(uint16_t can_id, bool enable);
    bool set_direction_invert(uint16_t can_id, bool invert);
    bool enable_motor(uint16_t can_id, bool enable);
    bool set_steps_per_rev(uint16_t can_id, uint16_t steps_per_rev);
    bool set_external_encoder(uint16_t can_id, bool enable);
    bool set_external_spi(uint16_t can_id, bool enable);
    bool set_endstop(uint16_t can_id, bool enable);
    bool set_limit_switch_active_low(uint16_t can_id, bool active_low);
    bool do_calibrate(uint16_t can_id);
    bool do_homing(uint16_t can_id, const HomingParams &p);
    bool do_auto_tune(uint16_t can_id, float min_angle, float max_angle);

    bool set_imu_id(uint16_t current_id, uint16_t new_id);
    bool reset_orientation(uint16_t imu_control_id);

    int registerAxis(uint16_t can_id);
    int registerImu(uint16_t can_id);

    const AxisState *getAxis(uint16_t can_id) const;
    const ImuState *getImu(uint16_t can_id) const;

    static constexpr size_t MAX_AXIS = 32;
    static constexpr size_t MAX_IMU = 16;

  private:
    Stream *_s = nullptr;

    // Simple RX ring buffer
    static constexpr size_t RX_BUF_SIZE = 512;
    uint8_t _rx[RX_BUF_SIZE];
    size_t _rx_head = 0; // next write
    size_t _rx_tail = 0; // next read

    // Caches
    struct AxisSlot
    {
      uint16_t id = 0xFFFF;
      AxisState st;
      bool valid = false;
    };
    struct ImuSlot
    {
      uint16_t id = 0xFFFF;
      ImuState st;
      bool valid = false;
    };
    AxisSlot _axis[MAX_AXIS];
    ImuSlot _imu[MAX_IMU];

    // helpers
    size_t rxAvailable() const;
    bool rxPeek(size_t offset, uint8_t &out) const;
    bool rxRead(uint8_t &out);
    bool rxReadBytes(uint8_t *dst, size_t n);
    void rxWriteBytes(const uint8_t *src, size_t n);

    static void decodeAxisConfig(const AxisConfigWire &w, AxisConfig &out);
    void handleFrame(uint16_t can_id, uint8_t cmd, const uint8_t *payload, uint8_t len);

    int findAxisSlot(uint16_t can_id) const;
    int findImuSlot(uint16_t can_id) const;
    int allocAxisSlot(uint16_t can_id);
    int allocImuSlot(uint16_t can_id);

    // Little-endian packing
    static void pack_u16(uint8_t *dst, uint16_t v);
    static void pack_f32(uint8_t *dst, float v);
  };

  // ================= Friendly wrappers =================
  class Stepper
  {
  public:
    Stepper(Bridge &b, uint16_t can_id) : _b(b), _id(can_id & 0x7FF)
    {
      _b.registerAxis(_id);
    }

    uint16_t id() const { return _id; }

    bool request_config() { return _b.request_config(_id); }
    bool enable_motor(bool en) { return _b.enable_motor(_id, en); }
    bool set_target_angle(float ang) { return _b.set_target_angle(_id, ang); }
    bool set_current_ma(uint16_t ma) { return _b.set_current_ma(_id, ma); }
    bool set_speed_limit_rps(float rps) { return _b.set_speed_limit_rps(_id, rps); }
    bool set_accel_limit_rps2(float rps2) { return _b.set_accel_limit_rps2(_id, rps2); }
    bool set_pid(float kp, float ki, float kd) { return _b.set_pid(_id, kp, ki, kd); }

    bool set_can_id(uint16_t new_id)
    {
      new_id &= 0x7FF;
      bool ok = _b.set_can_id(_id, new_id);
      if (ok)
      {
        _id = new_id;
        _b.registerAxis(_id);
      }
      return ok;
    }

    bool set_microsteps(uint16_t m) { return _b.set_microsteps(_id, m); }
    bool set_stealthchop(bool en) { return _b.set_stealthchop(_id, en); }
    bool set_external_mode(bool en) { return _b.set_external_mode(_id, en); }
    bool set_units_degrees(bool on) { return _b.set_units_degrees(_id, on); }
    bool set_encoder_invert(bool on) { return _b.set_encoder_invert(_id, on); }
    bool set_direction_invert(bool on) { return _b.set_direction_invert(_id, on); }
    bool set_external_encoder(bool on) { return _b.set_external_encoder(_id, on); }
    bool set_external_spi(bool on) { return _b.set_external_spi(_id, on); }
    bool set_endstop(bool on) { return _b.set_endstop(_id, on); }
    bool set_limit_switch_active_low(bool active_low) { return _b.set_limit_switch_active_low(_id, active_low); } // NEW wrapper
    bool do_calibrate() { return _b.do_calibrate(_id); }
    bool do_homing(const HomingParams &p) { return _b.do_homing(_id, p); }
    bool do_auto_tune(float min_angle, float max_angle) { return _b.do_auto_tune(_id, min_angle, max_angle); }

    const AxisState *state() const { return _b.getAxis(_id); }

  private:
    Bridge &_b;
    uint16_t _id;
  };

  class IMU
  {
  public:
    IMU(Bridge &b, uint16_t control_id = 0x003) : _b(b), _id(control_id & 0x7FF)
    {
      _b.registerImu(_id);
    }

    uint16_t id() const { return _id; }

    bool reset_orientation() { return _b.reset_orientation(_id); }

    bool set_can_id(uint16_t new_id)
    {
      new_id &= 0x7FF;
      bool ok = _b.set_imu_id(_id, new_id);
      if (ok)
      {
        _id = new_id;
        _b.registerImu(_id);
      }
      return ok;
    }

    const ImuState *state() const { return _b.getImu(_id); }

  private:
    Bridge &_b;
    uint16_t _id;
  };
}