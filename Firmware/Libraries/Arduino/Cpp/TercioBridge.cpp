#include "TercioBus.h"

namespace Tercio {

static const size_t AXIS_CONFIG_SIZE = sizeof(uint32_t) + 2 + 2 + 1 + 2 + 2 + 2 + 4*5 + 2;
static const size_t TELEM_TAIL_SIZE  = sizeof(float)*4 + 4;   // 4 floats + 4 bytes (B,B,B,B)
static const size_t IMU_PAYLOAD_SIZE = sizeof(float)*7;

static uint32_t millisNow() {
  return millis();
}

Bridge::Bridge(Stream& io)
  : _io(io), _rxLen(0) {
  memset(_axisUsed, 0, sizeof(_axisUsed));
  memset(_imuUsed, 0, sizeof(_imuUsed));
}

uint16_t Bridge::clamp11(uint16_t id) {
  return id & MAX_CAN_ID;
}

void Bridge::poll() {
  while (_io.available()) {
    if (_rxLen < RX_BUF_SIZE) {
      _rxBuf[_rxLen++] = static_cast<uint8_t>(_io.read());
    } else {
      _rxLen = 0;
    }
  }
  if (_rxLen >= HDR_SIZE) {
    processBuf();
  }
}

void Bridge::sendFrame(uint16_t canId, Cmd cmd, const uint8_t* payload, uint8_t len) {
  if (canId > MAX_CAN_ID || len > MAX_PAYLOAD_SIZE) return;
  uint8_t hdr[HDR_SIZE];
  hdr[0] = uint8_t(canId & 0xFF);
  hdr[1] = uint8_t((canId >> 8) & 0xFF);
  hdr[2] = static_cast<uint8_t>(cmd);
  hdr[3] = len;
  _io.write(hdr, HDR_SIZE);
  if (payload && len) _io.write(payload, len);
  _io.flush();
}

void Bridge::sendU16(uint16_t canId, Cmd cmd, uint16_t value) {
  uint8_t b[2];
  b[0] = uint8_t(value & 0xFF);
  b[1] = uint8_t((value >> 8) & 0xFF);
  sendFrame(canId, cmd, b, 2);
}

void Bridge::sendF32(uint16_t canId, Cmd cmd, float value) {
  uint8_t b[4];
  memcpy(b, &value, 4);
  sendFrame(canId, cmd, b, 4);
}

void Bridge::sendBool(uint16_t canId, Cmd cmd, bool value) {
  uint8_t b = value ? 1 : 0;
  sendFrame(canId, cmd, &b, 1);
}

void Bridge::processBuf() {
  size_t idx = 0;
  while (_rxLen - idx >= HDR_SIZE) {
    uint16_t canId = uint16_t(_rxBuf[idx]) | (uint16_t(_rxBuf[idx+1]) << 8);
    uint8_t cmd    = _rxBuf[idx+2];
    uint8_t len    = _rxBuf[idx+3];
    size_t total   = HDR_SIZE + len;
    if (_rxLen - idx < total) break;

    const uint8_t* payload = &_rxBuf[idx + HDR_SIZE];

    if (cmd == TELEMETRY_CMD) {
      handleTelemetry(canId, payload, len);
    } else if (cmd == GET_CONFIG_CMD) {
      handleConfig(canId, payload, len);
    } else if (cmd == IMU_TELEMETRY_CMD) {
      handleImuTelemetry(canId, payload, len);
    }

    idx += total;
  }

  if (idx && idx < _rxLen) {
    memmove(_rxBuf, _rxBuf + idx, _rxLen - idx);
  }
  _rxLen -= idx;
}

int Bridge::findAxisIndex(uint16_t canId) const {
  for (size_t i = 0; i < MAX_AXES; ++i) {
    if (_axisUsed[i] && _axes[i].config.canArbId == canId) return int(i);
  }
  return -1;
}

int Bridge::findImuIndex(uint16_t canId) const {
  (void)canId;
  for (size_t i = 0; i < MAX_AXES; ++i) {
    if (_imuUsed[i]) {
      // Just return first slot for now.
      return int(i);
    }
  }
  return -1;
}

int Bridge::allocAxisIndex(uint16_t canId) {
  int idx = findAxisIndex(canId);
  if (idx >= 0) return idx;
  for (size_t i = 0; i < MAX_AXES; ++i) {
    if (!_axisUsed[i]) {
      _axisUsed[i] = true;
      _axes[i].config.canArbId = canId;
      return int(i);
    }
  }
  return -1;
}

int Bridge::allocImuIndex(uint16_t canId) {
  (void)canId;
  for (size_t i = 0; i < MAX_AXES; ++i) {
    if (!_imuUsed[i]) {
      _imuUsed[i] = true;
      _imus[i].timestampMs = millisNow();
      return int(i);
    }
    // If used, just reuse index 0 for simplicity if we only have one IMU.
    if (_imuUsed[i] && i == 0) return 0;
  }
  return -1;
}

bool Bridge::parseAxisConfig(const uint8_t* b, size_t len, AxisConfig& out) const {
  if (len < AXIS_CONFIG_SIZE) return false;
  size_t o = 0;
  memcpy(&out.crc32, b + o, 4); o += 4;
  memcpy(&out.microsteps, b + o, 2); o += 2;
  memcpy(&out.stepsPerRev, b + o, 2); o += 2;
  out.units = b[o]; o += 1;
  uint16_t flags_u16;
  memcpy(&flags_u16, b + o, 2); o += 2;
  memcpy(&out.encZeroCounts, b + o, 2); o += 2;
  memcpy(&out.driver_mA, b + o, 2); o += 2;
  memcpy(&out.maxRPS,  b + o, 4); o += 4;
  memcpy(&out.maxRPS2, b + o, 4); o += 4;
  memcpy(&out.Kp,      b + o, 4); o += 4;
  memcpy(&out.Ki,      b + o, 4); o += 4;
  memcpy(&out.Kd,      b + o, 4); o += 4;
  memcpy(&out.canArbId, b + o, 2); o += 2;

  AxisFlags f;
  f.encInvert              = (flags_u16 & 0x01) != 0;
  f.dirInvert              = (flags_u16 & 0x02) != 0;
  f.stealthChop            = (flags_u16 & 0x04) != 0;
  f.externalMode           = (flags_u16 & 0x08) != 0;
  f.enableEndStop          = (flags_u16 & 0x10) != 0;
  f.externalEncoder        = (flags_u16 & 0x20) != 0;
  f.calibratedOnce         = (flags_u16 & 0x40) != 0;
  // bit 0x80 used to be externalSPI, now activeLow
  f.limitSwitchActiveLow   = (flags_u16 & 0x80) != 0;
  
  out.flags = f;
  return true;
}

bool Bridge::parseAxisTelemetry(const uint8_t* b, size_t len, AxisState& out) const {
  if (len < AXIS_CONFIG_SIZE + TELEM_TAIL_SIZE) return false;

  if (!parseAxisConfig(b, len, out.config)) return false;

  size_t offset = AXIS_CONFIG_SIZE;
  if (offset % 4 != 0) offset += (4 - (offset % 4));
  if (offset + TELEM_TAIL_SIZE > len) return false;

  const uint8_t* p = b + offset;
  memcpy(&out.currentSpeed, p, 4); p += 4;
  memcpy(&out.currentAngle, p, 4); p += 4;
  memcpy(&out.targetAngle,  p, 4); p += 4;
  memcpy(&out.temperature,  p, 4); p += 4;
  uint8_t stalled_u8   = *p++;
  uint8_t tuneState_u8 = *p++;
  uint8_t minT_u8      = *p++;
  uint8_t maxT_u8      = *p++;

  out.stalled     = stalled_u8 != 0;
  out.minTriggered = minT_u8 != 0;
  out.maxTriggered = maxT_u8 != 0;
  if (tuneState_u8 <= uint8_t(TuningState::DONE)) {
    out.tuneState = static_cast<TuningState>(tuneState_u8);
  } else {
    out.tuneState = TuningState::IDLE;
  }
  out.timestampMs = millisNow();
  return true;
}

bool Bridge::parseImuTelemetry(const uint8_t* b, size_t len, ImuState& out) const {
  if (len < IMU_PAYLOAD_SIZE) return false;
  const uint8_t* p = b;
  memcpy(&out.roll,  p, 4); p += 4;
  memcpy(&out.pitch, p, 4); p += 4;
  memcpy(&out.yaw,   p, 4); p += 4;
  memcpy(&out.ax,    p, 4); p += 4;
  memcpy(&out.ay,    p, 4); p += 4;
  memcpy(&out.az,    p, 4); p += 4;
  memcpy(&out.temp,  p, 4); p += 4;
  out.timestampMs = millisNow();
  return true;
}

void Bridge::handleTelemetry(uint16_t id, const uint8_t* payload, uint8_t len) {
  AxisState st;
  if (!parseAxisTelemetry(payload, len, st)) return;
  int idx = allocAxisIndex(st.config.canArbId);
  if (idx < 0) return;
  _axes[idx] = st;
}

void Bridge::handleConfig(uint16_t id, const uint8_t* payload, uint8_t len) {
  AxisConfig cfg;
  if (!parseAxisConfig(payload, len, cfg)) return;
  int idx = allocAxisIndex(cfg.canArbId);
  if (idx < 0) return;
  _axes[idx].config = cfg;
}

void Bridge::handleImuTelemetry(uint16_t id, const uint8_t* payload, uint8_t len) {
  ImuState st;
  if (!parseImuTelemetry(payload, len, st)) return;
  int idx = allocImuIndex(id);
  if (idx < 0) return;
  _imus[idx] = st;
}

bool Bridge::getAxisState(uint16_t canId, AxisState& out) const {
  int idx = findAxisIndex(canId);
  if (idx < 0) return false;
  out = _axes[idx];
  return true;
}

bool Bridge::getImuState(uint16_t canId, ImuState& out) const {
  (void)canId;
  for (size_t i = 0; i < MAX_AXES; ++i) {
    if (_imuUsed[i]) {
      out = _imus[i];
      return true;
    }
  }
  return false;
}

void Bridge::requestConfig(uint16_t canId) {
  sendFrame(clamp11(canId), Cmd::GET_CONFIG, nullptr, 0);
}

void Bridge::setTargetAngle(uint16_t canId, float angle) {
  sendF32(clamp11(canId), Cmd::TARGET_ANGLE, angle);
}

void Bridge::setCurrentMA(uint16_t canId, uint16_t mA) {
  sendU16(clamp11(canId), Cmd::SET_CURRENT_MA, mA);
}

void Bridge::setSpeedLimitRps(uint16_t canId, float rps) {
  sendF32(clamp11(canId), Cmd::SET_SPEED_LIMIT, rps);
}

void Bridge::setAccelLimitRps2(uint16_t canId, float rps2) {
  sendF32(clamp11(canId), Cmd::SET_ACCEL_LIMIT, rps2);
}

void Bridge::setPid(uint16_t canId, float kp, float ki, float kd) {
  uint8_t b[12];
  memcpy(b,      &kp, 4);
  memcpy(b + 4,  &ki, 4);
  memcpy(b + 8,  &kd, 4);
  sendFrame(clamp11(canId), Cmd::SET_PID, b, 12);
}

void Bridge::setCanId(uint16_t canId, uint16_t newId) {
  sendU16(clamp11(canId), Cmd::SET_ID, clamp11(newId));
}

void Bridge::setMicrosteps(uint16_t canId, uint16_t microsteps) {
  sendU16(clamp11(canId), Cmd::SET_MICROSTEPS, microsteps);
}

void Bridge::setStealthChop(uint16_t canId, bool enable) {
  sendBool(clamp11(canId), Cmd::SET_STEALTHCHOP, enable);
}

void Bridge::setExternalMode(uint16_t canId, bool enable) {
  sendBool(clamp11(canId), Cmd::SET_EXT_MODE, enable);
}

void Bridge::setUnitsDegrees(uint16_t canId, bool useDegrees) {
  uint8_t v = useDegrees ? 1 : 0;
  sendFrame(clamp11(canId), Cmd::SET_UNITS, &v, 1);
}

void Bridge::setEncoderInvert(uint16_t canId, bool enable) {
  sendBool(clamp11(canId), Cmd::SET_ENC_INVERT, enable);
}

void Bridge::setDirectionInvert(uint16_t canId, bool invert) {
  sendBool(clamp11(canId), Cmd::SET_DIR_INVERT, invert);
}

void Bridge::enableMotor(uint16_t canId, bool enable) {
  sendBool(clamp11(canId), Cmd::SET_ENABLED, enable);
}

void Bridge::setStepsPerRev(uint16_t canId, uint16_t stepsPerRev) {
  sendU16(clamp11(canId), Cmd::SET_STEPS_PER_REV, stepsPerRev);
}

void Bridge::setExternalEncoder(uint16_t canId, bool enable) {
  sendBool(clamp11(canId), Cmd::SET_EXT_ENCODER, enable);
}

void Bridge::setEndstop(uint16_t canId, bool enable) {
  sendBool(clamp11(canId), Cmd::SET_ENDSTOP, enable);
}

void Bridge::setLimitSwitchActiveLow(uint16_t canId, bool activeLow) {
  sendBool(clamp11(canId), Cmd::SET_LIMITSWITCH_ACTIVELOW, activeLow);
}

void Bridge::doCalibrate(uint16_t canId) {
  sendFrame(clamp11(canId), Cmd::DO_CALIBRATE, nullptr, 0);
}

void Bridge::doHoming(uint16_t canId, const HomingParams& p) {
  uint16_t cur = p.homingCurrent;
  if (cur > 0xFFFF) cur = 0xFFFF;

  uint8_t buf[14];
  uint8_t* b = buf;
  uint8_t useIn1 = (p.useIN1Trigger && !p.sensorlessHoming) ? 1 : 0;
  uint8_t sensorless = p.sensorlessHoming ? 1 : 0;
  uint8_t activeLow = (p.activeLow && !p.sensorlessHoming) ? 1 : 0;

  *b++ = useIn1;
  *b++ = sensorless;
  *b++ = uint8_t(cur & 0xFF);
  *b++ = uint8_t((cur >> 8) & 0xFF);
  memcpy(b, &p.offset, 4); b += 4;
  *b++ = activeLow;
  memcpy(b, &p.speed, 4); b += 4;
  *b++ = p.direction ? 1 : 0;

  sendFrame(clamp11(canId), Cmd::DO_HOMING, buf, uint8_t(b - buf));
}

void Bridge::doAutoTune(uint16_t canId, float minAngle, float maxAngle) {
  uint8_t b[8];
  memcpy(b,      &minAngle, 4);
  memcpy(b + 4,  &maxAngle, 4);
  sendFrame(clamp11(canId), Cmd::DO_AUTO_TUNE, b, 8);
}

void Bridge::setImuId(uint16_t currentId, uint16_t newId) {
  sendU16(clamp11(currentId), Cmd::SET_IMU_ID, clamp11(newId));
}

void Bridge::resetOrientation(uint16_t canId) {
  sendFrame(clamp11(canId), Cmd::RESET_ORIENT, nullptr, 0);
}

/* Stepper */

Stepper::Stepper(Bridge& b, uint16_t canId)
  : _bridge(b), _id(Bridge::clamp11(canId)) {}

void Stepper::setId(uint16_t newId) {
  _bridge.setCanId(_id, newId);
  _id = Bridge::clamp11(newId);
}

void Stepper::requestConfig()            { _bridge.requestConfig(_id); }
void Stepper::enableMotor(bool en)       { _bridge.enableMotor(_id, en); }
void Stepper::setTargetAngle(float ang)  { _bridge.setTargetAngle(_id, ang); }
void Stepper::setCurrentMA(uint16_t mA)  { _bridge.setCurrentMA(_id, mA); }
void Stepper::setSpeedLimitRps(float rps){ _bridge.setSpeedLimitRps(_id, rps); }
void Stepper::setAccelLimitRps2(float a) { _bridge.setAccelLimitRps2(_id, a); }
void Stepper::setPid(float kp,float ki,float kd){ _bridge.setPid(_id,kp,ki,kd); }
void Stepper::setMicrosteps(uint16_t m)  { _bridge.setMicrosteps(_id, m); }
void Stepper::setStealthChop(bool en)    { _bridge.setStealthChop(_id, en); }
void Stepper::setExternalMode(bool en)   { _bridge.setExternalMode(_id, en); }
void Stepper::setUnitsDegrees(bool on)   { _bridge.setUnitsDegrees(_id, on); }
void Stepper::setEncoderInvert(bool on)  { _bridge.setEncoderInvert(_id, on); }
void Stepper::setDirectionInvert(bool on){ _bridge.setDirectionInvert(_id,on); }
void Stepper::setExternalEncoder(bool on){ _bridge.setExternalEncoder(_id,on);}
void Stepper::setEndstop(bool on)        { _bridge.setEndstop(_id, on); }
void Stepper::setLimitSwitchActiveLow(bool a){ _bridge.setLimitSwitchActiveLow(_id,a);}
void Stepper::doCalibrate()              { _bridge.doCalibrate(_id); }
void Stepper::doHoming(const HomingParams& p){ _bridge.doHoming(_id, p); }
void Stepper::doAutoTune(float mn,float mx){ _bridge.doAutoTune(_id,mn,mx); }

bool Stepper::getAxisState(AxisState& out) const {
  return _bridge.getAxisState(_id, out);
}

/* IMU */

IMU::IMU(Bridge& b, uint16_t controlId)
  : _bridge(b), _id(Bridge::clamp11(controlId)) {}

void IMU::setId(uint16_t newId) {
  _bridge.setImuId(_id, newId);
  _id = Bridge::clamp11(newId);
}

void IMU::resetOrientation() {
  _bridge.resetOrientation(_id);
}

bool IMU::getState(ImuState& out) const {
  return _bridge.getImuState(_id, out);
}

bool IMU::getRoll(float& out) const {
  ImuState st;
  if (!getState(st)) return false;
  out = st.roll; return true;
}
bool IMU::getPitch(float& out) const {
  ImuState st;
  if (!getState(st)) return false;
  out = st.pitch; return true;
}
bool IMU::getYaw(float& out) const {
  ImuState st;
  if (!getState(st)) return false;
  out = st.yaw; return true;
}
bool IMU::getAccelX(float& out) const {
  ImuState st;
  if (!getState(st)) return false;
  out = st.ax; return true;
}
bool IMU::getAccelY(float& out) const {
  ImuState st;
  if (!getState(st)) return false;
  out = st.ay; return true;
}
bool IMU::getAccelZ(float& out) const {
  ImuState st;
  if (!getState(st)) return false;
  out = st.az; return true;
}
bool IMU::getTemperature(float& out) const {
  ImuState st;
  if (!getState(st)) return false;
  out = st.temp; return true;
}

} // namespace Tercio