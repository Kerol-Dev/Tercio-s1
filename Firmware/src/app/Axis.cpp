#include "app/Axis.h"

#include <cmath>

#include "drivers/Analog.h"
#include "drivers/CanBus.h"
#include "drivers/ControlTimer.h"
#include "drivers/Inputs.h"
#include "drivers/StepGenerator.h"
#include "drivers/Tmc2209.h"
#include "encoder/Encoder.h"

namespace {
using namespace proto;

constexpr uint16_t kHardFaults = fault::kHardMask;
constexpr float kTemperatureWarningMarginC = 10.0f;
constexpr float kTemperatureClearMarginC = 5.0f;
constexpr float kSupplyLowVolts = 8.0f;
constexpr float kStillVelocity = 0.02f;  // turns/s of controller output that still counts as standing
constexpr uint16_t kDiagDebounceMs = 5;

float orDefault(float requested, float limit) { return requested > 0.0f ? std::fmin(requested, limit) : limit; }
}  // namespace

// ---------------------------------------------------------------- setup ------

void Axis::begin(uint32_t nowMs) {
  nowMs_ = lastHostMs_ = nowMs;
  if (!selectEncoder()) raise(fault::kEncoder);
  applySettings();
  if (!hw_.driver.status().commOk) raise(fault::kDriverComm);
  if (settings_.enableOnBoot && !(faults_ & kHardFaults)) enable(true);
}

Encoder* Axis::encoder() const {
  return settings_.encoderType <= static_cast<uint8_t>(EncoderType::As5048a) ? hw_.encoders[settings_.encoderType]
                                                                               : nullptr;
}

bool Axis::selectEncoder() {
  if (activeEncoder_ >= 0) hw_.encoders[activeEncoder_]->end();
  activeEncoder_ = -1;
  homed_ = false;  // position tracking restarts from the single-turn reading
  Encoder* e = encoder();
  if (e == nullptr) return false;
  activeEncoder_ = static_cast<int8_t>(settings_.encoderType);
  const bool ok = e->begin();
  hw_.core.setEncoder(e, settings_.encoderZeroRaw);
  return ok;
}

void Axis::applySettings() {
  const Settings& s = settings_;
  const float motorSteps = static_cast<float>(s.fullStepsPerRev) * static_cast<float>(s.microsteps);
  const float stepsPerTurn = motorSteps * s.gearRatio;

  MotionCore::Config c;
  c.motorStepsPerTurn = motorSteps;
  c.stepsPerTurn = stepsPerTurn;
  c.direction = s.invertDirection ? -1 : 1;
  c.encoderSign = s.encoderInvert ? -1 : 1;
  c.gains.kp = s.kp;
  c.gains.ki = s.ki;
  c.gains.kd = s.kd;
  c.gains.deadband = s.positionDeadband;
  c.gains.integralLimit = 0.5f * s.maxVelocity;
  // Room above the profile speed to catch up, but never beyond the step generator.
  // While auto-tuning, the profile itself exceeds the configured limits.
  const bool tuning = procedure_ == Procedure::Tuning;
  const float stepLimit = StepGenerator::kMaxRate / stepsPerTurn;
  c.gains.maxVelocity = tuning ? stepLimit : std::fmin(2.0f * s.maxVelocity, stepLimit);
  c.gains.maxAcceleration = 4.0f * (tuning ? LimitTuner::kMaxAcceleration : s.maxAcceleration);
  c.followingErrorLimit = s.followingErrorLimit;
  c.stallTicks = static_cast<uint32_t>(s.stallTimeoutMs) * (control_timer::kRateHz / 1000u);
  c.softLimits = s.softLimitsActive();
  c.softMin = q32::fromTurns(s.softLimitMin);
  c.softMax = q32::fromTurns(s.softLimitMax);
  c.limitSwitches = s.limitSwitchesEnabled;
  c.stepDirIncrement = q32::fromTurns(1.0 / static_cast<double>(stepsPerTurn));
  hw_.core.configure(c);

  // A direction flip mirrors the frame around the current position; keep the
  // persisted zero consistent with it.
  const int8_t sign = static_cast<int8_t>(c.direction * c.encoderSign);
  if (sign != appliedSign_) {
    appliedSign_ = sign;
    if (hw_.core.snapshot().tracking) settings_.encoderZeroRaw = hw_.core.zeroRaw();
  }

  inputs::setLimitPolarity(s.limitSwitchActiveLow);
  inputs::setStepDirEnabled(s.stepDirMode);

  if (activeEncoder_ != static_cast<int8_t>(s.encoderType)) {
    // Parameter rules only allow this while disabled. The old calibration and
    // zero belong to the old sensor.
    settings_.calibrated = false;
    settings_.encoderZeroRaw = 0;
    if (!selectEncoder()) raise(fault::kEncoder);
  }
  updateDriver();

  if (s.stepDirMode != appliedStepDirMode_) {
    appliedStepDirMode_ = s.stepDirMode;
    if (enabled_ && procedure_ == Procedure::None && !(faults_ & kHardFaults)) resumeNormalMode();
  }
}

void Axis::updateDriver() {
  const Settings& s = settings_;
  Tmc2209::Config d;
  d.microsteps = s.microsteps;
  d.runCurrentMa = (procedure_ == Procedure::Homing && homing_.reducedCurrent()) ? s.homingCurrentMa : s.runCurrentMa;
  d.holdCurrentPct = s.holdCurrentPct;
  d.stealthChop = s.stealthChop;
  d.stealthChopMaxFullStepsPerSec = s.stealthChopMaxVel * static_cast<float>(s.fullStepsPerRev);
  hw_.driver.apply(d);
}

// ---------------------------------------------------------------- service ----

void Axis::service(uint32_t nowMs) {
  nowMs_ = nowMs;
  snapshot_ = hw_.core.snapshot();
  const uint8_t events = hw_.core.takeEvents();
  superviseFaults(nowMs, events);

  if (procedure_ != Procedure::None) {
    ProcedureContext ctx{hw_.core, settings_, snapshot_, events, nowMs};
    ProcedureResult result = ProcedureResult::Running;
    switch (procedure_) {
      case Procedure::Calibration: result = calibration_.service(ctx); break;
      case Procedure::Homing: result = homing_.service(ctx); break;
      case Procedure::Tuning: result = tuner_.service(ctx); break;
      case Procedure::None: break;
    }
    if (result != ProcedureResult::Running) finishProcedure(result);
    return;
  }

  if (!enabled_ || (faults_ & kHardFaults)) return;

  // The control tick already stopped and holds. In step/dir mode it stays
  // holding until ClearFaults re-arms the input.
  if (events & MotionCore::kEventStall) raise(fault::kStall);

  // External enable input gates the bridge in step/dir mode.
  if (settings_.stepDirMode && settings_.calibrated) {
    const bool open = inputs::stepDirEnable(settings_.stepDirEnableActiveLow);
    if (open != extGateOpen_) {
      extGateOpen_ = open;
      if (open)
        hw_.core.followStepDir();
      else
        hw_.core.setOff();
    }
  }

  // Command timeout: a host that goes quiet mid-motion stops the axis.
  const AxisState st = state();
  if (settings_.commandTimeoutMs && (st == AxisState::Moving || st == AxisState::Velocity) &&
      nowMs - lastHostMs_ > settings_.commandTimeoutMs) {
    hw_.core.stop(settings_.maxAcceleration);
    raise(fault::kCommandTimeout);
  }
}

void Axis::serviceSlow() {
  if (activeEncoder_ >= 0) hw_.encoders[activeEncoder_]->service();
}

void Axis::superviseFaults(uint32_t nowMs, uint8_t events) {
  const Settings& s = settings_;
  const Tmc2209::Status& driver = hw_.driver.status();
  const float temperature = analog::temperatureC();
  const Encoder* e = encoder();
  const MagnetStatus magnet = e ? e->magnet() : MagnetStatus{};

  if (events & MotionCore::kEventEncoderLost) raise(fault::kEncoder);
  if (snapshot_.tracking && !magnet.detected) raise(fault::kEncoder);
  if (!driver.commOk) raise(fault::kDriverComm);
  if (driver.overTemp) raise(fault::kDriverOverTemp);
  // DIAG reacts within a millisecond; DRV_STATUS (polled over UART) names the cause.
  diagMs_ = inputs::driverDiag() ? static_cast<uint16_t>(diagMs_ + 1) : 0;
  if (driver.shortCircuit || diagMs_ >= kDiagDebounceMs) raise(fault::kDriverFault);
  if (temperature >= s.overTemperatureC) raise(fault::kOverTemperature);

  uint16_t w = 0;
  if (temperature >= s.overTemperatureC - kTemperatureWarningMarginC) w |= warn::kTemperatureHigh;
  if (driver.preWarning) w |= warn::kDriverPreWarning;
  if (magnet.tooWeak) w |= warn::kMagnetWeak;
  if (magnet.tooStrong) w |= warn::kMagnetStrong;
  if (analog::supplyVolts() < kSupplyLowVolts) w |= warn::kSupplyLow;
  if (can_bus::errorPassive()) w |= warn::kCanErrorPassive;
  if (driver.openLoad && enabled_) w |= warn::kOpenLoad;
  warnings_ = w;
  (void)nowMs;
}

void Axis::raise(uint16_t fault) {
  if ((faults_ & fault) == fault) return;
  faults_ |= fault;
  if (fault & fault::kEncoder) homed_ = false;  // samples were missed: the frame is suspect
  if (fault & kHardFaults) {
    procedure_ = Procedure::None;
    pending_.valid = false;
    enabled_ = false;
    hw_.core.setOff();
    applySettings();  // drops any procedure-specific limits and currents
  }
}

// ---------------------------------------------------------------- commands ---

proto::Status Axis::enable(bool on) {
  if (!on) {
    const bool abortedProcedure = procedure_ != Procedure::None;
    procedure_ = Procedure::None;
    pending_.valid = false;
    enabled_ = false;
    hw_.core.setOff();
    if (abortedProcedure) applySettings();
    return Status::Ok;
  }
  if (faults_ & kHardFaults) return Status::Faulted;
  if (!enabled_) {
    enabled_ = true;
    resumeNormalMode();
  }
  return Status::Ok;
}

void Axis::resumeNormalMode() {
  if (!enabled_) {
    hw_.core.setOff();
  } else if (!settings_.calibrated) {
    hw_.core.setIdle();  // energised; closing the loop needs a known encoder direction
  } else if (settings_.stepDirMode) {
    extGateOpen_ = inputs::stepDirEnable(settings_.stepDirEnableActiveLow);
    if (extGateOpen_)
      hw_.core.followStepDir();
    else
      hw_.core.setOff();
  } else {
    hw_.core.hold();
  }
}

proto::Status Axis::checkMotionAllowed() const {
  if (faults_ & kHardFaults) return Status::Faulted;
  if (!enabled_) return Status::Disabled;
  if (!settings_.calibrated) return Status::NotCalibrated;
  if (procedure_ != Procedure::None || settings_.stepDirMode) return Status::Busy;
  return Status::Ok;
}

proto::Status Axis::startMove(q32::Pos target, float maxVelocity, float maxAcceleration, bool deferred) {
  const Status status = checkMotionAllowed();
  if (status != Status::Ok) return status;
  faults_ &= static_cast<uint16_t>(~(fault::kStall | fault::kCommandTimeout));
  const float v = orDefault(maxVelocity, settings_.maxVelocity);
  const float a = orDefault(maxAcceleration, settings_.maxAcceleration);
  if (deferred) {
    pending_ = {true, target, v, a};
    return Status::Ok;
  }
  hw_.core.moveTo(target, v, a);
  return Status::Ok;
}

proto::Status Axis::moveTo(double position, float maxVelocity, float maxAcceleration, bool deferred) {
  return startMove(q32::fromTurns(position), maxVelocity, maxAcceleration, deferred);
}

proto::Status Axis::moveBy(double delta, float maxVelocity, float maxAcceleration, bool deferred) {
  // Relative to where the axis is heading (read live, not from the 1 ms
  // snapshot), so back-to-back MoveBy commands add up.
  const MotionCore::Snapshot s = hw_.core.snapshot();
  const bool heading = s.mode == MotionCore::Mode::Closed && !s.velocityMode;
  const q32::Pos base = pending_.valid ? pending_.target : (heading ? s.target : s.position);
  return startMove(q32::fromTurns(q32::toTurns(base) + delta), maxVelocity, maxAcceleration, deferred);  // saturating
}

proto::Status Axis::setVelocity(float velocity, float maxAcceleration) {
  const Status status = checkMotionAllowed();
  if (status != Status::Ok) return status;
  faults_ &= static_cast<uint16_t>(~(fault::kStall | fault::kCommandTimeout));
  const float a = orDefault(maxAcceleration, settings_.maxAcceleration);
  const float v = std::fmax(-settings_.maxVelocity, std::fmin(settings_.maxVelocity, velocity));
  pending_.valid = false;
  if (v == 0.0f)
    hw_.core.stop(a);
  else
    hw_.core.runAt(v, a);
  return Status::Ok;
}

proto::Status Axis::stop() {
  if (procedure_ != Procedure::None) {
    procedure_ = Procedure::None;  // aborted by the host: no fault
    applySettings();                // run current and limits back to normal
    resumeNormalMode();
  }
  pending_.valid = false;
  hw_.core.stop(settings_.maxAcceleration);
  return Status::Ok;
}

proto::Status Axis::emergencyStop() { return enable(false); }

proto::Status Axis::setZero(double position) {
  if (procedure_ != Procedure::None || !isStill()) return Status::Busy;
  hw_.core.setPosition(q32::fromTurns(position));
  settings_.encoderZeroRaw = hw_.core.zeroRaw();
  return Status::Ok;
}

void Axis::sync() {
  if (!pending_.valid || checkMotionAllowed() != Status::Ok) return;
  hw_.core.moveTo(pending_.target, pending_.maxVelocity, pending_.maxAcceleration);
  pending_.valid = false;
}

proto::Status Axis::calibrate() {
  if (faults_ & kHardFaults) return Status::Faulted;
  if (procedure_ != Procedure::None) return Status::Busy;
  if (!snapshot_.tracking) return Status::Faulted;  // no encoder samples to calibrate against
  enabled_ = true;
  startProcedure(Procedure::Calibration);
  return Status::Ok;
}

proto::Status Axis::home() {
  const Status status = checkMotionAllowed();
  if (status != Status::Ok) return status;
  startProcedure(Procedure::Homing);
  return Status::Ok;
}

proto::Status Axis::autoTune(float min, float max) {
  if (!(max - min >= LimitTuner::minimumRange())) return Status::BadValue;  // must reach the first level
  const Status status = checkMotionAllowed();
  if (status != Status::Ok) return status;
  tunerMin_ = q32::fromTurns(min);
  tunerMax_ = q32::fromTurns(max);
  startProcedure(Procedure::Tuning);
  return Status::Ok;
}

proto::Status Axis::clearFaults() {
  const Tmc2209::Status& driver = hw_.driver.status();
  if (!driver.commOk || driver.overTemp || driver.shortCircuit || inputs::driverDiag()) return Status::Faulted;
  if (analog::temperatureC() >= settings_.overTemperatureC - kTemperatureClearMarginC) return Status::Faulted;
  if ((faults_ & fault::kEncoder) && !selectEncoder()) return Status::Faulted;
  faults_ = 0;
  // A stall in step/dir mode left the axis holding: re-arm the input.
  if (enabled_ && procedure_ == Procedure::None && settings_.stepDirMode) resumeNormalMode();
  return Status::Ok;
}

// ---------------------------------------------------------------- procedures -

void Axis::startProcedure(Procedure procedure) {
  procedure_ = procedure;
  pending_.valid = false;
  faults_ &= static_cast<uint16_t>(~(fault::kStall | fault::kCommandTimeout | fault::kCalibrationFailed |
                                     fault::kHomingFailed));
  ProcedureContext ctx{hw_.core, settings_, snapshot_, 0, nowMs_};
  switch (procedure) {
    case Procedure::Calibration: calibration_.start(ctx, encoder()->resolutionBits()); break;
    case Procedure::Homing:
      homed_ = false;
      homing_.start(ctx);
      break;
    case Procedure::Tuning: tuner_.start(ctx, tunerMin_, tunerMax_); break;
    case Procedure::None: break;
  }
  applySettings();  // homing current, tuning limits
}

void Axis::finishProcedure(ProcedureResult result) {
  const Procedure finished = procedure_;
  procedure_ = Procedure::None;
  const bool ok = result == ProcedureResult::Succeeded;

  switch (finished) {
    case Procedure::Calibration:
      if (ok) {
        settings_.encoderInvert = calibration_.encoderInverted();
        settings_.calibrated = true;
        homed_ = false;  // new zero, new frame
        applySettings();
        hw_.core.setPosition(0);
        settings_.encoderZeroRaw = hw_.core.zeroRaw();
        saveRequested_ = true;
      } else {
        faults_ |= fault::kCalibrationFailed;
      }
      break;
    case Procedure::Homing:
      if (ok) {
        homed_ = true;
        settings_.encoderZeroRaw = hw_.core.zeroRaw();
      } else {
        faults_ |= fault::kHomingFailed;
      }
      break;
    case Procedure::Tuning:
      if (ok) {
        settings_.maxVelocity = tuner_.velocity();
        settings_.maxAcceleration = tuner_.acceleration();
        saveRequested_ = true;
      } else {
        faults_ |= fault::kStall;
      }
      break;
    case Procedure::None:
      break;
  }
  applySettings();  // run current and limits back to normal (and any new tuning result)
  resumeNormalMode();
}

// ---------------------------------------------------------------- status -----

proto::AxisState Axis::state() const {
  if (faults_ & kHardFaults) return AxisState::Fault;
  if (!enabled_) return AxisState::Disabled;
  switch (procedure_) {
    case Procedure::Calibration: return AxisState::Calibrating;
    case Procedure::Homing: return AxisState::Homing;
    case Procedure::Tuning: return AxisState::Tuning;
    case Procedure::None: break;
  }
  if (settings_.stepDirMode && settings_.calibrated) return AxisState::StepDir;
  if (snapshot_.velocityMode) return AxisState::Velocity;
  if (!snapshot_.trajectoryDone) return AxisState::Moving;
  return AxisState::Holding;
}

uint8_t Axis::flags() const {
  uint8_t f = 0;
  if (enabled_) f |= flag::kEnabled;
  if (settings_.calibrated) f |= flag::kCalibrated;
  if (homed_) f |= flag::kHomed;
  if (snapshot_.trajectoryDone && snapshot_.settled) f |= flag::kSettled;
  if (snapshot_.limitMin) f |= flag::kLimitMin;
  if (snapshot_.limitMax) f |= flag::kLimitMax;
  if (pending_.valid) f |= flag::kMovePending;
  if (inputs::stepDirEnable(settings_.stepDirEnableActiveLow)) f |= flag::kExtEnable;
  return f;
}

uint8_t Axis::procedureStep() const {
  switch (procedure_) {
    case Procedure::Calibration: return calibration_.step();
    case Procedure::Homing: return homing_.step();
    case Procedure::Tuning: return tuner_.step();
    case Procedure::None: break;
  }
  return 0;
}

bool Axis::isStill() const {
  if (procedure_ != Procedure::None) return false;
  if (!enabled_) return true;
  const MotionCore::Snapshot s = hw_.core.snapshot();  // live: commands in this pass count
  return !s.velocityMode && s.mode != MotionCore::Mode::StepDir && s.trajectoryDone &&
         std::fabs(s.output) < kStillVelocity;
}

bool Axis::takeSaveRequest() {
  const bool requested = saveRequested_;
  saveRequested_ = false;
  return requested;
}
