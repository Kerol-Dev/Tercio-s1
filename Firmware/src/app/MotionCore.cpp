#include "app/MotionCore.h"

#include <cmath>

#include "drivers/ControlTimer.h"
#include "drivers/Inputs.h"
#include "drivers/Irq.h"
#include "drivers/StepGenerator.h"
#include "encoder/Encoder.h"

namespace {

constexpr float kDt = control_timer::kPeriod;
constexpr uint8_t kDebounceTicks = 10;         // 0.5 ms for limit switches
constexpr uint32_t kEncoderStaleTicks = 100;   // 5 ms without a sample: sensor lost
constexpr float kTrackerBandwidth = 600.0f;    // rad/s (~95 Hz) velocity estimator
constexpr float kStepDirVelocityAlpha = 0.02f; // ~2.5 ms smoothing of the pulse-rate feed-forward
constexpr float kSeekArmMarginS = 0.05f;       // ignore following error while ramping up

void debounce(uint8_t& counter, bool& state, bool active) {
  if (active) {
    if (counter < kDebounceTicks && ++counter == kDebounceTicks) state = true;
  } else if (counter > 0 && --counter == 0) {
    state = false;
  }
}

int32_t wrapSigned(int32_t value, int32_t cpr) {
  value &= cpr - 1;
  return value >= cpr / 2 ? value - cpr : value;
}

}  // namespace

// ---------------------------------------------------------------- setup ------

void MotionCore::attach(Encoder* encoder, StepGenerator* stepper) {
  stepper_ = stepper;
  setEncoder(encoder, 0);
}

void MotionCore::configure(const Config& config) {
  IrqLock lock;
  const int8_t oldSign = sign_;
  config_ = config;
  sign_ = static_cast<int8_t>(config.direction * config.encoderSign);
  if (sign_ != oldSign) {
    // Keep the reported position continuous across a direction change:
    // measured = oldSign·C − zero  =>  C = oldSign·(measured + zero), zero' = sign·C − measured.
    const q32::Pos countsQ = oldSign > 0 ? measured_ + zero_ : -(measured_ + zero_);
    zero_ = (sign_ > 0 ? countsQ : -countsQ) - measured_;
    if (mode_ != Mode::Off && mode_ != Mode::Idle && mode_ != Mode::OpenLoop) holdHere();
  }
  controller_.setGains(config.gains);
  applyBounds();
}

void MotionCore::setEncoder(Encoder* encoder, uint16_t zeroRaw) {
  IrqLock lock;
  encoder_ = encoder;
  bits_ = encoder ? encoder->resolutionBits() : 12;
  countsToTurns_ = 1.0f / static_cast<float>(int32_t{1} << bits_);
  tracker_.configure(bits_, kDt, kTrackerBandwidth);
  pendingZeroRaw_ = zeroRaw;
  tracking_ = false;
  zero_ = 0;
  if (mode_ != Mode::Off) enter(Mode::Idle);
}

// ---------------------------------------------------------------- tick -------

void MotionCore::tick() {
  // 1. Sense.
  uint16_t raw = 0;
  const bool fresh = encoder_ != nullptr && encoder_->poll(raw);
  if (!tracking_) {
    if (!fresh) {
      stepper_->setRate(0.0f);
      return;
    }
    const int32_t cpr = int32_t{1} << bits_;
    tracker_.reset(wrapSigned(static_cast<int32_t>(raw) - pendingZeroRaw_, cpr), raw);
    zero_ = 0;
    tracking_ = true;
    measured_ = countsToUser(q32::fromCounts(tracker_.counts(), bits_));
    holdHere();
  } else {
    tracker_.update(fresh, raw);
  }

  const q32::Pos counts =
      q32::fromCounts(tracker_.counts(), bits_) + q32::fromDelta(tracker_.extrapolation() * countsToTurns_);
  measured_ = countsToUser(counts);
  measuredVelocity_ = static_cast<float>(sign_) * tracker_.velocity() * countsToTurns_;

  if (tracker_.ticksSinceSample() > kEncoderStaleTicks && mode_ != Mode::Off) {
    events_ |= kEventEncoderLost;
    stepper_->setRate(0.0f);
    return;
  }

  debounce(limitMinCount_, limitMin_, inputs::limitMin());
  debounce(limitMaxCount_, limitMax_, inputs::limitMax());

  if (frozen_) {
    stepper_->setRate(0.0f);
    return;
  }

  // 2. Reference.
  bool moving = false;
  switch (mode_) {
    case Mode::Off:
    case Mode::Idle:
      reference_ = measured_;
      referenceVelocity_ = 0.0f;
      stepper_->setRate(0.0f);
      return;

    case Mode::OpenLoop:
      openLoop_.step(kDt);
      stepper_->setRate(openLoop_.velocity() * config_.motorStepsPerTurn);
      return;

    case Mode::StepDir: {
      const int32_t count = inputs::stepDirCount();
      int32_t pulses = count - lastStepCount_;
      lastStepCount_ = count;
      if (config_.limitSwitches && ((pulses < 0 && limitMin_) || (pulses > 0 && limitMax_))) pulses = 0;
      reference_ += pulses * config_.stepDirIncrement;
      const float pulseVelocity = static_cast<float>(pulses) * q32::diff(config_.stepDirIncrement, 0) / kDt;
      stepDirVelocity_ += kStepDirVelocityAlpha * (pulseVelocity - stepDirVelocity_);
      referenceVelocity_ = stepDirVelocity_;
      moving = pulses != 0 || std::fabs(stepDirVelocity_) > 1e-3f;
      break;
    }

    case Mode::Closed:
    case Mode::Seek:
      if (mode_ == Mode::Closed && config_.limitSwitches &&
          ((limitMin_ && trajectory_.velocity() < 0.0f) || (limitMax_ && trajectory_.velocity() > 0.0f))) {
        events_ |= kEventLimitStop;
        holdHere();
      }
      trajectory_.step(kDt);
      reference_ = trajectory_.position();
      referenceVelocity_ = trajectory_.velocity();
      moving = !trajectory_.done() || referenceVelocity_ != 0.0f;
      break;
  }

  // 3. Supervision: seek triggers and stall detection.
  const float error = q32::diff(reference_, measured_);
  bool errorHandled = false;
  if (mode_ == Mode::Seek) {
    ++seekTicks_;
    uint8_t cause = 0;
    if ((seekTriggers_ & kTriggerLimitMin) && limitMin_) cause = kTriggerLimitMin;
    if ((seekTriggers_ & kTriggerLimitMax) && limitMax_) cause = kTriggerLimitMax;
    if (seekTriggers_ & kTriggerFollowingError) {
      errorHandled = true;
      if (seekTicks_ > seekArmTicks_ && std::fabs(error) > seekErrorThreshold_) cause = kTriggerFollowingError;
    }
    if (cause) {
      seekCause_ = cause;
      events_ |= kEventSeekTriggered;
      holdHere();
      return;
    }
  }
  if (!errorHandled && config_.stallTicks) {
    if (std::fabs(error) > config_.followingErrorLimit) {
      if (++stallCount_ >= config_.stallTicks) {
        stallCount_ = 0;
        events_ |= kEventStall;
        holdHere();
        return;
      }
    } else {
      stallCount_ = 0;
    }
  }

  // 4. Control law -> step rate.
  const float velocity = controller_.update(error, referenceVelocity_, measuredVelocity_, moving, kDt);
  stepper_->setRate(velocity * config_.stepsPerTurn * static_cast<float>(config_.direction));
}

void MotionCore::holdHere() {
  if (mode_ == Mode::Seek || mode_ == Mode::StepDir) mode_ = Mode::Closed;
  trajectory_.reset(measured_);
  reference_ = measured_;
  referenceVelocity_ = 0.0f;
  controller_.reset();
  stallCount_ = 0;
  stepper_->setRate(0.0f);
}

// ---------------------------------------------------------------- commands ---

void MotionCore::enter(Mode mode) {
  const bool wasEnergised = mode_ != Mode::Off;
  mode_ = mode;
  if (mode == Mode::Off) {
    stepper_->enableDriver(false);
  } else if (!wasEnergised) {
    stepper_->enableDriver(true);
  }
  trajectory_.reset(measured_);
  reference_ = measured_;
  referenceVelocity_ = 0.0f;
  controller_.reset();
  stallCount_ = 0;
  stepper_->setRate(0.0f);
}

void MotionCore::applyBounds() { trajectory_.setBounds(config_.softLimits, config_.softMin, config_.softMax); }

void MotionCore::setOff() {
  IrqLock lock;
  enter(Mode::Off);
}

void MotionCore::setIdle() {
  IrqLock lock;
  enter(Mode::Idle);
}

void MotionCore::hold() {
  IrqLock lock;
  enter(Mode::Closed);
  applyBounds();
}

void MotionCore::moveTo(q32::Pos target, float maxVelocity, float maxAcceleration, bool bounded) {
  IrqLock lock;
  if (mode_ != Mode::Closed) enter(Mode::Closed);
  if (bounded)
    applyBounds();
  else
    trajectory_.setBounds(false, 0, 0);
  trajectory_.moveTo(target, maxVelocity, maxAcceleration);
}

void MotionCore::runAt(float velocity, float maxAcceleration) {
  IrqLock lock;
  if (mode_ != Mode::Closed) enter(Mode::Closed);
  applyBounds();
  trajectory_.runAt(velocity, maxAcceleration);
}

void MotionCore::stop(float maxAcceleration) {
  IrqLock lock;
  switch (mode_) {
    case Mode::Seek:
      mode_ = Mode::Closed;
      [[fallthrough]];
    case Mode::Closed:
      trajectory_.stop(maxAcceleration);
      break;
    case Mode::StepDir:
      holdHere();
      break;
    case Mode::OpenLoop:
      enter(Mode::Idle);  // never close the loop from here: the encoder sign may be unknown
      break;
    case Mode::Off:
    case Mode::Idle:
      break;
  }
}

void MotionCore::seek(float velocity, float maxAcceleration, uint8_t triggers, float errorThreshold) {
  IrqLock lock;
  enter(Mode::Seek);
  trajectory_.setBounds(false, 0, 0);  // homing establishes the frame; old limits do not apply
  trajectory_.runAt(velocity, maxAcceleration);
  seekTriggers_ = triggers;
  seekErrorThreshold_ = errorThreshold;
  seekArmTicks_ = static_cast<uint32_t>((std::fabs(velocity) / maxAcceleration + kSeekArmMarginS) / kDt);
  seekTicks_ = 0;
  seekCause_ = 0;
}

void MotionCore::followStepDir() {
  IrqLock lock;
  enter(Mode::StepDir);
  lastStepCount_ = inputs::stepDirCount();
  stepDirVelocity_ = 0.0f;
}

void MotionCore::openLoopMove(float motorTurns, float velocity, float acceleration) {
  IrqLock lock;
  if (mode_ != Mode::OpenLoop) {
    enter(Mode::OpenLoop);
    openLoop_.reset(0);
  }
  openLoop_.moveTo(openLoop_.position() + q32::fromDelta(motorTurns), velocity, acceleration);
}

void MotionCore::setPosition(q32::Pos position) {
  IrqLock lock;
  zero_ += measured_ - position;
  measured_ = position;
  if (mode_ == Mode::Closed || mode_ == Mode::Seek || mode_ == Mode::StepDir) holdHere();
  reference_ = measured_;
}

void MotionCore::freezeOutput(bool frozen) {
  IrqLock lock;
  frozen_ = frozen;
  if (frozen) stepper_->setRate(0.0f);
}

MotionCore::Snapshot MotionCore::snapshot() const {
  IrqLock lock;
  Snapshot s;
  s.mode = mode_;
  s.tracking = tracking_;
  s.position = measured_;
  s.reference = reference_;
  s.target = mode_ == Mode::Closed ? trajectory_.target() : reference_;
  s.velocity = measuredVelocity_;
  s.referenceVelocity = referenceVelocity_;
  s.followingError = q32::diff(reference_, measured_);
  s.output = controller_.output();
  s.trajectoryDone = trajectory_.done();
  s.velocityMode = mode_ == Mode::Closed && trajectory_.mode() == Trajectory::Mode::Velocity;
  s.settled = controller_.settled();
  s.openLoopDone = openLoop_.done();
  s.limitMin = limitMin_;
  s.limitMax = limitMax_;
  s.counts = tracker_.counts();
  s.seekCause = seekCause_;
  return s;
}

uint8_t MotionCore::takeEvents() {
  IrqLock lock;
  const uint8_t events = events_;
  events_ = 0;
  return events;
}

uint16_t MotionCore::zeroRaw() const {
  IrqLock lock;
  // Encoder count at which the user position is 0, then back onto the raw circle.
  const q32::Pos zeroCountsQ = sign_ > 0 ? zero_ : -zero_;
  const int32_t shift = 32 - bits_;
  const int64_t zeroCounts = (zeroCountsQ + (q32::Pos{1} << (shift - 1))) >> shift;
  const int64_t cpr = int64_t{1} << bits_;
  return static_cast<uint16_t>((tracker_.raw() + zeroCounts - tracker_.counts()) & (cpr - 1));
}
