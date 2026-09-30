#include "app/Params.h"

#include <cmath>
#include <cstddef>
#include <cstring>

namespace params {
namespace {

using proto::Param;

#define FIELD(name) static_cast<uint16_t>(offsetof(Settings, name))

constexpr Def kTable[] = {
    // id                            type        access                 field                       min       max
    {Param::NodeId,                  Type::U8,   Access::WhenStill,     FIELD(nodeId),              1,        proto::kMaxNodeId},
    {Param::TelemetryRateHz,         Type::U16,  Access::Always,        FIELD(telemetryRateHz),     0,        1000},
    {Param::CommandTimeoutMs,        Type::U16,  Access::Always,        FIELD(commandTimeoutMs),    0,        60000},

    {Param::FullStepsPerRev,         Type::U16,  Access::WhenStill,     FIELD(fullStepsPerRev),     20,       1000},
    {Param::Microsteps,              Type::U16,  Access::WhenStill,     FIELD(microsteps),          1,        256},
    {Param::RunCurrentMa,            Type::U16,  Access::Always,        FIELD(runCurrentMa),        50,       2000},
    {Param::HoldCurrentPct,          Type::U8,   Access::Always,        FIELD(holdCurrentPct),      0,        100},
    {Param::StealthChop,             Type::Bool, Access::WhenStill,     FIELD(stealthChop),         0,        1},
    {Param::StealthChopMaxVel,       Type::F32,  Access::Always,        FIELD(stealthChopMaxVel),   0,        100},
    {Param::InvertDirection,         Type::Bool, Access::WhenStill,     FIELD(invertDirection),     0,        1},
    {Param::GearRatio,               Type::F32,  Access::WhenStill,     FIELD(gearRatio),           0.01f,    1000},

    {Param::MaxVelocity,             Type::F32,  Access::Always,        FIELD(maxVelocity),         0.001f,   200},
    {Param::MaxAcceleration,         Type::F32,  Access::Always,        FIELD(maxAcceleration),     0.01f,    100000},
    {Param::Kp,                      Type::F32,  Access::Always,        FIELD(kp),                  0,        1000},
    {Param::Ki,                      Type::F32,  Access::Always,        FIELD(ki),                  0,        100000},
    {Param::Kd,                      Type::F32,  Access::Always,        FIELD(kd),                  0,        10},
    {Param::PositionDeadband,        Type::F32,  Access::Always,        FIELD(positionDeadband),    0,        0.1f},
    {Param::FollowingErrorLimit,     Type::F32,  Access::Always,        FIELD(followingErrorLimit), 0.001f,   1000},
    {Param::StallTimeoutMs,          Type::U16,  Access::Always,        FIELD(stallTimeoutMs),      0,        10000},
    {Param::SoftLimitMin,            Type::F32,  Access::Always,        FIELD(softLimitMin),        -1e7f,    1e7f},
    {Param::SoftLimitMax,            Type::F32,  Access::Always,        FIELD(softLimitMax),        -1e7f,    1e7f},
    {Param::EnableOnBoot,            Type::Bool, Access::Always,        FIELD(enableOnBoot),        0,        1},

    {Param::EncoderType,             Type::U8,   Access::WhenDisabled,  FIELD(encoderType),         0,        2},
    {Param::EncoderInvert,           Type::Bool, Access::WhenDisabled,  FIELD(encoderInvert),       0,        1},
    {Param::Calibrated,              Type::Bool, Access::ReadOnly,      FIELD(calibrated),          0,        1},

    {Param::LimitSwitchesEnabled,    Type::Bool, Access::Always,        FIELD(limitSwitchesEnabled),   0,     1},
    {Param::LimitSwitchActiveLow,    Type::Bool, Access::WhenStill,     FIELD(limitSwitchActiveLow),   0,     1},
    {Param::StepDirMode,             Type::Bool, Access::WhenStill,     FIELD(stepDirMode),            0,     1},
    {Param::StepDirEnableActiveLow,  Type::Bool, Access::Always,        FIELD(stepDirEnableActiveLow), 0,     1},

    {Param::HomingMode,              Type::U8,   Access::Always,        FIELD(homingMode),          0,        3},
    {Param::HomingVelocity,          Type::F32,  Access::Always,        FIELD(homingVelocity),      0.001f,   100},
    {Param::HomingCurrentMa,         Type::U16,  Access::Always,        FIELD(homingCurrentMa),     50,       2000},
    {Param::HomingBackoff,           Type::F32,  Access::Always,        FIELD(homingBackoff),       0,        1000},
    {Param::HomingStallError,        Type::F32,  Access::Always,        FIELD(homingStallError),    0.001f,   100},
    {Param::HomingTimeoutS,          Type::U16,  Access::Always,        FIELD(homingTimeoutS),      1,        3600},

    {Param::OverTemperatureC,        Type::F32,  Access::Always,        FIELD(overTemperatureC),    40,       125},
};

#undef FIELD

bool isPowerOfTwo(uint32_t v) { return v != 0 && (v & (v - 1)) == 0; }

// Extra constraints a plain range cannot express.
bool extraValid(Param id, float value) {
  if (id == Param::Microsteps) return isPowerOfTwo(static_cast<uint32_t>(value));
  return true;
}

float loadAsFloat(const Settings& s, const Def& d) {
  const auto* p = reinterpret_cast<const uint8_t*>(&s) + d.offset;
  switch (d.type) {
    case Type::Bool: { bool v; std::memcpy(&v, p, 1); return v ? 1.0f : 0.0f; }
    case Type::U8: { uint8_t v; std::memcpy(&v, p, 1); return v; }
    case Type::U16: { uint16_t v; std::memcpy(&v, p, 2); return v; }
    case Type::F32: { float v; std::memcpy(&v, p, 4); return v; }
  }
  return 0.0f;
}

void storeFromFloat(Settings& s, const Def& d, float value) {
  auto* p = reinterpret_cast<uint8_t*>(&s) + d.offset;
  switch (d.type) {
    case Type::Bool: { const bool v = value != 0.0f; std::memcpy(p, &v, 1); break; }
    case Type::U8: { const auto v = static_cast<uint8_t>(value); std::memcpy(p, &v, 1); break; }
    case Type::U16: { const auto v = static_cast<uint16_t>(value); std::memcpy(p, &v, 2); break; }
    case Type::F32: std::memcpy(p, &value, 4); break;
  }
}

}  // namespace

const Def* find(proto::Param id) {
  for (const Def& d : kTable)
    if (d.id == id) return &d;
  return nullptr;
}

uint32_t read(const Settings& s, const Def& def) {
  const float value = loadAsFloat(s, def);
  if (def.type == Type::F32) {
    uint32_t raw;
    std::memcpy(&raw, &value, 4);
    return raw;
  }
  return static_cast<uint32_t>(value);
}

proto::Status write(Settings& s, const Def& def, uint32_t raw) {
  if (def.access == Access::ReadOnly) return proto::Status::ReadOnly;

  float value;
  if (def.type == Type::F32) {
    std::memcpy(&value, &raw, 4);
    if (!std::isfinite(value)) return proto::Status::BadValue;
  } else {
    if (def.type == Type::Bool && raw > 1) return proto::Status::BadValue;
    if (raw > 0xFFFFu) return proto::Status::BadValue;
    value = static_cast<float>(raw);
  }
  if (value < def.min || value > def.max || !extraValid(def.id, value)) return proto::Status::BadValue;

  storeFromFloat(s, def, value);
  return proto::Status::Ok;
}

void sanitize(Settings& s) {
  const Settings defaults{};
  for (const Def& d : kTable) {
    const float value = loadAsFloat(s, d);
    if (!std::isfinite(value) || value < d.min || value > d.max || !extraValid(d.id, value))
      storeFromFloat(s, d, loadAsFloat(defaults, d));
  }
  if (s.encoderZeroRaw >= 16384) s.encoderZeroRaw = 0;
}

}  // namespace params
