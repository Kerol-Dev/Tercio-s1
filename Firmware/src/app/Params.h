#pragma once
// -----------------------------------------------------------------------------
// Parameter dictionary: maps protocol parameter ids onto Settings fields with
// type, range and access rules. Table-driven, so adding a parameter is one line.
// Hardware-independent (unit tested on the host).
// -----------------------------------------------------------------------------
#include <cstdint>

#include "app/Settings.h"
#include "protocol/Protocol.h"

namespace params {

enum class Type : uint8_t { Bool, U8, U16, F32 };

// When a parameter may be written.
enum class Access : uint8_t {
  Always,        // any time
  WhenStill,     // not while moving or running a procedure
  WhenDisabled,  // only with the bridge off
  ReadOnly,
};

struct Def {
  proto::Param id;
  Type type;
  Access access;
  uint16_t offset;  // byte offset into Settings
  float min;
  float max;
};

const Def* find(proto::Param id);

// Encodes the current value as 4 little-endian bytes.
uint32_t read(const Settings& s, const Def& def);

// Validates and stores `raw` (4-byte wire value). Returns Ok, BadValue or ReadOnly.
// Access rules are checked by the caller, which knows the axis state.
proto::Status write(Settings& s, const Def& def, uint32_t raw);

// Clamps every field of a freshly loaded record into its valid range, so a
// corrupted-but-CRC-valid record can never produce out-of-range settings.
void sanitize(Settings& s);

}  // namespace params
