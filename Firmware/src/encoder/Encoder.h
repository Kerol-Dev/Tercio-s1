#pragma once
// -----------------------------------------------------------------------------
// Absolute single-turn magnetic encoder. poll() runs in the 20 kHz control
// interrupt and must never block; everything slow (bring-up, diagnostics,
// bus recovery) happens in begin()/service() from the main loop.
// -----------------------------------------------------------------------------
#include <cstdint>

struct MagnetStatus {
  bool detected = false;
  bool tooWeak = false;
  bool tooStrong = false;
};

class Encoder {
 public:
  virtual ~Encoder() = default;

  // Main loop. False if the sensor does not respond.
  virtual bool begin() = 0;
  // Main loop: stop touching the bus (before switching to another encoder).
  virtual void end() = 0;
  // Control interrupt, every tick. True when `raw` holds a new sample.
  virtual bool poll(uint16_t& raw) = 0;
  // Main loop, ~100 Hz: schedules diagnostics, recovers a wedged bus.
  virtual void service() = 0;

  virtual uint8_t resolutionBits() const = 0;
  virtual MagnetStatus magnet() const = 0;
};
