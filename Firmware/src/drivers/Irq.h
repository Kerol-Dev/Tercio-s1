#pragma once
// Scoped interrupt lock for sharing state between the control interrupt and the
// main loop. Nests correctly (restores the previous PRIMASK). Keep scopes short:
// every microsecond here delays the 20 kHz control tick.
#include <Arduino.h>

class IrqLock {
 public:
  IrqLock() : primask_(__get_PRIMASK()) { __disable_irq(); }
  ~IrqLock() { __set_PRIMASK(primask_); }
  IrqLock(const IrqLock&) = delete;
  IrqLock& operator=(const IrqLock&) = delete;

 private:
  uint32_t primask_;
};
