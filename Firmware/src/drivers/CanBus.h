#pragma once
// -----------------------------------------------------------------------------
// FDCAN1 driver: CAN-FD 500 kbit/s arbitration / 2.5 Mbit/s data with bit-rate
// switching — the same timing ACANFD produced in firmware v1, so existing
// adapters keep working.
//
// A hardware filter passes only this node's command ID and the broadcast ID,
// so traffic from other nodes never reaches the CPU. Received frames are moved
// into a ring buffer from the interrupt, so a busy main loop loses nothing.
// (During a flash erase the CPU cannot take interrupts at all; the 3-frame
// hardware FIFO covers that ~25 ms window.) Bus-off is recovered automatically.
// -----------------------------------------------------------------------------
#include <cstdint>

struct CanFrame {
  uint16_t id = 0;
  uint8_t length = 0;  // bytes, 0..64
  uint8_t data[64] = {};
};

namespace can_bus {

// `acceptA` and `acceptB` are the two 11-bit IDs the node listens to.
bool begin(uint16_t acceptA, uint16_t acceptB, uint32_t irqPriority);
bool setFilter(uint16_t acceptA, uint16_t acceptB);

bool receive(CanFrame& frame);
// Queues an FD frame with BRS; the length is padded up to the next valid FD size.
bool send(uint16_t id, const uint8_t* data, uint8_t length);

// Bus-off recovery and error state tracking; call from the main loop.
void service(uint32_t nowMs);
bool errorPassive();
uint32_t droppedFrames();

}  // namespace can_bus
