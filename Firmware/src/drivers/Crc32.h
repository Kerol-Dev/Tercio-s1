#pragma once
// CRC-32 (IEEE 802.3, reflected polynomial 0xEDB88320), nibble-table variant:
// 64 bytes of table, ~2x faster than bitwise. Chainable: crc = update(crc, ...).
#include <cstddef>
#include <cstdint>

namespace crc32 {

inline uint32_t update(uint32_t crc, const void* data, size_t length) {
  static constexpr uint32_t kTable[16] = {
      0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC, 0x76DC4190, 0x6B6B51F4, 0x4DB26158, 0x5005713C,
      0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C, 0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C,
  };
  const auto* p = static_cast<const uint8_t*>(data);
  crc = ~crc;
  while (length--) {
    crc ^= *p++;
    crc = (crc >> 4) ^ kTable[crc & 0x0F];
    crc = (crc >> 4) ^ kTable[crc & 0x0F];
  }
  return ~crc;
}

}  // namespace crc32
