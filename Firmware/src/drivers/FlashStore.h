#pragma once
// -----------------------------------------------------------------------------
// Power-fail-safe, wear-levelled record storage in the last two flash pages.
//
// Records are appended to the active page; a page is erased only when the other
// one fills up, so a typical save programs a few dozen double-words (~1-2 ms)
// instead of erasing a page (~25 ms). A record is:
//
//   [magic u32][layout u16][length u16][sequence u32][crc32 u32][payload...]
//
// The header is written first. A power cut mid-write leaves a record whose CRC
// fails; the previous record (same page or the other page) stays valid.
//
// Flash programming stalls the CPU (single-bank part), so only save while the
// motor is not stepping.
// -----------------------------------------------------------------------------
#include <cstddef>
#include <cstdint>

class FlashStore {
 public:
  // Loads the newest valid record with a matching layout. Returns false if none.
  bool load(void* out, uint16_t size, uint16_t layout);
  bool save(const void* data, uint16_t size, uint16_t layout);
  // Called from the NMI handler on a double-bit ECC error.
  static void onEccError();

 private:
  struct Slot {
    uint32_t address = 0;
    uint32_t sequence = 0;
    bool valid = false;
  };
  struct PageScan {
    Slot newest;           // newest valid record in the page
    uint32_t freeOffset;   // first erased byte, or kPageSize when the tail is unusable
  };

  PageScan scan(uint32_t page) const;
  static uint32_t pageAddress(uint32_t page);
  static bool imageOverlapsStore();
};
