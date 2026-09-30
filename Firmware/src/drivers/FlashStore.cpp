#include "drivers/FlashStore.h"

#include <Arduino.h>

#include <cstring>

#include "drivers/Crc32.h"

namespace {

constexpr uint32_t kMagic = 0x54525331;  // "TRS1"
constexpr uint32_t kPageSize = FLASH_PAGE_SIZE;
constexpr uint32_t kHeaderSize = 16;
constexpr uint32_t kErased32 = 0xFFFFFFFFu;

// Set (via the NMI) when a read hits a double-bit ECC error, which is what a
// double-word interrupted mid-programming looks like.
volatile bool g_eccError = false;

uint32_t alignUp8(uint32_t n) { return (n + 7u) & ~7u; }

uint32_t read32(uint32_t address) { return *reinterpret_cast<volatile const uint32_t*>(address); }
uint16_t read16(uint32_t address) { return *reinterpret_cast<volatile const uint16_t*>(address); }

uint32_t pageCount() { return (static_cast<uint32_t>(*reinterpret_cast<const uint16_t*>(FLASHSIZE_BASE)) * 1024u) / kPageSize; }

void flushDataCache() {
  __HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_DATA_CACHE_ENABLE();
}

uint32_t recordCrc(uint16_t layout, uint16_t length, uint32_t sequence, const void* payload) {
  uint32_t crc = crc32::update(0, &layout, sizeof layout);
  crc = crc32::update(crc, &length, sizeof length);
  crc = crc32::update(crc, &sequence, sizeof sequence);
  return crc32::update(crc, payload, length);
}

}  // namespace

extern "C" uint32_t _sidata, _sdata, _edata;

void FlashStore::onEccError() { g_eccError = true; }

uint32_t FlashStore::pageAddress(uint32_t page) { return FLASH_BASE + page * kPageSize; }

bool FlashStore::imageOverlapsStore() {
  const uint32_t imageEnd = reinterpret_cast<uint32_t>(&_sidata) +
                            (reinterpret_cast<uint32_t>(&_edata) - reinterpret_cast<uint32_t>(&_sdata));
  return imageEnd > pageAddress(pageCount() - 2);
}

FlashStore::PageScan FlashStore::scan(uint32_t page) const {
  const uint32_t base = pageAddress(page);
  PageScan result{{}, kPageSize};
  uint32_t offset = 0;

  while (offset + kHeaderSize <= kPageSize) {
    g_eccError = false;
    const uint32_t address = base + offset;
    const uint32_t magic = read32(address);
    if (!g_eccError && magic == kErased32 && read32(address + 4) == kErased32 &&
        read32(address + 8) == kErased32 && read32(address + 12) == kErased32) {
      result.freeOffset = offset;
      return result;
    }
    const uint16_t length = read16(address + 6);
    const uint32_t slot = alignUp8(kHeaderSize + length);
    if (g_eccError || magic != kMagic || offset + slot > kPageSize) break;  // unusable tail

    const uint16_t layout = read16(address + 4);
    const uint32_t sequence = read32(address + 8);
    const uint32_t storedCrc = read32(address + 12);
    const auto* payload = reinterpret_cast<const void*>(address + kHeaderSize);
    const bool valid = storedCrc == recordCrc(layout, length, sequence, payload) && !g_eccError;
    if (valid && (!result.newest.valid || sequence > result.newest.sequence))
      result.newest = {address, sequence, true};
    offset += slot;
  }
  result.freeOffset = (offset + kHeaderSize <= kPageSize) ? kPageSize : offset;
  return result;
}

bool FlashStore::load(void* out, uint16_t size, uint16_t layout) {
  const uint32_t first = pageCount() - 2;
  const PageScan a = scan(first), b = scan(first + 1);
  const Slot& best = (b.newest.valid && (!a.newest.valid || b.newest.sequence > a.newest.sequence)) ? b.newest : a.newest;
  if (!best.valid) return false;
  if (read16(best.address + 4) != layout || read16(best.address + 6) != size) return false;
  std::memcpy(out, reinterpret_cast<const void*>(best.address + kHeaderSize), size);
  return true;
}

bool FlashStore::save(const void* data, uint16_t size, uint16_t layout) {
  if (imageOverlapsStore() || kHeaderSize + size > kPageSize) return false;

  const uint32_t first = pageCount() - 2;
  const PageScan scans[2] = {scan(first), scan(first + 1)};
  const uint32_t slot = alignUp8(kHeaderSize + size);

  // Append to the page holding the newest record; otherwise start the other page.
  int active = (scans[1].newest.valid && (!scans[0].newest.valid || scans[1].newest.sequence > scans[0].newest.sequence)) ? 1 : 0;
  const uint32_t sequence = (scans[active].newest.valid ? scans[active].newest.sequence : 0) + 1;
  bool erase = false;
  if (scans[active].freeOffset + slot > kPageSize) {
    active ^= 1;
    erase = scans[active].freeOffset != 0;  // other page not blank: erase it
  }
  const uint32_t address = pageAddress(first + active) + (erase ? 0 : scans[active].freeOffset);

  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
  bool ok = true;

  if (erase) {
    FLASH_EraseInitTypeDef eraseInit{};
    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.Banks = FLASH_BANK_1;
    eraseInit.Page = first + active;
    eraseInit.NbPages = 1;
    uint32_t pageError = 0;
    ok = HAL_FLASHEx_Erase(&eraseInit, &pageError) == HAL_OK;
  }

  const uint32_t crc = recordCrc(layout, size, sequence, data);
  const uint64_t header0 = kMagic | (static_cast<uint64_t>(layout) << 32) | (static_cast<uint64_t>(size) << 48);
  const uint64_t header1 = sequence | (static_cast<uint64_t>(crc) << 32);
  ok = ok && HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, header0) == HAL_OK;
  ok = ok && HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + 8, header1) == HAL_OK;

  const auto* bytes = static_cast<const uint8_t*>(data);
  for (uint32_t offset = 0; ok && offset < size; offset += 8) {
    uint64_t word = ~0ull;
    std::memcpy(&word, bytes + offset, (size - offset) < 8 ? (size - offset) : 8);
    ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + kHeaderSize + offset, word) == HAL_OK;
  }

  HAL_FLASH_Lock();
  flushDataCache();
  return ok && std::memcmp(reinterpret_cast<const void*>(address + kHeaderSize), data, size) == 0;
}
