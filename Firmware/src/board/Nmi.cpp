// Non-maskable interrupt: the two recoverable causes on this board.
#include <Arduino.h>

#include "drivers/FlashStore.h"

extern "C" void NMI_Handler(void) {
  // Crystal failure (clock security system): restart; the boot-time check then
  // finds no crystal and runs from HSI.
  if (RCC->CIFR & RCC_CIFR_CSSF) {
    RCC->CICR = RCC_CICR_CSSC;
    NVIC_SystemReset();
  }
  // Double-bit ECC error while reading the settings pages (a record cut off by
  // power loss): flag it so the record is skipped, instead of hard-faulting.
  if (FLASH->ECCR & FLASH_ECCR_ECCD) {
    FLASH->ECCR |= FLASH_ECCR_ECCD;  // write 1 to clear
    FlashStore::onEccError();
    return;
  }
  while (true) {
  }
}
