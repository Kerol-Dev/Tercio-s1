#include "drivers/CanBus.h"

#include <Arduino.h>

#include <cstring>

#include "board/Board.h"

static_assert(board::kCanRxPin == PA11 && board::kCanTxPin == PA12, "FDCAN1 is wired to PA11/PA12 (AF9)");

namespace can_bus {
namespace {

// Bit timing for a 170 MHz FDCAN kernel clock (PCLK1), identical to ACANFD's
// choice in firmware v1: 85 MHz time quanta,
//   arbitration 500 kbit/s = 1 + 126 + 43 tq (sample point 74.7 %)
//   data        2.5 Mbit/s = 1 + 24 + 9 tq   (sample point 73.5 %)
// Transceiver delay compensation offset: half a data bit.
constexpr uint32_t kKernelClock = 170000000;
constexpr uint32_t kPrescaler = 2;
constexpr uint32_t kNominalSeg1 = 126, kNominalSeg2 = 43;
constexpr uint32_t kDataSeg1 = 24, kDataSeg2 = 9;
constexpr uint32_t kTdcOffset = 34;
constexpr uint32_t kBusOffRetryMs = 50;

constexpr uint8_t kDlcBytes[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

constexpr uint8_t kRxSlots = 16;  // power of two
FDCAN_HandleTypeDef g_can{};
CanFrame g_rx[kRxSlots];
volatile uint8_t g_rxHead = 0;  // written by the interrupt
volatile uint8_t g_rxTail = 0;  // written by the main loop
volatile uint32_t g_dropped = 0;
uint32_t g_lastRecoveryMs = 0;

bool configureFilter(uint16_t a, uint16_t b) {
  FDCAN_FilterTypeDef filter{};
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_DUAL;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = a & 0x7FFu;
  filter.FilterID2 = b & 0x7FFu;
  return HAL_FDCAN_ConfigFilter(&g_can, &filter) == HAL_OK &&
         HAL_FDCAN_ConfigGlobalFilter(&g_can, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) == HAL_OK;
}

}  // namespace

bool begin(uint16_t acceptA, uint16_t acceptB, uint32_t irqPriority) {
  if (HAL_RCC_GetPCLK1Freq() != kKernelClock) return false;  // bit timing assumes 170 MHz

  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef pins{};
  pins.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  pins.Mode = GPIO_MODE_AF_PP;
  pins.Pull = GPIO_NOPULL;
  pins.Speed = GPIO_SPEED_FREQ_HIGH;
  pins.Alternate = GPIO_AF9_FDCAN1;
  HAL_GPIO_Init(GPIOA, &pins);

  RCC_PeriphCLKInitTypeDef clock{};
  clock.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  clock.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) return false;
  __HAL_RCC_FDCAN_CLK_ENABLE();

  g_can.Instance = FDCAN1;
  FDCAN_InitTypeDef& init = g_can.Init;
  init.ClockDivider = FDCAN_CLOCK_DIV1;
  init.FrameFormat = FDCAN_FRAME_FD_BRS;
  init.Mode = FDCAN_MODE_NORMAL;
  init.AutoRetransmission = ENABLE;  // IDs are unique per node, so retrying is safe
  init.TransmitPause = DISABLE;
  init.ProtocolException = DISABLE;
  init.NominalPrescaler = kPrescaler;
  init.NominalSyncJumpWidth = kNominalSeg2;
  init.NominalTimeSeg1 = kNominalSeg1;
  init.NominalTimeSeg2 = kNominalSeg2;
  init.DataPrescaler = kPrescaler;
  init.DataSyncJumpWidth = kDataSeg2;
  init.DataTimeSeg1 = kDataSeg1;
  init.DataTimeSeg2 = kDataSeg2;
  init.StdFiltersNbr = 1;
  init.ExtFiltersNbr = 0;
  init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&g_can) != HAL_OK) return false;

  if (!configureFilter(acceptA, acceptB)) return false;
  if (HAL_FDCAN_ConfigTxDelayCompensation(&g_can, kTdcOffset, 0) != HAL_OK) return false;
  if (HAL_FDCAN_EnableTxDelayCompensation(&g_can) != HAL_OK) return false;
  if (HAL_FDCAN_ActivateNotification(&g_can, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) return false;

  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, irqPriority, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  return HAL_FDCAN_Start(&g_can) == HAL_OK;
}

bool setFilter(uint16_t acceptA, uint16_t acceptB) {
  // Filter elements live in message RAM and may be rewritten while running.
  // Stopping the controller instead would flush the TX FIFO (and a pending reply).
  return configureFilter(acceptA, acceptB);
}

bool receive(CanFrame& frame) {
  const uint8_t tail = g_rxTail;
  if (tail == g_rxHead) return false;
  frame = g_rx[tail];
  __DMB();
  g_rxTail = static_cast<uint8_t>((tail + 1) & (kRxSlots - 1));
  return true;
}

bool send(uint16_t id, const uint8_t* data, uint8_t length) {
  if (length > 64) return false;
  uint8_t dlc = 0;
  while (kDlcBytes[dlc] < length) ++dlc;

  uint8_t buffer[64];
  std::memcpy(buffer, data, length);
  std::memset(buffer + length, 0, kDlcBytes[dlc] - length);

  FDCAN_TxHeaderTypeDef header{};
  header.Identifier = id & 0x7FFu;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = dlc;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_ON;
  header.FDFormat = FDCAN_FD_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0;

  if (HAL_FDCAN_GetTxFifoFreeLevel(&g_can) == 0 || HAL_FDCAN_AddMessageToTxFifoQ(&g_can, &header, buffer) != HAL_OK) {
    g_dropped = g_dropped + 1;
    return false;
  }
  return true;
}

void service(uint32_t nowMs) {
  // After bus-off the FDCAN parks itself in INIT; leaving INIT starts the
  // standard recovery (128 x 11 recessive bits) in hardware.
  if ((FDCAN1->PSR & FDCAN_PSR_BO) && nowMs - g_lastRecoveryMs >= kBusOffRetryMs) {
    g_lastRecoveryMs = nowMs;
    CLEAR_BIT(FDCAN1->CCCR, FDCAN_CCCR_INIT);
  }
}

bool errorPassive() { return (FDCAN1->PSR & (FDCAN_PSR_EP | FDCAN_PSR_BO)) != 0; }
uint32_t droppedFrames() { return g_dropped; }

}  // namespace can_bus

extern "C" {

void FDCAN1_IT0_IRQHandler(void) { HAL_FDCAN_IRQHandler(&can_bus::g_can); }

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* handle, uint32_t) {
  using namespace can_bus;
  FDCAN_RxHeaderTypeDef header;
  while (HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0) > 0) {
    const uint8_t head = g_rxHead;
    const uint8_t next = static_cast<uint8_t>((head + 1) & (kRxSlots - 1));
    if (next == g_rxTail) {  // ring full: drop the oldest hardware entry
      uint8_t scratch[64];
      HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header, scratch);
      g_dropped = g_dropped + 1;
      continue;
    }
    CanFrame& frame = g_rx[head];
    if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header, frame.data) != HAL_OK) break;
    frame.id = static_cast<uint16_t>(header.Identifier);
    frame.length = kDlcBytes[header.DataLength & 0x0Fu];
    // Classic CAN: DLC 9..15 still means 8 bytes.
    if (header.FDFormat == FDCAN_CLASSIC_CAN && frame.length > 8) frame.length = 8;
    __DMB();
    g_rxHead = next;
  }
}

}  // extern "C"
