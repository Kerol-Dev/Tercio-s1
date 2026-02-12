#include <Arduino.h>
#include <ACANFD_STM32_from_cpp.h>
#include <ACANFD_STM32_Settings.h>
#include <ACANFD_STM32.h>
#include <cstring>
#include "CanCmdBus.h"


// -----------------------------------------------------------------------------
// CanCmdBus Implementation
// -----------------------------------------------------------------------------
namespace CanCmdBus
{
  // ---------------------------------------------------------------------------
  // Internal Data Structures
  // ---------------------------------------------------------------------------
  struct CommandHandlerEntry
  {
    uint8_t commandId;
    Handler handlerFunc;
  };

  namespace
  {
    CommandHandlerEntry g_handlerTable[CANCMD_MAX_HANDLERS];
    uint8_t             g_handlerCount = 0;
    uint16_t            g_filterId     = 0;
    uint16_t            g_filterMask   = 0;
    uint16_t            g_defaultTxId  = 0;
    uint8_t             g_rxBuffer[64]; // Static buffer to avoid stack allocation in poll()
  }


  // ---------------------------------------------------------------------------
  // Internal Helper Functions
  // ---------------------------------------------------------------------------

  // Maps requested payload length to the nearest valid CAN-FD DLC length
  static uint8_t normalizeFdLength(uint8_t length)
  {
    static constexpr uint8_t validLengths[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    
    for (uint8_t step : validLengths)
    {
      if (length <= step)
        return step;
    }
    return 64;
  }

  static uint8_t findHandlerIndex(uint8_t commandId)
  {
    for (uint8_t i = 0; i < g_handlerCount; ++i)
    {
      if (g_handlerTable[i].commandId == commandId)
        return i;
    }
    return 0xFF;
  }

  // Checks and recovers from FDCAN hardware error states (Bus Off / Error Passive)
  static void manageHardwareState()
  {
    // PSR (Protocol Status Register)
    const uint32_t protocolStatus = FDCAN1->PSR;

    // Check for Bus_Off (BO) or Error_Passive (EP) bits
    if (protocolStatus & (FDCAN_PSR_BO | FDCAN_PSR_EP))
    {
      // Clearing the Error Counter Register (ECR) helps reset error logic
      // in some manual recovery scenarios, though typically the hardware
      // auto-recovers after 128 occurrences of 11 consecutive recessive bits.
      FDCAN1->ECR = 0;
    }
  }


  // ---------------------------------------------------------------------------
  // Public API
  // ---------------------------------------------------------------------------

  bool begin(uint32_t nominalBps, uint8_t dataFactor, uint8_t rxPin, uint8_t txPin, bool normalMode)
  {
    ACANFD_STM32_Settings settings(nominalBps, (DataBitRateFactor)dataFactor);
    
    settings.mModuleMode = normalMode ? ACANFD_STM32_Settings::NORMAL_FD 
                                      : ACANFD_STM32_Settings::EXTERNAL_LOOP_BACK;
    
    settings.mRxPin = rxPin;
    settings.mTxPin = txPin;
    
    // Disable automatic retransmission for deterministic behavior in control loops
    settings.mEnableRetransmission = false; 

    const uint32_t errorCode = fdcan1.beginFD(settings);
    if (errorCode != 0)
      return false;

    g_handlerCount = 0;
    std::memset(g_handlerTable, 0, sizeof(g_handlerTable));
    
    return true;
  }

  void poll()
  {
    // Ensure hardware is healthy before attempting reception
    manageHardwareState();

    CANFDMessage message;

    while (fdcan1.receiveFD0(message))
    {
      const uint16_t messageId = static_cast<uint16_t>(message.id & 0x7FF);

      // Apply software ID filtering
      if ((messageId & g_filterMask) != (g_filterId & g_filterMask))
        continue;

      if (message.len == 0 || message.len > 64)
        continue;

      const uint8_t commandId = message.data[0];
      const uint8_t payloadLen = (message.len > 1) ? static_cast<uint8_t>(message.len - 1) : 0;

      const uint8_t handlerIdx = findHandlerIndex(commandId);
      if (handlerIdx == 0xFF || g_handlerTable[handlerIdx].handlerFunc == nullptr)
        continue;

      if (payloadLen > 0)
      {
        std::memcpy(g_rxBuffer, &message.data[1], payloadLen);
      }

      CmdFrame frame = {
        messageId, 
        commandId, 
        (payloadLen > 0) ? g_rxBuffer : nullptr, 
        payloadLen
      };

      // Execute handler safely
      #if defined(__cpp_exceptions)
      try
      {
        g_handlerTable[handlerIdx].handlerFunc(frame);
      }
      catch (...)
      {
        // Suppress exceptions in ISR/Poll context
      }
      #else
        g_handlerTable[handlerIdx].handlerFunc(frame);
      #endif
    }
  }

  bool send(uint16_t id, uint8_t commandId, const void *data, uint8_t length)
  {
    if (length > 63)
      length = 63;

    CANFDMessage message;
    message.id = static_cast<uint32_t>(id & 0x7FF);

    const uint8_t totalLength = 1 + length;
    message.len = normalizeFdLength(totalLength);

    std::memset(message.data, 0, sizeof(message.data));
    message.data[0] = commandId;
    
    if (data != nullptr && length > 0)
    {
      std::memcpy(&message.data[1], data, length);
    }

    // Attempt transmission. Non-zero return indicates error (e.g. FIFO full)
    const uint32_t status = fdcan1.tryToSendReturnStatusFD(message);

    if (status != 0)
    {
      return false;
    }
    
    return true;
  }

  bool registerHandler(uint8_t commandId, Handler handler)
  {
    const uint8_t existingIdx = findHandlerIndex(commandId);
    
    if (existingIdx != 0xFF)
    {
      g_handlerTable[existingIdx].handlerFunc = handler;
      return true;
    }
    
    if (g_handlerCount >= CANCMD_MAX_HANDLERS)
      return false;
      
    g_handlerTable[g_handlerCount++] = {commandId, handler};
    return true;
  }

  void setIdFilter(uint16_t filter, uint16_t mask)
  {
    g_filterId = filter & 0x7FF;
    g_filterMask = mask & 0x7FF;
  }

  void setDefaultTxId(uint16_t id)
  {
    g_defaultTxId = id & 0x7FF;
  }

  bool sendDefault(uint8_t commandId, const void *data, uint8_t length)
  {
    return send(g_defaultTxId, commandId, data, length);
  }

} // namespace CanCmdBus