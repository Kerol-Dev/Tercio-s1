#include "Arduino.h"
#include <AS5048A.h>


// -----------------------------------------------------------------------------
// Constants and Register Definitions
// -----------------------------------------------------------------------------
namespace Registers
{
  constexpr uint16_t CLEAR_ERROR_FLAG        = 0x0001;
  constexpr uint16_t PROGRAMMING_CONTROL     = 0x0003;
  constexpr uint16_t OTP_ZERO_POS_HIGH       = 0x0016;
  constexpr uint16_t OTP_ZERO_POS_LOW        = 0x0017;
  constexpr uint16_t DIAG_AGC                = 0x3FFD;
  constexpr uint16_t MAGNITUDE               = 0x3FFE;
  constexpr uint16_t ANGLE                   = 0x3FFF;
}

namespace Flags
{
  // Error Register Flags
  constexpr uint8_t  ERROR_PARITY            = 0x04;
  constexpr uint8_t  ERROR_COMMAND_INVALID   = 0x02;
  constexpr uint8_t  ERROR_FRAMING           = 0x01;

  // Diagnostic Register Flags
  constexpr uint16_t DIAG_COMP_HIGH          = 0x2000;
  constexpr uint16_t DIAG_COMP_LOW           = 0x1000;
  constexpr uint16_t DIAG_COF                = 0x0800;
  constexpr uint16_t DIAG_OCF                = 0x0400;
  constexpr uint8_t  DIAG_AGC_MASK           = 0xFF;
}

static constexpr double MAX_VALUE = 8191.0;
static constexpr uint16_t MASK_DATA = 0x3FFF; // Mask for 14-bit data


// -----------------------------------------------------------------------------
// AS5048A Implementation
// -----------------------------------------------------------------------------

AS5048A::AS5048A(byte cs, bool debug)
    : _cs(cs),
      errorFlag(false),
      ocfFlag(false),
      position(0),
      debug(debug),
      esp32_delay(0)
{
}

void AS5048A::begin()
{
    setDelay();
    this->settings = SPISettings(3000000, MSBFIRST, SPI_MODE1);
    pinMode(this->_cs, OUTPUT);
    SPI.begin();
}

void AS5048A::close()
{
    SPI.end();
}

uint8_t AS5048A::spiCalcEvenParity(uint16_t value)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < 16; i++)
    {
        if (value & 0x1)
            count++;
        value >>= 1;
    }
    return count & 0x1;
}

int16_t AS5048A::getRotation()
{
    uint16_t data = AS5048A::getRawRotation();
    int16_t rotation = static_cast<int16_t>(data) - static_cast<int16_t>(this->position);
    
    if (rotation > static_cast<int16_t>(MAX_VALUE))
        rotation = -((MASK_DATA) - rotation); 

    return rotation;
}

int16_t AS5048A::getRawRotation()
{
    return AS5048A::read(Registers::ANGLE);
}

double AS5048A::getRotationInDegrees()
{
    int16_t rotation = getRotation();
    // Maps -8192..8191 to 0..360
    double degrees = 360.0 * (rotation + MAX_VALUE) / (MAX_VALUE * 2.0);
    return degrees;
}

double AS5048A::getRotationInRadians()
{
    int16_t rotation = getRotation();
    // Maps -8192..8191 to 0..2PI
    double radians = PI * (rotation + MAX_VALUE) / MAX_VALUE;
    return radians;
}

uint16_t AS5048A::getState()
{
    return AS5048A::read(Registers::DIAG_AGC);
}

void AS5048A::printState()
{
    if (this->debug)
    {
        uint16_t data = AS5048A::getState();
        if (AS5048A::error())
        {
            Serial.print("Error bit was set!");
        }
        Serial.println(data, BIN);
    }
}

uint8_t AS5048A::getGain()
{
    uint16_t data = AS5048A::getState();
    return static_cast<uint8_t>(data & Flags::DIAG_AGC_MASK);
}

String AS5048A::getDiagnostic()
{
    uint16_t data = AS5048A::getState();
    
    if (data & Flags::DIAG_COMP_HIGH)
        return "COMP high";
    
    if (data & Flags::DIAG_COMP_LOW)
        return "COMP low";
    
    if (data & Flags::DIAG_COF)
        return "CORDIC overflow";
    
    if ((data & Flags::DIAG_OCF) && !ocfFlag)
    {
        ocfFlag = true;
        return "Offset compensation finished";
    }
    
    return "";
}

String AS5048A::getErrors()
{
    uint16_t error = AS5048A::read(Registers::CLEAR_ERROR_FLAG);
    
    if (error & Flags::ERROR_PARITY)
        return "Parity Error";
    
    if (error & Flags::ERROR_COMMAND_INVALID)
        return "Command invalid";
    
    if (error & Flags::ERROR_FRAMING)
        return "Framing error";
    
    return "";
}

void AS5048A::setZeroPosition(uint16_t position)
{
    this->position = position % MASK_DATA;
}

uint16_t AS5048A::getZeroPosition()
{
    return this->position;
}

bool AS5048A::error()
{
    return this->errorFlag;
}

uint16_t AS5048A::read(uint16_t registerAddress)
{
    uint16_t command = 0x4000; // Read command (Bit 14 set)
    command |= registerAddress;

    // Add parity bit (Bit 15)
    if (spiCalcEvenParity(command))
        command |= 0x8000;

    if (this->debug)
    {
        Serial.print("Read (0x");
        Serial.print(registerAddress, HEX);
        Serial.print(") with command: 0b");
        Serial.println(command, BIN);
    }

    SPI.beginTransaction(this->settings);

    // Send address
    digitalWrite(this->_cs, LOW);
    SPI.transfer16(command);
    digitalWrite(this->_cs, HIGH);

    delay(this->esp32_delay);

    // Read response (send dummy 0x00)
    digitalWrite(this->_cs, LOW);
    uint16_t response = SPI.transfer16(0x00);
    digitalWrite(this->_cs, HIGH);

    SPI.endTransaction();

    if (this->debug)
    {
        Serial.print("Read returned: ");
        Serial.println(response, BIN);
    }

    // Check error bit (Bit 14)
    if (response & 0x4000)
    {
        if (this->debug)
            Serial.println("Setting error bit");
        this->errorFlag = true;
    }
    else
    {
        this->errorFlag = false;
    }

    // Mask out Parity (15) and Error (14) bits
    return response & MASK_DATA;
}

uint16_t AS5048A::write(uint16_t registerAddress, uint16_t data)
{
    // 1. Send Write Command
    uint16_t command = 0x0000; // Write command (Bit 14 clear)
    command |= registerAddress;

    if (spiCalcEvenParity(command))
        command |= 0x8000;

    if (this->debug)
    {
        Serial.print("Write (0x");
        Serial.print(registerAddress, HEX);
        Serial.print(") with command: 0b");
        Serial.println(command, BIN);
    }

    SPI.beginTransaction(this->settings);

    digitalWrite(this->_cs, LOW);
    SPI.transfer16(command);
    digitalWrite(this->_cs, HIGH);

    // 2. Send Data
    uint16_t dataPacket = data;
    if (spiCalcEvenParity(dataPacket))
        dataPacket |= 0x8000;

    if (this->debug)
    {
        Serial.print("Sending data to write: ");
        Serial.println(dataPacket, BIN);
    }

    digitalWrite(this->_cs, LOW);
    SPI.transfer16(dataPacket);
    digitalWrite(this->_cs, HIGH);

    delay(this->esp32_delay);

    // 3. Read Confirmation (optional, but standard for this flow)
    digitalWrite(this->_cs, LOW);
    uint16_t response = SPI.transfer16(0x0000);
    digitalWrite(this->_cs, HIGH);

    SPI.endTransaction();

    return response & MASK_DATA;
}

void AS5048A::setDelay()
{
    this->esp32_delay = 0;
    if (this->debug)
    {
        Serial.println("Device not detected");
    }
}