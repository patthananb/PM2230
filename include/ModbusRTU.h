#ifndef MODBUSRTU_H
#define MODBUSRTU_H

#include <Arduino.h>

// ============================================================================
// ModbusRTU Class - Raw Modbus RTU master (no external library)
// ============================================================================
class ModbusRTU {
public:
    ModbusRTU(HardwareSerial& serial, int8_t dePin = -1);

    void begin(uint32_t baudRate, uint32_t config, int8_t rxPin, int8_t txPin);

    /// Read Holding Registers (Function Code 0x03)
    bool readHoldingRegisters(uint8_t slaveAddr, uint16_t startReg,
                              uint16_t quantity, uint16_t* result);

    void setResponseTimeout(uint32_t ms);

private:
    HardwareSerial& _serial;
    int8_t          _dePin;
    uint32_t        _baudRate          = 9600;
    uint32_t        _frameDelayUs      = 1750;
    uint32_t        _responseTimeoutMs = 1000;

    void     _setTransmitMode(bool tx);
    int      _receiveFrame(uint8_t* buf, uint8_t expected, uint32_t timeoutMs);
    uint16_t _calculateCRC(const uint8_t* data, uint8_t len);
};

#endif // MODBUSRTU_H
