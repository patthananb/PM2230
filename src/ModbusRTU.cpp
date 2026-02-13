#include "ModbusRTU.h"

// ============================================================================
// Constructor
// ============================================================================
ModbusRTU::ModbusRTU(HardwareSerial& serial, int8_t dePin)
    : _serial(serial), _dePin(dePin) {}

// ============================================================================
// Initialization
// ============================================================================
void ModbusRTU::begin(uint32_t baudRate, uint32_t config, int8_t rxPin, int8_t txPin) {
    _serial.begin(baudRate, config, rxPin, txPin);
    _baudRate = baudRate;

    // Inter-frame delay: 3.5 character times
    // At baud rates >= 19200, fixed 1750 us per Modbus spec
    if (baudRate >= 19200) {
        _frameDelayUs = 1750;
    } else {
        _frameDelayUs = (uint32_t)(3.5f * 11.0f * 1000000.0f / (float)baudRate);
    }

    if (_dePin >= 0) {
        pinMode(_dePin, OUTPUT);
        digitalWrite(_dePin, LOW); // Receive mode
    }
}

// ============================================================================
// Read Holding Registers (Function Code 0x03)
// ============================================================================
bool ModbusRTU::readHoldingRegisters(uint8_t slaveAddr, uint16_t startReg,
                                     uint16_t quantity, uint16_t* result) {
    // Build request: [Addr][0x03][RegHi][RegLo][QtyHi][QtyLo][CRC_Lo][CRC_Hi]
    uint8_t request[8];
    request[0] = slaveAddr;
    request[1] = 0x03;
    request[2] = (startReg >> 8) & 0xFF;
    request[3] = startReg & 0xFF;
    request[4] = (quantity >> 8) & 0xFF;
    request[5] = quantity & 0xFF;

    uint16_t crc = _calculateCRC(request, 6);
    request[6] = crc & 0xFF;         // CRC Lo
    request[7] = (crc >> 8) & 0xFF;  // CRC Hi

    // Clear RX buffer
    while (_serial.available()) _serial.read();

    // Transmit
    _setTransmitMode(true);
    _serial.write(request, 8);
    _serial.flush();
    _setTransmitMode(false);

    delayMicroseconds(_frameDelayUs);

    // Receive response
    uint8_t expectedLen = 3 + (2 * quantity) + 2;
    uint8_t response[256];
    int received = _receiveFrame(response, expectedLen, _responseTimeoutMs);

    if (received < expectedLen) return false;
    if (response[0] != slaveAddr) return false;
    if (response[1] != 0x03) return false;
    if (response[2] != quantity * 2) return false;

    // Verify CRC
    uint16_t respCRC = response[received - 2] | (response[received - 1] << 8);
    if (respCRC != _calculateCRC(response, received - 2)) return false;

    // Extract register values (big-endian)
    for (uint16_t i = 0; i < quantity; i++) {
        result[i] = (response[3 + i * 2] << 8) | response[3 + i * 2 + 1];
    }
    return true;
}

void ModbusRTU::setResponseTimeout(uint32_t ms) {
    _responseTimeoutMs = ms;
}

// ============================================================================
// Private helpers
// ============================================================================
void ModbusRTU::_setTransmitMode(bool tx) {
    if (_dePin >= 0) {
        digitalWrite(_dePin, tx ? HIGH : LOW);
        delayMicroseconds(100);
    }
}

int ModbusRTU::_receiveFrame(uint8_t* buf, uint8_t expected, uint32_t timeoutMs) {
    uint32_t start = millis();
    uint8_t  idx   = 0;
    while (idx < expected && (millis() - start) < timeoutMs) {
        if (_serial.available()) {
            buf[idx++] = _serial.read();
            start = millis(); // reset per-byte timeout
        }
    }
    return idx;
}

uint16_t ModbusRTU::_calculateCRC(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
            else              { crc >>= 1; }
        }
    }
    return crc;
}
