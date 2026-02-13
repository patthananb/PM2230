#include <Arduino.h>

// ============================================================================
// ModbusRTU Class - Raw Modbus RTU master (no external library)
// ============================================================================
class ModbusRTU {
public:
    ModbusRTU(HardwareSerial& serial, int8_t dePin = -1)
        : _serial(serial), _dePin(dePin) {}

    void begin(uint32_t baudRate, uint32_t config, int8_t rxPin, int8_t txPin) {
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

    /// Read Holding Registers (Function Code 0x03)
    bool readHoldingRegisters(uint8_t slaveAddr, uint16_t startReg,
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

    void setResponseTimeout(uint32_t ms) { _responseTimeoutMs = ms; }

private:
    HardwareSerial& _serial;
    int8_t          _dePin;
    uint32_t        _baudRate         = 9600;
    uint32_t        _frameDelayUs     = 1750;
    uint32_t        _responseTimeoutMs = 1000;

    void _setTransmitMode(bool tx) {
        if (_dePin >= 0) {
            digitalWrite(_dePin, tx ? HIGH : LOW);
            delayMicroseconds(100);
        }
    }

    int _receiveFrame(uint8_t* buf, uint8_t expected, uint32_t timeoutMs) {
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

    uint16_t _calculateCRC(const uint8_t* data, uint8_t len) {
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
};

// ============================================================================
// PM2230 Register Map (from Schneider PM2xxx Register List)
// All metering registers are FLOAT32 (2 × 16-bit, big-endian word order)
// ============================================================================
namespace PM2230Reg {
    // Current (A)
    constexpr uint16_t CURRENT_A       = 3000;
    constexpr uint16_t CURRENT_B       = 3002;
    constexpr uint16_t CURRENT_C       = 3004;
    constexpr uint16_t CURRENT_N       = 3006;
    constexpr uint16_t CURRENT_AVG     = 3010;

    // Voltage Line-to-Line (V)
    constexpr uint16_t VOLTAGE_AB      = 3020;
    constexpr uint16_t VOLTAGE_BC      = 3022;
    constexpr uint16_t VOLTAGE_CA      = 3024;
    constexpr uint16_t VOLTAGE_LL_AVG  = 3026;

    // Voltage Line-to-Neutral (V)
    constexpr uint16_t VOLTAGE_AN      = 3028;
    constexpr uint16_t VOLTAGE_BN      = 3030;
    constexpr uint16_t VOLTAGE_CN      = 3032;

    // Active Power (kW)
    constexpr uint16_t ACTIVE_POWER_A     = 3054;
    constexpr uint16_t ACTIVE_POWER_B     = 3056;
    constexpr uint16_t ACTIVE_POWER_C     = 3058;
    constexpr uint16_t ACTIVE_POWER_TOTAL = 3060;

    // Reactive Power (kVAR)
    constexpr uint16_t REACTIVE_POWER_A     = 3062;
    constexpr uint16_t REACTIVE_POWER_B     = 3064;
    constexpr uint16_t REACTIVE_POWER_C     = 3066;
    constexpr uint16_t REACTIVE_POWER_TOTAL = 3068;

    // Apparent Power (kVA)
    constexpr uint16_t APPARENT_POWER_A     = 3070;
    constexpr uint16_t APPARENT_POWER_B     = 3072;
    constexpr uint16_t APPARENT_POWER_C     = 3074;
    constexpr uint16_t APPARENT_POWER_TOTAL = 3076;

    // Power Factor
    constexpr uint16_t PF_A     = 3078;
    constexpr uint16_t PF_B     = 3080;
    constexpr uint16_t PF_C     = 3082;
    constexpr uint16_t PF_TOTAL = 3084;

    // Frequency (Hz)
    constexpr uint16_t FREQUENCY = 3110;
}

// ============================================================================
// PM2230 Data Structure
// ============================================================================
struct PM2230Data {
    // Line-to-Neutral Voltage (V)
    float Van = 0, Vbn = 0, Vcn = 0;
    // Line-to-Line Voltage (V)
    float Vab = 0, Vbc = 0, Vca = 0;
    // Line Current (A)  — phase currents are the line currents
    float Ia = 0, Ib = 0, Ic = 0;
    // Neutral Current (A)
    float In = 0;
    // Active Power (kW)
    float Pa = 0, Pb = 0, Pc = 0, Ptotal = 0;
    // Reactive Power (kVAR)
    float Qa = 0, Qb = 0, Qc = 0, Qtotal = 0;
    // Apparent Power (kVA)
    float Sa = 0, Sb = 0, Sc = 0, Stotal = 0;
    // Power Factor
    float PFa = 0, PFb = 0, PFc = 0, PFtotal = 0;
    // Frequency (Hz)
    float frequency = 0;
};

// ============================================================================
// PM2230 Power Meter Class
// ============================================================================
class PM2230 {
public:
    PM2230(ModbusRTU& modbus, uint8_t slaveAddress = 1)
        : _modbus(modbus), _slaveAddr(slaveAddress) {}

    PM2230Data data;

    bool readAll() {
        bool ok = true;
        ok &= readVoltage();
        ok &= readCurrent();
        ok &= readPower();
        ok &= readPowerFactor();
        ok &= readFrequency();
        return ok;
    }

    /// Read Line-to-Line and Line-to-Neutral voltages
    bool readVoltage() {
        uint16_t regs[6];

        // L-L: Vab, Vbc, Vca  (3020–3025, 3 × FLOAT32 = 6 regs)
        if (!_modbus.readHoldingRegisters(_slaveAddr, PM2230Reg::VOLTAGE_AB, 6, regs))
            return false;
        data.Vab = _toFloat(regs[0], regs[1]);
        data.Vbc = _toFloat(regs[2], regs[3]);
        data.Vca = _toFloat(regs[4], regs[5]);

        // L-N: Van, Vbn, Vcn  (3028–3033, 3 × FLOAT32 = 6 regs)
        if (!_modbus.readHoldingRegisters(_slaveAddr, PM2230Reg::VOLTAGE_AN, 6, regs))
            return false;
        data.Van = _toFloat(regs[0], regs[1]);
        data.Vbn = _toFloat(regs[2], regs[3]);
        data.Vcn = _toFloat(regs[4], regs[5]);

        return true;
    }

    /// Read phase currents and neutral current
    bool readCurrent() {
        // Ia, Ib, Ic, In  (3000–3007, 4 × FLOAT32 = 8 regs)
        uint16_t regs[8];
        if (!_modbus.readHoldingRegisters(_slaveAddr, PM2230Reg::CURRENT_A, 8, regs))
            return false;
        data.Ia = _toFloat(regs[0], regs[1]);
        data.Ib = _toFloat(regs[2], regs[3]);
        data.Ic = _toFloat(regs[4], regs[5]);
        data.In = _toFloat(regs[6], regs[7]);
        return true;
    }

    /// Read Active, Reactive and Apparent power (all 3 phases + total)
    bool readPower() {
        // P, Q, S  (3054–3077, 12 × FLOAT32 = 24 regs)
        uint16_t regs[24];
        if (!_modbus.readHoldingRegisters(_slaveAddr, PM2230Reg::ACTIVE_POWER_A, 24, regs))
            return false;

        data.Pa     = _toFloat(regs[0],  regs[1]);
        data.Pb     = _toFloat(regs[2],  regs[3]);
        data.Pc     = _toFloat(regs[4],  regs[5]);
        data.Ptotal = _toFloat(regs[6],  regs[7]);

        data.Qa     = _toFloat(regs[8],  regs[9]);
        data.Qb     = _toFloat(regs[10], regs[11]);
        data.Qc     = _toFloat(regs[12], regs[13]);
        data.Qtotal = _toFloat(regs[14], regs[15]);

        data.Sa     = _toFloat(regs[16], regs[17]);
        data.Sb     = _toFloat(regs[18], regs[19]);
        data.Sc     = _toFloat(regs[20], regs[21]);
        data.Stotal = _toFloat(regs[22], regs[23]);

        return true;
    }

    /// Read Power Factor (3 phases + total)
    bool readPowerFactor() {
        uint16_t regs[8];
        if (!_modbus.readHoldingRegisters(_slaveAddr, PM2230Reg::PF_A, 8, regs))
            return false;
        data.PFa     = _toFloat(regs[0], regs[1]);
        data.PFb     = _toFloat(regs[2], regs[3]);
        data.PFc     = _toFloat(regs[4], regs[5]);
        data.PFtotal = _toFloat(regs[6], regs[7]);
        return true;
    }

    /// Read Frequency
    bool readFrequency() {
        uint16_t regs[2];
        if (!_modbus.readHoldingRegisters(_slaveAddr, PM2230Reg::FREQUENCY, 2, regs))
            return false;
        data.frequency = _toFloat(regs[0], regs[1]);
        return true;
    }

    /// Print all readings to Serial monitor
    void printReadings() {
        Serial.println("========== PM2230 Power Meter Readings ==========");

        Serial.println("--- Voltage Line-to-Neutral ---");
        Serial.printf("  Van: %.2f V\n", data.Van);
        Serial.printf("  Vbn: %.2f V\n", data.Vbn);
        Serial.printf("  Vcn: %.2f V\n", data.Vcn);

        Serial.println("--- Voltage Line-to-Line ---");
        Serial.printf("  Vab: %.2f V\n", data.Vab);
        Serial.printf("  Vbc: %.2f V\n", data.Vbc);
        Serial.printf("  Vca: %.2f V\n", data.Vca);

        Serial.println("--- Current ---");
        Serial.printf("  Ia:  %.3f A\n", data.Ia);
        Serial.printf("  Ib:  %.3f A\n", data.Ib);
        Serial.printf("  Ic:  %.3f A\n", data.Ic);
        Serial.printf("  In:  %.3f A\n", data.In);

        Serial.println("--- Active Power (P) ---");
        Serial.printf("  Pa: %.3f kW   Pb: %.3f kW   Pc: %.3f kW\n",
                       data.Pa, data.Pb, data.Pc);
        Serial.printf("  P total: %.3f kW\n", data.Ptotal);

        Serial.println("--- Reactive Power (Q) ---");
        Serial.printf("  Qa: %.3f kVAR  Qb: %.3f kVAR  Qc: %.3f kVAR\n",
                       data.Qa, data.Qb, data.Qc);
        Serial.printf("  Q total: %.3f kVAR\n", data.Qtotal);

        Serial.println("--- Apparent Power (S) ---");
        Serial.printf("  Sa: %.3f kVA   Sb: %.3f kVA   Sc: %.3f kVA\n",
                       data.Sa, data.Sb, data.Sc);
        Serial.printf("  S total: %.3f kVA\n", data.Stotal);

        Serial.println("--- Power Factor ---");
        Serial.printf("  PFa: %.3f   PFb: %.3f   PFc: %.3f\n",
                       data.PFa, data.PFb, data.PFc);
        Serial.printf("  PF total: %.3f\n", data.PFtotal);

        Serial.printf("--- Frequency: %.2f Hz ---\n", data.frequency);
        Serial.println("=================================================");
    }

private:
    ModbusRTU& _modbus;
    uint8_t    _slaveAddr;

    /// Convert two 16-bit registers to IEEE 754 FLOAT32 (big-endian word order)
    float _toFloat(uint16_t regHi, uint16_t regLo) {
        uint32_t raw = ((uint32_t)regHi << 16) | regLo;
        float value;
        memcpy(&value, &raw, sizeof(float));
        return value;
    }
};

// ============================================================================
// Configuration — adjust pins to match your wiring
// ============================================================================
#define RS485_RX_PIN    16  // ESP32-S3 GPIO for RS485 RX
#define RS485_TX_PIN    17  // ESP32-S3 GPIO for RS485 TX
#define RS485_DE_PIN    18  // ESP32-S3 GPIO for RS485 DE/RE direction control
#define MODBUS_BAUD     9600
#define MODBUS_SLAVE_ID 1

// ============================================================================
// Global objects
// ============================================================================
ModbusRTU modbus(Serial1, RS485_DE_PIN);
PM2230    powerMeter(modbus, MODBUS_SLAVE_ID);

// ============================================================================
// Setup & Loop
// ============================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("PM2230 Power Meter — Modbus RTU Reader");
    Serial.println("Initializing RS485...");

    modbus.begin(MODBUS_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

    Serial.println("Ready.\n");
}

void loop() {
    Serial.println("Reading PM2230...");

    if (powerMeter.readAll()) {
        powerMeter.printReadings();
    } else {
        Serial.println("ERROR: Failed to read PM2230 — check wiring & slave ID");
    }

    delay(5000);
}