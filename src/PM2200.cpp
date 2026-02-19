#include "PM2200.h"

// ============================================================================
// Constructor
// ============================================================================
PM2200::PM2200(ModbusRTU& modbus, uint8_t slaveAddress)
    : _modbus(modbus), _slaveAddr(slaveAddress) {}

// ============================================================================
// Read all parameters
// ============================================================================
bool PM2200::readAll() {
    bool ok = true;
    ok &= readVoltage();
    ok &= readCurrent();
    ok &= readPower();
    ok &= readPowerFactor();
    ok &= readFrequency();
    return ok;
}

// ============================================================================
// Read Line-to-Line and Line-to-Neutral voltages
// ============================================================================
bool PM2200::readVoltage() {
    uint16_t regs[6];

    // L-L: Vab, Vbc, Vca  (3020–3025, 3 × FLOAT32 = 6 regs)
    if (!_modbus.readHoldingRegisters(_slaveAddr, PM2200Reg::VOLTAGE_AB, 6, regs))
        return false;
    data.Vab = _toFloat(regs[0], regs[1]);
    data.Vbc = _toFloat(regs[2], regs[3]);
    data.Vca = _toFloat(regs[4], regs[5]);

    // L-N: Van, Vbn, Vcn  (3028–3033, 3 × FLOAT32 = 6 regs)
    if (!_modbus.readHoldingRegisters(_slaveAddr, PM2200Reg::VOLTAGE_AN, 6, regs))
        return false;
    data.Van = _toFloat(regs[0], regs[1]);
    data.Vbn = _toFloat(regs[2], regs[3]);
    data.Vcn = _toFloat(regs[4], regs[5]);

    return true;
}

// ============================================================================
// Read phase currents (PM2200 has no neutral current input)
// ============================================================================
bool PM2200::readCurrent() {
    // Ia, Ib, Ic  (3000–3005, 3 × FLOAT32 = 6 regs)
    uint16_t regs[6];
    if (!_modbus.readHoldingRegisters(_slaveAddr, PM2200Reg::CURRENT_A, 6, regs))
        return false;
    data.Ia = _toFloat(regs[0], regs[1]);
    data.Ib = _toFloat(regs[2], regs[3]);
    data.Ic = _toFloat(regs[4], regs[5]);
    return true;
}

// ============================================================================
// Read Active, Reactive and Apparent power (all 3 phases + total)
// ============================================================================
bool PM2200::readPower() {
    // P, Q, S  (3054–3077, 12 × FLOAT32 = 24 regs)
    uint16_t regs[24];
    if (!_modbus.readHoldingRegisters(_slaveAddr, PM2200Reg::ACTIVE_POWER_A, 24, regs))
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

// ============================================================================
// Read Power Factor (3 phases + total)
// ============================================================================
bool PM2200::readPowerFactor() {
    uint16_t regs[8];
    if (!_modbus.readHoldingRegisters(_slaveAddr, PM2200Reg::PF_A, 8, regs))
        return false;
    data.PFa     = _toFloat(regs[0], regs[1]);
    data.PFb     = _toFloat(regs[2], regs[3]);
    data.PFc     = _toFloat(regs[4], regs[5]);
    data.PFtotal = _toFloat(regs[6], regs[7]);
    return true;
}

// ============================================================================
// Read Frequency
// ============================================================================
bool PM2200::readFrequency() {
    uint16_t regs[2];
    if (!_modbus.readHoldingRegisters(_slaveAddr, PM2200Reg::FREQUENCY, 2, regs))
        return false;
    data.frequency = _toFloat(regs[0], regs[1]);
    return true;
}

// ============================================================================
// Print all readings to Serial monitor
// ============================================================================
void PM2200::printReadings() {
    Serial.println("========== PM2200 Power Meter Readings ==========");

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

// ============================================================================
// Print error message when the power meter is not responding
// ============================================================================
void PM2200::printNotResponding() {
    Serial.printf("[PM2200] ERROR: Device at slave address %d is not responding.\n", _slaveAddr);
}

// ============================================================================
// IEEE 754 FLOAT32 conversion (two 16-bit registers → float)
// ============================================================================
float PM2200::_toFloat(uint16_t regHi, uint16_t regLo) {
    uint32_t raw = ((uint32_t)regHi << 16) | regLo;
    float value;
    memcpy(&value, &raw, sizeof(float));
    return value;
}
