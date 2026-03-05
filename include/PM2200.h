#ifndef PM2200_H
#define PM2200_H

#include <Arduino.h>
#include "ModbusRTU.h"

// ============================================================================
// PM2200 Register Map (from Schneider PM2xxx Register List)
// All metering registers are FLOAT32 (2 × 16-bit, big-endian word order)
// ============================================================================
namespace PM2200Reg {
    // Register addresses are 0-based on the wire (datasheet value - 1).
    // e.g. datasheet shows 3000 → send 2999 over Modbus.

    // Current (A)
    constexpr uint16_t CURRENT_A       = 2999;
    constexpr uint16_t CURRENT_B       = 3001;
    constexpr uint16_t CURRENT_C       = 3003;
    constexpr uint16_t CURRENT_AVG     = 3009;

    // Voltage Line-to-Line (V)
    constexpr uint16_t VOLTAGE_AB      = 3019;
    constexpr uint16_t VOLTAGE_BC      = 3021;
    constexpr uint16_t VOLTAGE_CA      = 3023;
    constexpr uint16_t VOLTAGE_LL_AVG  = 3025;

    // Voltage Line-to-Neutral (V)
    constexpr uint16_t VOLTAGE_AN      = 3027;
    constexpr uint16_t VOLTAGE_BN      = 3029;
    constexpr uint16_t VOLTAGE_CN      = 3031;

    // Active Power (kW)
    constexpr uint16_t ACTIVE_POWER_A     = 3053;
    constexpr uint16_t ACTIVE_POWER_B     = 3055;
    constexpr uint16_t ACTIVE_POWER_C     = 3057;
    constexpr uint16_t ACTIVE_POWER_TOTAL = 3059;

    // Reactive Power (kVAR)
    constexpr uint16_t REACTIVE_POWER_A     = 3061;
    constexpr uint16_t REACTIVE_POWER_B     = 3063;
    constexpr uint16_t REACTIVE_POWER_C     = 3065;
    constexpr uint16_t REACTIVE_POWER_TOTAL = 3067;

    // Apparent Power (kVA)
    constexpr uint16_t APPARENT_POWER_A     = 3069;
    constexpr uint16_t APPARENT_POWER_B     = 3071;
    constexpr uint16_t APPARENT_POWER_C     = 3073;
    constexpr uint16_t APPARENT_POWER_TOTAL = 3075;

    // Power Factor
    constexpr uint16_t PF_A     = 3077;
    constexpr uint16_t PF_B     = 3079;
    constexpr uint16_t PF_C     = 3081;
    constexpr uint16_t PF_TOTAL = 3083;

    // Frequency (Hz)
    constexpr uint16_t FREQUENCY = 3109;
}

// ============================================================================
// PM2200 Data Structure
// ============================================================================
struct PM2200Data {
    // Line-to-Neutral Voltage (V)
    float Van = 0, Vbn = 0, Vcn = 0;
    // Line-to-Line Voltage (V)
    float Vab = 0, Vbc = 0, Vca = 0;
    // Line Current (A)
    float Ia = 0, Ib = 0, Ic = 0;
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
// PM2200 Power Meter Class
// ============================================================================
class PM2200 {
public:
    PM2200(ModbusRTU& modbus, uint8_t slaveAddress = 1);

    PM2200Data data;

    bool readAll();
    bool readVoltage();
    bool readCurrent();
    bool readPower();
    bool readPowerFactor();
    bool readFrequency();
    void printReadings();
    void printNotResponding();

    void setSlaveAddress(uint8_t addr) { _slaveAddr = addr; }

private:
    ModbusRTU& _modbus;
    uint8_t    _slaveAddr;

    /// Convert two 16-bit registers to IEEE 754 FLOAT32 (big-endian word order)
    float _toFloat(uint16_t regHi, uint16_t regLo);
};

#endif // PM2200_H
