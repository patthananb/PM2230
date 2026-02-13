#ifndef PM2230_H
#define PM2230_H

#include <Arduino.h>
#include "ModbusRTU.h"

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
    // Line Current (A) — phase currents are the line currents
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
    PM2230(ModbusRTU& modbus, uint8_t slaveAddress = 1);

    PM2230Data data;

    bool readAll();
    bool readVoltage();
    bool readCurrent();
    bool readPower();
    bool readPowerFactor();
    bool readFrequency();
    void printReadings();

private:
    ModbusRTU& _modbus;
    uint8_t    _slaveAddr;

    /// Convert two 16-bit registers to IEEE 754 FLOAT32 (big-endian word order)
    float _toFloat(uint16_t regHi, uint16_t regLo);
};

#endif // PM2230_H
