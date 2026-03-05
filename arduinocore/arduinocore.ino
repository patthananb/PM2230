// ============================================================================
// PM2200 Power Meter — Modbus RTU Reader (Single-file Arduino sketch)
// Target: ESP32-C6
// Adapted from working XY-MD02 raw Modbus logic
// ============================================================================

#include <Arduino.h>
#include <string.h>  // for memcpy

// ============================================================================
// Configuration
// ============================================================================
#define RS485_RX_PIN    6
#define RS485_TX_PIN    7
#define RS485_DE_PIN    4
#define MODBUS_BAUD     9600
#define SLAVE_ADDR      2

// ============================================================================
// PM2200 Register Map — datasheet address minus 1 (0-based on wire)
// e.g. datasheet 3000 → wire 2999
// ============================================================================
namespace PM2200Reg {
    constexpr uint16_t ENERGY_KWH       = 2699;  // datasheet 2700 — Active Energy Delivered (kWh)
    constexpr uint16_t CURRENT_A        = 2999;  // datasheet 3000
    constexpr uint16_t VOLTAGE_AB       = 3019;  // datasheet 3020
    constexpr uint16_t VOLTAGE_AN       = 3027;  // datasheet 3028
    constexpr uint16_t ACTIVE_POWER_A   = 3053;  // datasheet 3054
    constexpr uint16_t REACTIVE_POWER_A = 3061;  // datasheet 3062
    constexpr uint16_t APPARENT_POWER_A = 3069;  // datasheet 3070
    constexpr uint16_t PF_A             = 3077;  // datasheet 3078
    constexpr uint16_t FREQUENCY        = 3109;  // datasheet 3110
}

// ============================================================================
// PM2200 Data Structure
// ============================================================================
struct PM2200Data {
    float Van = 0, Vbn = 0, Vcn = 0;
    float Ia = 0, Ib = 0, Ic = 0;
    float energy_kwh = 0;              // Active Energy Delivered (kWh)
    float Pa = 0, Pb = 0, Pc = 0, Ptotal = 0;
    float Qa = 0, Qb = 0, Qc = 0, Qtotal = 0;
    float Sa = 0, Sb = 0, Sc = 0, Stotal = 0;
};

// ============================================================================
// Global objects — same pattern as working code
// ============================================================================
HardwareSerial RS485(1);
PM2200Data data;

// ============================================================================
// Modbus exception code → string (mirrors Python exception_str())
// ============================================================================
const char* exception_str(byte code) {
    switch (code) {
        case 1: return "Illegal Function";
        case 2: return "Illegal Data Address";
        case 3: return "Illegal Data Value";
        case 4: return "Slave Device Failure";
        default: return "Unknown";
    }
}

// ============================================================================
// CRC-16/Modbus — identical to working code
// ============================================================================
uint16_t calc_modbus_crc(const byte* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) { crc >>= 1; crc ^= 0xA001; }
            else         { crc >>= 1; }
        }
    }
    return crc;
}

// ============================================================================
// Read Modbus registers — adapted from working code
// func_code: 0x03 = Holding Registers, 0x04 = Input Registers
// result buffer must be at least 3 + num_regs*2 + 2 bytes
// Returns 0 on success, -1 on error
// ============================================================================
int read_regs(byte dev_addr, byte func_code,
              uint16_t start_reg_addr, size_t num_regs, byte* result) {
    byte req_frame[8];
    req_frame[0] = dev_addr;
    req_frame[1] = func_code;
    req_frame[2] = (start_reg_addr >> 8) & 0xFF;
    req_frame[3] = start_reg_addr & 0xFF;
    req_frame[4] = (num_regs >> 8) & 0xFF;
    req_frame[5] = num_regs & 0xFF;

    uint16_t crc = calc_modbus_crc(req_frame, 6);
    req_frame[6] = crc & 0xFF;
    req_frame[7] = (crc >> 8) & 0xFF;

    // Print request frame for debugging
    Serial.print("  TX [");
    for (int i = 0; i < 8; i++) {
        Serial.printf("%02X", req_frame[i]);
        if (i < 7) Serial.print(" ");
    }
    Serial.println("]");

    // Flush any stale bytes
    while (RS485.available()) RS485.read();

    // Transmit
    digitalWrite(RS485_DE_PIN, HIGH);
    RS485.write(req_frame, 8);
    RS485.flush();
    digitalWrite(RS485_DE_PIN, LOW);

    delay(5);

    // Expected response: addr(1) + fc(1) + byte_count(1) + data(num_regs*2) + crc(2)
    size_t expected = 3 + num_regs * 2 + 2;
    byte resp_frame[256] = {0};
    size_t resp_len = RS485.readBytes(resp_frame, expected);

    if (resp_len == 0) {
        Serial.println("  ERROR: no response");
        return -1;
    }

    // Exception response: addr(1) + (fc|0x80)(1) + ex_code(1) + crc(2) = 5 bytes
    if (resp_len >= 3 && resp_frame[1] == (func_code | 0x80)) {
        byte ex = resp_frame[2];
        Serial.printf("  EXCEPTION: %s\n", exception_str(ex));
        return -2;
    }

    if (resp_len < expected) {
        Serial.printf("  ERROR: short response (%d of %d bytes)\n", resp_len, expected);
        return -1;
    }

    uint16_t expected_crc   = (resp_frame[resp_len - 1] << 8) | resp_frame[resp_len - 2];
    uint16_t calculated_crc = calc_modbus_crc(resp_frame, resp_len - 2);

    if (calculated_crc != expected_crc) {
        Serial.printf("  ERROR: CRC mismatch (got 0x%04X, expected 0x%04X)\n",
                      calculated_crc, expected_crc);
        return -1;
    }

    memcpy(result, resp_frame, resp_len);
    return 0;
}

// ============================================================================
// Convert two raw response bytes at offset to uint16_t (big-endian)
// ============================================================================
uint16_t bytes_to_u16(const byte* resp, int offset) {
    return (resp[offset] << 8) | resp[offset + 1];
}

// ============================================================================
// Convert two consecutive uint16_t registers to IEEE 754 float
// ============================================================================
float regs_to_float(uint16_t hi, uint16_t lo) {
    uint32_t raw = ((uint32_t)hi << 16) | lo;
    float value;
    memcpy(&value, &raw, sizeof(float));
    return value;
}

// ============================================================================
// Read a block of FLOAT32 registers and store into a float array
// data_offset: byte offset in response where register data starts (usually 3)
// ============================================================================
bool read_float_block(uint16_t start_reg, size_t num_regs, float* out) {
    byte resp[256];
    // FC03 only — PM2200 does not support FC04
    if (read_regs(SLAVE_ADDR, 0x03, start_reg, num_regs, resp) != 0) {
        return false;
    }
    for (size_t i = 0; i < num_regs / 2; i++) {
        uint16_t hi = bytes_to_u16(resp, 3 + i * 4);
        uint16_t lo = bytes_to_u16(resp, 3 + i * 4 + 2);
        out[i] = regs_to_float(hi, lo);
    }
    return true;
}

// ============================================================================
// Probe phase — mirrors Python probe loop
// Tests FC03 on reg 2999 (datasheet 3000). Prints OK/EXCEPTION/ERROR.
// Returns true if meter responded successfully.
// ============================================================================
bool probe() {
    Serial.println("==================================================");
    Serial.println("PROBING — scanning FC and register addresses");
    Serial.println("==================================================");

    byte resp[256];
    int rc = read_regs(SLAVE_ADDR, 0x03, PM2200Reg::CURRENT_A, 2, resp);

    if (rc == 0) {
        float val = regs_to_float(bytes_to_u16(resp, 3), bytes_to_u16(resp, 5));
        Serial.printf("  FC03  reg  %u  -> OK  value=%.4f\n", PM2200Reg::CURRENT_A, val);
        Serial.println("==================================================");
        Serial.printf("\nSUCCESS: FC03, start reg=%u\n", PM2200Reg::CURRENT_A);
        Serial.println("Polling Van, Vbn, Vcn every 2s ...\n");
        return true;
    } else if (rc == -2) {
        // exception message already printed by read_regs()
        Serial.println("==================================================");
    } else {
        Serial.printf("  FC03  reg  %u  -> (no response)\n", PM2200Reg::CURRENT_A);
        Serial.println("==================================================");
        Serial.println("\nERROR: Meter did not respond successfully to any probe.");
        Serial.println("Check: wiring, slave address, baud rate.");
        Serial.println("Confirmed config: 9600 baud, 8N1, slave 2.");
    }
    return false;
}

// ============================================================================
// Read all PM2200 values
// ============================================================================
bool readAll() {
    float buf[4];
    bool ok = true;

    // Line-to-neutral voltages Van, Vbn, Vcn (6 registers = 3 floats)
    Serial.print("  Reading Voltage L-N... ");
    if (read_float_block(PM2200Reg::VOLTAGE_AN, 6, buf)) {
        data.Van = buf[0]; data.Vbn = buf[1]; data.Vcn = buf[2];
        Serial.println("OK");
    } else { Serial.println("FAIL"); ok = false; }

    // Currents Ia, Ib, Ic (6 registers = 3 floats)
    Serial.print("  Reading Current... ");
    if (read_float_block(PM2200Reg::CURRENT_A, 6, buf)) {
        data.Ia = buf[0]; data.Ib = buf[1]; data.Ic = buf[2];
        Serial.println("OK");
    } else { Serial.println("FAIL"); ok = false; }

    // Active Energy Delivered kWh (2 registers = 1 float)
    Serial.print("  Reading Energy kWh... ");
    if (read_float_block(PM2200Reg::ENERGY_KWH, 2, buf)) {
        data.energy_kwh = buf[0];
        Serial.println("OK");
    } else { Serial.println("FAIL"); ok = false; }

    // Active power Pa, Pb, Pc, Ptotal (8 registers = 4 floats)
    Serial.print("  Reading Active Power... ");
    if (read_float_block(PM2200Reg::ACTIVE_POWER_A, 8, buf)) {
        data.Pa = buf[0]; data.Pb = buf[1]; data.Pc = buf[2]; data.Ptotal = buf[3];
        Serial.println("OK");
    } else { Serial.println("FAIL"); ok = false; }

    // Reactive power Qa, Qb, Qc, Qtotal (8 registers = 4 floats)
    Serial.print("  Reading Reactive Power... ");
    if (read_float_block(PM2200Reg::REACTIVE_POWER_A, 8, buf)) {
        data.Qa = buf[0]; data.Qb = buf[1]; data.Qc = buf[2]; data.Qtotal = buf[3];
        Serial.println("OK");
    } else { Serial.println("FAIL"); ok = false; }

    // Apparent power Sa, Sb, Sc, Stotal (8 registers = 4 floats)
    Serial.print("  Reading Apparent Power... ");
    if (read_float_block(PM2200Reg::APPARENT_POWER_A, 8, buf)) {
        data.Sa = buf[0]; data.Sb = buf[1]; data.Sc = buf[2]; data.Stotal = buf[3];
        Serial.println("OK");
    } else { Serial.println("FAIL"); ok = false; }

    return ok;
}

// ============================================================================
// Print readings
// ============================================================================
void printReadings() {
    Serial.println("========== PM2200 Power Meter Readings ==========");

    Serial.println("--- Voltage Line-to-Neutral ---");
    Serial.printf("  Van: %.2f V   Vbn: %.2f V   Vcn: %.2f V\n", data.Van, data.Vbn, data.Vcn);

    Serial.println("--- Current ---");
    Serial.printf("  Ia: %.3f A   Ib: %.3f A   Ic: %.3f A\n", data.Ia, data.Ib, data.Ic);

    Serial.println("--- Energy ---");
    Serial.printf("  Energy Delivered: %.3f kWh\n", data.energy_kwh);

    Serial.println("--- Active Power (P) ---");
    Serial.printf("  Pa: %.3f kW   Pb: %.3f kW   Pc: %.3f kW\n", data.Pa, data.Pb, data.Pc);
    Serial.printf("  P total: %.3f kW\n", data.Ptotal);

    Serial.println("--- Reactive Power (Q) ---");
    Serial.printf("  Qa: %.3f kVAR  Qb: %.3f kVAR  Qc: %.3f kVAR\n", data.Qa, data.Qb, data.Qc);
    Serial.printf("  Q total: %.3f kVAR\n", data.Qtotal);

    Serial.println("--- Apparent Power (S) ---");
    Serial.printf("  Sa: %.3f kVA   Sb: %.3f kVA   Sc: %.3f kVA\n", data.Sa, data.Sb, data.Sc);
    Serial.printf("  S total: %.3f kVA\n", data.Stotal);

    Serial.println("=================================================");
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
    Serial.begin(115200);
    // Wait up to 5s for USB CDC to enumerate on the host.
    // If a terminal is already open this returns immediately; if not, we
    // continue after 5s so the board still boots unattended.
    while (!Serial && millis() < 5000) { delay(10); }

    Serial.println("ESP32 is alive!");
    Serial.println("PM2200 Power Meter — Modbus RTU Reader");
    Serial.println("Initializing RS485...");

    pinMode(RS485_DE_PIN, OUTPUT);
    digitalWrite(RS485_DE_PIN, LOW);  // default receive mode

    RS485.begin(MODBUS_BAUD, SERIAL_8N1);  // explicit 8N1 — no parity
    RS485.setPins(RS485_RX_PIN, RS485_TX_PIN);
    RS485.setTimeout(2000);  // 2s read timeout, same as working Python code

    Serial.printf("Ready. Slave address: %d\n\n", SLAVE_ADDR);

    probe();
}

// ============================================================================
// Loop — read and print all measurements every 5s
// ============================================================================
void loop() {
    Serial.println("\nReading PM2200...");
    if (readAll()) {
        printReadings();
    } else {
        Serial.println("ERROR: one or more reads failed — check wiring & slave ID");
    }
    delay(5000);
}
