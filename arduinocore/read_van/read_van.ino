// ============================================================================
// PM2200 Power Meter — Read Van only
// Sends a single Modbus FC03 request for Van (phase A to neutral voltage)
// Slave address: 2, Baud: 9600, Parity: None (8N1)
// Target: ESP32-C6
// ============================================================================

#include <Arduino.h>

// ---- RS485 pin config -------------------------------------------------------
#define RS485_RX_PIN    6
#define RS485_TX_PIN    7
#define RS485_DE_PIN    4
#define MODBUS_BAUD     9600
#define SLAVE_ADDR      2

// Van wire address = datasheet 3028 − 1 = 3027
#define VAN_REG         3027
#define NUM_REGS        2       // FLOAT32 = 2 registers

HardwareSerial RS485(1);

// ---- CRC-16/Modbus ----------------------------------------------------------
uint16_t calc_modbus_crc(const uint8_t *buf, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else              crc >>= 1;
        }
    }
    return crc;
}

// ---- Send FC03 request & read response, return Van in *out -----------------
bool read_van(float *out) {
    // Build FC03 frame
    uint8_t frame[8];
    frame[0] = SLAVE_ADDR;
    frame[1] = 0x03;                    // FC03 Read Holding Registers
    frame[2] = (VAN_REG >> 8) & 0xFF;  // Register address high
    frame[3] =  VAN_REG       & 0xFF;  // Register address low
    frame[4] = (NUM_REGS >> 8) & 0xFF; // Number of registers high
    frame[5] =  NUM_REGS       & 0xFF; // Number of registers low
    uint16_t crc = calc_modbus_crc(frame, 6);
    frame[6] = crc & 0xFF;             // CRC low
    frame[7] = (crc >> 8) & 0xFF;      // CRC high

    // Transmit
    digitalWrite(RS485_DE_PIN, HIGH);   // Enable driver
    delay(1);
    RS485.write(frame, 8);
    RS485.flush();                       // Wait until TX complete
    delay(1);
    digitalWrite(RS485_DE_PIN, LOW);    // Switch to receive

    Serial.print("TX: ");
    for (int i = 0; i < 8; i++) {
        if (frame[i] < 0x10) Serial.print('0');
        Serial.print(frame[i], HEX);
        Serial.print(' ');
    }
    Serial.println();

    // Expected response: slave(1) + FC(1) + byte_count(1) + data(4) + CRC(2) = 9 bytes
    uint8_t resp[9] = {0};
    int n = RS485.readBytes(resp, 9);

    Serial.print("RX (");
    Serial.print(n);
    Serial.print(" bytes): ");
    for (int i = 0; i < n; i++) {
        if (resp[i] < 0x10) Serial.print('0');
        Serial.print(resp[i], HEX);
        Serial.print(' ');
    }
    Serial.println();

    if (n < 9) {
        Serial.println("ERROR: short/no response");
        return false;
    }

    // Check for Modbus exception
    if (resp[1] & 0x80) {
        Serial.print("ERROR: Modbus exception code ");
        Serial.println(resp[2], HEX);
        return false;
    }

    // Validate CRC
    uint16_t resp_crc = calc_modbus_crc(resp, 7);
    uint16_t recv_crc = resp[7] | ((uint16_t)resp[8] << 8);
    if (resp_crc != recv_crc) {
        Serial.println("ERROR: CRC mismatch");
        return false;
    }

    // Decode FLOAT32 (big-endian word order: reg[0]=high word, reg[1]=low word)
    uint32_t raw = ((uint32_t)resp[3] << 24) |
                   ((uint32_t)resp[4] << 16) |
                   ((uint32_t)resp[5] <<  8) |
                    (uint32_t)resp[6];
    memcpy(out, &raw, 4);
    return true;
}

// ---- Setup ------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000) { delay(10); }

    pinMode(RS485_DE_PIN, OUTPUT);
    digitalWrite(RS485_DE_PIN, LOW);

    RS485.begin(MODBUS_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    RS485.setTimeout(2000);

    Serial.println("=== PM2200 Van reader ===");
    Serial.printf("Slave: %d, Baud: %d, Register: %d (0x%04X), Parity: None\n",
                  SLAVE_ADDR, MODBUS_BAUD, VAN_REG, VAN_REG);
}

// ---- Loop -------------------------------------------------------------------
void loop() {
    float van = 0;
    if (read_van(&van)) {
        Serial.printf("Van = %.2f V\n", van);
    }
    Serial.println("---");
    delay(2000);
}
