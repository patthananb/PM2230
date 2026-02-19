#include <Arduino.h>
#include "ModbusRTU.h"
#include "PM2200.h"

// ============================================================================
// Configuration — adjust pins to match your wiring
// ============================================================================
#define RS485_RX_PIN    14  // ESP32 GPIO for RS485 RX
#define RS485_TX_PIN    15  // ESP32 GPIO for RS485 TX
#define RS485_DE_PIN    18  // ESP32 GPIO for RS485 DE/RE direction control
#define MODBUS_BAUD     9600
#define MODBUS_CONFIG   SERIAL_8E1  // Schneider default: 8 data, Even parity, 1 stop

// ============================================================================
// Global objects
// ============================================================================
ModbusRTU modbus(Serial1, RS485_DE_PIN);
PM2200    powerMeter(modbus, 1);  // temporary address, will be updated after scan

uint8_t foundSlaveAddr = 0;
bool    slaveFound     = false;

// ============================================================================
// Scan Modbus bus for a responding device (addresses 1–247)
// ============================================================================
uint8_t scanForSlave() {
    uint16_t reg;
    // Use a short timeout so the scan doesn't take forever
    modbus.setResponseTimeout(500);

    Serial.println("Scanning Modbus addresses 1-20 ...");
    for (uint8_t addr = 1; addr <= 20; addr++) {
        Serial.printf("  scanning address %d ...\n", addr);
        // Try reading 1 register (Current A @ 3000) as a probe
        if (modbus.readHoldingRegisters(addr, 3000, 1, &reg)) {
            return addr;
        }
    }
    return 0;  // not found
}

// ============================================================================
// Setup & Loop
// ============================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("ESP32 is alive!");
    Serial.println("PM2200 Power Meter — Modbus RTU Reader");
    Serial.println("Initializing RS485...");

    modbus.begin(MODBUS_BAUD, MODBUS_CONFIG, RS485_RX_PIN, RS485_TX_PIN);

    Serial.println("Ready.\n");

    // --- Auto-detect slave address ---
    foundSlaveAddr = scanForSlave();

    if (foundSlaveAddr != 0) {
        slaveFound = true;
        Serial.printf("\n*** Found PM2200 at slave address: %d ***\n\n", foundSlaveAddr);
        powerMeter.setSlaveAddress(foundSlaveAddr);
        // Restore normal timeout for readings
        modbus.setResponseTimeout(1000);
    } else {
        Serial.println("\n*** ERROR: No Modbus device found on bus! ***");
        Serial.println("Check wiring, baud rate, and parity settings.");
    }
}

void loop() {
    if (!slaveFound) {
        Serial.println("No device found. Retrying scan...");
        foundSlaveAddr = scanForSlave();
        if (foundSlaveAddr != 0) {
            slaveFound = true;
            Serial.printf("\n*** Found PM2200 at slave address: %d ***\n\n", foundSlaveAddr);
            powerMeter.setSlaveAddress(foundSlaveAddr);
            modbus.setResponseTimeout(1000);
        } else {
            delay(5000);
            return;
        }
    }

    Serial.println("Reading PM2200...");

    if (powerMeter.readAll()) {
        powerMeter.printReadings();
    } else {
        Serial.println("ERROR: Failed to read PM2200 — check wiring & slave ID");
    }

    delay(5000);
}