#include <Arduino.h>
#include "ModbusRTU.h"
#include "PM2230.h"

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