# PM2200 Power Meter — ESP32-C6 Modbus RTU Reader

## Overview

This project reads electrical measurements from a **Schneider PM2200** power meter using **Modbus RTU** over an **RS485** serial bus, running on an **ESP32-C6** microcontroller.

No external Modbus library is used — the Modbus RTU protocol is implemented from scratch in a single Arduino sketch (`arduinocore/arduinocore.ino`).

### Measurements Read

| Parameter | Registers | Unit |
|-----------|-----------|------|
| Voltage Van, Vbn, Vcn | 3027–3032 (wire) | V |
| Current Ia, Ib, Ic | 2999–3004 | A |
| Energy Delivered | 2699–2700 | kWh |
| Active Power Pa, Pb, Pc, P total | 3053–3060 | kW |
| Reactive Power Qa, Qb, Qc, Q total | 3061–3068 | kVAR |
| Apparent Power Sa, Sb, Sc, S total | 3069–3076 | kVA |

---

## Hardware

### Physical Hardware Chain

```
┌─────────────────────────────────┐        ┌──────────────────┐        ┌─────────────────┐
│           ESP32-C6              │        │  RS485 Module    │        │  PM2200 Meter   │
│                                 │        │  (e.g. MAX485)   │        │                 │
│  GPIO7 (TX) ──────────────────▶ │ DI     │                  │        │                 │
│  GPIO6 (RX) ◀────────────────── │ RO     │              A ──│──────▶ │ A (D+)          │
│  GPIO4 (DE) ──────────────────▶ │ DE+RE  │              B ──│──────▶ │ B (D−)          │
│                                 │        │                  │        │                 │
│  USB CDC (Serial) ──▶ PC        │        │  Half-duplex     │        │  Slave addr: 2  │
│                                 │        │  RS485 bus       │        │  9600 baud, 8N1 │
└─────────────────────────────────┘        └──────────────────┘        └─────────────────┘
```

### Wiring

| ESP32-C6 Pin | RS485 Module | Signal |
|-------------|-------------|--------|
| GPIO 7 | DI (Driver Input) | TX |
| GPIO 6 | RO (Receiver Output) | RX |
| GPIO 4 | DE + RE (tied together) | Direction control |
| 3.3V or 5V | VCC | Check module spec — MAX485 needs 5V |
| GND | GND | Ground |

Serial config: **9600 baud, 8N1**

---

## Build & Flash

### Compile (CDCOnBoot=cdc is required for USB Serial on ESP32-C6)

```bash
arduino-cli compile --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc arduinocore/arduinocore.ino
```

### Export binaries

```bash
arduino-cli compile --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc --export-binaries arduinocore/arduinocore.ino
```

### Flash with esptool

```bash
ESPTOOL=~/Library/Arduino15/packages/esp32/tools/esptool_py/5.1.0/esptool
BUILD=arduinocore/build/esp32.esp32.esp32c6

$ESPTOOL --chip esp32c6 --port /dev/tty.usbmodem101 --baud 460800 \
  --before default-reset --after hard-reset write-flash -z \
  --flash-mode dio --flash-freq 80m --flash-size detect \
  0x0     $BUILD/arduinocore.ino.bootloader.bin \
  0x8000  $BUILD/arduinocore.ino.partitions.bin \
  0xe000  $BUILD/boot_app0.bin \
  0x10000 $BUILD/arduinocore.ino.bin
```

### Serial Monitor (Python — arduino-cli monitor fails on ESP32-C6)

```bash
PYTHONUNBUFFERED=1 .venv/bin/python3 -u -c "
import serial, time
s = serial.Serial('/dev/tty.usbmodem101', 115200, timeout=3)
start = time.time()
while time.time() - start < 60:
    line = s.readline()
    if line:
        print(line.decode(errors='replace').rstrip(), flush=True)
s.close()
"
```

---

## Software Architecture

### Call Chain (`arduinocore.ino`)

```
setup() / loop()
    └── readAll()
            ├── read_float_block(reg, num_regs)
            │       └── read_regs(slave, FC03, start_reg, num_regs)
            │               ├── build Modbus FC03 request frame
            │               ├── DE pin HIGH → transmit frame via RS485
            │               ├── DE pin LOW  → wait for response
            │               └── validate CRC → return raw bytes
            └── regs_to_float(hi, lo)  — IEEE 754 register pair → float
```

### Key Functions

| Function | Role |
|----------|------|
| `read_regs()` | Builds FC03 frame, drives DE pin, reads and CRC-validates response |
| `read_float_block()` | Calls `read_regs()`, converts each register pair to IEEE 754 float |
| `readAll()` | Reads all measurement groups; prints OK/FAIL per group |
| `calc_modbus_crc()` | CRC-16/Modbus (poly `0xA001`, init `0xFFFF`) |
| `printReadings()` | Formats and prints all values over USB CDC Serial |
| `probe()` | Runs once at startup; tests FC03 on reg 2999 to verify meter is responding |

---

## PM2200 Register Map

> All wire addresses = datasheet address − 1 (Modbus is 0-based on the wire)

| Parameter | Datasheet | Wire (use this) | Size | Unit |
|-----------|-----------|-----------------|------|------|
| Active Energy Delivered | 2700 | **2699** | FLOAT32 | kWh |
| Current Ia | 3000 | **2999** | FLOAT32 | A |
| Current Ib | 3002 | **3001** | FLOAT32 | A |
| Current Ic | 3004 | **3003** | FLOAT32 | A |
| Voltage Van | 3028 | **3027** | FLOAT32 | V |
| Voltage Vbn | 3030 | **3029** | FLOAT32 | V |
| Voltage Vcn | 3032 | **3031** | FLOAT32 | V |
| Active Power Pa | 3054 | **3053** | FLOAT32 | kW |
| Active Power Pb | 3056 | **3055** | FLOAT32 | kW |
| Active Power Pc | 3058 | **3057** | FLOAT32 | kW |
| Active Power Total | 3060 | **3059** | FLOAT32 | kW |
| Reactive Power Qa | 3062 | **3061** | FLOAT32 | kVAR |
| Reactive Power Qb | 3064 | **3063** | FLOAT32 | kVAR |
| Reactive Power Qc | 3066 | **3065** | FLOAT32 | kVAR |
| Reactive Power Total | 3068 | **3067** | FLOAT32 | kVAR |
| Apparent Power Sa | 3070 | **3069** | FLOAT32 | kVA |
| Apparent Power Sb | 3072 | **3071** | FLOAT32 | kVA |
| Apparent Power Sc | 3074 | **3073** | FLOAT32 | kVA |
| Apparent Power Total | 3076 | **3075** | FLOAT32 | kVA |

---

## ⚠️ Important Warnings (Confirmed by Live Testing)

### 1. FC04 is NOT supported — use FC03 only

```
02 84 01  →  Exception: Illegal Function
```

### 2. Register addresses are 0-based on the wire (datasheet − 1)

Using the datasheet address directly returns:
```
02 83 03  →  Exception: Illegal Data Value
```

### 3. Parity must be None (8N1), not Even (8E1)

Schneider docs say 8E1 but this unit only responds to 8N1.

---

## Modbus RTU Protocol Reference

### Frame Structure (FC03 — Read Holding Registers)

**Request (8 bytes):**
```
┌──────────┬──────────┬───────────────┬──────────────┬───────────┐
│ Slave ID │ Function │ Start Register│   Quantity   │    CRC    │
│  1 byte  │   0x03   │   2 bytes     │   2 bytes    │  2 bytes  │
└──────────┴──────────┴───────────────┴──────────────┴───────────┘
```

**Response:**
```
┌──────────┬──────────┬────────────┬─────────────────────────┬───────────┐
│ Slave ID │ Function │ Byte Count │     Register Data       │    CRC    │
│  1 byte  │   0x03   │  N×2 bytes │  N × 2 bytes (Hi, Lo)   │  2 bytes  │
└──────────┴──────────┴────────────┴─────────────────────────┴───────────┘
```

### CRC-16/Modbus

```
CRC = 0xFFFF
for each byte:
    CRC = CRC XOR byte
    repeat 8 times:
        if (CRC & 1):  CRC = (CRC >> 1) XOR 0xA001
        else:          CRC = CRC >> 1
```
Transmitted **low byte first**, then high byte.

### IEEE 754 FLOAT32 — Register Pair to Float

Each value = 2 consecutive registers (32 bits total, big-endian word order):

```cpp
float regs_to_float(uint16_t regHi, uint16_t regLo) {
    uint32_t raw = ((uint32_t)regHi << 16) | regLo;
    float value;
    memcpy(&value, &raw, sizeof(float));
    return value;
}
```

---

## Utility Scripts

| Script | Purpose |
|--------|---------|
| `test_pm2200.py` | Test meter directly via USB-RS485 adapter (no ESP32 needed). Probes then polls Van/Vbn/Vcn every 2s. |
| `sniff_rs485.py` | Passively sniffs all Modbus traffic on the bus; decodes FC03 requests/responses and exception frames. |

### Setup

```bash
python3 -m venv .venv
.venv/bin/pip install pyserial
```

