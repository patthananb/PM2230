# PM2230 Power Meter — Code Explanation

## Overview

This project reads electrical measurements from a **Schneider PM2230** power meter using **Modbus RTU** over an **RS485** serial bus, running on an **ESP32-S3** microcontroller.

No external Modbus library is used — the protocol is implemented from scratch in OOP style.

---

## Code Architecture

```
┌──────────────┐        ┌──────────┐        ┌────────────┐
│   main.cpp   │───────▶│  PM2230  │───────▶│  ModbusRTU │──── RS485 ──── PM2230 Meter
│  (setup/loop)│        │  (meter) │        │  (master)  │
└──────────────┘        └──────────┘        └────────────┘
```

### Classes

| Class | Responsibility |
|-------|---------------|
| `ModbusRTU` | Low-level Modbus RTU master — builds frames, calculates CRC, sends/receives via RS485 |
| `PM2230` | High-level meter abstraction — knows the register addresses, reads FLOAT32 values, stores results in `PM2230Data` |
| `PM2230Data` | Plain struct holding all measurement values (voltages, currents, power, PF, frequency) |

### Flow

1. `setup()` — Initializes USB Serial (debug) and RS485 Serial via `ModbusRTU::begin()`
2. `loop()` — Every 5 seconds calls `PM2230::readAll()` which reads all register groups, then prints results
3. `PM2230::readAll()` calls individual methods: `readVoltage()`, `readCurrent()`, `readPower()`, `readPowerFactor()`, `readFrequency()`
4. Each read method calls `ModbusRTU::readHoldingRegisters()` with the appropriate register address and quantity
5. Raw 16-bit register pairs are converted to `float` via IEEE 754 conversion

---

## Modbus RTU Protocol

### What is Modbus RTU?

Modbus RTU (Remote Terminal Unit) is a serial communication protocol. "RTU" means data is transmitted in **binary** (not ASCII). Each byte is sent as-is on the wire, making it compact and efficient.

### RS485 Physical Layer

RS485 is a **differential** electrical standard using two wires (A and B). It supports:
- Long distances (up to 1200m)
- Multiple devices on one bus (up to 32 nodes)
- Half-duplex communication (one talks at a time)

The **DE (Driver Enable)** pin controls the RS485 transceiver direction:
- `DE = HIGH` → Transmit mode (ESP32 sends data)
- `DE = LOW` → Receive mode (ESP32 listens for response)

### Frame Structure (Function Code 0x03 — Read Holding Registers)

#### Request Frame (Master → Slave)

```
┌──────────┬──────────┬───────────────┬──────────────┬───────────┐
│ Slave ID │ Function │ Start Register│   Quantity   │    CRC    │
│  1 byte  │  1 byte  │   2 bytes     │   2 bytes    │  2 bytes  │
│          │  (0x03)  │  (Hi, Lo)     │  (Hi, Lo)    │ (Lo, Hi)  │
└──────────┴──────────┴───────────────┴──────────────┴───────────┘
     8 bytes total
```

**Example: Read Voltage A-N (register 3028, quantity 2)**

| Byte | Value | Meaning |
|------|-------|---------|
| 0 | `0x01` | Slave address 1 |
| 1 | `0x03` | Function code: Read Holding Registers |
| 2 | `0x0B` | Start register high byte (3028 = 0x0BD4) |
| 3 | `0xD4` | Start register low byte |
| 4 | `0x00` | Quantity high byte |
| 5 | `0x02` | Quantity low byte (2 registers = 1 FLOAT32) |
| 6 | `0xXX` | CRC low byte |
| 7 | `0xXX` | CRC high byte |

#### Response Frame (Slave → Master)

```
┌──────────┬──────────┬────────────┬─────────────────────────┬───────────┐
│ Slave ID │ Function │ Byte Count │     Register Data       │    CRC    │
│  1 byte  │  1 byte  │   1 byte   │  N × 2 bytes            │  2 bytes  │
│          │  (0x03)  │  (N×2)     │  (Hi, Lo per register)  │ (Lo, Hi)  │
└──────────┴──────────┴────────────┴─────────────────────────┴───────────┘
```

**Example: Response for Voltage A-N = 230.5 V**

| Byte | Value | Meaning |
|------|-------|---------|
| 0 | `0x01` | Slave address |
| 1 | `0x03` | Function code echo |
| 2 | `0x04` | Byte count (2 regs × 2 bytes = 4) |
| 3 | `0x43` | Register 3028 high byte |
| 4 | `0x66` | Register 3028 low byte |
| 5 | `0x80` | Register 3029 high byte |
| 6 | `0x00` | Register 3029 low byte |
| 7 | `0xXX` | CRC low byte |
| 8 | `0xXX` | CRC high byte |

### CRC-16 Calculation

Every Modbus RTU frame ends with a 16-bit CRC (Cyclic Redundancy Check) for error detection.

**Algorithm (CRC-16/Modbus):**
1. Initialize CRC = `0xFFFF`
2. For each byte in the message:
   - XOR the byte into the low byte of CRC
   - Repeat 8 times:
     - If the LSB of CRC is 1: shift right by 1, XOR with `0xA001`
     - If the LSB is 0: shift right by 1
3. The result is the 16-bit CRC (transmitted **low byte first**, then high byte)

```
CRC = 0xFFFF
for each byte:
    CRC = CRC XOR byte
    repeat 8 times:
        if (CRC & 1):
            CRC = (CRC >> 1) XOR 0xA001
        else:
            CRC = CRC >> 1
```

### Inter-frame Timing

Modbus RTU uses **silence** (no data) to delimit frames:
- **3.5 character times** of silence = frame boundary
- At 9600 baud: 1 character = 11 bits → 3.5 chars ≈ **4.01 ms**
- At ≥19200 baud: fixed at **1.75 ms** (per Modbus specification)

---

## IEEE 754 FLOAT32 Conversion

### Why is this needed?

The PM2230 stores measurement values as **IEEE 754 single-precision floating point** numbers. Each value occupies **2 consecutive Modbus registers** (2 × 16 bits = 32 bits).

### IEEE 754 Single-Precision Format

```
 31  30       23  22                    0
┌───┬──────────┬──────────────────────────┐
│ S │ Exponent │       Mantissa           │
│1b │  8 bits  │       23 bits            │
└───┴──────────┴──────────────────────────┘
```

| Field | Bits | Description |
|-------|------|-------------|
| **S** (Sign) | Bit 31 | `0` = positive, `1` = negative |
| **Exponent** | Bits 30–23 | Biased exponent (bias = 127) |
| **Mantissa** | Bits 22–0 | Fractional part (implicit leading 1) |

**Formula:**

$$value = (-1)^S \times 2^{(Exponent - 127)} \times (1 + Mantissa)$$

### Worked Example: 230.5 V

**Step 1: Convert 230.5 to binary**
- 230 = `11100110`
- 0.5 = `.1`
- 230.5 = `11100110.1`

**Step 2: Normalize**
- `1.11001101 × 2^7`
- Exponent = 7 + 127 (bias) = **134** = `10000110`
- Mantissa = `11001101 00000000 0000000` (23 bits, drop the leading 1)

**Step 3: Assemble 32 bits**

```
S  Exponent   Mantissa
0  10000110   11001101 00000000 0000000

Hex: 0x43668000
```

**Step 4: Split into two 16-bit Modbus registers**

```
Register Hi (first) : 0x4366
Register Lo (second): 0x8000
```

### Code Conversion (Register Pair → Float)

The PM2230 transmits registers in **big-endian word order** (high register first):

```cpp
float _toFloat(uint16_t regHi, uint16_t regLo) {
    uint32_t raw = ((uint32_t)regHi << 16) | regLo;
    float value;
    memcpy(&value, &raw, sizeof(float));
    return value;
}
```

1. `regHi` is shifted left 16 bits and combined with `regLo` to form a 32-bit integer
2. `memcpy` reinterprets those 32 bits as an IEEE 754 float (no undefined behavior, unlike pointer casts)

---

## PM2230 Register Map (used in this code)

| Parameter | Register | Size | Unit |
|-----------|----------|------|------|
| Current A | 3000 | FLOAT32 (2 regs) | A |
| Current B | 3002 | FLOAT32 | A |
| Current C | 3004 | FLOAT32 | A |
| Current N | 3006 | FLOAT32 | A |
| Voltage A-B | 3020 | FLOAT32 | V |
| Voltage B-C | 3022 | FLOAT32 | V |
| Voltage C-A | 3024 | FLOAT32 | V |
| Voltage A-N | 3028 | FLOAT32 | V |
| Voltage B-N | 3030 | FLOAT32 | V |
| Voltage C-N | 3032 | FLOAT32 | V |
| Active Power A | 3054 | FLOAT32 | kW |
| Active Power B | 3056 | FLOAT32 | kW |
| Active Power C | 3058 | FLOAT32 | kW |
| Active Power Total | 3060 | FLOAT32 | kW |
| Reactive Power A | 3062 | FLOAT32 | kVAR |
| Reactive Power B | 3064 | FLOAT32 | kVAR |
| Reactive Power C | 3066 | FLOAT32 | kVAR |
| Reactive Power Total | 3068 | FLOAT32 | kVAR |
| Apparent Power A | 3070 | FLOAT32 | kVA |
| Apparent Power B | 3072 | FLOAT32 | kVA |
| Apparent Power C | 3074 | FLOAT32 | kVA |
| Apparent Power Total | 3076 | FLOAT32 | kVA |
| Power Factor A | 3078 | FLOAT32 | — |
| Power Factor B | 3080 | FLOAT32 | — |
| Power Factor C | 3082 | FLOAT32 | — |
| Power Factor Total | 3084 | FLOAT32 | — |
| Frequency | 3110 | FLOAT32 | Hz |

> Register addresses are sourced from: `Public_PM2xxx_PMC Register List_v1001.xls`

---

## Wiring Summary

| ESP32-S3 Pin | RS485 Module | Description |
|-------------|-------------|-------------|
| GPIO 16 | RO (Receiver Out) | UART RX |
| GPIO 17 | DI (Driver In) | UART TX |
| GPIO 18 | DE + RE (tied) | Direction control |
| 3.3V | VCC | Power |
| GND | GND | Ground |

RS485 module A/B lines connect to the PM2230 meter's Modbus RS485 terminal.
# PM2230
