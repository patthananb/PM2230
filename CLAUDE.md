# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-C6 firmware that reads electrical measurements from a **Schneider PM2230/PM2200** power meter using **Modbus RTU over RS485**. The Modbus protocol is implemented from scratch — no external Modbus library is used.

## Build & Flash Commands

Uses [PlatformIO](https://platformio.org/). All commands are run from the repo root:

```bash
pio run                    # compile
pio run -t upload          # compile and flash to device
pio device monitor         # open serial monitor (115200 baud)
pio run -t upload && pio device monitor  # flash then monitor
pio run -t clean           # clean build artifacts
```

## Architecture

```
main.cpp  →  PM2200  →  ModbusRTU  →  RS485  →  PM2230 meter
```

| File | Role |
|------|------|
| `src/main.cpp` | `setup()`/`loop()`, RS485 pin config, auto-scans bus for slave address |
| `include/PM2200.h` / `src/PM2200.cpp` | Meter abstraction: register map (`PM2200Reg` namespace), `PM2200Data` struct, read methods |
| `include/ModbusRTU.h` / `src/ModbusRTU.cpp` | Raw Modbus RTU master: frame building, CRC-16, half-duplex DE pin control, receive with timeout |

**Data flow:** `PM2200::readAll()` calls individual `readVoltage()`, `readCurrent()`, `readPower()`, `readPowerFactor()`, `readFrequency()` methods, each calling `ModbusRTU::readHoldingRegisters()`. Raw 16-bit register pairs are converted to `float` via IEEE 754 (`memcpy` trick, big-endian word order).

## Hardware Configuration (in `main.cpp`)

| Define | Value | Purpose |
|--------|-------|---------|
| `RS485_RX_PIN` | 14 | UART RX from RS485 RO |
| `RS485_TX_PIN` | 15 | UART TX to RS485 DI |
| `RS485_DE_PIN` | 18 | Direction control (DE+RE tied) |
| `MODBUS_BAUD` | 9600 | Default Schneider baud rate |
| `MODBUS_CONFIG` | `SERIAL_8E1` | **8 data bits, Even parity, 1 stop** — Schneider default |

Serial1 is used for RS485; `Serial` (USB CDC) is used for debug output.

## Key Implementation Details

- **Slave auto-discovery:** On startup, `scanForSlave()` scans addresses 1–20 using a 500 ms timeout. After finding a device, timeout is restored to 1000 ms.
- **Inter-frame delay:** At 9600 baud = 4.01 ms (3.5 × 11 bits × 1/baud). At ≥19200 baud fixed at 1750 µs per spec.
- **CRC:** Standard CRC-16/Modbus (poly `0xA001`, init `0xFFFF`), transmitted low byte first.
- **FLOAT32:** Each measurement is 2 consecutive registers. `regHi << 16 | regLo` → `memcpy` into `float`.
- **Register addressing:** 0-based (e.g., Current A = 3000, not 30001 as shown in some Schneider docs).

## Register Map (from `datasheet/Public_PM2xxx_PMC Register List_v1001.xls`)

Registers start at 3000. All metering values are FLOAT32 (2 registers each). Key groups:
- 3000–3005: Currents Ia, Ib, Ic
- 3020–3025: Line-to-line voltages Vab, Vbc, Vca
- 3028–3033: Line-to-neutral voltages Van, Vbn, Vcn
- 3054–3077: Active (P), Reactive (Q), Apparent (S) power — 3 phases + total
- 3078–3085: Power factors — 3 phases + total
- 3110–3111: Frequency

## Utility Script

`extract_registers.py` parses the `.xls` datasheet to extract PM2230-compatible registers matching keywords (Current, Voltage, Power, Frequency, Factor). Requires `xlrd`: `pip install xlrd`.

## Note on Naming

The class is named `PM2200` (supports the broader PM2xxx family), but the target hardware is a **PM2230**. The `README.md` describes the PM2230 and references ESP32-S3 wiring — the active board in `platformio.ini` is **ESP32-C6** (pins differ from the README).
