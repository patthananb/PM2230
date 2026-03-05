# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-C6 firmware that reads electrical measurements from a **Schneider PM2200** power meter using **Modbus RTU over RS485**.

There are two parallel implementations in this repo:

| Path | Build tool | Status |
|------|-----------|--------|
| `src/main.cpp` + `include/` | PlatformIO (`pio run`) | Original OOP implementation |
| `arduinocore/arduinocore.ino` | Arduino CLI (`arduino-cli`) | **Active working implementation** |

## Build & Flash Commands

### Arduino CLI (arduinocore.ino — active)

```bash
# Compile — CDCOnBoot=cdc is REQUIRED for Serial (USB CDC) to work on ESP32-C6
arduino-cli compile --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc arduinocore/arduinocore.ino

# Export binaries for esptool flashing
arduino-cli compile --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc --export-binaries arduinocore/arduinocore.ino

# Flash using esptool directly (more reliable than arduino-cli upload on ESP32-C6)
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

### Serial Monitor (ESP32-C6 USB CDC)

`arduino-cli monitor` and `pio device monitor` do NOT work in non-interactive shells (termios error). Use Python instead:

```bash
# Read serial output from ESP32
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

**Important notes for ESP32-C6 USB CDC:**
- `CDCOnBoot=cdc` must be set at compile time — without it `Serial` produces no output
- `arduino-cli upload` sometimes fails on ESP32-C6; use esptool directly
- The sketch uses `while (!Serial && millis() < 5000)` — open the serial port within 5s of boot to catch setup() output
- If `arduino-cli upload` fails with "No serial data received", press BOOT+RESET manually to enter download mode

### PlatformIO (src/main.cpp — original)

```bash
pio run                    # compile
pio run -t upload          # compile and flash
pio run -t clean
# Note: pio device monitor fails in non-interactive shells — use Python reader above
```

### Python test script (USB-RS485 adapter, no ESP32 needed)

```bash
# Create venv first time
python3 -m venv .venv && .venv/bin/pip install pyserial

# Run — confirmed working, reads Van/Vbn/Vcn ~229.5V
.venv/bin/python3 test_pm2200.py
```

## Architecture (arduinocore.ino)

```
setup()/loop()  →  readAll()  →  read_float_block()  →  read_regs()  →  HardwareSerial RS485  →  PM2200 meter
```

All logic is in a single `.ino` file. No external Modbus library. Key functions:

| Function | Role |
|----------|------|
| `read_regs()` | Builds Modbus FC03 frame, sends via RS485 (DE pin HIGH), reads response, validates CRC |
| `read_float_block()` | Calls `read_regs()`, converts register pairs to IEEE 754 float array |
| `readAll()` | Reads all meter values with per-group status prints (OK/FAIL) |
| `calc_modbus_crc()` | CRC-16/Modbus (poly 0xA001, init 0xFFFF) |
| `printReadings()` | Prints all values to USB CDC Serial |

## Hardware Configuration (confirmed by live testing)

| Define | Value | Notes |
|--------|-------|-------|
| `RS485_RX_PIN` | **6** | UART RX from RS485 module RO pin |
| `RS485_TX_PIN` | **7** | UART TX to RS485 module DI pin |
| `RS485_DE_PIN` | **4** | DE/RE direction control (tie DE+RE together on module) |
| `MODBUS_BAUD` | 9600 | |
| Serial config | **SERIAL_8N1** | No parity — confirmed working |
| `SLAVE_ADDR` | **2** | Fixed address of this meter |
| RS485 timeout | **2000 ms** | `RS485.setTimeout(2000)` |

`HardwareSerial RS485(1)` is used for RS485. `Serial` (USB CDC) is used for debug output.

## Key Implementation Details (confirmed by live testing)

- **Function code: FC03 only.** The PM2200 does NOT support FC04 (Input Registers). Using FC04 returns exception `84 01` (Illegal Function). Always use FC03 (Holding Registers).
- **Register addressing: datasheet value − 1.** The datasheet lists registers starting at 3000, but the wire address is 0-based. E.g. datasheet 3000 → wire address 2999. Using 3000 directly returns exception `83 03` (Illegal Data Value).
- **Parity: 8N1, not 8E1.** Schneider documentation says Even parity, but this unit only responds to No parity.
- **Slave address: 2.** Fixed — no need to scan.
- **Read timeout: 2000 ms.** Consistent with Python test results.
- **FLOAT32:** Each measurement = 2 consecutive registers. `regHi << 16 | regLo` → `memcpy` into `float` (big-endian word order).
- **DE pin:** HIGH = transmit, LOW = receive. Must go LOW before calling `readBytes()`.
- **RS485.begin():** Must pass `SERIAL_8N1` explicitly — `RS485.begin(MODBUS_BAUD, SERIAL_8N1)`.
- **USB CDC Serial:** Use `while (!Serial && millis() < 5000) { delay(10); }` in setup() — not `while(!Serial)` (blocks forever headless) and not `delay(2000)` (output lost before port opens).

## Register Map (wire addresses = datasheet − 1)

| Parameter | Datasheet | Wire address | Size |
|-----------|-----------|-------------|------|
| Current Ia, Ib, Ic | 3000–3005 | **2999–3004** | FLOAT32 each |
| Voltage Vab, Vbc, Vca | 3020–3025 | **3019–3024** | FLOAT32 each |
| Voltage Van, Vbn, Vcn | 3028–3033 | **3027–3032** | FLOAT32 each |
| Active Power Pa–Ptotal | 3054–3061 | **3053–3060** | FLOAT32 each |
| Reactive Power Qa–Qtotal | 3062–3069 | **3061–3068** | FLOAT32 each |
| Apparent Power Sa–Stotal | 3070–3077 | **3069–3076** | FLOAT32 each |
| Power Factor PFa–PFtotal | 3078–3085 | **3077–3084** | FLOAT32 each |
| Frequency | 3110–3111 | **3109–3110** | FLOAT32 |

> Source: `datasheet/Public_PM2xxx_PMC Register List_v1001.xls`

## Hardware Wiring (confirmed working)

### ESP32-C6 → RS485 Module

| ESP32-C6 GPIO | RS485 Module Pin | Signal |
|---|---|---|
| GPIO 7 | DI (Driver Input) | TX — data ESP32 sends |
| GPIO 6 | RO (Receiver Output) | RX — data ESP32 receives |
| GPIO 4 | DE + RE (tied together) | Direction control |
| 3.3V or 5V | VCC | Check module spec — MAX485 needs 5V |
| GND | GND | Common ground |

### RS485 Module → PM2200 Meter

| RS485 Module | PM2200 Terminal |
|---|---|
| A | A (D+) |
| B | B (D−) |

### Common wiring mistakes
- **A/B swapped** — most common cause of no response. Try reversing A and B first.
- **DE pin floating** — if DE is always HIGH, ESP32 drives the bus and can never receive. If always LOW, nothing is transmitted.
- **5V module powered from 3.3V** — MAX485-based modules need 5V VCC; 3.3V causes weak differential signal.
- **Wrong GPIO pins** — old README shows GPIO 14/15/18, those are wrong. Use 6/7/4.

## USB-RS485 Test Script (`test_pm2200.py`)

Standalone Python script for testing the meter directly via a USB-to-RS485 adapter (no ESP32 needed).

- Port: `/dev/tty.usbserial-A50285BI` (macOS — check with `ls /dev/tty.usb*`)
- **Confirmed working:** reads Van/Vbn/Vcn ≈ 229.5V
- Use this to verify the meter and A/B polarity before debugging ESP32 wiring
- Run with both devices on the bus to check if ESP32 RS485 module is holding DE HIGH

## Agentic Workflow (for spawning parallel agents)

When spawning agents to debug this project, use these three roles:

| Agent | Type | Task |
|-------|------|------|
| **Serial Monitor** | Bash | Read `/dev/tty.usbmodem101` at 115200 baud using `.venv/bin/python3` with pyserial. Capture output for ~45s and report all lines received. |
| **Code Fix** | general-purpose | Read `arduinocore/arduinocore.ino`, identify issues, apply fixes. Do not change confirmed-working values (pins, registers, baud, slave addr, FC03). |
| **Hardware Advice** | general-purpose | Read CLAUDE.md and README.md, give hardware checklist for ESP32-C6 → RS485 module → PM2200 wiring. |

**Serial monitor agent note:** Must use Python pyserial, not `arduino-cli monitor` or `pio device monitor` — both fail with termios error in non-interactive shells.

