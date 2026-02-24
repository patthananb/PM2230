#!/usr/bin/env python3
"""
PM2200 — Probe script: scans FC03/FC04 across known register addresses
to find what the meter actually responds to.
Port and slave address auto-detected / configurable below.
"""

import serial
import struct
import time

# ============================================================================
# Configuration
# ============================================================================
PORT    = "/dev/tty.usbserial-A50285BI"
BAUD    = 9600
SLAVE   = 2
TIMEOUT = 2.0

# All register addresses are datasheet value minus 1 (0-based on wire)
# e.g. datasheet 3000 → wire 2999

# ============================================================================
# CRC-16/Modbus
# ============================================================================
def calc_crc(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])

# ============================================================================
# Send one Modbus request, return raw response bytes or None on error
# ============================================================================
def modbus_request(ser, slave, fc, start_reg, num_regs):
    req = bytes([slave, fc,
                 (start_reg >> 8) & 0xFF, start_reg & 0xFF,
                 (num_regs  >> 8) & 0xFF, num_regs  & 0xFF])
    req += calc_crc(req)

    ser.reset_input_buffer()
    ser.write(req)

    # Read extra bytes to account for echo from USB-RS485 adapter DE switching
    expected = 3 + num_regs * 2 + 2
    raw = ser.read(expected + 8)
    if not raw:
        return None

    # Strip leading bytes until we find the slave address byte
    for i in range(len(raw)):
        if raw[i] == slave:
            resp = raw[i:]
            if len(resp) >= expected:
                return resp[:expected]
    return raw[:expected]  # fallback

# ============================================================================
# Decode Modbus exception code
# ============================================================================
def exception_str(code):
    codes = {1: "Illegal Function", 2: "Illegal Data Address",
             3: "Illegal Data Value", 4: "Slave Device Failure"}
    return codes.get(code, f"Unknown ({code:#04x})")

# ============================================================================
# Convert response bytes to float
# ============================================================================
def resp_to_float(resp):
    hi = (resp[3] << 8) | resp[4]
    lo = (resp[5] << 8) | resp[6]
    raw = (hi << 16) | lo
    return struct.unpack('>f', raw.to_bytes(4, 'big'))[0]

# ============================================================================
# Main — probe then poll
# ============================================================================
def main():
    print(f"Opening {PORT} at {BAUD} baud, slave={SLAVE} ...\n")
    ser = serial.Serial(PORT, BAUD, bytesize=8, parity='N',
                        stopbits=1, timeout=TIMEOUT)

    # --- Probe phase ---
    probe_regs = [2999]   # datasheet 3000 → wire 2999
    probe_fcs  = [0x03]   # FC03 only (FC04 unsupported by PM2200)

    working_fc  = None
    working_reg = None

    print("=" * 50)
    print("PROBING — scanning FC and register addresses")
    print("=" * 50)

    for fc in probe_fcs:
        for reg in probe_regs:
            resp = modbus_request(ser, SLAVE, fc, reg, 2)
            if not resp:
                print(f"  FC{fc:02X}  reg {reg:5d}  -> (no response)")
                continue

            # Check for exception response
            if resp[1] == (fc | 0x80):
                ex = resp[2] if len(resp) > 2 else 0
                print(f"  FC{fc:02X}  reg {reg:5d}  -> EXCEPTION: {exception_str(ex)}")
                continue

            # Validate CRC
            if len(resp) >= 4:
                crc_ok = calc_crc(resp[:-2]) == resp[-2:]
                if crc_ok:
                    val = resp_to_float(resp)
                    print(f"  FC{fc:02X}  reg {reg:5d}  -> OK  value={val:.4f}")
                    if working_fc is None:
                        working_fc  = fc
                        working_reg = reg
                else:
                    print(f"  FC{fc:02X}  reg {reg:5d}  -> CRC ERROR  raw={resp.hex(' ').upper()}")
            else:
                print(f"  FC{fc:02X}  reg {reg:5d}  -> short resp={resp.hex(' ').upper()}")

    print("=" * 50)

    if working_fc is None:
        print("\nERROR: Meter did not respond successfully to any probe.")
        print("Check: wiring, slave address, baud rate. Confirmed config: 9600 baud, 8N1, slave 2.")
        ser.close()
        return

    print(f"\nSUCCESS: FC{working_fc:02X}, start reg={working_reg}")
    print(f"Polling Van, Vbn, Vcn every 2s ...\n")

    # Van=3027, Vbn=3029, Vcn=3031 (datasheet 3028/3030/3032 minus 1)
    # Read 6 registers in one request to get all 3 floats
    REG_VAN   = 3027
    NUM_REGS  = 6   # 3 floats × 2 registers each

    # --- Poll phase ---
    while True:
        resp = modbus_request(ser, SLAVE, 0x03, REG_VAN, NUM_REGS)
        expected_len = 3 + NUM_REGS * 2 + 2
        if resp and len(resp) == expected_len and resp[1] == 0x03:
            Van = resp_to_float(resp)
            Vbn = resp_to_float(resp[4:])   # shift by 4 bytes (2 regs)
            Vcn = resp_to_float(resp[8:])   # shift by 8 bytes (4 regs)
            print(f"  Van: {Van:.2f} V   Vbn: {Vbn:.2f} V   Vcn: {Vcn:.2f} V")
        else:
            raw = resp.hex(' ').upper() if resp else '(none)'
            print(f"  ERROR: {raw}")
        time.sleep(2)

if __name__ == "__main__":
    main()
