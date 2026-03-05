#!/usr/bin/env python3
"""
RS485 bus sniffer — passively listens to all traffic on the bus.

Connects the USB-RS485 adapter in receive-only mode (no DE toggling).
Groups bytes into Modbus frames using inter-byte gap timing, then decodes
each frame as a Modbus RTU request or response.

Usage:
    .venv/bin/python3 sniff_rs485.py

Wiring:
    USB-RS485 adapter A/B → same A/B terminals as the PM2200 meter.
    Make sure DE/RE on the adapter are pulled LOW (receive only).
    The ESP32 + adapter can both be on the bus at the same time.
"""

import serial
import serial.tools.list_ports
import sys
import time
import struct

# ============================================================================
# Configuration
# ============================================================================
BAUD            = 9600
# Inter-frame gap: Modbus spec = 3.5 char times.
# At 9600 baud, 1 char ≈ 1.04 ms  →  3.5 chars ≈ 3.65 ms.
# Use 15 ms to be safe across busy terminals.
FRAME_GAP_S     = 0.015

# ============================================================================
# CRC-16/Modbus
# ============================================================================
def calc_crc(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def crc_ok(frame: bytes) -> bool:
    if len(frame) < 4:
        return False
    expected = (frame[-1] << 8) | frame[-2]
    return calc_crc(frame[:-2]) == expected

# ============================================================================
# Modbus FC names
# ============================================================================
FC_NAMES = {
    0x01: "Read Coils",
    0x02: "Read Discrete Inputs",
    0x03: "Read Holding Regs",
    0x04: "Read Input Regs",
    0x05: "Write Single Coil",
    0x06: "Write Single Reg",
    0x10: "Write Multiple Regs",
}

EXCEPTION_CODES = {
    1: "Illegal Function",
    2: "Illegal Data Address",
    3: "Illegal Data Value",
    4: "Slave Device Failure",
}

# ============================================================================
# Decode a single Modbus frame
# ============================================================================
def decode_frame(raw: bytes, ts: float) -> str:
    if len(raw) < 4:
        return f"  [too short — {raw.hex(' ').upper()}]"

    addr = raw[0]
    fc   = raw[1]
    crc  = "CRC OK" if crc_ok(raw) else "CRC ERR"

    # Exception response
    if fc & 0x80:
        real_fc  = fc & 0x7F
        ex_code  = raw[2] if len(raw) > 2 else 0
        ex_str   = EXCEPTION_CODES.get(ex_code, f"Unknown ({ex_code:#04x})")
        return (f"  EXCEPTION  addr={addr}  FC{real_fc:02X}  ex={ex_str}  [{crc}]"
                f"  {raw.hex(' ').upper()}")

    fc_name = FC_NAMES.get(fc, f"FC{fc:02X}")

    # FC03/FC04 request: addr(1)+fc(1)+start(2)+count(2)+crc(2) = 8 bytes
    if fc in (0x03, 0x04) and len(raw) == 8:
        start = (raw[2] << 8) | raw[3]
        count = (raw[4] << 8) | raw[5]
        return (f"  REQUEST    addr={addr}  {fc_name}  "
                f"start_reg={start} (ds={start+1})  count={count}  [{crc}]")

    # FC03/FC04 response: addr(1)+fc(1)+byte_count(1)+data(N)+crc(2)
    if fc in (0x03, 0x04) and len(raw) >= 5:
        byte_count = raw[2]
        data_bytes  = raw[3:3+byte_count]
        floats = []
        for i in range(0, len(data_bytes) - 1, 4):
            if i + 4 <= len(data_bytes):
                hi  = (data_bytes[i]   << 8) | data_bytes[i+1]
                lo  = (data_bytes[i+2] << 8) | data_bytes[i+3]
                raw32 = ((hi << 16) | lo).to_bytes(4, 'big')
                val = struct.unpack('>f', raw32)[0]
                floats.append(f"{val:.4f}")
        floats_str = "  floats=[" + ", ".join(floats) + "]" if floats else ""
        return (f"  RESPONSE   addr={addr}  {fc_name}  "
                f"bytes={byte_count}{floats_str}  [{crc}]")

    # Generic
    return (f"  FRAME      addr={addr}  {fc_name}  "
            f"len={len(raw)}  [{crc}]  {raw.hex(' ').upper()}")

# ============================================================================
# Port scanner — finds USB-serial adapters and lets the user pick one
# ============================================================================
def scan_ports():
    # Grab all ports; prefer USB-serial adapter devices
    all_ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)

    # RS485 adapter chips: FTDI, CH340/CH341, CP2102, PL2303
    adapter_ports = [p for p in all_ports if any(k in (p.device + (p.description or '') +
                     (p.manufacturer or '')).lower()
                     for k in ('ftdi', 'ch340', 'ch341', 'cp210', 'pl2303', 'usbserial'))]

    # Fallback: any USB port that isn't ESP32 JTAG/CDC
    usb_ports = [p for p in all_ports if 'usb' in p.device.lower()
                 and 'espressif' not in (p.manufacturer or '').lower()
                 and 'jtag' not in (p.description or '').lower()]

    candidates = adapter_ports or usb_ports or all_ports

    if not candidates:
        print("ERROR: No serial ports found. Plug in the USB-RS485 adapter and retry.")
        sys.exit(1)

    if len(candidates) == 1:
        p = candidates[0]
        print(f"Auto-selected: {p.device}  ({p.description})")
        return p.device

    print("Multiple serial ports found — choose one:")
    for i, p in enumerate(candidates):
        mfr = f"  [{p.manufacturer}]" if p.manufacturer else ""
        print(f"  [{i}] {p.device}  {p.description}{mfr}")

    while True:
        try:
            choice = input(f"Enter number (0-{len(candidates)-1}): ").strip()
            idx = int(choice)
            if 0 <= idx < len(candidates):
                return candidates[idx].device
        except (ValueError, EOFError):
            pass
        print("  Invalid — try again.")

# ============================================================================
# Main sniffer loop
# ============================================================================
def main():
    port = scan_ports()
    print(f"Opening {port} at {BAUD} baud (passive listen) ...")
    print(f"Frame gap threshold: {FRAME_GAP_S*1000:.0f} ms")
    print("Listening for RS485 traffic — Ctrl+C to stop\n")

    ser = serial.Serial(port, BAUD, bytesize=8, parity='N',
                        stopbits=1, timeout=FRAME_GAP_S)

    frame_buf  = bytearray()
    last_byte_ts = None
    frame_count  = 0

    try:
        while True:
            try:
                chunk = ser.read(256)   # returns after timeout if nothing arrives
            except serial.SerialException as e:
                print(f"\nSerial error: {e}")
                print("Adapter disconnected or port lost. Reconnect and restart.")
                break
            now = time.time()

            if chunk:
                # If there was a gap since the last byte, flush the previous frame
                if last_byte_ts and (now - last_byte_ts) > FRAME_GAP_S and frame_buf:
                    frame_count += 1
                    ts_str = time.strftime("%H:%M:%S", time.localtime(last_byte_ts))
                    print(f"[{ts_str}] frame #{frame_count}  raw: {bytes(frame_buf).hex(' ').upper()}")
                    print(decode_frame(bytes(frame_buf), last_byte_ts))
                    frame_buf.clear()

                frame_buf.extend(chunk)
                last_byte_ts = now

            else:
                # Timeout — flush whatever is buffered
                if frame_buf:
                    frame_count += 1
                    ts_str = time.strftime("%H:%M:%S", time.localtime(last_byte_ts))
                    print(f"[{ts_str}] frame #{frame_count}  raw: {bytes(frame_buf).hex(' ').upper()}")
                    print(decode_frame(bytes(frame_buf), last_byte_ts))
                    frame_buf.clear()
                    last_byte_ts = None

    except KeyboardInterrupt:
        print(f"\nStopped. {frame_count} frames captured.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
