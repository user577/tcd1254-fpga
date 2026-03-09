#!/usr/bin/env python3
"""
capture.py — Host interface for TCD1254 FPGA CCD readout on iCEstick.

Compatible with the STM32 serial protocol:
  Command:  12 bytes ('ER' + SH[4] + ICG[4] + mode[1] + avg[1])
  Position: 2 bytes (uint16 LE, value/10 = pixels, 0xFFFF = no shadow)
  Raw:      0xAA 0x55 sync + N×2 bytes (uint16 LE per pixel)

Usage:
  python capture.py                     # Position mode (default)
  python capture.py --raw               # Capture and plot one raw frame
  python capture.py --raw --save        # Save raw frame to CSV
  python capture.py --port COM5         # Specify COM port
  python capture.py --baud 921600       # Specify baud rate
"""

import argparse
import struct
import sys
import time
from collections import deque

import serial
import serial.tools.list_ports


# ---- Constants matching reference firmware ----
TOTAL_PIXELS = 2200
BAUD_RATE = 921_600
NO_SHADOW = 0xFFFF

# Default CCD timing (in fM cycles)
DEFAULT_SH = 20
DEFAULT_ICG = 500_000


def list_ports():
    """List available serial ports."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return []
    print("Available ports:")
    for p in ports:
        print(f"  {p.device:10s}  {p.description}")
    return ports


def build_command(sh_period, icg_period, mode, avg_count):
    """Build 12-byte command matching STM32 protocol."""
    cmd = bytearray(12)
    cmd[0] = 0x45  # 'E'
    cmd[1] = 0x52  # 'R'
    struct.pack_into('>I', cmd, 2, sh_period)
    struct.pack_into('>I', cmd, 6, icg_period)
    cmd[10] = mode
    cmd[11] = avg_count
    return bytes(cmd)


def open_port(port, baud):
    """Open serial port with appropriate settings."""
    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(0.1)
    ser.reset_input_buffer()
    return ser


def run_position_mode(ser, sh, icg, avg):
    """Continuous position readout — prints shadow position at each update."""
    cmd = build_command(sh, icg, mode=1, avg_count=avg)
    ser.write(cmd)
    print(f"Position mode started (SH={sh}, ICG={icg}, avg={avg})")
    print("Press Ctrl+C to stop.\n")

    history = deque(maxlen=200)
    count = 0
    t0 = time.time()

    try:
        while True:
            data = ser.read(2)
            if len(data) < 2:
                continue

            value = struct.unpack('<H', data)[0]
            count += 1

            if value == NO_SHADOW:
                pos_str = "NO SHADOW"
            else:
                pos_px = value / 10.0
                history.append(pos_px)
                pos_str = f"{pos_px:7.1f} px"

            # Print with FPS counter
            elapsed = time.time() - t0
            fps = count / elapsed if elapsed > 0 else 0

            if len(history) > 1:
                jitter = max(history) - min(history)
                print(f"\r  Pos: {pos_str}  |  FPS: {fps:6.1f}  |  "
                      f"Range: {jitter:5.1f} px  |  N={count}", end="", flush=True)
            else:
                print(f"\r  Pos: {pos_str}  |  FPS: {fps:6.1f}  |  N={count}",
                      end="", flush=True)

    except KeyboardInterrupt:
        print(f"\n\nStopped after {count} readings in {elapsed:.1f}s "
              f"({fps:.1f} FPS)")


def capture_raw_frame(ser, sh, icg):
    """Capture a single raw frame and return pixel array."""
    cmd = build_command(sh, icg, mode=2, avg_count=1)
    ser.write(cmd)

    # Expected: 0xAA 0x55 + TOTAL_PIXELS * 2 bytes
    expected_bytes = 2 + TOTAL_PIXELS * 2
    raw = b''
    t0 = time.time()
    timeout = 5.0  # seconds

    while len(raw) < expected_bytes + 256:
        if time.time() - t0 > timeout:
            break
        chunk = ser.read(min(1024, expected_bytes + 256 - len(raw)))
        if chunk:
            raw += chunk

    # Find sync marker
    sync_pos = -1
    for i in range(len(raw) - 1):
        if raw[i] == 0xAA and raw[i + 1] == 0x55:
            sync_pos = i
            break

    if sync_pos < 0:
        print(f"ERROR: Sync marker not found in {len(raw)} bytes")
        return None

    data_start = sync_pos + 2
    if len(raw) < data_start + TOTAL_PIXELS * 2:
        print(f"ERROR: Incomplete frame ({len(raw) - data_start} / "
              f"{TOTAL_PIXELS * 2} data bytes)")
        return None

    # Decode 12-bit pixel values (uint16 LE)
    pixels = []
    for i in range(TOTAL_PIXELS):
        offset = data_start + i * 2
        val = struct.unpack_from('<H', raw, offset)[0]
        pixels.append(val & 0x0FFF)  # Mask to 12 bits

    print(f"Captured {len(pixels)} pixels "
          f"(min={min(pixels)}, max={max(pixels)}, "
          f"mean={sum(pixels)/len(pixels):.0f})")

    return pixels


def plot_frame(pixels, title="TCD1254 Raw Frame"):
    """Plot a raw frame using matplotlib."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed — skipping plot")
        return

    fig, ax = plt.subplots(figsize=(14, 5))
    ax.plot(pixels, linewidth=0.5)
    ax.set_xlabel("Pixel Index")
    ax.set_ylabel("ADC Value (12-bit)")
    ax.set_title(title)
    ax.set_xlim(0, len(pixels))
    ax.set_ylim(0, 4095)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


def save_frame_csv(pixels, filename="frame.csv"):
    """Save raw frame to CSV."""
    with open(filename, 'w') as f:
        f.write("pixel_index,adc_value\n")
        for i, v in enumerate(pixels):
            f.write(f"{i},{v}\n")
    print(f"Saved {len(pixels)} pixels to {filename}")


def main():
    parser = argparse.ArgumentParser(
        description="TCD1254 FPGA CCD Readout — Host Interface")
    parser.add_argument('--port', '-p', type=str, default=None,
                        help='Serial port (auto-detect if omitted)')
    parser.add_argument('--baud', '-b', type=int, default=BAUD_RATE,
                        help=f'Baud rate (default: {BAUD_RATE})')
    parser.add_argument('--raw', '-r', action='store_true',
                        help='Capture raw frame instead of position mode')
    parser.add_argument('--save', '-s', action='store_true',
                        help='Save raw frame to CSV')
    parser.add_argument('--sh', type=int, default=DEFAULT_SH,
                        help=f'SH period in fM cycles (default: {DEFAULT_SH})')
    parser.add_argument('--icg', type=int, default=DEFAULT_ICG,
                        help=f'ICG period in fM cycles (default: {DEFAULT_ICG})')
    parser.add_argument('--avg', type=int, default=4, choices=range(1, 16),
                        help='Averaging count 1-15 (default: 4)')
    parser.add_argument('--list', '-l', action='store_true',
                        help='List available serial ports and exit')
    args = parser.parse_args()

    if args.list:
        list_ports()
        return

    # Auto-detect port if not specified
    port = args.port
    if port is None:
        ports = serial.tools.list_ports.comports()
        # Look for FTDI or iCEstick
        for p in ports:
            desc = (p.description or '').lower()
            if 'ftdi' in desc or 'ft2232' in desc or 'icestick' in desc:
                port = p.device
                print(f"Auto-detected: {port} ({p.description})")
                break
        if port is None:
            if ports:
                port = ports[0].device
                print(f"Using first available port: {port}")
            else:
                print("ERROR: No serial ports found")
                list_ports()
                sys.exit(1)

    print(f"Opening {port} at {args.baud} baud...")
    ser = open_port(port, args.baud)

    try:
        if args.raw:
            pixels = capture_raw_frame(ser, args.sh, args.icg)
            if pixels:
                if args.save:
                    save_frame_csv(pixels)
                plot_frame(pixels)
        else:
            run_position_mode(ser, args.sh, args.icg, args.avg)
    finally:
        ser.close()
        print("Port closed.")


if __name__ == '__main__':
    main()
