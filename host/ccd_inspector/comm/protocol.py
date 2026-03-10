"""FPGA command/response protocol for TCD1254 CCD readout.

Existing command format (12 bytes, backward-compatible):
    'E' 'R' + SH[4 BE] + ICG[4 BE] + mode[1] + avg[1]

Extended flash command (8 bytes):
    'F' 'L' + lamp_mask[1] + flash_delay_us[2 BE] + flash_duration_us[2 BE] + flags[1]

    lamp_mask:  Bitmask of lamps to fire (bits 0-7 = lamps 0-7)
    flash_delay_us:  Delay from ICG rising edge to flash trigger (0 = start of integration)
    flash_duration_us:  Flash pulse width in microseconds
    flags:  bit 0 = auto-sequence (cycle through set bits one lamp per frame)
            bit 1 = capture raw frame after each flash (vs position mode)

Response formats:
    Position mode (1): 2 bytes uint16 LE per frame (0xFFFF = no shadow)
    Raw mode (2):      0xAA 0x55 sync + 2200 * 2 bytes uint16 LE
    Multi-angle (3):   0xAA lamp_id[1] + 2200 * 2 bytes uint16 LE (per lamp in sequence)
"""

from __future__ import annotations

import struct

import numpy as np

# Hardware constants
TOTAL_PIXELS = 2200
EFFECTIVE_START = 32
EFFECTIVE_END = 2080
EFFECTIVE_PIXELS = EFFECTIVE_END - EFFECTIVE_START  # 2048
FM_FREQ = 4_000_000
BAUD_RATE = 921_600
ADC_BITS = 12
ADC_MAX = (1 << ADC_BITS) - 1  # 4095
MAX_LAMPS = 8

# Protocol constants
CMD_HEADER = b'ER'
CMD_LENGTH = 12
FLASH_CMD_HEADER = b'FL'
FLASH_CMD_LENGTH = 8
SYNC_MARKER = b'\xAA\x55'
MULTI_SYNC_MARKER = b'\xAA'  # followed by lamp_id byte
NO_SHADOW = 0xFFFF
RAW_FRAME_BYTES = 2 + TOTAL_PIXELS * 2  # 4402

# Modes
MODE_POSITION = 1
MODE_RAW = 2
MODE_MULTI_ANGLE = 3

# Flash flags
FLASH_FLAG_AUTO_SEQUENCE = 0x01
FLASH_FLAG_RAW_CAPTURE = 0x02


def build_command(
    sh_period: int = 20,
    icg_period: int = 500_000,
    mode: int = MODE_POSITION,
    avg_count: int = 4,
) -> bytes:
    """Build 12-byte exposure command packet."""
    cmd = bytearray(CMD_LENGTH)
    cmd[0:2] = CMD_HEADER
    struct.pack_into('>I', cmd, 2, sh_period)
    struct.pack_into('>I', cmd, 6, icg_period)
    cmd[10] = mode & 0xFF
    cmd[11] = avg_count & 0xFF
    return bytes(cmd)


def build_flash_command(
    lamp_mask: int = 0x01,
    flash_delay_us: int = 0,
    flash_duration_us: int = 500,
    auto_sequence: bool = False,
    raw_capture: bool = True,
) -> bytes:
    """Build 8-byte flash lamp command.

    Args:
        lamp_mask: Bitmask of lamps to fire (bits 0-7).
        flash_delay_us: Delay from ICG edge to flash trigger.
        flash_duration_us: Flash pulse width in microseconds.
        auto_sequence: Cycle through lamps one per frame.
        raw_capture: Capture raw frame (vs position mode) per flash.
    """
    cmd = bytearray(FLASH_CMD_LENGTH)
    cmd[0:2] = FLASH_CMD_HEADER
    cmd[2] = lamp_mask & 0xFF
    struct.pack_into('>H', cmd, 3, min(flash_delay_us, 0xFFFF))
    struct.pack_into('>H', cmd, 5, min(flash_duration_us, 0xFFFF))
    flags = 0
    if auto_sequence:
        flags |= FLASH_FLAG_AUTO_SEQUENCE
    if raw_capture:
        flags |= FLASH_FLAG_RAW_CAPTURE
    cmd[7] = flags
    return bytes(cmd)


def parse_position(data: bytes) -> float | None:
    """Parse 2-byte position response.

    Returns shadow center in pixels (0.1 px resolution), or None if no shadow.
    """
    if len(data) < 2:
        return None
    value = struct.unpack('<H', data[:2])[0]
    if value == NO_SHADOW:
        return None
    return value / 10.0


def parse_raw_frame(data: bytes) -> np.ndarray | None:
    """Find sync marker and extract pixel array.

    Returns (2200,) uint16 array masked to 12 bits, or None on failure.
    """
    sync_pos = data.find(SYNC_MARKER)
    if sync_pos < 0:
        return None

    data_start = sync_pos + 2
    needed = TOTAL_PIXELS * 2
    if len(data) < data_start + needed:
        return None

    pixels = np.frombuffer(data, dtype='<u2', count=TOTAL_PIXELS, offset=data_start)
    return (pixels & ADC_MAX).copy()


def parse_multi_angle_frame(data: bytes) -> tuple[int, np.ndarray] | None:
    """Parse a multi-angle frame response.

    Returns (lamp_id, pixels) tuple, or None on failure.
    Frame format: 0xAA + lamp_id[1] + 2200 * uint16 LE
    """
    # Find 0xAA byte that isn't part of 0xAA55 sync
    for i in range(len(data) - 1):
        if data[i] == 0xAA and data[i + 1] != 0x55:
            lamp_id = data[i + 1]
            data_start = i + 2
            needed = TOTAL_PIXELS * 2
            if len(data) < data_start + needed:
                return None
            pixels = np.frombuffer(
                data, dtype='<u2', count=TOTAL_PIXELS, offset=data_start
            )
            return lamp_id, (pixels & ADC_MAX).copy()
    return None


def lamp_mask_to_ids(mask: int) -> list[int]:
    """Convert bitmask to list of lamp IDs."""
    return [i for i in range(MAX_LAMPS) if mask & (1 << i)]


def lamp_ids_to_mask(ids: list[int]) -> int:
    """Convert list of lamp IDs to bitmask."""
    mask = 0
    for i in ids:
        if 0 <= i < MAX_LAMPS:
            mask |= 1 << i
    return mask
