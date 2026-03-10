"""Exposure timing calculator.

Converts between human-readable units (ms, fps) and FPGA register values
(SH/ICG periods in fM cycles).
"""

from __future__ import annotations

from ccd_inspector.comm.protocol import FM_FREQ, TOTAL_PIXELS

# Readout takes TOTAL_PIXELS * 4 fM cycles (pixel rate = fM/4)
READOUT_CYCLES = TOTAL_PIXELS * 4  # 8800
MIN_SH = 4
MIN_ICG = READOUT_CYCLES + 200  # 9000 — readout + margin


def ms_to_icg(integration_ms: float) -> int:
    """Convert integration time in ms to ICG period in fM cycles."""
    return max(MIN_ICG, int(integration_ms * FM_FREQ / 1000))


def icg_to_ms(icg_period: int) -> float:
    """Convert ICG period in fM cycles to integration time in ms."""
    return icg_period / FM_FREQ * 1000


def fps_to_icg(target_fps: float) -> int:
    """Convert target frame rate to ICG period.

    Frame period = ICG period / fM.  Readout is included in ICG.
    """
    if target_fps <= 0:
        return MIN_ICG
    icg = int(FM_FREQ / target_fps)
    return max(MIN_ICG, icg)


def icg_to_fps(icg_period: int) -> float:
    """Convert ICG period to achievable frame rate."""
    if icg_period <= 0:
        return 0.0
    return FM_FREQ / icg_period


def max_fps() -> float:
    """Maximum achievable frame rate (limited by readout time)."""
    return icg_to_fps(MIN_ICG)


def validate_timing(sh: int, icg: int) -> tuple[bool, str]:
    """Validate SH/ICG parameter constraints.

    Returns (ok, message).
    """
    if sh < MIN_SH:
        return False, f"SH must be >= {MIN_SH} fM cycles"
    if icg < MIN_ICG:
        return False, f"ICG must be >= {MIN_ICG} fM cycles ({icg_to_ms(MIN_ICG):.2f} ms min)"
    if sh >= icg:
        return False, "SH must be less than ICG"
    return True, "OK"
