"""CCD frame data model."""

from __future__ import annotations

from dataclasses import dataclass, field
import time

import numpy as np

from ccd_inspector.comm.protocol import (
    EFFECTIVE_END,
    EFFECTIVE_START,
    FM_FREQ,
    TOTAL_PIXELS,
)


@dataclass
class CCDFrame:
    """A single CCD readout frame with metadata."""

    pixels: np.ndarray  # (2200,) uint16
    timestamp: float = field(default_factory=time.perf_counter)
    sh_period: int = 20
    icg_period: int = 500_000
    lamp_id: int | None = None
    frame_number: int = 0

    @property
    def integration_time_ms(self) -> float:
        return self.icg_period / FM_FREQ * 1000

    @property
    def fps(self) -> float:
        total_us = self.icg_period / FM_FREQ * 1e6
        return 1e6 / total_us if total_us > 0 else 0.0

    @property
    def effective_pixels(self) -> np.ndarray:
        """2048 effective photosites (TCD1254 datasheet)."""
        return self.pixels[EFFECTIVE_START:EFFECTIVE_END]

    @property
    def peak(self) -> int:
        return int(np.max(self.effective_pixels))

    @property
    def mean(self) -> float:
        return float(np.mean(self.effective_pixels))

    @property
    def is_saturated(self) -> bool:
        return self.peak >= 4090
