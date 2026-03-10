"""Calibration data management.

Handles dark frame subtraction, flat field correction, and pixel-to-mm
coordinate mapping for the TCD1254 CCD sensor.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ccd_inspector.comm.protocol import (
    ADC_MAX,
    EFFECTIVE_END,
    EFFECTIVE_START,
    TOTAL_PIXELS,
)


@dataclass
class CalibrationData:
    """Stores and applies CCD calibration."""

    dark_frame: np.ndarray | None = None
    flat_field: np.ndarray | None = None
    pixel_pitch_mm: float = 0.014       # TCD1254: 14 um pixel pitch
    sensor_offset_mm: float = 0.0       # Offset from mechanical reference

    def correct(self, pixels: np.ndarray) -> np.ndarray:
        """Apply dark subtraction and flat-field correction."""
        result = pixels.astype(np.float64)

        if self.dark_frame is not None:
            result = result - self.dark_frame.astype(np.float64)
            result = np.clip(result, 0, None)

        if self.flat_field is not None:
            flat = self.flat_field.astype(np.float64)
            mean_flat = np.mean(flat[EFFECTIVE_START:EFFECTIVE_END])
            safe = np.where(flat > 10, flat, mean_flat)
            result = result * (mean_flat / safe)

        return np.clip(result, 0, ADC_MAX).astype(np.uint16)

    def pixel_to_mm(self, pixel_index: float) -> float:
        """Convert pixel index to mm from sensor center."""
        center = (EFFECTIVE_START + EFFECTIVE_END) / 2
        return (pixel_index - center) * self.pixel_pitch_mm + self.sensor_offset_mm

    def mm_to_pixel(self, mm: float) -> float:
        """Convert mm from sensor center to pixel index."""
        center = (EFFECTIVE_START + EFFECTIVE_END) / 2
        return (mm - self.sensor_offset_mm) / self.pixel_pitch_mm + center

    def width_px_to_mm(self, width_px: float) -> float:
        """Convert width in pixels to mm."""
        return width_px * self.pixel_pitch_mm

    def capture_dark(self, frames: list[np.ndarray]):
        """Average multiple dark frames (lens capped, no illumination)."""
        if not frames:
            return
        stacked = np.stack([f.astype(np.float64) for f in frames])
        self.dark_frame = np.mean(stacked, axis=0).astype(np.uint16)

    def capture_flat(self, frames: list[np.ndarray]):
        """Average multiple flat field frames (uniform illumination).

        Should be captured after dark frame so dark can be subtracted first.
        """
        if not frames:
            return
        stacked = np.stack([f.astype(np.float64) for f in frames])
        avg = np.mean(stacked, axis=0)

        if self.dark_frame is not None:
            avg = avg - self.dark_frame.astype(np.float64)
            avg = np.clip(avg, 1, None)

        self.flat_field = avg.astype(np.uint16)

    def calibrate_pitch(
        self,
        pixels: np.ndarray,
        known_spacing_mm: float,
        edge_positions: list[float],
    ):
        """Calibrate pixel pitch from known feature spacing.

        Args:
            pixels: CCD frame with calibration target.
            known_spacing_mm: Physical spacing between calibration features.
            edge_positions: Detected edge positions (pixel indices) of
                calibration features. Needs at least 2 edges.
        """
        if len(edge_positions) < 2:
            return

        # Compute mean spacing in pixels between consecutive edges
        spacings = [
            edge_positions[i + 1] - edge_positions[i]
            for i in range(len(edge_positions) - 1)
        ]
        mean_spacing_px = np.mean(spacings)

        if mean_spacing_px > 0:
            self.pixel_pitch_mm = known_spacing_mm / mean_spacing_px

    def save(self, directory: Path):
        """Save calibration data to directory."""
        directory.mkdir(parents=True, exist_ok=True)

        if self.dark_frame is not None:
            np.save(directory / 'dark_frame.npy', self.dark_frame)
        if self.flat_field is not None:
            np.save(directory / 'flat_field.npy', self.flat_field)

        # Save scalar params
        import json
        params = {
            'pixel_pitch_mm': self.pixel_pitch_mm,
            'sensor_offset_mm': self.sensor_offset_mm,
        }
        (directory / 'calibration.json').write_text(json.dumps(params, indent=2))

    @classmethod
    def load(cls, directory: Path) -> CalibrationData:
        """Load calibration data from directory."""
        cal = cls()

        dark_path = directory / 'dark_frame.npy'
        if dark_path.exists():
            cal.dark_frame = np.load(dark_path)

        flat_path = directory / 'flat_field.npy'
        if flat_path.exists():
            cal.flat_field = np.load(flat_path)

        params_path = directory / 'calibration.json'
        if params_path.exists():
            import json
            params = json.loads(params_path.read_text())
            cal.pixel_pitch_mm = params.get('pixel_pitch_mm', 0.014)
            cal.sensor_offset_mm = params.get('sensor_offset_mm', 0.0)

        return cal
