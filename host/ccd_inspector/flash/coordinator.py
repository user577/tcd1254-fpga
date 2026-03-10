"""Orchestrates FPGA-triggered flash capture sequences.

The FPGA handles all timing — the coordinator just:
1. Sends the flash config (which lamps, delay, duration, auto-sequence flag)
2. Sends the exposure config (SH, ICG, mode)
3. Collects the resulting frames, tagged by lamp ID
"""

from __future__ import annotations

import logging
from collections import defaultdict

import numpy as np
from PySide6.QtCore import QObject, Signal

from ccd_inspector.comm.protocol import (
    MODE_MULTI_ANGLE,
    MODE_RAW,
    build_flash_command,
    lamp_ids_to_mask,
)
from ccd_inspector.comm.serial_link import SerialLink
from ccd_inspector.core.frame import CCDFrame
from ccd_inspector.flash.lamp_config import LampArrayConfig
from ccd_inspector.flash.sequence import FlashSequence

log = logging.getLogger(__name__)


class FlashCaptureCoordinator(QObject):
    """Manages multi-angle flash capture sequences via FPGA."""

    # Emitted when a full sequence completes: {lamp_id: CCDFrame, ...}
    sequence_complete = Signal(dict)
    # Emitted for each individual frame as it arrives
    frame_captured = Signal(int, np.ndarray)  # lamp_id, pixels
    # Progress: (current_step, total_steps)
    progress = Signal(int, int)

    def __init__(self, serial_link: SerialLink, parent=None):
        super().__init__(parent)
        self._link = serial_link
        self._config: LampArrayConfig | None = None
        self._active_sequence: FlashSequence | None = None
        self._collected_frames: dict[int, CCDFrame] = {}
        self._expected_lamps: list[int] = []
        self._collecting = False

    def set_lamp_config(self, config: LampArrayConfig):
        self._config = config

    def start_sequence(self, sequence: FlashSequence, sh: int = 20, icg: int = 40_000):
        """Start a multi-angle capture sequence.

        Uses short integration time (default 10ms = 40000 fM cycles) since
        flash illumination dominates over ambient.
        """
        if not self._link.is_connected:
            log.error("Cannot start sequence: not connected")
            return

        self._active_sequence = sequence
        self._collected_frames.clear()
        self._expected_lamps = [step.lamp_id for step in sequence.steps]
        self._collecting = True

        # Tell FPGA which lamps and timing
        lamp_ids = [step.lamp_id for step in sequence.steps]
        mask = lamp_ids_to_mask(lamp_ids)

        # Use the first step's timing (could be per-step in future FPGA rev)
        delay = sequence.steps[0].flash_delay_us if sequence.steps else 0
        duration = sequence.steps[0].flash_duration_us if sequence.steps else 500

        self._link.send_flash_command(
            lamp_mask=mask,
            flash_delay_us=delay,
            flash_duration_us=duration,
            auto_sequence=True,
            raw_capture=True,
        )

        # Set exposure and start capture
        self._link.send_command(sh=sh, icg=icg, mode=MODE_RAW)
        self._link.frame_received.connect(self._on_frame)
        self._link.start_continuous(MODE_RAW)

        log.info(
            "Started sequence '%s': %d lamps, mask=0x%02X",
            sequence.name, len(lamp_ids), mask,
        )

    def stop(self):
        self._collecting = False
        self._link.frame_received.disconnect(self._on_frame)
        self._link.stop_continuous()

    def _on_frame(self, pixels: np.ndarray, timestamp: float):
        if not self._collecting or not self._expected_lamps:
            return

        # Assign frames to lamps in sequence order
        step_idx = len(self._collected_frames)
        if step_idx >= len(self._expected_lamps):
            # Sequence complete
            self._finish()
            return

        lamp_id = self._expected_lamps[step_idx]
        step = self._active_sequence.steps[step_idx]

        frame = CCDFrame(
            pixels=pixels,
            timestamp=timestamp,
            sh_period=self._link._sh,
            icg_period=self._link._icg,
            lamp_id=lamp_id,
            frame_number=step_idx,
        )

        self._collected_frames[lamp_id] = frame
        self.frame_captured.emit(lamp_id, pixels)
        self.progress.emit(step_idx + 1, len(self._expected_lamps))

        log.debug("Captured lamp %d (step %d/%d)",
                  lamp_id, step_idx + 1, len(self._expected_lamps))

        if len(self._collected_frames) >= len(self._expected_lamps):
            self._finish()

    def _finish(self):
        self._collecting = False
        self._link.stop_continuous()
        self.sequence_complete.emit(dict(self._collected_frames))
        log.info("Sequence complete: %d frames", len(self._collected_frames))
