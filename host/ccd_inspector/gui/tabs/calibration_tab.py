"""Calibration workflow tab.

Guided steps:
1. Dark frame capture (lens capped)
2. Flat field capture (uniform illumination)
3. Pixel pitch calibration (known target)
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ccd_inspector.comm.protocol import MODE_RAW
from ccd_inspector.comm.ccd_link import CcdLink
from ccd_inspector.core.calibration import CalibrationData
from ccd_inspector.core.config import CALIBRATION_DIR
from ccd_inspector.gui.widgets.waveform_widget import WaveformWidget


class CalibrationTab(QWidget):
    """Guided calibration workflow."""

    def __init__(
        self,
        link: CcdLink,
        calibration: CalibrationData,
        parent=None,
    ):
        super().__init__(parent)
        self._link = link
        self._calibration = calibration
        self._capture_buffer: list[np.ndarray] = []
        self._capture_target = 0
        self._capture_mode = ""  # 'dark' or 'flat'

        self._build_ui()
        self._connect_signals()
        self._update_status()

    def _build_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        # Left panel: calibration steps
        ctrl_panel = QWidget()
        ctrl_layout = QVBoxLayout(ctrl_panel)
        ctrl_panel.setMaximumWidth(380)

        # -- Capture settings --
        cap_group = QGroupBox("Capture Settings")
        cap_form = QFormLayout(cap_group)

        self._spin_num_frames = QSpinBox()
        self._spin_num_frames.setRange(1, 100)
        self._spin_num_frames.setValue(16)
        cap_form.addRow("Frames to Average:", self._spin_num_frames)

        ctrl_layout.addWidget(cap_group)

        # -- Step 1: Dark Frame --
        dark_group = QGroupBox("1. Dark Frame")
        dark_layout = QVBoxLayout(dark_group)

        dark_layout.addWidget(QLabel(
            "Cap the lens or block all light.\n"
            "Captures the sensor's dark current baseline."
        ))

        self._btn_dark = QPushButton("Capture Dark Frame")
        dark_layout.addWidget(self._btn_dark)

        self._lbl_dark_status = QLabel("Not captured")
        dark_layout.addWidget(self._lbl_dark_status)

        ctrl_layout.addWidget(dark_group)

        # -- Step 2: Flat Field --
        flat_group = QGroupBox("2. Flat Field")
        flat_layout = QVBoxLayout(flat_group)

        flat_layout.addWidget(QLabel(
            "Place a uniform diffuser in front of the sensor.\n"
            "Normalizes per-pixel sensitivity variation."
        ))

        self._btn_flat = QPushButton("Capture Flat Field")
        flat_layout.addWidget(self._btn_flat)

        self._lbl_flat_status = QLabel("Not captured")
        flat_layout.addWidget(self._lbl_flat_status)

        ctrl_layout.addWidget(flat_group)

        # -- Step 3: Pixel Pitch --
        pitch_group = QGroupBox("3. Pixel Pitch")
        pitch_form = QFormLayout(pitch_group)

        pitch_form.addRow(QLabel(
            "Place a ruler/target with known spacing."
        ))

        self._spin_known_spacing = QDoubleSpinBox()
        self._spin_known_spacing.setRange(0.1, 100.0)
        self._spin_known_spacing.setValue(1.0)
        self._spin_known_spacing.setSuffix(" mm")
        self._spin_known_spacing.setDecimals(3)
        pitch_form.addRow("Known Spacing:", self._spin_known_spacing)

        self._spin_pixel_pitch = QDoubleSpinBox()
        self._spin_pixel_pitch.setRange(0.001, 1.0)
        self._spin_pixel_pitch.setValue(0.014)
        self._spin_pixel_pitch.setSuffix(" mm")
        self._spin_pixel_pitch.setDecimals(4)
        pitch_form.addRow("Pixel Pitch:", self._spin_pixel_pitch)

        self._btn_auto_pitch = QPushButton("Auto-Calibrate from Edges")
        pitch_form.addRow(self._btn_auto_pitch)

        ctrl_layout.addWidget(pitch_group)

        # -- Save/Load --
        io_group = QGroupBox("Save / Load")
        io_layout = QHBoxLayout(io_group)

        self._btn_save = QPushButton("Save Calibration")
        self._btn_load = QPushButton("Load Calibration")
        self._btn_clear = QPushButton("Clear All")
        io_layout.addWidget(self._btn_save)
        io_layout.addWidget(self._btn_load)
        io_layout.addWidget(self._btn_clear)

        ctrl_layout.addWidget(io_group)

        # Progress bar
        self._progress = QProgressBar()
        self._progress.setVisible(False)
        ctrl_layout.addWidget(self._progress)

        self._lbl_status = QLabel()
        self._lbl_status.setWordWrap(True)
        ctrl_layout.addWidget(self._lbl_status)

        ctrl_layout.addStretch()

        # Right panel: preview waveform
        self._waveform = WaveformWidget()

        splitter_widget = QWidget()
        splitter_layout = QHBoxLayout(splitter_widget)
        splitter_layout.setContentsMargins(0, 0, 0, 0)
        splitter_layout.addWidget(ctrl_panel, stretch=0)
        splitter_layout.addWidget(self._waveform, stretch=1)

        layout.addWidget(splitter_widget)

    def _connect_signals(self):
        self._btn_dark.clicked.connect(lambda: self._start_capture('dark'))
        self._btn_flat.clicked.connect(lambda: self._start_capture('flat'))
        self._btn_save.clicked.connect(self._save_calibration)
        self._btn_load.clicked.connect(self._load_calibration)
        self._btn_clear.clicked.connect(self._clear_calibration)
        self._btn_auto_pitch.clicked.connect(self._auto_calibrate_pitch)
        self._spin_pixel_pitch.valueChanged.connect(self._on_pitch_changed)
        self._link.frame_received.connect(self._on_frame)

    def _start_capture(self, mode: str):
        if not self._link.is_connected:
            self._lbl_status.setText("Not connected")
            return

        self._capture_mode = mode
        self._capture_buffer.clear()
        self._capture_target = self._spin_num_frames.value()

        self._progress.setRange(0, self._capture_target)
        self._progress.setValue(0)
        self._progress.setVisible(True)

        label = "dark frame" if mode == 'dark' else "flat field"
        self._lbl_status.setText(f"Capturing {label}: 0/{self._capture_target}...")

        # Start continuous raw capture
        self._link.start_continuous(MODE_RAW)

    def _on_frame(self, pixels: np.ndarray, timestamp: float):
        self._waveform.update_frame(pixels)

        if not self._capture_mode:
            return

        self._capture_buffer.append(pixels.copy())
        count = len(self._capture_buffer)
        self._progress.setValue(count)

        label = "dark frame" if self._capture_mode == 'dark' else "flat field"
        self._lbl_status.setText(
            f"Capturing {label}: {count}/{self._capture_target}..."
        )

        if count >= self._capture_target:
            self._finish_capture()

    def _finish_capture(self):
        mode = self._capture_mode
        self._capture_mode = ""
        self._link.stop_continuous()
        self._progress.setVisible(False)

        if mode == 'dark':
            self._calibration.capture_dark(self._capture_buffer)
            self._lbl_dark_status.setText(
                f"Captured ({len(self._capture_buffer)} frames averaged)"
            )
            self._lbl_dark_status.setStyleSheet("color: #44ff44;")
            self._lbl_status.setText("Dark frame captured")
        elif mode == 'flat':
            self._calibration.capture_flat(self._capture_buffer)
            self._lbl_flat_status.setText(
                f"Captured ({len(self._capture_buffer)} frames averaged)"
            )
            self._lbl_flat_status.setStyleSheet("color: #44ff44;")
            self._lbl_status.setText("Flat field captured")

        self._capture_buffer.clear()

    def _save_calibration(self):
        directory = QFileDialog.getExistingDirectory(
            self, "Save Calibration", str(CALIBRATION_DIR)
        )
        if directory:
            self._calibration.save(Path(directory))
            self._lbl_status.setText(f"Saved to {directory}")

    def _load_calibration(self):
        directory = QFileDialog.getExistingDirectory(
            self, "Load Calibration", str(CALIBRATION_DIR)
        )
        if directory:
            loaded = CalibrationData.load(Path(directory))
            self._calibration.dark_frame = loaded.dark_frame
            self._calibration.flat_field = loaded.flat_field
            self._calibration.pixel_pitch_mm = loaded.pixel_pitch_mm
            self._calibration.sensor_offset_mm = loaded.sensor_offset_mm
            self._update_status()
            self._spin_pixel_pitch.setValue(loaded.pixel_pitch_mm)
            self._lbl_status.setText(f"Loaded from {directory}")

    def _clear_calibration(self):
        self._calibration.dark_frame = None
        self._calibration.flat_field = None
        self._update_status()
        self._lbl_status.setText("Calibration cleared")

    def _update_status(self):
        if self._calibration.dark_frame is not None:
            self._lbl_dark_status.setText("Loaded")
            self._lbl_dark_status.setStyleSheet("color: #44ff44;")
        else:
            self._lbl_dark_status.setText("Not captured")
            self._lbl_dark_status.setStyleSheet("")

        if self._calibration.flat_field is not None:
            self._lbl_flat_status.setText("Loaded")
            self._lbl_flat_status.setStyleSheet("color: #44ff44;")
        else:
            self._lbl_flat_status.setText("Not captured")
            self._lbl_flat_status.setStyleSheet("")

    def _on_pitch_changed(self, value: float):
        self._calibration.pixel_pitch_mm = value

    def _auto_calibrate_pitch(self):
        """Auto-calibrate pixel pitch from detected edges in current frame."""
        from ccd_inspector.processing.edge_detect import EdgeDetector

        if self._waveform._curve.getData()[1] is None:
            self._lbl_status.setText("No frame data — capture a frame first")
            return

        # Get current waveform data
        _, pixels = self._waveform._curve.getData()
        if pixels is None or len(pixels) == 0:
            return

        detector = EdgeDetector(threshold=30.0)
        edges = detector.detect_edges(np.array(pixels, dtype=np.uint16))

        if len(edges) < 2:
            self._lbl_status.setText(
                f"Need at least 2 edges, found {len(edges)}"
            )
            return

        positions = [e.position for e in edges]
        known_spacing = self._spin_known_spacing.value()
        self._calibration.calibrate_pitch(
            np.array(pixels, dtype=np.uint16),
            known_spacing,
            positions,
        )
        self._spin_pixel_pitch.setValue(self._calibration.pixel_pitch_mm)
        self._lbl_status.setText(
            f"Calibrated: {self._calibration.pixel_pitch_mm:.4f} mm/px "
            f"from {len(edges)} edges"
        )
