"""Live CCD waveform display tab."""

from __future__ import annotations

import time

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from ccd_inspector.comm.protocol import MODE_RAW
from ccd_inspector.comm.serial_link import SerialLink
from ccd_inspector.core.frame import CCDFrame
from ccd_inspector.gui.widgets.histogram_widget import HistogramWidget
from ccd_inspector.gui.widgets.waveform_widget import WaveformWidget


class LiveViewTab(QWidget):
    """Real-time CCD waveform and histogram display."""

    def __init__(self, serial_link: SerialLink, parent=None):
        super().__init__(parent)
        self._link = serial_link
        self._frame_count = 0
        self._fps_time = time.perf_counter()
        self._fps = 0.0
        self._last_frame: CCDFrame | None = None

        self._build_ui()
        self._connect_signals()

        # FPS counter update
        self._fps_timer = QTimer(self)
        self._fps_timer.timeout.connect(self._update_fps_display)
        self._fps_timer.start(500)

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        # Top toolbar
        toolbar = QHBoxLayout()
        self._btn_start = QPushButton("Start")
        self._btn_stop = QPushButton("Stop")
        self._btn_stop.setEnabled(False)
        self._btn_freeze = QPushButton("Freeze")
        self._btn_freeze.setCheckable(True)

        self._lbl_fps = QLabel("FPS: --")
        self._lbl_peak = QLabel("Peak: --")
        self._lbl_mean = QLabel("Mean: --")
        self._lbl_status = QLabel("Disconnected")

        toolbar.addWidget(self._btn_start)
        toolbar.addWidget(self._btn_stop)
        toolbar.addWidget(self._btn_freeze)
        toolbar.addStretch()
        toolbar.addWidget(self._lbl_fps)
        toolbar.addWidget(QLabel(" | "))
        toolbar.addWidget(self._lbl_peak)
        toolbar.addWidget(QLabel(" | "))
        toolbar.addWidget(self._lbl_mean)
        toolbar.addWidget(QLabel(" | "))
        toolbar.addWidget(self._lbl_status)

        layout.addLayout(toolbar)

        # Waveform + histogram splitter
        splitter = QSplitter(Qt.Orientation.Vertical)

        self._waveform = WaveformWidget()
        splitter.addWidget(self._waveform)

        self._histogram = HistogramWidget()
        self._histogram.setMaximumHeight(150)
        splitter.addWidget(self._histogram)

        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)

        layout.addWidget(splitter)

    def _connect_signals(self):
        self._btn_start.clicked.connect(self._start_capture)
        self._btn_stop.clicked.connect(self._stop_capture)
        self._link.frame_received.connect(self._on_frame)
        self._link.connection_changed.connect(self._on_connection_changed)
        self._link.error_occurred.connect(self._on_error)

    def _start_capture(self):
        if not self._link.is_connected:
            self._lbl_status.setText("Not connected!")
            return
        self._frame_count = 0
        self._fps_time = time.perf_counter()
        self._link.start_continuous(MODE_RAW)
        self._btn_start.setEnabled(False)
        self._btn_stop.setEnabled(True)
        self._lbl_status.setText("Capturing...")

    def _stop_capture(self):
        self._link.stop_continuous()
        self._btn_start.setEnabled(True)
        self._btn_stop.setEnabled(False)
        self._lbl_status.setText("Stopped")

    def _on_frame(self, pixels: np.ndarray, timestamp: float):
        self._frame_count += 1

        frame = CCDFrame(
            pixels=pixels,
            timestamp=timestamp,
            sh_period=self._link._sh,
            icg_period=self._link._icg,
            frame_number=self._frame_count,
        )
        self._last_frame = frame

        if not self._btn_freeze.isChecked():
            self._waveform.update_frame(pixels)
            self._histogram.update_frame(pixels)
            self._lbl_peak.setText(f"Peak: {frame.peak}")
            self._lbl_mean.setText(f"Mean: {frame.mean:.0f}")

    def _update_fps_display(self):
        now = time.perf_counter()
        dt = now - self._fps_time
        if dt > 0:
            self._fps = self._frame_count / dt
        self._lbl_fps.setText(f"FPS: {self._fps:.1f}")
        self._frame_count = 0
        self._fps_time = now

    def _on_connection_changed(self, connected: bool):
        self._lbl_status.setText("Connected" if connected else "Disconnected")

    def _on_error(self, msg: str):
        self._lbl_status.setText(f"Error: {msg}")
