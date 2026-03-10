"""Interactive exposure / CCD timing control tab."""

from __future__ import annotations

import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QSpinBox,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from ccd_inspector.comm.protocol import MODE_RAW
from ccd_inspector.comm.ccd_link import CcdLink
from ccd_inspector.core.timing import (
    MIN_ICG,
    MIN_SH,
    fps_to_icg,
    icg_to_fps,
    icg_to_ms,
    max_fps,
    ms_to_icg,
    validate_timing,
)
from ccd_inspector.gui.widgets.waveform_widget import WaveformWidget


# Logarithmic slider helpers
_LOG_STEPS = 1000
_MS_MIN = 2.5    # ~400 FPS
_MS_MAX = 5000.0  # 0.2 FPS

def _slider_to_ms(val: int) -> float:
    """Map slider [0, _LOG_STEPS] to integration time on log scale."""
    import math
    frac = val / _LOG_STEPS
    return _MS_MIN * math.exp(frac * math.log(_MS_MAX / _MS_MIN))

def _ms_to_slider(ms: float) -> int:
    import math
    ms = max(_MS_MIN, min(_MS_MAX, ms))
    frac = math.log(ms / _MS_MIN) / math.log(_MS_MAX / _MS_MIN)
    return int(frac * _LOG_STEPS)


class ExposureTab(QWidget):
    """Interactive SH/ICG control with live waveform preview."""

    def __init__(self, link: CcdLink, parent=None):
        super().__init__(parent)
        self._link = link
        self._updating = False  # guard against signal loops

        self._build_ui()
        self._connect_signals()
        self._set_defaults()

    def _build_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        # Left panel: controls
        ctrl_panel = QWidget()
        ctrl_layout = QVBoxLayout(ctrl_panel)
        ctrl_panel.setMaximumWidth(350)

        # -- Integration time group --
        integ_group = QGroupBox("Integration Time")
        integ_layout = QVBoxLayout(integ_group)

        self._slider_integ = QSlider(Qt.Orientation.Horizontal)
        self._slider_integ.setRange(0, _LOG_STEPS)
        integ_layout.addWidget(self._slider_integ)

        integ_form = QFormLayout()
        self._spin_ms = QDoubleSpinBox()
        self._spin_ms.setRange(_MS_MIN, _MS_MAX)
        self._spin_ms.setDecimals(2)
        self._spin_ms.setSuffix(" ms")
        integ_form.addRow("Time:", self._spin_ms)

        self._spin_fps = QDoubleSpinBox()
        self._spin_fps.setRange(0.2, max_fps())
        self._spin_fps.setDecimals(1)
        self._spin_fps.setSuffix(" Hz")
        integ_form.addRow("Frame Rate:", self._spin_fps)

        self._spin_icg = QSpinBox()
        self._spin_icg.setRange(MIN_ICG, 20_000_000)
        self._spin_icg.setSuffix(" fM")
        integ_form.addRow("ICG Period:", self._spin_icg)

        integ_layout.addLayout(integ_form)
        ctrl_layout.addWidget(integ_group)

        # -- SH period group --
        sh_group = QGroupBox("Shift Gate (SH)")
        sh_form = QFormLayout(sh_group)

        self._spin_sh = QSpinBox()
        self._spin_sh.setRange(MIN_SH, 1000)
        self._spin_sh.setValue(20)
        self._spin_sh.setSuffix(" fM")
        sh_form.addRow("SH Period:", self._spin_sh)

        self._lbl_sh_us = QLabel()
        sh_form.addRow("Duration:", self._lbl_sh_us)

        ctrl_layout.addWidget(sh_group)

        # -- Averaging --
        avg_group = QGroupBox("Averaging")
        avg_form = QFormLayout(avg_group)

        self._spin_avg = QSpinBox()
        self._spin_avg.setRange(1, 15)
        self._spin_avg.setValue(4)
        avg_form.addRow("Frame Avg:", self._spin_avg)

        ctrl_layout.addWidget(avg_group)

        # -- Validation --
        self._lbl_validation = QLabel()
        self._lbl_validation.setWordWrap(True)
        self._lbl_validation.setStyleSheet("color: #ff4444;")
        ctrl_layout.addWidget(self._lbl_validation)

        # -- Apply / Auto-expose --
        btn_layout = QHBoxLayout()
        self._btn_apply = QPushButton("Apply")
        self._btn_apply.setDefault(True)
        self._btn_auto = QPushButton("Auto-Expose")
        btn_layout.addWidget(self._btn_apply)
        btn_layout.addWidget(self._btn_auto)
        ctrl_layout.addLayout(btn_layout)

        ctrl_layout.addStretch()

        # Right panel: live preview
        self._waveform = WaveformWidget()

        # Layout
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(ctrl_panel)
        splitter.addWidget(self._waveform)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        layout.addWidget(splitter)

    def _connect_signals(self):
        self._slider_integ.valueChanged.connect(self._on_slider_changed)
        self._spin_ms.valueChanged.connect(self._on_ms_changed)
        self._spin_fps.valueChanged.connect(self._on_fps_changed)
        self._spin_icg.valueChanged.connect(self._on_icg_changed)
        self._spin_sh.valueChanged.connect(self._on_sh_changed)
        self._btn_apply.clicked.connect(self._apply_settings)
        self._btn_auto.clicked.connect(self._auto_expose)
        self._link.frame_received.connect(self._on_frame)

    def _set_defaults(self):
        self._updating = True
        self._spin_ms.setValue(125.0)  # ICG=500000 / 4MHz
        self._updating = False
        self._on_ms_changed(125.0)

    # -- Coupled value updates (break loops with _updating flag) --

    def _on_slider_changed(self, val: int):
        if self._updating:
            return
        self._updating = True
        ms = _slider_to_ms(val)
        self._spin_ms.setValue(ms)
        self._spin_icg.setValue(ms_to_icg(ms))
        self._spin_fps.setValue(icg_to_fps(ms_to_icg(ms)))
        self._updating = False
        self._validate()

    def _on_ms_changed(self, ms: float):
        if self._updating:
            return
        self._updating = True
        icg = ms_to_icg(ms)
        self._slider_integ.setValue(_ms_to_slider(ms))
        self._spin_icg.setValue(icg)
        self._spin_fps.setValue(icg_to_fps(icg))
        self._updating = False
        self._validate()

    def _on_fps_changed(self, fps: float):
        if self._updating:
            return
        self._updating = True
        icg = fps_to_icg(fps)
        ms = icg_to_ms(icg)
        self._slider_integ.setValue(_ms_to_slider(ms))
        self._spin_ms.setValue(ms)
        self._spin_icg.setValue(icg)
        self._updating = False
        self._validate()

    def _on_icg_changed(self, icg: int):
        if self._updating:
            return
        self._updating = True
        ms = icg_to_ms(icg)
        self._slider_integ.setValue(_ms_to_slider(ms))
        self._spin_ms.setValue(ms)
        self._spin_fps.setValue(icg_to_fps(icg))
        self._updating = False
        self._validate()

    def _on_sh_changed(self, sh: int):
        self._lbl_sh_us.setText(f"{sh / 4.0:.2f} us")
        self._validate()

    def _validate(self):
        ok, msg = validate_timing(self._spin_sh.value(), self._spin_icg.value())
        self._lbl_validation.setText("" if ok else msg)
        self._btn_apply.setEnabled(ok)

    def _apply_settings(self):
        if not self._link.is_connected:
            self._lbl_validation.setText("Not connected")
            return
        self._link.send_command(
            sh=self._spin_sh.value(),
            icg=self._spin_icg.value(),
            avg=self._spin_avg.value(),
        )
        self._lbl_validation.setText("")
        self._lbl_validation.setStyleSheet("color: #44ff44;")
        self._lbl_validation.setText("Applied")

    def _auto_expose(self):
        """Simple auto-exposure: adjust ICG until peak is ~75% of full scale."""
        if not self._link.is_connected:
            return
        # Will be implemented with a feedback loop using frame_received
        # For now, just a placeholder
        self._lbl_validation.setStyleSheet("color: #ffaa00;")
        self._lbl_validation.setText("Auto-expose: not yet implemented")

    def _on_frame(self, pixels: np.ndarray, timestamp: float):
        self._waveform.update_frame(pixels)
