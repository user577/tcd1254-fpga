"""Flash lamp array configuration and sequence control tab."""

from __future__ import annotations

import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QSpinBox,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ccd_inspector.comm.serial_link import SerialLink
from ccd_inspector.flash.coordinator import FlashCaptureCoordinator
from ccd_inspector.flash.lamp_config import LampArrayConfig, LampConfig
from ccd_inspector.flash.sequence import FlashSequence
from ccd_inspector.gui.widgets.waveform_widget import WaveformWidget


class FlashSequenceTab(QWidget):
    """Configure flash lamp array and run capture sequences."""

    def __init__(self, serial_link: SerialLink, parent=None):
        super().__init__(parent)
        self._link = serial_link
        self._config = LampArrayConfig.default_4_angle()
        self._coordinator = FlashCaptureCoordinator(serial_link)
        self._coordinator.set_lamp_config(self._config)

        self._build_ui()
        self._connect_signals()
        self._refresh_lamp_table()

    def _build_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        # Left panel: lamp config + sequence controls
        ctrl_panel = QWidget()
        ctrl_layout = QVBoxLayout(ctrl_panel)
        ctrl_panel.setMaximumWidth(420)

        # -- Lamp array configuration --
        lamp_group = QGroupBox("Lamp Array")
        lamp_layout = QVBoxLayout(lamp_group)

        self._lamp_table = QTableWidget()
        self._lamp_table.setColumnCount(6)
        self._lamp_table.setHorizontalHeaderLabels([
            "En", "ID", "Name", "Angle", "Duration (us)", "Delay (us)",
        ])
        self._lamp_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.ResizeToContents
        )
        self._lamp_table.setMaximumHeight(180)
        lamp_layout.addWidget(self._lamp_table)

        lamp_btn_layout = QHBoxLayout()
        self._btn_add_lamp = QPushButton("Add Lamp")
        self._btn_remove_lamp = QPushButton("Remove")
        self._btn_reset_lamps = QPushButton("Reset to Default")
        lamp_btn_layout.addWidget(self._btn_add_lamp)
        lamp_btn_layout.addWidget(self._btn_remove_lamp)
        lamp_btn_layout.addWidget(self._btn_reset_lamps)
        lamp_layout.addLayout(lamp_btn_layout)

        ctrl_layout.addWidget(lamp_group)

        # -- Capture settings --
        capture_group = QGroupBox("Capture Settings")
        capture_form = QFormLayout(capture_group)

        self._spin_flash_icg = QSpinBox()
        self._spin_flash_icg.setRange(9000, 2_000_000)
        self._spin_flash_icg.setValue(40_000)
        self._spin_flash_icg.setSuffix(" fM")
        capture_form.addRow("Integration (ICG):", self._spin_flash_icg)

        self._lbl_flash_ms = QLabel("10.0 ms")
        capture_form.addRow("Integration Time:", self._lbl_flash_ms)

        self._spin_flash_sh = QSpinBox()
        self._spin_flash_sh.setRange(4, 1000)
        self._spin_flash_sh.setValue(20)
        self._spin_flash_sh.setSuffix(" fM")
        capture_form.addRow("SH Period:", self._spin_flash_sh)

        ctrl_layout.addWidget(capture_group)

        # -- Sequence controls --
        seq_group = QGroupBox("Sequence")
        seq_layout = QVBoxLayout(seq_group)

        btn_row1 = QHBoxLayout()
        self._btn_test_fire = QPushButton("Test Fire Selected")
        self._btn_round_robin = QPushButton("Round Robin Capture")
        btn_row1.addWidget(self._btn_test_fire)
        btn_row1.addWidget(self._btn_round_robin)
        seq_layout.addLayout(btn_row1)

        self._btn_stop_seq = QPushButton("Stop")
        self._btn_stop_seq.setEnabled(False)
        seq_layout.addWidget(self._btn_stop_seq)

        self._lbl_seq_status = QLabel()
        self._lbl_seq_status.setWordWrap(True)
        seq_layout.addWidget(self._lbl_seq_status)

        ctrl_layout.addWidget(seq_group)
        ctrl_layout.addStretch()

        # Right panel: multi-angle waveform display
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)

        self._lbl_angle = QLabel("Waveform: waiting for capture...")
        right_layout.addWidget(self._lbl_angle)

        self._waveform = WaveformWidget()
        right_layout.addWidget(self._waveform)

        # Captured frames summary
        self._results_table = QTableWidget()
        self._results_table.setColumnCount(4)
        self._results_table.setHorizontalHeaderLabels([
            "Lamp", "Angle", "Peak", "Mean",
        ])
        self._results_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        self._results_table.setMaximumHeight(150)
        right_layout.addWidget(self._results_table)

        # Layout
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(ctrl_panel)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        layout.addWidget(splitter)

    def _connect_signals(self):
        self._btn_add_lamp.clicked.connect(self._add_lamp)
        self._btn_remove_lamp.clicked.connect(self._remove_lamp)
        self._btn_reset_lamps.clicked.connect(self._reset_lamps)
        self._btn_test_fire.clicked.connect(self._test_fire)
        self._btn_round_robin.clicked.connect(self._run_round_robin)
        self._btn_stop_seq.clicked.connect(self._stop_sequence)
        self._spin_flash_icg.valueChanged.connect(self._update_flash_ms)

        self._coordinator.frame_captured.connect(self._on_frame_captured)
        self._coordinator.sequence_complete.connect(self._on_sequence_complete)
        self._coordinator.progress.connect(self._on_progress)

    def _refresh_lamp_table(self):
        self._lamp_table.setRowCount(len(self._config.lamps))
        for i, lamp in enumerate(self._config.lamps):
            # Enabled checkbox
            chk = QTableWidgetItem()
            chk.setCheckState(
                Qt.CheckState.Checked if lamp.enabled else Qt.CheckState.Unchecked
            )
            self._lamp_table.setItem(i, 0, chk)

            self._lamp_table.setItem(i, 1, QTableWidgetItem(str(lamp.lamp_id)))
            self._lamp_table.setItem(i, 2, QTableWidgetItem(lamp.name))
            self._lamp_table.setItem(i, 3, QTableWidgetItem(f"{lamp.angle_deg:.0f}"))
            self._lamp_table.setItem(i, 4, QTableWidgetItem(str(lamp.flash_duration_us)))
            self._lamp_table.setItem(i, 5, QTableWidgetItem(str(lamp.flash_delay_us)))

    def _read_lamp_table(self):
        """Read lamp config back from table."""
        lamps = []
        for i in range(self._lamp_table.rowCount()):
            enabled = self._lamp_table.item(i, 0).checkState() == Qt.CheckState.Checked
            lamp_id = int(self._lamp_table.item(i, 1).text())
            name = self._lamp_table.item(i, 2).text()
            angle = float(self._lamp_table.item(i, 3).text())
            duration = int(self._lamp_table.item(i, 4).text())
            delay = int(self._lamp_table.item(i, 5).text())
            lamps.append(LampConfig(
                lamp_id=lamp_id, name=name, angle_deg=angle,
                enabled=enabled, flash_duration_us=duration,
                flash_delay_us=delay,
            ))
        self._config = LampArrayConfig(lamps=lamps)
        self._coordinator.set_lamp_config(self._config)

    def _add_lamp(self):
        next_id = len(self._config.lamps)
        if next_id >= 8:
            return
        self._config.lamps.append(LampConfig(lamp_id=next_id))
        self._refresh_lamp_table()

    def _remove_lamp(self):
        row = self._lamp_table.currentRow()
        if row >= 0 and len(self._config.lamps) > 1:
            self._config.lamps.pop(row)
            self._refresh_lamp_table()

    def _reset_lamps(self):
        self._config = LampArrayConfig.default_4_angle()
        self._coordinator.set_lamp_config(self._config)
        self._refresh_lamp_table()

    def _update_flash_ms(self, icg: int):
        ms = icg / 4_000_000 * 1000
        self._lbl_flash_ms.setText(f"{ms:.1f} ms")

    def _test_fire(self):
        """Fire a single flash for the selected lamp."""
        if not self._link.is_connected:
            self._lbl_seq_status.setText("Not connected")
            return

        self._read_lamp_table()
        row = self._lamp_table.currentRow()
        if row < 0:
            row = 0

        lamp = self._config.lamps[row]
        seq = FlashSequence.single_lamp(lamp)
        self._coordinator.start_sequence(
            seq,
            sh=self._spin_flash_sh.value(),
            icg=self._spin_flash_icg.value(),
        )
        self._btn_stop_seq.setEnabled(True)
        self._lbl_seq_status.setText(f"Firing {lamp.name}...")

    def _run_round_robin(self):
        """Capture one frame per enabled lamp."""
        if not self._link.is_connected:
            self._lbl_seq_status.setText("Not connected")
            return

        self._read_lamp_table()
        seq = FlashSequence.round_robin(self._config, include_ambient=False)
        if not seq.steps:
            self._lbl_seq_status.setText("No lamps enabled")
            return

        self._coordinator.start_sequence(
            seq,
            sh=self._spin_flash_sh.value(),
            icg=self._spin_flash_icg.value(),
        )
        self._btn_stop_seq.setEnabled(True)
        self._btn_round_robin.setEnabled(False)
        self._lbl_seq_status.setText(
            f"Round robin: 0/{len(seq.steps)} lamps..."
        )

    def _stop_sequence(self):
        self._coordinator.stop()
        self._btn_stop_seq.setEnabled(False)
        self._btn_round_robin.setEnabled(True)
        self._lbl_seq_status.setText("Stopped")

    def _on_frame_captured(self, lamp_id: int, pixels: np.ndarray):
        lamp = self._config.get_lamp(lamp_id)
        name = lamp.name if lamp else f"Lamp {lamp_id}"
        self._lbl_angle.setText(f"Waveform: {name}")
        self._waveform.update_frame(pixels)

    def _on_progress(self, current: int, total: int):
        self._lbl_seq_status.setText(f"Capturing: {current}/{total} lamps...")

    def _on_sequence_complete(self, frames: dict):
        self._btn_stop_seq.setEnabled(False)
        self._btn_round_robin.setEnabled(True)

        self._results_table.setRowCount(len(frames))
        for i, (lamp_id, frame) in enumerate(sorted(frames.items())):
            lamp = self._config.get_lamp(lamp_id)
            name = lamp.name if lamp else f"Lamp {lamp_id}"
            angle = lamp.angle_deg if lamp else 0

            self._results_table.setItem(i, 0, QTableWidgetItem(name))
            self._results_table.setItem(i, 1, QTableWidgetItem(f"{angle:.0f}"))
            self._results_table.setItem(i, 2, QTableWidgetItem(str(frame.peak)))
            self._results_table.setItem(i, 3, QTableWidgetItem(f"{frame.mean:.0f}"))

        self._lbl_seq_status.setText(
            f"Sequence complete: {len(frames)} frames captured"
        )
