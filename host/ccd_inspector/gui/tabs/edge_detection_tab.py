"""Edge detection tuning and results tab."""

from __future__ import annotations

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QSlider,
    QSpinBox,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ccd_inspector.comm.ccd_link import CcdLink
from ccd_inspector.processing.edge_detect import EdgeDetector
from ccd_inspector.processing.part_classifier import PartClassifier
from ccd_inspector.gui.widgets.waveform_widget import WaveformWidget


class EdgeDetectionTab(QWidget):
    """Interactive edge detection parameter tuning with live results."""

    def __init__(self, link: CcdLink, parent=None):
        super().__init__(parent)
        self._link = link
        self._last_pixels: np.ndarray | None = None
        self._detector = EdgeDetector()
        self._classifier = PartClassifier()

        self._build_ui()
        self._connect_signals()

    def _build_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        # Left panel: parameters
        ctrl_panel = QWidget()
        ctrl_layout = QVBoxLayout(ctrl_panel)
        ctrl_panel.setMaximumWidth(320)

        # -- Edge detection params --
        edge_group = QGroupBox("Edge Detection")
        edge_form = QFormLayout(edge_group)

        self._spin_gradient_window = QSpinBox()
        self._spin_gradient_window.setRange(1, 100)
        self._spin_gradient_window.setValue(20)
        edge_form.addRow("Gradient Window:", self._spin_gradient_window)

        self._spin_smooth = QDoubleSpinBox()
        self._spin_smooth.setRange(0.0, 20.0)
        self._spin_smooth.setValue(3.0)
        self._spin_smooth.setSingleStep(0.5)
        edge_form.addRow("Smooth Sigma:", self._spin_smooth)

        self._spin_threshold = QDoubleSpinBox()
        self._spin_threshold.setRange(1.0, 1000.0)
        self._spin_threshold.setValue(50.0)
        self._spin_threshold.setSingleStep(10.0)
        edge_form.addRow("Threshold:", self._spin_threshold)

        self._spin_min_width = QSpinBox()
        self._spin_min_width.setRange(1, 500)
        self._spin_min_width.setValue(10)
        edge_form.addRow("Min Width (px):", self._spin_min_width)

        self._spin_max_width = QSpinBox()
        self._spin_max_width.setRange(10, 2200)
        self._spin_max_width.setValue(1500)
        edge_form.addRow("Max Width (px):", self._spin_max_width)

        ctrl_layout.addWidget(edge_group)

        # -- Part classification params --
        class_group = QGroupBox("Part Classification")
        class_form = QFormLayout(class_group)

        self._spin_part_width = QDoubleSpinBox()
        self._spin_part_width.setRange(0.1, 100.0)
        self._spin_part_width.setValue(5.0)
        self._spin_part_width.setSuffix(" mm")
        class_form.addRow("Known Width:", self._spin_part_width)

        self._spin_pixel_pitch = QDoubleSpinBox()
        self._spin_pixel_pitch.setRange(0.001, 1.0)
        self._spin_pixel_pitch.setValue(0.014)
        self._spin_pixel_pitch.setDecimals(4)
        self._spin_pixel_pitch.setSuffix(" mm")
        class_form.addRow("Pixel Pitch:", self._spin_pixel_pitch)

        self._spin_tolerance = QDoubleSpinBox()
        self._spin_tolerance.setRange(1.0, 50.0)
        self._spin_tolerance.setValue(15.0)
        self._spin_tolerance.setSuffix(" %")
        class_form.addRow("Width Tolerance:", self._spin_tolerance)

        ctrl_layout.addWidget(class_group)

        # -- Actions --
        self._btn_reprocess = QPushButton("Re-process")
        ctrl_layout.addWidget(self._btn_reprocess)

        self._lbl_status = QLabel()
        self._lbl_status.setWordWrap(True)
        ctrl_layout.addWidget(self._lbl_status)

        ctrl_layout.addStretch()

        # Right panel: waveform + results table
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)

        self._waveform = WaveformWidget()
        right_layout.addWidget(self._waveform, stretch=3)

        # Results table
        self._table = QTableWidget()
        self._table.setColumnCount(7)
        self._table.setHorizontalHeaderLabels([
            "Object", "Center (px)", "Width (px)", "Width (mm)",
            "Sharpness", "Class", "Confidence",
        ])
        self._table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        self._table.setMaximumHeight(200)
        right_layout.addWidget(self._table, stretch=1)

        # Layout
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(ctrl_panel)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        layout.addWidget(splitter)

    def _connect_signals(self):
        self._link.frame_received.connect(self._on_frame)
        self._btn_reprocess.clicked.connect(self._reprocess)

        # Live update on parameter change
        for spin in (
            self._spin_gradient_window, self._spin_smooth,
            self._spin_threshold, self._spin_min_width, self._spin_max_width,
            self._spin_part_width, self._spin_pixel_pitch, self._spin_tolerance,
        ):
            spin.valueChanged.connect(self._reprocess)

    def _on_frame(self, pixels: np.ndarray, timestamp: float):
        self._last_pixels = pixels.copy()
        self._reprocess()

    def _reprocess(self):
        if self._last_pixels is None:
            return

        # Update detector params
        self._detector = EdgeDetector(
            gradient_window=self._spin_gradient_window.value(),
            smooth_sigma=self._spin_smooth.value(),
            threshold=self._spin_threshold.value(),
            min_width_px=self._spin_min_width.value(),
            max_width_px=self._spin_max_width.value(),
        )

        self._classifier = PartClassifier(
            known_part_width_mm=self._spin_part_width.value(),
            pixel_pitch_mm=self._spin_pixel_pitch.value(),
            width_tolerance_pct=self._spin_tolerance.value(),
        )

        pixels = self._last_pixels

        # Detect edges and objects
        edges = self._detector.detect_edges(pixels)
        objects = self._detector.detect_objects(pixels)

        # Show waveform with edge markers
        self._waveform.update_frame(pixels)
        self._waveform.set_edges([e.position_dict for e in edges])

        # Classify each object
        self._table.setRowCount(len(objects))
        for i, obj in enumerate(objects):
            measurement = self._classifier.classify(obj, pixels)

            self._table.setItem(i, 0, QTableWidgetItem(str(i + 1)))
            self._table.setItem(i, 1, QTableWidgetItem(f"{measurement.center_px:.1f}"))
            self._table.setItem(i, 2, QTableWidgetItem(f"{measurement.width_px:.1f}"))
            self._table.setItem(i, 3, QTableWidgetItem(f"{measurement.width_mm:.2f}"))
            self._table.setItem(i, 4, QTableWidgetItem(f"{measurement.edge_sharpness:.0f}"))

            class_item = QTableWidgetItem(measurement.classification)
            if measurement.classification == 'single':
                class_item.setForeground(Qt.GlobalColor.green)
            elif measurement.classification == 'grouped':
                class_item.setForeground(Qt.GlobalColor.red)
            else:
                class_item.setForeground(Qt.GlobalColor.yellow)
            self._table.setItem(i, 5, class_item)

            self._table.setItem(i, 6, QTableWidgetItem(f"{measurement.confidence:.0%}"))

        self._lbl_status.setText(
            f"Edges: {len(edges)}  |  Objects: {len(objects)}"
        )
