"""System Control tab — RP2040 configuration, motor control, sort monitoring."""

from __future__ import annotations

import logging

from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import (
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ccd_inspector.comm.rp2040_link import RP2040Link

log = logging.getLogger(__name__)

AXIS_NAMES = ["Feeder", "Conveyor", "Singulate", "Sort"]


class SystemControlTab(QWidget):
    """RP2040 system controller interface."""

    def __init__(self, rp2040: RP2040Link, parent=None):
        super().__init__(parent)
        self._rp = rp2040
        self._build_ui()
        self._connect_signals()

        # Poll status every 500 ms when connected
        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._poll_status)

    def _build_ui(self):
        layout = QVBoxLayout(self)

        # Connection
        conn_box = QGroupBox("RP2040 Connection")
        conn_lay = QHBoxLayout(conn_box)

        self._lbl_rp_status = QLabel("Disconnected")
        conn_lay.addWidget(self._lbl_rp_status)

        self._btn_rp_connect = QPushButton("Auto-Connect")
        conn_lay.addWidget(self._btn_rp_connect)

        self._btn_rp_disconnect = QPushButton("Disconnect")
        self._btn_rp_disconnect.setEnabled(False)
        conn_lay.addWidget(self._btn_rp_disconnect)

        conn_lay.addStretch()
        layout.addWidget(conn_box)

        # Sub-tabs for different control areas
        sub_tabs = QTabWidget()
        layout.addWidget(sub_tabs)

        sub_tabs.addTab(self._build_status_panel(), "Status")
        sub_tabs.addTab(self._build_motor_panel(), "Motors")
        sub_tabs.addTab(self._build_sort_panel(), "Sort Config")
        sub_tabs.addTab(self._build_control_panel(), "Control")

    def _build_status_panel(self) -> QWidget:
        w = QWidget()
        layout = QVBoxLayout(w)

        # System state
        state_box = QGroupBox("System State")
        state_lay = QFormLayout(state_box)
        self._lbl_state = QLabel("---")
        self._lbl_fpga = QLabel("---")
        state_lay.addRow("Sort State:", self._lbl_state)
        state_lay.addRow("FPGA Running:", self._lbl_fpga)
        layout.addWidget(state_box)

        # Axis status
        axes_box = QGroupBox("Axes")
        axes_lay = QFormLayout(axes_box)
        self._axis_labels = []
        for name in AXIS_NAMES:
            lbl = QLabel("---")
            axes_lay.addRow(f"{name}:", lbl)
            self._axis_labels.append(lbl)
        layout.addWidget(axes_box)

        # Sort stats
        stats_box = QGroupBox("Sort Statistics")
        stats_lay = QFormLayout(stats_box)
        self._lbl_inspected = QLabel("0")
        self._lbl_accepted = QLabel("0")
        self._lbl_rejected = QLabel("0")
        self._lbl_rescanned = QLabel("0")
        self._lbl_cycles = QLabel("0")
        stats_lay.addRow("Inspected:", self._lbl_inspected)
        stats_lay.addRow("Accepted:", self._lbl_accepted)
        stats_lay.addRow("Rejected:", self._lbl_rejected)
        stats_lay.addRow("Rescanned:", self._lbl_rescanned)
        stats_lay.addRow("Cycles:", self._lbl_cycles)

        btn_reset_stats = QPushButton("Reset Stats")
        btn_reset_stats.clicked.connect(self._reset_stats)
        stats_lay.addRow("", btn_reset_stats)
        layout.addWidget(stats_box)

        layout.addStretch()
        return w

    def _build_motor_panel(self) -> QWidget:
        w = QWidget()
        layout = QVBoxLayout(w)

        # Homing
        home_box = QGroupBox("Homing")
        home_lay = QHBoxLayout(home_box)
        btn_home_all = QPushButton("Home All")
        btn_home_all.clicked.connect(lambda: self._rp.home_all())
        home_lay.addWidget(btn_home_all)
        for i, name in enumerate(AXIS_NAMES):
            btn = QPushButton(f"Home {name}")
            btn.clicked.connect(lambda checked, ax=i: self._rp.home_axis(ax))
            home_lay.addWidget(btn)
        layout.addWidget(home_box)

        # Manual jog
        jog_box = QGroupBox("Manual Move")
        jog_lay = QFormLayout(jog_box)

        self._spin_axis = QSpinBox()
        self._spin_axis.setRange(0, 3)
        jog_lay.addRow("Axis:", self._spin_axis)

        self._spin_steps = QSpinBox()
        self._spin_steps.setRange(-100000, 100000)
        self._spin_steps.setValue(500)
        jog_lay.addRow("Steps:", self._spin_steps)

        self._spin_speed = QSpinBox()
        self._spin_speed.setRange(100, 50000)
        self._spin_speed.setValue(2000)
        jog_lay.addRow("Speed:", self._spin_speed)

        btn_move = QPushButton("Move")
        btn_move.clicked.connect(self._do_move)
        jog_lay.addRow("", btn_move)

        layout.addWidget(jog_box)

        # E-stop
        estop_box = QGroupBox("Emergency")
        estop_lay = QHBoxLayout(estop_box)
        btn_estop = QPushButton("E-STOP ALL")
        btn_estop.setStyleSheet("background-color: #cc0000; color: white; font-weight: bold; padding: 10px;")
        btn_estop.clicked.connect(lambda: self._rp.estop())
        estop_lay.addWidget(btn_estop)
        layout.addWidget(estop_box)

        layout.addStretch()
        return w

    def _build_sort_panel(self) -> QWidget:
        w = QWidget()
        layout = QFormLayout(w)

        self._edit_min_w = QLineEdit("5.0")
        self._edit_max_w = QLineEdit("50.0")
        self._edit_grouped = QLineEdit("80.0")
        self._spin_rescan = QSpinBox()
        self._spin_rescan.setRange(0, 10)
        self._spin_rescan.setValue(3)

        layout.addRow("Min shadow width (px):", self._edit_min_w)
        layout.addRow("Max shadow width (px):", self._edit_max_w)
        layout.addRow("Grouped threshold (px):", self._edit_grouped)
        layout.addRow("Rescan attempts:", self._spin_rescan)

        self._edit_feed_steps = QLineEdit("500")
        self._edit_feed_speed = QLineEdit("2000")
        self._edit_settle_ms = QLineEdit("50")

        layout.addRow("Feed steps:", self._edit_feed_steps)
        layout.addRow("Feed speed:", self._edit_feed_speed)
        layout.addRow("Settle time (ms):", self._edit_settle_ms)

        btn_apply = QPushButton("Apply Sort Config")
        btn_apply.clicked.connect(self._apply_sort_config)
        layout.addRow("", btn_apply)

        btn_save = QPushButton("Save to Flash")
        btn_save.clicked.connect(lambda: self._rp.save_config())
        layout.addRow("", btn_save)

        return w

    def _build_control_panel(self) -> QWidget:
        w = QWidget()
        layout = QVBoxLayout(w)

        ctrl_box = QGroupBox("Sort Control")
        ctrl_lay = QHBoxLayout(ctrl_box)

        btn_start = QPushButton("START")
        btn_start.setStyleSheet("background-color: #00aa00; color: white; font-weight: bold; padding: 10px;")
        btn_start.clicked.connect(lambda: self._rp.start())
        ctrl_lay.addWidget(btn_start)

        btn_stop = QPushButton("STOP")
        btn_stop.setStyleSheet("background-color: #cc8800; color: white; font-weight: bold; padding: 10px;")
        btn_stop.clicked.connect(lambda: self._rp.stop())
        ctrl_lay.addWidget(btn_stop)

        btn_estop = QPushButton("E-STOP")
        btn_estop.setStyleSheet("background-color: #cc0000; color: white; font-weight: bold; padding: 10px;")
        btn_estop.clicked.connect(lambda: self._rp.estop())
        ctrl_lay.addWidget(btn_estop)

        layout.addWidget(ctrl_box)

        # FPGA quick controls
        fpga_box = QGroupBox("FPGA (via RP2040)")
        fpga_lay = QHBoxLayout(fpga_box)

        btn_trigger = QPushButton("Trigger Capture")
        btn_trigger.clicked.connect(self._trigger)
        fpga_lay.addWidget(btn_trigger)

        btn_shadow = QPushButton("Read Shadow")
        btn_shadow.clicked.connect(self._read_shadow)
        fpga_lay.addWidget(btn_shadow)

        self._lbl_shadow = QLabel("---")
        fpga_lay.addWidget(self._lbl_shadow)

        layout.addWidget(fpga_box)
        layout.addStretch()
        return w

    def _connect_signals(self):
        self._btn_rp_connect.clicked.connect(self._rp_connect)
        self._btn_rp_disconnect.clicked.connect(self._rp_disconnect)

    def _rp_connect(self):
        port = RP2040Link.auto_detect_port()
        if not port:
            self._lbl_rp_status.setText("No RP2040 found")
            return

        if self._rp.connect(port):
            self._lbl_rp_status.setText(f"Connected: {port}")
            self._btn_rp_connect.setEnabled(False)
            self._btn_rp_disconnect.setEnabled(True)
            self._poll_timer.start(500)
            self._load_config_values()
        else:
            self._lbl_rp_status.setText("Connection failed")

    def _rp_disconnect(self):
        self._poll_timer.stop()
        self._rp.disconnect()
        self._lbl_rp_status.setText("Disconnected")
        self._btn_rp_connect.setEnabled(True)
        self._btn_rp_disconnect.setEnabled(False)

    def _poll_status(self):
        if not self._rp.is_connected:
            self._poll_timer.stop()
            return

        try:
            status = self._rp.get_status()
            self._lbl_state.setText(status.state.upper())
            self._lbl_fpga.setText("Yes" if status.fpga_running else "No")

            for i, ax in enumerate(status.axes):
                parts = []
                parts.append(f"pos={ax.position}")
                if ax.busy:
                    parts.append("BUSY")
                if ax.homed:
                    parts.append("homed")
                if ax.home_switch:
                    parts.append("sw")
                if ax.enabled:
                    parts.append("en")
                self._axis_labels[i].setText("  ".join(parts))

            stats = self._rp.get_stats()
            self._lbl_inspected.setText(str(stats.inspected))
            self._lbl_accepted.setText(str(stats.accepted))
            self._lbl_rejected.setText(str(stats.rejected))
            self._lbl_rescanned.setText(str(stats.rescanned))
            self._lbl_cycles.setText(str(stats.cycles))

        except Exception as exc:
            log.warning("Status poll failed: %s", exc)

    def _load_config_values(self):
        try:
            cfg = self._rp.get_config()
            sort = cfg.get("sort", {})
            self._edit_min_w.setText(str(sort.get("min_w", 5.0)))
            self._edit_max_w.setText(str(sort.get("max_w", 50.0)))
            self._edit_grouped.setText(str(sort.get("grouped", 80.0)))
            self._spin_rescan.setValue(sort.get("rescan", 3))
        except Exception as exc:
            log.warning("Failed to load config: %s", exc)

    def _apply_sort_config(self):
        try:
            self._rp.set_config("sort.min_w", self._edit_min_w.text())
            self._rp.set_config("sort.max_w", self._edit_max_w.text())
            self._rp.set_config("sort.grouped", self._edit_grouped.text())
            self._rp.set_config("sort.rescan", self._spin_rescan.value())
            self._rp.set_config("seq.feed_steps", self._edit_feed_steps.text())
            self._rp.set_config("seq.feed_speed", self._edit_feed_speed.text())
            self._rp.set_config("seq.settle_ms", self._edit_settle_ms.text())
        except Exception as exc:
            log.error("Failed to apply sort config: %s", exc)

    def _do_move(self):
        self._rp.move(
            self._spin_axis.value(),
            self._spin_steps.value(),
            self._spin_speed.value(),
        )

    def _reset_stats(self):
        self._rp.reset_stats()

    def _trigger(self):
        self._rp.trigger_capture()

    def _read_shadow(self):
        px = self._rp.read_shadow()
        if px < 0:
            self._lbl_shadow.setText("No shadow")
        else:
            self._lbl_shadow.setText(f"{px:.1f} px")
