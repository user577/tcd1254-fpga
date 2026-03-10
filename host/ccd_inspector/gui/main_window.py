"""Main application window."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QStatusBar,
    QTabWidget,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

from ccd_inspector.comm.protocol import BAUD_RATE
from ccd_inspector.comm.serial_link import SerialLink
from ccd_inspector.gui.tabs.edge_detection_tab import EdgeDetectionTab
from ccd_inspector.gui.tabs.exposure_tab import ExposureTab
from ccd_inspector.gui.tabs.live_view_tab import LiveViewTab


class MainWindow(QMainWindow):
    """TCD1254 Inspector main window."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("TCD1254 CCD Inspector")
        self.resize(1200, 700)

        # Shared serial link
        self._link = SerialLink(self)

        self._build_toolbar()
        self._build_tabs()
        self._build_statusbar()
        self._connect_signals()

        self._refresh_ports()

    def _build_toolbar(self):
        toolbar = QToolBar("Connection")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        toolbar.addWidget(QLabel(" Port: "))
        self._combo_port = QComboBox()
        self._combo_port.setMinimumWidth(120)
        toolbar.addWidget(self._combo_port)

        self._btn_refresh = QPushButton("Refresh")
        toolbar.addWidget(self._btn_refresh)

        self._btn_connect = QPushButton("Connect")
        toolbar.addWidget(self._btn_connect)

        self._btn_disconnect = QPushButton("Disconnect")
        self._btn_disconnect.setEnabled(False)
        toolbar.addWidget(self._btn_disconnect)

    def _build_tabs(self):
        self._tabs = QTabWidget()
        self.setCentralWidget(self._tabs)

        self._live_tab = LiveViewTab(self._link)
        self._exposure_tab = ExposureTab(self._link)

        self._edge_tab = EdgeDetectionTab(self._link)

        self._tabs.addTab(self._live_tab, "Live View")
        self._tabs.addTab(self._exposure_tab, "Exposure")
        self._tabs.addTab(self._edge_tab, "Edge Detection")
        # Future tabs:
        # self._tabs.addTab(FlashSequenceTab(...), "Flash Array")
        # self._tabs.addTab(InspectionTab(...), "Inspection")
        # self._tabs.addTab(CalibrationTab(...), "Calibration")

    def _build_statusbar(self):
        self._statusbar = QStatusBar()
        self.setStatusBar(self._statusbar)
        self._lbl_conn_status = QLabel("Disconnected")
        self._statusbar.addPermanentWidget(self._lbl_conn_status)

    def _connect_signals(self):
        self._btn_refresh.clicked.connect(self._refresh_ports)
        self._btn_connect.clicked.connect(self._connect)
        self._btn_disconnect.clicked.connect(self._disconnect)
        self._link.connection_changed.connect(self._on_connection_changed)
        self._link.error_occurred.connect(self._on_error)

    def _refresh_ports(self):
        self._combo_port.clear()
        ports = SerialLink.list_ports()
        for p in ports:
            label = f"{p['device']}  ({p['description']})" if p['description'] else p['device']
            self._combo_port.addItem(label, p['device'])

        # Try to auto-select
        auto = SerialLink.auto_detect_port()
        if auto:
            for i in range(self._combo_port.count()):
                if self._combo_port.itemData(i) == auto:
                    self._combo_port.setCurrentIndex(i)
                    break

    def _connect(self):
        port = self._combo_port.currentData()
        if not port:
            self._statusbar.showMessage("No port selected", 3000)
            return
        ok = self._link.connect(port, BAUD_RATE)
        if not ok:
            self._statusbar.showMessage(f"Failed to connect to {port}", 3000)

    def _disconnect(self):
        self._link.stop_continuous()
        self._link.disconnect()

    def _on_connection_changed(self, connected: bool):
        self._btn_connect.setEnabled(not connected)
        self._btn_disconnect.setEnabled(connected)
        self._combo_port.setEnabled(not connected)
        if connected:
            self._lbl_conn_status.setText(f"Connected: {self._link.port_name}")
            self._statusbar.showMessage("Connected", 2000)
        else:
            self._lbl_conn_status.setText("Disconnected")

    def _on_error(self, msg: str):
        self._statusbar.showMessage(f"Error: {msg}", 5000)

    def closeEvent(self, event):
        self._link.stop_continuous()
        self._link.disconnect()
        super().closeEvent(event)
