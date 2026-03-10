"""Abstract CCD link interface for frame acquisition and control.

All GUI tabs program against this interface, allowing transparent switching
between the FPGA UART path (SerialLink) and the RP2040 USB CDC path
(RP2040Link).
"""

from __future__ import annotations

from abc import abstractmethod

import numpy as np
from PySide6.QtCore import QObject, Signal

from ccd_inspector.comm.protocol import MODE_POSITION, MODE_RAW  # noqa: F401


class CcdLink(QObject):
    """Abstract interface for CCD communication (FPGA UART or RP2040 USB)."""

    frame_received = Signal(np.ndarray, float)   # pixels, timestamp
    position_received = Signal(object)            # float | None
    connection_changed = Signal(bool)
    error_occurred = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

    # ---- Properties ----

    @property
    @abstractmethod
    def is_connected(self) -> bool: ...

    @property
    @abstractmethod
    def port_name(self) -> str: ...

    # ---- Connection ----

    @abstractmethod
    def connect(self, port: str, **kwargs) -> bool: ...

    @abstractmethod
    def disconnect(self): ...

    # ---- CCD commands ----

    @abstractmethod
    def send_command(self, sh=None, icg=None, mode=None, avg=None): ...

    @abstractmethod
    def send_flash_command(self, lamp_mask=0x01, flash_delay_us=0,
                           flash_duration_us=500, auto_sequence=False,
                           raw_capture=True): ...

    # ---- Continuous acquisition ----

    @abstractmethod
    def start_continuous(self, mode=MODE_RAW): ...

    @abstractmethod
    def stop_continuous(self): ...

    # ---- Port enumeration ----

    @staticmethod
    @abstractmethod
    def list_ports() -> list[dict]: ...

    @staticmethod
    @abstractmethod
    def auto_detect_port() -> str | None: ...
