"""Threaded serial port manager with Qt signal integration."""

from __future__ import annotations

import logging
import time
import numpy as np
import serial
import serial.tools.list_ports
from PySide6.QtCore import QObject, QThread, Signal

from ccd_inspector.comm.ccd_link import CcdLink
from ccd_inspector.comm.protocol import (
    BAUD_RATE,
    MODE_POSITION,
    MODE_RAW,
    NO_SHADOW,
    RAW_FRAME_BYTES,
    SYNC_MARKER,
    TOTAL_PIXELS,
    build_command,
    build_flash_command,
    parse_position,
    parse_raw_frame,
)

log = logging.getLogger(__name__)


class _ReaderWorker(QObject):
    """Runs in a QThread — reads serial data and emits frames."""

    frame_received = Signal(np.ndarray, float)  # pixels, timestamp
    position_received = Signal(object)  # float | None
    error_occurred = Signal(str)

    def __init__(self, ser: serial.Serial, mode: int):
        super().__init__()
        self._ser = ser
        self._mode = mode
        self._running = False

    def run(self):
        self._running = True
        buf = bytearray()

        try:
            while self._running:
                waiting = self._ser.in_waiting
                if waiting == 0:
                    time.sleep(0.001)
                    continue

                chunk = self._ser.read(min(waiting, 8192))
                if not chunk:
                    continue

                if self._mode == MODE_POSITION:
                    self._handle_position(chunk)
                elif self._mode == MODE_RAW:
                    buf.extend(chunk)
                    buf = self._extract_frames(buf)

        except serial.SerialException as exc:
            if self._running:
                self.error_occurred.emit(str(exc))
        except Exception as exc:
            self.error_occurred.emit(f"Reader error: {exc}")

    def _handle_position(self, data: bytes):
        # Position data comes as 2-byte chunks
        for i in range(0, len(data) - 1, 2):
            pos = parse_position(data[i : i + 2])
            self.position_received.emit(pos)

    def _extract_frames(self, buf: bytearray) -> bytearray:
        """Pull complete frames from buffer, emit signals, return remainder."""
        while True:
            idx = buf.find(SYNC_MARKER)
            if idx < 0:
                # Keep last byte in case it's the start of a sync marker
                if len(buf) > 1:
                    buf = buf[-1:]
                return buf

            end = idx + RAW_FRAME_BYTES
            if len(buf) < end:
                # Incomplete frame — trim junk before sync and wait
                buf = buf[idx:]
                return buf

            frame_data = bytes(buf[idx:end])
            pixels = parse_raw_frame(frame_data)
            if pixels is not None:
                self.frame_received.emit(pixels, time.perf_counter())

            buf = buf[end:]

    def stop(self):
        self._running = False


class SerialLink(CcdLink):
    """Thread-safe serial port manager for direct FPGA UART connection."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._ser: serial.Serial | None = None
        self._thread: QThread | None = None
        self._worker: _ReaderWorker | None = None
        self._current_mode = MODE_POSITION
        self._sh = 20
        self._icg = 500_000
        self._avg = 4

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    @property
    def port_name(self) -> str:
        return self._ser.port if self._ser else ""

    def connect(self, port: str, baud: int = BAUD_RATE) -> bool:
        """Open serial port. Returns True on success."""
        self.disconnect()
        try:
            self._ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(0.05)
            self._ser.reset_input_buffer()
            self.connection_changed.emit(True)
            log.info("Connected to %s at %d baud", port, baud)
            return True
        except serial.SerialException as exc:
            self.error_occurred.emit(f"Failed to open {port}: {exc}")
            self._ser = None
            return False

    def disconnect(self):
        """Stop reader and close port."""
        self._stop_reader()
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None
        self.connection_changed.emit(False)

    def send_command(
        self,
        sh: int | None = None,
        icg: int | None = None,
        mode: int | None = None,
        avg: int | None = None,
    ):
        """Send exposure command. None values keep current settings."""
        if not self.is_connected:
            return
        if sh is not None:
            self._sh = sh
        if icg is not None:
            self._icg = icg
        if mode is not None:
            self._current_mode = mode
        if avg is not None:
            self._avg = avg

        cmd = build_command(self._sh, self._icg, self._current_mode, self._avg)
        self._ser.write(cmd)
        log.debug(
            "Sent command: SH=%d ICG=%d mode=%d avg=%d",
            self._sh, self._icg, self._current_mode, self._avg,
        )

    def send_flash_command(
        self,
        lamp_mask: int = 0x01,
        flash_delay_us: int = 0,
        flash_duration_us: int = 500,
        auto_sequence: bool = False,
        raw_capture: bool = True,
    ):
        """Send flash lamp configuration command."""
        if not self.is_connected:
            return
        cmd = build_flash_command(
            lamp_mask, flash_delay_us, flash_duration_us,
            auto_sequence, raw_capture,
        )
        self._ser.write(cmd)
        log.debug(
            "Sent flash cmd: mask=0x%02X delay=%dus dur=%dus seq=%s raw=%s",
            lamp_mask, flash_delay_us, flash_duration_us,
            auto_sequence, raw_capture,
        )

    def start_continuous(self, mode: int = MODE_RAW):
        """Start continuous frame acquisition in a background thread."""
        if not self.is_connected:
            self.error_occurred.emit("Not connected")
            return

        self._stop_reader()
        self._current_mode = mode
        self.send_command(mode=mode)

        self._worker = _ReaderWorker(self._ser, mode)
        self._thread = QThread()
        self._worker.moveToThread(self._thread)

        self._worker.frame_received.connect(self.frame_received)
        self._worker.position_received.connect(self.position_received)
        self._worker.error_occurred.connect(self._on_reader_error)
        self._thread.started.connect(self._worker.run)

        self._thread.start()
        log.info("Started continuous capture (mode=%d)", mode)

    def stop_continuous(self):
        """Stop continuous acquisition."""
        self._stop_reader()

    def _stop_reader(self):
        if self._worker:
            self._worker.stop()
        if self._thread and self._thread.isRunning():
            self._thread.quit()
            self._thread.wait(2000)
        self._worker = None
        self._thread = None

    def _on_reader_error(self, msg: str):
        log.error("Reader error: %s", msg)
        self.error_occurred.emit(msg)

    @staticmethod
    def list_ports() -> list[dict]:
        """List available serial ports."""
        ports = serial.tools.list_ports.comports()
        return [
            {
                "device": p.device,
                "description": p.description or "",
                "hwid": p.hwid or "",
            }
            for p in sorted(ports, key=lambda p: p.device)
        ]

    @staticmethod
    def auto_detect_port() -> str | None:
        """Try to find a likely FPGA UART port."""
        for p in serial.tools.list_ports.comports():
            desc = (p.description or "").lower()
            if any(kw in desc for kw in ("ftdi", "ft2232", "ft232", "usb-serial", "ch340")):
                return p.device
        ports = serial.tools.list_ports.comports()
        return ports[0].device if ports else None
