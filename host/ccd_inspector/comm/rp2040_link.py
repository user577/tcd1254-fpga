"""RP2040 USB serial command interface with CcdLink frame acquisition.

The RP2040 microcontroller is the system controller -- it manages steppers,
sort sequence, and communicates with the FPGA over SPI. Python connects to
the RP2040 over USB CDC serial using a simple text protocol for configuration
and monitoring, plus binary frame streaming for CCD data.

Text protocol (RP2040-specific commands):
    Send:  "COMMAND args...\n"
    Recv:  "OK ...\n" or "ERR ...\n" or "{json}\n"

Binary frame protocol (when streaming):
    0xAA 0x55 + len_hi len_lo + <len bytes of pixel data>
    Pixel data: TOTAL_PIXELS * uint16 LE, masked to 12 bits.
"""

from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import serial
import serial.tools.list_ports
from PySide6.QtCore import QObject, QThread, Signal

from ccd_inspector.comm.ccd_link import CcdLink
from ccd_inspector.comm.protocol import (
    ADC_MAX,
    MODE_POSITION,
    MODE_RAW,
    SYNC_MARKER,
    TOTAL_PIXELS,
    parse_position,
    parse_raw_frame,
    RAW_FRAME_BYTES,
)

log = logging.getLogger(__name__)

RP2040_BAUD = 115_200  # USB CDC (baud doesn't matter, but set for consistency)
TIMEOUT = 2.0


# ---------------------------------------------------------------------------
# Data classes for RP2040-specific status
# ---------------------------------------------------------------------------

@dataclass
class AxisStatus:
    """Status of a single stepper axis."""
    position: int = 0
    busy: bool = False
    homed: bool = False
    home_switch: bool = False
    enabled: bool = False


@dataclass
class SortStats:
    """Sort sequence statistics."""
    inspected: int = 0
    accepted: int = 0
    rejected: int = 0
    rescanned: int = 0
    no_part: int = 0
    cycles: int = 0


@dataclass
class SystemStatus:
    """Full system status."""
    state: str = "unknown"
    fpga_running: bool = False
    axes: list[AxisStatus] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Reader worker for binary frame streaming
# ---------------------------------------------------------------------------

class _RP2040ReaderWorker(QObject):
    """Runs in a QThread -- reads serial data and emits frames.

    Handles the mixed protocol: binary frames (0xAA 0x55 sync) and
    text response lines can be interleaved during streaming.
    """

    frame_received = Signal(np.ndarray, float)   # pixels, timestamp
    position_received = Signal(object)            # float | None
    error_occurred = Signal(str)
    text_line_received = Signal(str)              # text responses during streaming

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
        """Position data comes as 2-byte chunks."""
        for i in range(0, len(data) - 1, 2):
            pos = parse_position(data[i : i + 2])
            self.position_received.emit(pos)

    def _extract_frames(self, buf: bytearray) -> bytearray:
        """Pull complete frames from buffer, emit signals, return remainder.

        Frame format: 0xAA 0x55 + 2200 * uint16 LE (RAW_FRAME_BYTES total).
        Any bytes before a sync marker that look like text lines are emitted
        via text_line_received.
        """
        while True:
            idx = buf.find(SYNC_MARKER)
            if idx < 0:
                # No sync marker -- check for text lines in the buffer
                self._extract_text_lines(buf)
                # Keep last byte in case it's the start of a sync marker
                if len(buf) > 1:
                    buf = buf[-1:]
                return buf

            # Emit any text data before the sync marker
            if idx > 0:
                self._extract_text_lines(buf[:idx])

            end = idx + RAW_FRAME_BYTES
            if len(buf) < end:
                # Incomplete frame -- trim junk before sync and wait
                buf = buf[idx:]
                return buf

            frame_data = bytes(buf[idx:end])
            pixels = parse_raw_frame(frame_data)
            if pixels is not None:
                self.frame_received.emit(pixels, time.perf_counter())

            buf = buf[end:]

    def _extract_text_lines(self, data: bytearray | bytes):
        """Extract and emit complete text lines from data."""
        try:
            text = bytes(data).decode("ascii", errors="replace")
        except Exception:
            return
        for line in text.split("\n"):
            line = line.strip()
            if line:
                self.text_line_received.emit(line)

    def stop(self):
        self._running = False


# ---------------------------------------------------------------------------
# RP2040Link -- CcdLink + RP2040-specific commands
# ---------------------------------------------------------------------------

class RP2040Link(CcdLink):
    """CcdLink implementation for the RP2040 system controller.

    Provides both the CcdLink interface (frame acquisition, exposure control)
    and RP2040-specific methods (motion, homing, config, sort control).
    """

    # Additional signal for text responses during streaming
    text_response_received = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._ser: serial.Serial | None = None
        self._thread: QThread | None = None
        self._worker: _RP2040ReaderWorker | None = None
        self._current_mode = MODE_POSITION
        self._sh = 20
        self._icg = 500_000
        self._avg = 1
        self._streaming = False

    # ================================================================
    # CcdLink interface
    # ================================================================

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    @property
    def port_name(self) -> str:
        return self._ser.port if self._ser else ""

    def connect(self, port: str, **kwargs) -> bool:
        """Open USB serial connection to RP2040.

        Keyword args:
            timeout (float): Serial timeout in seconds (default 2.0).
        """
        timeout = kwargs.get("timeout", TIMEOUT)
        self.disconnect()
        try:
            self._ser = serial.Serial(port, RP2040_BAUD, timeout=timeout)
            time.sleep(0.1)
            self._ser.reset_input_buffer()

            # Verify connection with PING
            resp = self._command("PING")
            if resp.strip() != "PONG":
                log.warning("Unexpected PING response: %r", resp)
                self._ser.close()
                self._ser = None
                return False

            log.info("Connected to RP2040 on %s", port)
            self.connection_changed.emit(True)
            return True
        except (serial.SerialException, OSError) as exc:
            log.error("Failed to connect to %s: %s", port, exc)
            self.error_occurred.emit(f"Failed to connect to {port}: {exc}")
            self._ser = None
            return False

    def disconnect(self):
        """Stop streaming, close serial connection."""
        self.stop_continuous()
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
        """Send exposure command via RP2040 text protocol.

        None values keep current settings.
        """
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

        self._command_ok(
            f"EXPO {self._sh} {self._icg} {self._current_mode} {self._avg}"
        )
        log.debug(
            "Sent EXPO: SH=%d ICG=%d mode=%d avg=%d",
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
        """Send flash lamp configuration via RP2040 text protocol."""
        if not self.is_connected:
            return
        flags = 0
        if auto_sequence:
            flags |= 0x01
        if raw_capture:
            flags |= 0x02
        self._command_ok(
            f"FLASH {lamp_mask} {flash_delay_us} {flash_duration_us} {flags}"
        )
        log.debug(
            "Sent FLASH: mask=0x%02X delay=%dus dur=%dus flags=0x%02X",
            lamp_mask, flash_delay_us, flash_duration_us, flags,
        )

    def start_continuous(self, mode: int = MODE_RAW):
        """Start continuous frame acquisition via RP2040.

        Sends EXPO to configure mode, then STREAM_START, then spawns
        a reader thread that parses binary frames from the serial port.
        """
        if not self.is_connected:
            self.error_occurred.emit("Not connected")
            return

        self._stop_reader()
        self._current_mode = mode
        self.send_command(mode=mode)
        self._command("STREAM_START")
        self._streaming = True

        # Spawn reader thread
        self._worker = _RP2040ReaderWorker(self._ser, mode)
        self._thread = QThread()
        self._worker.moveToThread(self._thread)

        self._worker.frame_received.connect(self.frame_received)
        self._worker.position_received.connect(self.position_received)
        self._worker.error_occurred.connect(self._on_reader_error)
        self._worker.text_line_received.connect(self.text_response_received)
        self._thread.started.connect(self._worker.run)

        self._thread.start()
        log.info("Started continuous capture via RP2040 (mode=%d)", mode)

    def stop_continuous(self):
        """Stop continuous acquisition."""
        self._stop_reader()
        if self._streaming and self.is_connected:
            try:
                self._ser.write(b"STREAM_STOP\n")
                self._ser.flush()
                time.sleep(0.05)
                self._ser.reset_input_buffer()
            except Exception:
                pass
        self._streaming = False

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
        """Try to find the RP2040 USB CDC port.

        Looks for Raspberry Pi Pico / RP2040 VID:PID (2E8A:000A).
        """
        for p in serial.tools.list_ports.comports():
            # RP2040 USB CDC: VID=0x2E8A, PID=0x000A
            if p.vid == 0x2E8A and p.pid == 0x000A:
                return p.device
            # Also match description
            desc = (p.description or "").lower()
            if "pico" in desc or "rp2040" in desc:
                return p.device
        return None

    # ================================================================
    # Single-frame capture (blocking convenience method)
    # ================================================================

    def capture_frame(self) -> np.ndarray | None:
        """Capture and download a single frame. Blocking.

        Sends the FRAME command, reads the 0xAA 0x55 header + pixel data,
        and returns a (TOTAL_PIXELS,) uint16 array masked to 12 bits.
        Returns None on timeout or protocol error.
        """
        if not self.is_connected:
            return None
        try:
            self._ser.write(b"FRAME\n")
            self._ser.flush()

            # Read until we find the sync marker or timeout
            # The RP2040 may echo the command or send an OK first
            buf = bytearray()
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                waiting = self._ser.in_waiting
                if waiting > 0:
                    buf.extend(self._ser.read(min(waiting, 8192)))
                else:
                    time.sleep(0.005)

                # Look for sync marker
                idx = buf.find(SYNC_MARKER)
                if idx >= 0 and len(buf) >= idx + RAW_FRAME_BYTES:
                    frame_data = bytes(buf[idx : idx + RAW_FRAME_BYTES])
                    pixels = parse_raw_frame(frame_data)
                    return pixels

            log.warning("capture_frame timed out (%d bytes received)", len(buf))
            return None
        except Exception as exc:
            log.error("capture_frame failed: %s", exc)
            return None

    # ================================================================
    # Internal helpers
    # ================================================================

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

    # ================================================================
    # Low-level text command interface
    # ================================================================

    def _command(self, cmd: str) -> str:
        """Send a command and read the response line."""
        if not self.is_connected:
            raise ConnectionError("Not connected to RP2040")

        self._ser.write((cmd + "\n").encode())
        self._ser.flush()
        line = self._ser.readline().decode(errors="replace").strip()
        return line

    def _command_json(self, cmd: str) -> dict[str, Any]:
        """Send a command and parse JSON response."""
        resp = self._command(cmd)
        return json.loads(resp)

    def _command_ok(self, cmd: str) -> bool:
        """Send a command and check for OK response."""
        resp = self._command(cmd)
        if resp.startswith("OK"):
            return True
        log.error("Command failed: %s -> %s", cmd, resp)
        return False

    # ================================================================
    # RP2040-specific commands (not part of CcdLink interface)
    # ================================================================

    # ---- Status ----

    def ping(self) -> bool:
        """Check if RP2040 is responsive."""
        try:
            return self._command("PING").strip() == "PONG"
        except Exception:
            return False

    def get_status(self) -> SystemStatus:
        """Get full system status."""
        data = self._command_json("STATUS")
        status = SystemStatus(
            state=data.get("state", "unknown"),
            fpga_running=data.get("fpga_running", False),
        )
        for ax in data.get("axes", []):
            status.axes.append(AxisStatus(
                position=ax.get("pos", 0),
                busy=ax.get("busy", False),
                homed=ax.get("homed", False),
                home_switch=ax.get("home_sw", False),
                enabled=ax.get("en", False),
            ))
        return status

    def get_stats(self) -> SortStats:
        """Get sort statistics."""
        data = self._command_json("STATS")
        return SortStats(
            inspected=data.get("inspected", 0),
            accepted=data.get("accepted", 0),
            rejected=data.get("rejected", 0),
            rescanned=data.get("rescanned", 0),
            no_part=data.get("no_part", 0),
            cycles=data.get("cycles", 0),
        )

    def reset_stats(self) -> bool:
        """Reset sort statistics."""
        return self._command_ok("STATS_RESET")

    # ---- Configuration ----

    def get_config(self) -> dict[str, Any]:
        """Get current system configuration."""
        return self._command_json("CONFIG_GET")

    def set_config(self, key: str, value: Any) -> bool:
        """Set a configuration parameter.

        Keys: auto_start, auto_home, default_sh, default_icg,
              sort.min_w, sort.max_w, sort.grouped, sort.rescan,
              seq.feed_steps, seq.feed_speed, seq.present_steps, seq.settle_ms
        """
        return self._command_ok(f"CONFIG_SET {key}={value}")

    def save_config(self) -> bool:
        """Persist current config to flash."""
        return self._command_ok("CONFIG_SAVE")

    def reset_config(self) -> bool:
        """Reset to factory defaults."""
        return self._command_ok("CONFIG_RESET")

    # ---- Sort control ----

    def start(self) -> bool:
        """Start autonomous sorting."""
        return self._command_ok("START")

    def stop(self) -> bool:
        """Stop after current cycle."""
        return self._command_ok("STOP")

    def estop(self) -> bool:
        """Emergency stop."""
        return self._command_ok("ESTOP")

    # ---- Homing ----

    def home_all(self) -> bool:
        """Home all axes."""
        return self._command_ok("HOME")

    def home_axis(self, axis: int) -> bool:
        """Home a single axis (0-3)."""
        return self._command_ok(f"HOME {axis}")

    # ---- Motion ----

    def move(self, axis: int, steps: int, speed: int) -> bool:
        """Move an axis by steps at speed."""
        return self._command_ok(f"MOVE {axis} {steps} {speed}")

    def jog(self, axis: int, direction: bool, speed: int) -> bool:
        """Start jogging an axis."""
        return self._command_ok(f"JOG {axis} {1 if direction else 0} {speed}")

    def halt(self, axis: int) -> bool:
        """Stop an axis."""
        return self._command_ok(f"HALT {axis}")

    def enable_axis(self, axis: int, enable: bool) -> bool:
        """Enable/disable a stepper driver."""
        return self._command_ok(f"ENABLE {axis} {1 if enable else 0}")

    # ---- FPGA config (via RP2040 SPI bridge) ----

    def set_exposure(self, sh: int, icg: int, mode: int = 1, avg: int = 1) -> bool:
        """Set FPGA exposure parameters via RP2040."""
        return self._command_ok(f"EXPO {sh} {icg} {mode} {avg}")

    def set_flash(self, mask: int, delay_us: int, duration_us: int,
                  flags: int = 0) -> bool:
        """Set FPGA flash parameters via RP2040."""
        return self._command_ok(f"FLASH {mask} {delay_us} {duration_us} {flags}")

    def trigger_capture(self) -> bool:
        """Trigger a single FPGA capture."""
        return self._command_ok("TRIGGER")

    def read_shadow(self) -> float:
        """Read shadow detection result. Returns pixel position or -1.0."""
        data = self._command_json("SHADOW")
        return data.get("shadow_px", -1.0)
