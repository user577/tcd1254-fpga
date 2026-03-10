"""RP2040 USB serial command interface.

The RP2040 microcontroller is the system controller — it manages steppers,
sort sequence, and communicates with the FPGA over SPI. Python connects to
the RP2040 over USB CDC serial using a simple text protocol for configuration
and monitoring.

Protocol: text-based, one command per line.
    Send:  "COMMAND args...\n"
    Recv:  "OK ...\n" or "ERR ...\n" or "{json}\n"
"""

from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass, field
from typing import Any

import serial
import serial.tools.list_ports

log = logging.getLogger(__name__)

RP2040_BAUD = 115_200  # USB CDC (baud doesn't matter, but set for consistency)
TIMEOUT = 2.0


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


class RP2040Link:
    """Thread-safe interface to the RP2040 controller over USB serial."""

    def __init__(self):
        self._ser: serial.Serial | None = None

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    @property
    def port_name(self) -> str:
        return self._ser.port if self._ser else ""

    # ---- Connection ----

    def connect(self, port: str, timeout: float = TIMEOUT) -> bool:
        """Open USB serial connection to RP2040."""
        self.disconnect()
        try:
            self._ser = serial.Serial(port, RP2040_BAUD, timeout=timeout)
            time.sleep(0.1)
            self._ser.reset_input_buffer()

            # Verify connection
            resp = self._command("PING")
            if resp.strip() != "PONG":
                log.warning("Unexpected PING response: %r", resp)
                self._ser.close()
                self._ser = None
                return False

            log.info("Connected to RP2040 on %s", port)
            return True
        except (serial.SerialException, OSError) as exc:
            log.error("Failed to connect to %s: %s", port, exc)
            self._ser = None
            return False

    def disconnect(self):
        """Close serial connection."""
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    # ---- Low-level command interface ----

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
        log.error("Command failed: %s → %s", cmd, resp)
        return False

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

    # ---- Port detection ----

    @staticmethod
    def list_ports() -> list[dict[str, str]]:
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
