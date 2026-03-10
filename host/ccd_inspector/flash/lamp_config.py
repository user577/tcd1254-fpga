"""Flash lamp array configuration.

The FPGA controls flash lamp GPIO outputs synchronized to the CCD integration
cycle (ICG/SH timing). The Python side configures which lamps fire and when
via the 'FL' command protocol. The FPGA handles the actual timing.

Lamp GPIO outputs are on J3 connector spare pins (active-high, accent through
MOSFET driver to xenon/LED flash modules).

Timing within the CCD frame:
    ICG ───┐                              ┌───
           └──────────────────────────────┘
    SH  ──────┐  ┌───────────────────────────
              └──┘
    Flash ────────────[delay]─┐  ┌────────────
                              └──┘ (duration)
    Readout                          ▓▓▓▓▓▓▓▓

The flash fires during the integration window (after SH, before readout).
Delay is measured from ICG falling edge.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class LampConfig:
    """Configuration for a single flash lamp."""

    lamp_id: int          # 0-7, maps to FPGA GPIO bit
    name: str = ""
    angle_deg: float = 0.0      # Illumination angle from sensor normal
    enabled: bool = True
    flash_duration_us: int = 500   # Pulse width
    flash_delay_us: int = 0        # Delay from ICG edge
    notes: str = ""

    def __post_init__(self):
        if not self.name:
            self.name = f"Lamp {self.lamp_id}"


@dataclass
class LampArrayConfig:
    """Configuration for the full lamp array."""

    lamps: list[LampConfig] = field(default_factory=list)

    @property
    def enabled_mask(self) -> int:
        """Bitmask of enabled lamps."""
        mask = 0
        for lamp in self.lamps:
            if lamp.enabled:
                mask |= 1 << lamp.lamp_id
        return mask

    @property
    def enabled_lamps(self) -> list[LampConfig]:
        return [l for l in self.lamps if l.enabled]

    @property
    def enabled_count(self) -> int:
        return sum(1 for l in self.lamps if l.enabled)

    def get_lamp(self, lamp_id: int) -> LampConfig | None:
        for lamp in self.lamps:
            if lamp.lamp_id == lamp_id:
                return lamp
        return None

    @classmethod
    def default_4_angle(cls) -> LampArrayConfig:
        """Default 4-lamp config at 0/45/90/135 degrees."""
        return cls(lamps=[
            LampConfig(lamp_id=0, name="Top",     angle_deg=0),
            LampConfig(lamp_id=1, name="Angled",  angle_deg=45),
            LampConfig(lamp_id=2, name="Side",    angle_deg=90),
            LampConfig(lamp_id=3, name="Back",    angle_deg=135),
        ])
