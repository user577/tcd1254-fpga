"""Flash capture sequences.

Defines multi-angle capture sequences where the FPGA fires one lamp per frame
and streams the corresponding raw frame back. The FPGA auto-sequences through
enabled lamps when FLASH_FLAG_AUTO_SEQUENCE is set.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from ccd_inspector.flash.lamp_config import LampArrayConfig, LampConfig


@dataclass
class FlashSequenceStep:
    """One step in a flash capture sequence."""

    lamp_id: int
    flash_delay_us: int = 0
    flash_duration_us: int = 500

    @classmethod
    def from_lamp(cls, lamp: LampConfig) -> FlashSequenceStep:
        return cls(
            lamp_id=lamp.lamp_id,
            flash_delay_us=lamp.flash_delay_us,
            flash_duration_us=lamp.flash_duration_us,
        )


@dataclass
class FlashSequence:
    """A complete multi-angle capture sequence."""

    name: str
    steps: list[FlashSequenceStep] = field(default_factory=list)
    include_ambient: bool = True  # Capture one frame with no flash (dark reference)

    @property
    def total_frames(self) -> int:
        """Total frames including optional ambient."""
        return len(self.steps) + (1 if self.include_ambient else 0)

    @classmethod
    def round_robin(cls, config: LampArrayConfig,
                    include_ambient: bool = True) -> FlashSequence:
        """Fire each enabled lamp once, in order."""
        steps = [FlashSequenceStep.from_lamp(l) for l in config.enabled_lamps]
        return cls(name="Round Robin", steps=steps, include_ambient=include_ambient)

    @classmethod
    def single_lamp(cls, lamp: LampConfig) -> FlashSequence:
        """Single lamp capture."""
        return cls(
            name=f"Single: {lamp.name}",
            steps=[FlashSequenceStep.from_lamp(lamp)],
            include_ambient=False,
        )
