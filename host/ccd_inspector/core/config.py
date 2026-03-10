"""System configuration — load/save from TOML."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
import json

from ccd_inspector.comm.protocol import BAUD_RATE
from ccd_inspector.flash.lamp_config import LampArrayConfig, LampConfig


# Default config directory
if __import__('sys').platform == 'win32':
    _CONFIG_DIR = Path.home() / 'AppData' / 'Local' / 'TCD1254Inspector'
else:
    _CONFIG_DIR = Path.home() / '.config' / 'tcd1254-inspector'

CONFIG_DIR = _CONFIG_DIR
CONFIG_FILE = CONFIG_DIR / 'config.json'
CALIBRATION_DIR = CONFIG_DIR / 'calibration'


@dataclass
class SerialConfig:
    port: str = ""
    baud: int = BAUD_RATE
    auto_detect: bool = True


@dataclass
class ExposureDefaults:
    sh_period: int = 20
    icg_period: int = 500_000
    avg_count: int = 4


@dataclass
class EdgeDetectionDefaults:
    gradient_window: int = 20
    smooth_sigma: float = 3.0
    threshold: float = 50.0
    min_width_px: int = 10
    max_width_px: int = 1500


@dataclass
class PartConfig:
    known_part_width_mm: float = 5.0
    pixel_pitch_mm: float = 0.014
    width_tolerance_pct: float = 15.0


@dataclass
class SystemConfig:
    serial: SerialConfig = field(default_factory=SerialConfig)
    exposure: ExposureDefaults = field(default_factory=ExposureDefaults)
    edge_detection: EdgeDetectionDefaults = field(default_factory=EdgeDetectionDefaults)
    part: PartConfig = field(default_factory=PartConfig)
    lamps: list[dict] = field(default_factory=list)
    dark_frame_path: str = ""
    flat_field_path: str = ""

    def get_lamp_config(self) -> LampArrayConfig:
        """Build LampArrayConfig from stored lamp dicts."""
        if not self.lamps:
            return LampArrayConfig.default_4_angle()
        return LampArrayConfig(
            lamps=[LampConfig(**d) for d in self.lamps]
        )

    def set_lamp_config(self, config: LampArrayConfig):
        self.lamps = [
            {
                'lamp_id': l.lamp_id,
                'name': l.name,
                'angle_deg': l.angle_deg,
                'enabled': l.enabled,
                'flash_duration_us': l.flash_duration_us,
                'flash_delay_us': l.flash_delay_us,
            }
            for l in config.lamps
        ]

    def save(self, path: Path | None = None):
        path = path or CONFIG_FILE
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            'serial': asdict(self.serial),
            'exposure': asdict(self.exposure),
            'edge_detection': asdict(self.edge_detection),
            'part': asdict(self.part),
            'lamps': self.lamps,
            'dark_frame_path': self.dark_frame_path,
            'flat_field_path': self.flat_field_path,
        }
        path.write_text(json.dumps(data, indent=2))

    @classmethod
    def load(cls, path: Path | None = None) -> SystemConfig:
        path = path or CONFIG_FILE
        if not path.exists():
            return cls()
        try:
            data = json.loads(path.read_text())
            cfg = cls()
            if 'serial' in data:
                cfg.serial = SerialConfig(**data['serial'])
            if 'exposure' in data:
                cfg.exposure = ExposureDefaults(**data['exposure'])
            if 'edge_detection' in data:
                cfg.edge_detection = EdgeDetectionDefaults(**data['edge_detection'])
            if 'part' in data:
                cfg.part = PartConfig(**data['part'])
            cfg.lamps = data.get('lamps', [])
            cfg.dark_frame_path = data.get('dark_frame_path', '')
            cfg.flat_field_path = data.get('flat_field_path', '')
            return cfg
        except Exception:
            return cls()
