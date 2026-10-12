"""Runtime configuration (environment variables, all with safe defaults)."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path

APP_DIR = Path(__file__).resolve().parent
ROOT = APP_DIR.parents[1]


def _bot_params() -> dict:
    """Generated per-size constants (tools/gen_params.py -> bot_params.json)."""
    return json.loads((APP_DIR / "bot_params.json").read_text())


@dataclass
class Settings:
    """Settings for one Pi process."""

    size: str = field(default_factory=lambda: os.environ.get("PIPEBOT_SIZE", "M"))
    # serial:///dev/ttyACM0?baud=921600  |  tcp://127.0.0.1:7000 (SIL simulator)
    link: str = field(default_factory=lambda: os.environ.get("PIPEBOT_LINK", "tcp://127.0.0.1:7000"))
    camera: str = field(default_factory=lambda: os.environ.get("PIPEBOT_CAMERA", "auto"))  # auto | libcamera | synthetic
    recordings: Path = field(default_factory=lambda: Path(os.environ.get("PIPEBOT_RECORDINGS", str(ROOT / "pi" / "recordings"))))
    static_dir: Path = field(default_factory=lambda: Path(os.environ.get("PIPEBOT_STATIC", str(ROOT / "app" / "dist"))))
    video_w: int = 1280
    video_h: int = 720
    video_fps: int = 30
    ping_hz: float = 4.0
    ack_timeout_s: float = 0.3
    stale_after_s: float = 0.3  # telemetry older than this is "stale" in the UI
    drive_ttl_ms: int = 300
    expand_ttl_ms: int = 500
    max_commands_per_s: int = 50
    tether_length_m: float = field(default_factory=lambda: float(os.environ.get("PIPEBOT_TETHER_M", "50")))

    @property
    def params(self) -> dict:
        """Constants for the configured size."""
        return _bot_params()[self.size]
