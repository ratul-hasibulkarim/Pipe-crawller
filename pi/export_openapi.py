"""Write pi/api/openapi.yaml from the FastAPI app (python3 pi/export_openapi.py [--check])."""

from __future__ import annotations

import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from pi.app.main import create_app  # noqa: E402

OUT = ROOT / "pi" / "api" / "openapi.yaml"


def main() -> int:
    """Generate or check."""
    text = yaml.safe_dump(create_app().openapi(), sort_keys=False)
    if "--check" in sys.argv:
        return 0 if OUT.exists() and OUT.read_text() == text else 1
    OUT.write_text(text)
    print(f"wrote {OUT.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
