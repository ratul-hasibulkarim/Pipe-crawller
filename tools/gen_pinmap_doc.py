"""firmware/include/pinmap.h -> docs/pinmap.md (pinmap.h is the single source of truth).

python3 tools/gen_pinmap_doc.py [--check]
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "firmware" / "include" / "pinmap.h"
OUT = ROOT / "docs" / "pinmap.md"


def render() -> str:
    """Markdown table from the `// PIN |` rows."""
    rows = []
    section = ""
    for line in SRC.read_text().splitlines():
        m = re.match(r"\s*// ---- (.+?) -+", line)
        if m:
            section = m.group(1).strip()
        m = re.match(r"\s*// PIN \| (.+?) \| (.+?) \| (.+)$", line)
        if m:
            rows.append((section, *[g.strip() for g in m.groups()]))
    out = [
        "# Controller pin map",
        "",
        "Generated from `firmware/include/pinmap.h` by `tools/gen_pinmap_doc.py` - edit the header, not this file.",
        "",
        "| Group | Pin | Peripheral | Function / alternate |",
        "|---|---|---|---|",
    ]
    out += [f"| {s} | {p} | {per} | {fn} |" for s, p, per, fn in rows]
    out += [
        "",
        "I2C addresses: BNO085 0x4A, VL53L1X 0x29/0x2A, SHT31 0x44, BMP390 0x77, INA219 0x40 (drive) / 0x41 (expansion),",
        "ADS1115 0x48, TCA9548A 0x70 (encoder bus) with AS5600 0x36 on mux channels 0-4.",
        "",
    ]
    return "\n".join(out)


def main() -> int:
    """Generate or check."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    text = render()
    if a.check:
        if not OUT.exists() or OUT.read_text() != text:
            print("docs/pinmap.md is stale")
            return 1
        return 0
    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(text)
    print(f"wrote {OUT.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
