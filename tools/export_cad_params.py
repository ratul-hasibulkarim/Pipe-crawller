#!/usr/bin/env python3
"""Export bot_params.yaml as SolidWorks / Fusion 360 / FreeCAD parameter tables.

    python tools/export_cad_params.py            # writes cad/exports/<size>/params_*.{txt,csv}

* params_solidworks.txt - SolidWorks Tools > Equations > Import: one global variable per
  line ("lever_length_mm" = 116). Link sketch dimensions to them and the model follows
  the YAML when you re-import.
* params.csv - name, value, unit, comment: Fusion 360 (Modify > Change Parameters, via the
  ParameterIO add-in) or a FreeCAD spreadsheet.

Includes the derived values a CAD user needs (lever angle range, stroke, forces) from
calc/design_calcs.py, so the design checks and the CAD use the same numbers.
"""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "calc"))

import design_calcs as dc  # noqa: E402
import mechanism as mech  # noqa: E402


def flatten(prefix: str, d: Any, out: dict[str, float]) -> None:
    """Numeric leaves of a nested dict as prefix_key names."""
    if isinstance(d, dict):
        for k, v in d.items():
            flatten(f"{prefix}_{k}" if prefix else str(k), v, out)
    elif isinstance(d, (int, float)) and not isinstance(d, bool):
        out[re.sub(r"[^A-Za-z0-9_]", "_", prefix)] = float(d)


def unit(name: str) -> str:
    """Return the unit implied by the name suffix."""
    for suf, u in (
        ("_n_mm", "N/mm"),
        ("_mm_s", "mm/s"),
        ("_w_m2k", "W/m2K"),
        ("_k_per_w", "K/W"),
        ("_mm", "mm"),
        ("_deg", "deg"),
        ("_n", "N"),
        ("_kg", "kg"),
        ("_nm", "N*m"),
        ("_mpa", "MPa"),
        ("_v", "V"),
        ("_a", "A"),
        ("_s", "s"),
    ):
        if name.endswith(suf):
            return u
    return ""


def table(p: dict[str, Any], size: str) -> dict[str, float]:
    """All numbers for one size."""
    out: dict[str, float] = {}
    flatten("", p["sizes"][size], out)
    flatten("cad", p["cad"], out)
    d, _ = dc.evaluate_size(p, size)
    th0, th1 = mech.stroke(p, size)
    out.update(
        {
            "lever_angle_retracted_deg": math.degrees(th0),
            "lever_angle_extended_deg": math.degrees(th1),
            "wheel_reach_retracted_dia_mm": mech.pipe_for_theta(mech.arm_geometry(p, size), th0),
            "wheel_reach_extended_dia_mm": mech.pipe_for_theta(mech.arm_geometry(p, size), th1),
            "compliance_travel_mm": p["common"]["arm_compliance_travel_mm"],
            "arm_angle_deg": p["common"]["arm_angle_deg"],
        }
    )
    derived = ("grip_per_arm_n", "grip_limit_n", "arm_structural_load_n", "lever_torque_max_nm", "sector_tooth_force_n", "expansion_motor_torque_nm")
    for k in (*derived, "bend_radius_mm"):
        out[f"calc_{k}"] = d[k]
    return out


def main() -> None:
    """Write the tables for every size."""
    p = dc.load_params()
    for size in p["sizes"]:
        t = table(p, size)
        out = ROOT / "cad" / "exports" / size
        out.mkdir(parents=True, exist_ok=True)
        sw = [f'"{k}" = {v:.6g}{"mm" if unit(k) == "mm" else ("deg" if unit(k) == "deg" else "")}' for k, v in sorted(t.items())]
        (out / "params_solidworks.txt").write_text("\n".join(sw) + "\n")
        rows = ["name,value,unit"] + [f"{k},{v:.6g},{unit(k)}" for k, v in sorted(t.items())]
        (out / "params.csv").write_text("\n".join(rows) + "\n")
        print(f"{size}: {len(t)} parameters -> {out.relative_to(ROOT)}/params_solidworks.txt, params.csv")


if __name__ == "__main__":
    main()
