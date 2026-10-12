"""Exploded-view renderer.

python3 cad/exploded_view.py --size M [--out cad/exports/M/exploded.png]
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from cad.assembly import Instance, Robot, rot, to_loc, trans  # noqa: E402
from cad.render import render  # noqa: E402

FRONT_POD = (
    "body_front_shell",
    "electronics_tray",
    "compute",
    "mcu_board",
    "buck",
    "camera_window",
    "led_ring_mount",
    "nose_bumper",
    "gimbal",
    "camera_module",
    "tilt_servo",
    "pan_gearmotor",
    "slip_ring",
    "articulation_yoke_front",
)
REAR_POD = ("body_rear_shell", "tether_strain_relief", "expansion_motor", "planetary", "battery_caddy", "cell_18650")


def explode_offset(r: Robot, it: Instance) -> np.ndarray:
    """Displacement for one instance in the exploded view."""
    sz = r.sz
    k = sz.rb
    n = it.part.name
    if it.group.startswith("arm"):
        i = int(it.group[-1])
        a = math.radians(sz.arm_angles()[i])
        radial = np.array([math.cos(a), math.sin(a), 0.0])
        level = 1.6
        if n.startswith(("wheel", "bearing")):
            level = 3.2
        elif n.startswith(("drive", "timing", "belt", "drive_belt")):
            level = 2.6
        elif n.startswith(("rocker", "compliance", "slide", "stub")):
            level = 2.2
        elif n.startswith(("expansion_worm", "bevel_pinion", "worm_bearing")):
            level = 0.6
        return radial * k * level
    if n.startswith(FRONT_POD) or it.group == "gimbal":
        dz = 1.2 * k
        if n.startswith(("camera_window", "led_ring", "nose_bumper")):
            dz += 1.5 * k
        if n.startswith(("electronics_tray", "compute", "mcu", "buck")):
            return np.array([0.0, -2.2 * k, dz * 0.3])
        return np.array([0.0, 0.0, dz])
    if n.startswith(REAR_POD):
        dz = -1.2 * k
        if n.startswith("tether"):
            dz -= 1.2 * k
        if n.startswith(("cell", "battery")):
            return np.array([0.0, -2.2 * k, dz * 0.4])
        return np.array([0.0, 0.0, dz])
    if n.startswith("articulation") or n.startswith("joint_boot"):
        return np.array([0.0, 0.0, 0.6 * k])
    if n.startswith("bevel_crown"):
        return np.array([0.0, 0.0, -0.6 * k])
    return np.zeros(3)


def render_exploded(r: Robot, path: Path, theta_frac: float = 0.6) -> None:
    """Render the exploded view."""
    th = r.sz.theta_min + theta_frac * (r.sz.theta_max - r.sz.theta_min)
    view = rot((0, 1, 0), 90)
    items = []
    for it in r.instances(th):
        if it.group == "seal":
            continue
        d = explode_offset(r, it)
        m = view @ trans(*d) @ it.m
        items.append((it.part.shape.moved(to_loc(m)), it.part.color, 1.0))
    render(items, path, f"PIPEBOT-X {r.sz.name} - exploded", elev=24, azim=-65, tol=0.8, size=(11, 7))


def main() -> int:
    """CLI."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--size", default="M")
    ap.add_argument("--out", type=Path)
    a = ap.parse_args()
    r = Robot(a.size)
    out = a.out or ROOT / "cad" / "exports" / a.size / "exploded.png"
    render_exploded(r, out)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
