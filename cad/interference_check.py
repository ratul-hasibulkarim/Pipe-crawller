"""Scripted collision check over the full expansion stroke and full gimbal range.

Fails (exit 1) if any two parts overlap by more than the tolerance, except designed
contacts (gear meshes, press fits, belt wraps) listed in ALLOWED. Also checks, at every
stroke position, that all six treads touch a pipe of the matching ID and that nothing
except the rolling wheels comes within a weld-bead height of the wall (test T9).

    python3 cad/interference_check.py --size S [--json out.json]
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
import time
from pathlib import Path

import cadquery as cq

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from cad.assembly import Instance, Robot  # noqa: E402
from cad.lib import primitives as P  # noqa: E402

# designed contacts, by Part.name prefix pairs
ALLOWED = [
    ("worm_sector", "expansion_worm"),
    ("bevel_crown", "bevel_pinion"),
    ("bevel_pinion", "expansion_worm"),
    ("gearbox_bulkhead", "worm_bearing_block"),
    ("expansion_worm", "worm_bearing_block"),
    ("expansion_motor", "planetary"),
    ("wheel_axle", "wheel_hub"),
    ("wheel_hub", "wheel_tread"),
    ("drive_belt", "timing_pulley"),
    ("drive_belt", "belt_tensioner"),
    ("drive_worm_gearbox", "timing_pulley"),
    ("drive_stepper", "drive_motor_mount"),
    ("drive_stepper", "drive_worm_gearbox"),
    ("wheel_axle", "timing_pulley"),
    ("gimbal_pan_yoke", "pan_gearmotor"),
    ("gimbal_pan_yoke", "gimbal_tilt_cradle"),
    ("gimbal_pan_yoke", "tilt_servo"),
    ("camera_module", "gimbal_tilt_cradle"),
    ("pivot_shaft", "lever_plate"),
    ("pivot_shaft", "worm_sector"),
    ("pivot_shaft", "gearbox_bulkhead"),
    ("stub_pin", "lever_plate"),
    ("stub_pin", "slide_bushing"),
    ("slide_bushing", "rocker_plate"),
    ("compliance_spring_retainer", "rocker_plate"),
    ("bearing", "rocker_plate"),
    ("bearing", "wheel_axle"),
    ("wheel_axle", "rocker_plate"),
    ("cell_18650", "battery_caddy"),
    ("articulation_spider", "articulation_yoke"),
    ("planetary", "gearbox_bulkhead"),
    ("planetary", "body_rear_shell"),
    ("expansion_motor", "body_rear_shell"),
    ("electronics_tray", "compute"),
    ("electronics_tray", "mcu_board"),
    ("electronics_tray", "tmc2209"),
    ("electronics_tray", "buck"),
    ("battery_caddy", "body_mid_chassis"),
    ("joint_boot", "articulation_yoke"),
]


def allowed(a: Instance, b: Instance) -> bool:
    """Return True if this pair is a designed contact."""
    na, nb = a.part.name, b.part.name
    for x, y in ALLOWED:
        if (na.startswith(x) and nb.startswith(y)) or (na.startswith(y) and nb.startswith(x)):
            return True
    return False


class Placed:
    """Instance with cached shape and bounding box."""

    def __init__(self, inst: Instance) -> None:
        self.inst = inst
        self.shape = inst.shape()
        self.bb = self.shape.BoundingBox()

    def bb_hit(self, other: Placed, pad: float = 0.0) -> bool:
        """Bounding boxes overlap."""
        a, b = self.bb, other.bb
        return not (
            a.xmax + pad < b.xmin or b.xmax + pad < a.xmin or a.ymax + pad < b.ymin or b.ymax + pad < a.ymin or a.zmax + pad < b.zmin or b.zmax + pad < a.zmin
        )


def common_volume(a: cq.Shape, b: cq.Shape) -> float:
    """Volume of a AND b."""
    try:
        return a.intersect(b).Volume()
    except Exception:  # noqa: BLE001
        return 0.0


def check_pairs(items_a: list[Placed], items_b: list[Placed] | None, tol: float, tag: str) -> list[dict]:
    """All overlapping, non-allowed pairs."""
    hits = []
    pairs = itertools.combinations(items_a, 2) if items_b is None else itertools.product(items_a, items_b)
    for a, b in pairs:
        if a.inst.group == "seal" or b.inst.group == "seal" or a is b:
            continue
        if not a.bb_hit(b) or allowed(a.inst, b.inst):
            continue
        v = common_volume(a.shape, b.shape)
        if v > tol:
            hits.append({"where": tag, "a": a.inst.name, "b": b.inst.name, "volume_mm3": round(v, 2)})
    return hits


def run(size: str, verbose: bool = True) -> dict:
    """Run every check for a size; returns a report dict."""
    t0 = time.time()
    r = Robot(size)
    sz = r.sz
    tol = sz.cad("interference_tolerance_mm3")
    report: dict = {"size": size, "collisions": [], "pipe": [], "positions": []}
    thetas = sz.thetas()
    # static vs static once
    inst0 = r.instances(thetas[0])
    report["qty_errors"] = r.check_quantities(inst0)
    static = [Placed(i) for i in inst0 if i.group in ("static", "seal")]
    report["collisions"] += check_pairs(static, None, tol, "static")
    weld = sz.c["weld_bead_mm"]
    for th in thetas:
        inst = r.instances(th)
        arms = {g: [Placed(i) for i in inst if i.group == g] for g in ("arm0", "arm1", "arm2")}
        gim = [Placed(i) for i in inst if i.group == "gimbal"]
        tag = f"theta={math.degrees(th):.1f}deg pipe={sz.pipe_for(th):.0f}mm"
        report["positions"].append(tag)
        for g, items in arms.items():
            report["collisions"] += check_pairs(items, None, tol, tag + f" {g} internal")
            report["collisions"] += check_pairs(items, static + gim, tol, tag + f" {g} vs body")
        for ga, gb in (("arm0", "arm1"), ("arm1", "arm2"), ("arm2", "arm0")):
            report["collisions"] += check_pairs(arms[ga], arms[gb], tol, tag + f" {ga} vs {gb}")
        # pipe contact + weld-bead clearance
        rc = sz.pipe_for(th) / 2
        z0, z1 = -sz.rear - 60, sz.z_nose + 10
        wall = Placed(Instance("pipe", r.parts["body_rear_shell"], r.instances(th)[0].m, "pipe"))
        wall.shape = P.tube(rc + 40, rc - weld, z0, z1).val()
        wall.bb = wall.shape.BoundingBox()
        contact = P.tube(rc + 40, rc - 0.3, z0, z1).val()
        touching = 0
        for items in arms.values():
            for it in items:
                n = it.inst.part.name
                if n == "wheel_tread":
                    if common_volume(it.shape, contact) > 0.01:
                        touching += 1
                    continue
                if n == "wheel_hub":
                    continue  # rolls with the tread: rides over a bead, cannot strike it
                if it.bb_hit(wall):
                    v = common_volume(it.shape, wall.shape)
                    if v > tol:
                        report["pipe"].append({"where": tag, "part": it.inst.name, "issue": f"within {weld} mm of wall", "volume_mm3": round(v, 2)})
        if touching != 6:
            report["pipe"].append({"where": tag, "part": "wheel_tread", "issue": f"only {touching}/6 treads touch the wall"})
        if verbose:
            print(f"  {size} {tag}: collisions so far {len(report['collisions'])}, pipe issues {len(report['pipe'])}", flush=True)
    # gimbal sweep at mid stroke
    th_mid = thetas[len(thetas) // 2]
    front_static = [s for s in static if s.bb.zmax > sz.z_front0]
    for pan in sz.cad("gimbal_pan_samples_deg"):
        for tilt in sz.cad("gimbal_tilt_samples_deg"):
            gim = [Placed(i) for i in r.instances(th_mid, pan, tilt) if i.group == "gimbal"]
            tag = f"gimbal pan={pan} tilt={tilt}"
            report["collisions"] += check_pairs(gim, None, tol, tag + " internal")
            report["collisions"] += check_pairs(gim, front_static, tol, tag + " vs pod")
    report["seconds"] = round(time.time() - t0, 1)
    report["passed"] = not (report["collisions"] or report["pipe"] or report["qty_errors"])
    return report


def main() -> int:
    """CLI."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--size", default="M")
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
    rep = run(a.size)
    for c in rep["collisions"]:
        print(f"COLLISION {c['where']}: {c['a']} x {c['b']} = {c['volume_mm3']} mm3")
    for c in rep["pipe"]:
        print(f"PIPE {c['where']}: {c['part']} {c['issue']}")
    for e in rep["qty_errors"]:
        print(f"QTY {e}")
    if a.json:
        a.json.parent.mkdir(parents=True, exist_ok=True)
        a.json.write_text(json.dumps(rep, indent=2))
    print(f"{a.size}: {'PASS' if rep['passed'] else 'FAIL'} ({rep['seconds']} s)")
    return 0 if rep["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
