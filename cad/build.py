"""`make cad` entry point: build every size, export STEP/STL/PNG, run the interference
check, and regenerate BOM, mass report, fastener table and assembly manual.

    python3 cad/build.py [--sizes S M L] [--skip-interference] [--no-stl] [--no-thumbs]
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import cadquery as cq
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from cad import interference_check  # noqa: E402
from cad.assembly import Robot, rot, to_loc  # noqa: E402
from cad.exploded_view import render_exploded  # noqa: E402
from cad.lib import primitives as P  # noqa: E402
from cad.lib.part import Part  # noqa: E402
from cad.modules import E_accessories  # noqa: E402
from cad.render import render  # noqa: E402

EXPORTS = ROOT / "cad" / "exports"
DOCS = ROOT / "docs"
TORQUE_NM = {"M2": 0.25, "M2.5": 0.45, "M3": 0.8, "M4": 1.8}  # SHCS into 6061 / heat-set insert (use 60% in inserts)
PROC_MULT = {"CNC": (6.0, 15.0), "FDM": (3.0, 2.0), "cast": (4.0, 3.0), "vacuum": (4.0, 5.0), "moulded": (3.0, 1.0)}


def part_cost(pt: Part, mats: dict) -> float:
    """Unit cost estimate (USD)."""
    if pt.cost is not None:
        return pt.cost
    mat = mats[pt.material]
    mass = pt.unit_mass(mats)
    for key, (mult, minimum) in PROC_MULT.items():
        if key in mat["process"]:
            return round(max(minimum, mass * mat["cost_per_kg"] * mult), 2)
    return round(mass * mat["cost_per_kg"], 2)


def print_orientation_errors(pt: Part) -> list[str]:
    """FDM rule: bearing bores must end up vertical on the build plate."""
    if pt.print_rot is None or not pt.bearing_bores:
        return []
    axis, ang = pt.print_rot
    m = rot(axis, ang)[:3, :3]
    errs = []
    for b in pt.bearing_bores:
        v = m @ np.asarray(b, float)
        if abs(abs(v[2]) - 1.0) > 1e-6:
            errs.append(f"{pt.name}: bearing bore {b} not vertical in print orientation")
    return errs


def export_part(pt: Part, out: Path, stl: bool, thumb: bool) -> None:
    """STEP (+STL, +PNG) for one part in its own frame."""
    (out / "step").mkdir(parents=True, exist_ok=True)
    cq.exporters.export(pt.solid, str(out / "step" / f"{pt.name}.step"))
    if stl:
        (out / "stl").mkdir(parents=True, exist_ok=True)
        shape = pt.solid
        if pt.print_rot is not None:
            shape = pt.solid.rotate((0, 0, 0), pt.print_rot[0], pt.print_rot[1])
        cq.exporters.export(shape, str(out / "stl" / f"{pt.name}.stl"), tolerance=0.05, angularTolerance=0.2)
    if thumb:
        render([(pt.shape, pt.color, 1.0)], out / "png" / f"{pt.name}.png", pt.name, tol=0.2, size=(4, 4))


def views(r: Robot, out: Path) -> None:
    """Assembly iso view + stroke comparison inside a cut-away pipe."""
    sz = r.sz
    view = rot((0, 1, 0), 90)
    th_mid = (sz.theta_min + sz.theta_max) / 2
    items = [(it.part.shape.moved(to_loc(view @ it.m)), it.part.color, 1.0) for it in r.instances(th_mid) if it.group != "seal"]
    render(items, out / "assembly.png", f"PIPEBOT-X {sz.name} - {sz.pipe_for(th_mid):.0f} mm pipe", elev=20, azim=-60, tol=0.6, size=(11, 7))
    for tag, th in (("min", sz.theta_min), ("max", sz.theta_max)):
        rc = sz.pipe_for(th) / 2
        z0, z1 = -sz.rear - 20, sz.z_nose + 20
        pipe = P.tube(rc + 4, rc, z0, z1).cut(P.box(-rc - 10, rc + 10, -rc - 10, 0, z0 - 1, z1 + 1))
        items = [(pipe.val().moved(to_loc(view)), (0.55, 0.6, 0.65), 0.25)]
        items += [(it.part.shape.moved(to_loc(view @ it.m)), it.part.color, 1.0) for it in r.instances(th) if it.group != "seal"]
        render(items, out / f"stroke_{tag}.png", f"{sz.name}: gripping {sz.pipe_for(th):.0f} mm pipe (theta {math.degrees(th):.1f} deg)",
               elev=10, azim=-90, tol=0.8, size=(11, 5))
    front = rot((0, 0, 1), 0)
    items = [(it.part.shape.moved(to_loc(front @ it.m)), it.part.color, 1.0) for it in r.instances(sz.theta_max) if it.group != "seal"]
    render(items, out / "front_view_max.png", f"{sz.name}: nose view at max pipe", elev=90, azim=-90, tol=0.8, size=(6, 6))


def mass_and_bom(r: Robot) -> tuple[dict, list[dict], dict[str, int]]:
    """Mass per module, CG, BOM rows, fastener tally."""
    mats = r.p["materials"]
    inst = r.instances((r.sz.theta_min + r.sz.theta_max) / 2)
    by_mod: dict[str, float] = {}
    total = 0.0
    mom = np.zeros(3)
    for it in inst:
        m = it.part.unit_mass(mats)
        mod = it.part.module
        by_mod[mod] = by_mod.get(mod, 0.0) + m
        total += m
        c = it.shape().Center()
        mom += m * np.array([c.x, c.y, c.z])
    cg = mom / total
    rows = []
    fast: dict[str, int] = {}
    for pt in sorted(r.parts.values(), key=lambda q: (q.module, q.name)):
        unit_cost = part_cost(pt, mats)
        mass = pt.unit_mass(mats)
        proc = pt.process or ("purchased" if pt.purchased else mats[pt.material]["process"])
        rows.append({
            "size": r.sz.name, "part": pt.name, "module": pt.module, "qty": pt.qty, "material": pt.material, "process": proc,
            "unit_mass_g": round(mass * 1000, 1), "unit_cost_usd": unit_cost, "line_cost_usd": round(unit_cost * pt.qty, 2),
            "vendor": "TBD (quote)" if not pt.purchased else "TBD (catalogue)", "notes": pt.note,
        })
        for f, n in pt.fasteners.items():
            fast[f] = fast.get(f, 0) + n * max(pt.qty, 0)
    cat = r.p["catalog"]
    extra = [("tether (per m)", 1, cat["tether_cost_per_m"]), ("compliance spring", 6, 0.5),
             ("hall sensor + magnet", 6, cat["hall_sensor"]["cost"]), ("LED white 1 W", 4, cat["led_white_1w"]["cost"]),
             ("LED IR 850 nm", 2, cat["led_ir_850"]["cost"]), ("VL53L1X ToF", 2, cat["tof_vl53l1x"]["cost"]),
             ("BNO085 IMU", 1, cat["imu_bno085"]["cost"]), ("SHT31 / BMP390 / INA219 breakouts", 1, 15.0)]
    for name, q, c in extra:
        rows.append({"size": r.sz.name, "part": name, "module": "purchased", "qty": q, "material": "-", "process": "purchased",
                     "unit_mass_g": "", "unit_cost_usd": c, "line_cost_usd": round(q * c, 2), "vendor": "TBD (catalogue)", "notes": ""})
    for f, n in sorted(fast.items()):
        rows.append({"size": r.sz.name, "part": f, "module": "fastener", "qty": n, "material": "A2 stainless", "process": "purchased",
                     "unit_mass_g": "", "unit_cost_usd": cat["fastener_cost"], "line_cost_usd": round(n * cat["fastener_cost"], 2),
                     "vendor": "TBD", "notes": ""})
    allow = r.p["cad"]["mass_allowance_frac"]
    mass = {"by_module_kg": {k: round(v, 3) for k, v in sorted(by_mod.items())}, "total_kg": round(total, 3),
            "allowance_frac": allow, "with_allowance_kg": round(total * (1 + allow), 3),
            "target_kg": r.sz.s["mass_target_kg"], "cg_mm": [round(float(v), 2) for v in cg],
            "cg_radial_offset_mm": round(float(math.hypot(cg[0], cg[1])), 2), "cg_offset_param_mm": r.sz.s["cg_offset_mm"]}
    return mass, rows, fast


def write_assembly_manual(results: dict, p: dict) -> None:
    """docs/assembly_manual.md (numbered steps, part names, torque specs)."""
    lines = ["# PIPEBOT-X assembly manual", "",
             "Generated by `cad/build.py` from `params/bot_params.yaml`. Part names match the STEP files in",
             "`cad/exports/<size>/step/` and the BOM (`docs/bom.csv`). Torques are for A2 SHCS into 6061;",
             "into heat-set inserts use 60 %. Use medium threadlocker on every screw in the arm and gearbox.", "",
             "| Thread | Torque into Al (N·m) | Into insert (N·m) |", "|---|---|---|"]
    for k, v in TORQUE_NM.items():
        lines.append(f"| {k} | {v:.2f} | {0.6 * v:.2f} |")
    for size, res in results.items():
        s = p["sizes"][size]
        f = p["cad"]["fastener"][size]
        lines += ["", f"## Size {size} ({s['pipe_id_min_mm']}-{s['pipe_id_max_mm']} mm pipe)", "",
                  "### A. Gearbox and rear pod (flooded gearbox: pack with marine grease, NLGI 2)",
                  f"1. Press 2x `worm_bearing_block` into each of the three radial bores of `gearbox_bulkhead`.",
                  "2. Slide `bevel_pinion` onto each `expansion_worm` shaft (set screw on the flat), insert the shafts from the outside.",
                  f"3. Fit `bevel_crown` on the `planetary_*` output shaft; bolt planetary to the `expansion_motor_*` face ({f}, {TORQUE_NM[f]:.2f} N·m).",
                  "4. Insert motor+planetary into `body_rear_shell` from the rear until the motor face seats on the internal ring; screw the shell to the gearbox (O-ring `service_hatch_gasket` greased).",
                  "5. Check by hand: turning the motor shaft turns all three worms; with the motor unpowered, a worm cannot be back-driven from the sector side (self-lock).",
                  "6. Route the tether through `tether_strain_relief`, clamp the Kevlar member in the collet (pull test 200 N for 60 s), fit the cap with its O-ring.",
                  "", "### B. Arms (repeat x3)",
                  "7. Slide the `pivot_shaft` through lug, `lever_plate`, `worm_sector`, `lever_plate`, lug; E-clips both ends. Mesh the sector at the retracted end tooth (expansion at min).",
                  "8. Assemble the rocker: press thin-section bearings into both `rocker_plate`s, fit `wheel_axle`s through bearings, `wheel_hub`+`wheel_tread`s and the GT2 `timing_pulley`s on the belt side.",
                  "9. Bolt `drive_stepper_*` + `drive_motor_mount` + `drive_worm_gearbox_*` between the rocker plates; fit `drive_belt_GT2`, tension with `belt_tensioner` (deflection 1 mm at 5 N on the long span).",
                  "10. Insert `slide_bushing`s + `stub_pin`s, drop the grip spring in each slot, close with `compliance_spring_retainer` (hall sensor faces the magnet in the pin).",
                  "11. Pin the rocker to the lever tip. Check: rocker pitches freely +/-10 deg and slides +/-5 mm against the spring.",
                  "", "### C. Spine, joint, front pod",
                  "12. Bolt `body_mid_chassis` (spine) to the gearbox front face; pass all wires through the spine (no wire crosses an arm joint).",
                  "13. " + ("Slide the 2x2 18650 pack in `battery_caddy` into the spine (M only)." if size == "M" else ("Fit the 4S2P ring pack (`battery_caddy`) around the expansion motor (L only)." if size == "L" else "S is tether-powered: no battery.")),
                  "14. Fit `articulation_yoke_rear`, `articulation_spider`, `articulation_yoke_front`; slide `joint_boot` over and clamp. Leave a 30 mm service loop of the wiring inside the boot.",
                  "15. Fit compute board, controller PCB and buck on `electronics_tray`; slide the tray into `body_front_shell`.",
                  "16. Gimbal: `slip_ring_6w` and `pan_gearmotor_*` to the tray front; `tilt_servo_*` into `gimbal_pan_yoke`; camera into `gimbal_tilt_cradle`; cradle pins into yoke arms. Check +/-90 deg tilt hard stops and continuous pan.",
                  "17. `led_ring_mount` with 4 white + 2 IR LEDs and the front VL53L1X; `camera_window` dome with O-ring; `nose_bumper` into its groove.",
                  "", "### D. Final checks",
                  f"18. Mass {res['mass']['with_allowance_kg']} kg incl. wiring allowance (target {res['mass']['target_kg']} kg); CG radial offset {res['mass']['cg_radial_offset_mm']} mm.",
                  "19. Run `tests/acceptance` T4 (diameter sweep) dry before the first pipe run; then T1 hold test.",
                  "20. IP68: 30 min at 1 m (T13) with the gearbox flooded and the dry pods sealed."]
    (DOCS / "assembly_manual.md").write_text("\n".join(lines) + "\n")


def main() -> int:
    """Build everything; exit 1 on any failed check."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sizes", nargs="+", default=["S", "M", "L"])
    ap.add_argument("--skip-interference", action="store_true")
    ap.add_argument("--no-stl", action="store_true")
    ap.add_argument("--no-thumbs", action="store_true")
    a = ap.parse_args()
    DOCS.mkdir(exist_ok=True)
    results: dict = {}
    all_rows: list[dict] = []
    failures: list[str] = []
    p = None
    for size in a.sizes:
        t0 = time.time()
        r = Robot(size)
        p = r.p
        out = EXPORTS / size
        out.mkdir(parents=True, exist_ok=True)
        errs = r.check_quantities(r.instances(r.sz.theta_min))
        for pt in r.parts.values():
            errs += print_orientation_errors(pt)
            if pt.module not in ("purchased",) and pt.material in r.p["materials"] and "FDM" in r.p["materials"][pt.material]["process"]:
                if r.p["cad"]["min_wall_mm"] < 2.5:
                    errs.append("min_wall_mm below the 2.5 mm FDM rule")
            export_part(pt, out, not a.no_stl, not a.no_thumbs)
        r.assembly((r.sz.theta_min + r.sz.theta_max) / 2).export(str(out / "assembly.step"))
        views(r, out)
        render_exploded(r, out / "exploded.png")
        mass, rows, fast = mass_and_bom(r)
        all_rows += rows
        if mass["with_allowance_kg"] > mass["target_kg"]:
            errs.append(f"mass {mass['with_allowance_kg']} kg (incl. allowance) over target {mass['target_kg']} kg")
        if mass["cg_radial_offset_mm"] > mass["cg_offset_param_mm"]:
            errs.append(f"CG offset {mass['cg_radial_offset_mm']} mm > {mass['cg_offset_param_mm']} mm used in the roll calc")
        res = {"mass": mass, "fasteners": fast, "errors": errs}
        if not a.skip_interference:
            rep = interference_check.run(size, verbose=False)
            (out / "interference.json").write_text(json.dumps(rep, indent=2))
            res["interference_passed"] = rep["passed"]
            if not rep["passed"]:
                errs.append(f"interference: {len(rep['collisions'])} collisions, {len(rep['pipe'])} pipe issues")
        results[size] = res
        failures += [f"{size}: {e}" for e in errs]
        print(f"[{size}] built in {time.time() - t0:.0f} s, mass {mass['total_kg']} kg / {mass['target_kg']} kg, errors: {len(errs)}", flush=True)
    # reel (common)
    assert p is not None
    reel = E_accessories.tether_reel(p)
    for pt in reel:
        export_part(pt, EXPORTS / "common", not a.no_stl, not a.no_thumbs)
    render([(pt.shape, pt.color, 1.0) for pt in reel], EXPORTS / "common" / "tether_reel.png", "Tether reel", tol=0.8)
    # docs
    with open(DOCS / "bom.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(all_rows[0].keys()))
        w.writeheader()
        w.writerows(all_rows)
    lines = ["# Mass report", "", "Generated by `cad/build.py`. Made parts: CAD volume x density; purchased parts: catalogue mass.", ""]
    for size, res in results.items():
        m = res["mass"]
        lines += [f"## Size {size}", "", "| Module | Mass (kg) |", "|---|---|"]
        lines += [f"| {k} | {v:.3f} |" for k, v in m["by_module_kg"].items()]
        flag = "OVER TARGET" if m["with_allowance_kg"] > m["target_kg"] else "within target"
        lines += [f"| **CAD total** | **{m['total_kg']:.3f}** |",
                  f"| + {m['allowance_frac'] * 100:.0f} % wiring/grease/potting/fasteners | {m['with_allowance_kg'] - m['total_kg']:.3f} |",
                  f"| **Estimated flight mass** | **{m['with_allowance_kg']:.3f}** |", "",
                  f"Target {m['target_kg']} kg: {flag}. CG at {m['cg_mm']} mm (radial offset {m['cg_radial_offset_mm']} mm; "
                  f"roll calc assumes {m['cg_offset_param_mm']} mm).", ""]
    (DOCS / "mass_report.md").write_text("\n".join(lines))
    fl = ["# Fastener table", "", "Per robot, generated by `cad/build.py` (rule 6: M2/M2.5/M3/M4 SHCS + heat-set inserts only).", ""]
    for size, res in results.items():
        fl += [f"## Size {size}", "", "| Fastener | Qty |", "|---|---|"] + [f"| {k} | {v} |" for k, v in sorted(res["fasteners"].items())] + [""]
    (DOCS / "fastener_table.md").write_text("\n".join(fl))
    write_assembly_manual(results, p)
    (EXPORTS / "build_report.json").write_text(json.dumps(results, indent=2))
    for f in failures:
        print("FAIL", f)
    print("CAD BUILD", "PASSED" if not failures else "FAILED")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
