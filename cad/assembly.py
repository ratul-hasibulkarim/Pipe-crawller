"""Full PIPEBOT-X assembly for any lever angle and camera pan/tilt.

Builds every part and places every instance for a given lever angle (theta) and camera pan/tilt. Used by exports, interference check, mass
report, BOM and renders.

    python3 cad/assembly.py --size M [--theta-frac 0.5] [--out cad/exports/M/assembly.step]
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import cadquery as cq
import numpy as np
from OCP.gp import gp_Trsf

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from cad.lib import primitives as P  # noqa: E402
from cad.lib.params import Size, load  # noqa: E402
from cad.lib.part import COLORS, Part  # noqa: E402
from cad.modules import A_body, B_expansion, C_wheel, D_gimbal, E_accessories  # noqa: E402


# ---------------------------------------------------------------------------- transforms
def rot(axis: tuple[float, float, float], deg: float) -> np.ndarray:
    """4x4 rotation about an axis through the origin."""
    a = np.asarray(axis, float)
    a /= np.linalg.norm(a)
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    x, y, z = a
    r = np.array(
        [
            [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
            [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
            [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)],
        ]
    )
    m = np.eye(4)
    m[:3, :3] = r
    return m


def trans(x: float = 0, y: float = 0, z: float = 0) -> np.ndarray:
    """4x4 translation."""
    m = np.eye(4)
    m[:3, 3] = (x, y, z)
    return m


def to_loc(m: np.ndarray) -> cq.Location:
    """Numpy 4x4 -> cq.Location."""
    t = gp_Trsf()
    t.SetValues(*m[0, :4], *m[1, :4], *m[2, :4])
    return cq.Location(t)


@dataclass
class Instance:
    """One placed copy of a part."""

    name: str
    part: Part
    m: np.ndarray
    group: str  # static | arm0..2 | gimbal | seal

    def shape(self) -> cq.Shape:
        """Placed OCC shape."""
        return self.part.shape.moved(to_loc(self.m))


# ---------------------------------------------------------------------------- purchased extras
def purchased_extras(sz: Size, tray: Part, caddy: Part | None) -> dict[str, Part]:
    """Boards, bearings and cells (purchased) built in their own frames."""
    cat = sz.p["catalog"]
    out: dict[str, Part] = {}
    b = sz.bearing
    out["bearing"] = Part(
        f"bearing_{sz.s['drive']['bearing']}",
        "purchased",
        P.bearing(b["id"], b["od"], b["w"]).rotate((0, 0, 0), (1, 0, 0), 90),
        "steel",
        qty=12,
        purchased=True,
        mass_kg=b["mass"],
        cost=b["cost"],
        color=COLORS["steel"],
    )
    bd = cat["board"][sz.s["power"]["compute_board"]]
    out["compute"] = Part(
        f"compute_{sz.s['power']['compute_board'].replace(' ', '_')}",
        "purchased",
        P.box(0, bd["h"], -bd["w"] / 2, bd["w"] / 2, 0, bd["l"]),
        "PCB",
        purchased=True,
        mass_kg=bd["mass"],
        cost=bd["cost"],
        color=COLORS["pcb"],
    )
    mc = cat["board"]["mcu"]
    out["mcu"] = Part(
        "mcu_board_RP2040",
        "purchased",
        P.box(-mc["h"], 0, -mc["w"] / 2, mc["w"] / 2, 0, mc["l"]),
        "PCB",
        purchased=True,
        mass_kg=mc["mass"],
        cost=mc["cost"],
        color=COLORS["pcb"],
    )
    bk = cat["board"]["buck"]
    out["buck"] = Part(
        "buck_48v_12v_5v",
        "purchased",
        P.box(-bk["h"], 0, -bk["w"] / 2, bk["w"] / 2, 0, bk["l"]),
        "PCB",
        purchased=True,
        mass_kg=bk["mass"],
        cost=bk["cost"],
        color=COLORS["pcb"],
    )
    if caddy is not None:
        c = cat["cell_18650"]
        out["cell"] = Part(
            "cell_18650",
            "purchased",
            P.disc(c["d"] / 2, 0, c["l"]),
            "Li-ion",
            qty=len(caddy.meta["cells"]),
            purchased=True,
            mass_kg=c["mass"],
            cost=c["cost"],
            color=COLORS["led"],
        )
    return out


# ---------------------------------------------------------------------------- build + place
class Robot:
    """All parts of one size and their placements."""

    def __init__(self, size: str, p: dict | None = None) -> None:
        self.p = p or load()
        self.sz = Size(size, self.p)
        sz = self.sz
        parts: list[Part] = []
        for mod in (A_body, B_expansion, C_wheel, D_gimbal, E_accessories):
            parts += mod.build(sz)
        self.parts = {pt.name: pt for pt in parts}
        caddy = self.parts.get("battery_caddy")
        self.extras = purchased_extras(sz, self.parts["electronics_tray"], caddy)
        for pt in self.extras.values():
            self.parts[pt.name] = pt

    def get(self, prefix: str) -> Part:
        """Part by exact name or unique prefix."""
        if prefix in self.parts:
            return self.parts[prefix]
        hits = [v for k, v in self.parts.items() if k.startswith(prefix)]
        if len(hits) != 1:
            raise KeyError(f"{prefix}: {len(hits)} matches")
        return hits[0]

    def instances(self, theta: float, pan: float = 0.0, tilt: float = 0.0) -> list[Instance]:
        """Place every instance for lever angle theta (rad) and gimbal pan/tilt (deg)."""
        sz = self.sz
        out: list[Instance] = []
        eye = np.eye(4)
        static = [
            "body_rear_shell",
            "tether_strain_relief",
            "body_mid_chassis",
            "body_front_shell",
            "electronics_tray",
            "gearbox_bulkhead",
            "bevel_crown",
            "camera_window",
            "led_ring_mount",
            "nose_bumper",
            "articulation_yoke_rear",
            "articulation_yoke_front",
            "articulation_spider",
            "joint_boot",
        ]
        if "battery_caddy" in self.parts:
            static.append("battery_caddy")
        for n in static:
            out.append(Instance(n, self.parts[n], eye, "static"))
        for n in ("expansion_motor", "planetary", "pan_gearmotor", "slip_ring"):
            pt = self.get(n)
            out.append(Instance(pt.name, pt, eye, "static"))
        # seals (excluded from interference: compressed in grooves by design)
        g = self.parts["service_hatch_gasket"]
        zs = [
            -sz.rear + sz.cap_len + 2.75,
            sz.z_gearbox0 - 2.25,
            sz.mid - sz.bh / 2,
            sz.z_dome - sz.cad("led_ring_thickness_mm") - 2.25,
            sz.z_dome - 1,
            sz.z_front0 + sz.bh / 2,
        ]
        for k, z in enumerate(zs):
            out.append(Instance(f"service_hatch_gasket#{k}", g, trans(z=z), "seal"))
        # electronics in the tray
        tr = self.parts["electronics_tray"].meta
        bx = tr["board_x"]
        out.append(Instance(self.extras["compute"].name, self.extras["compute"], trans(bx, 0, tr["z0"] + 1), "static"))
        t = sz.cad("min_wall_mm")
        out.append(Instance("mcu_board_RP2040", self.extras["mcu"], trans(bx - t, 0, tr["z0"] + 1), "static"))
        mcu_l = self.p["catalog"]["board"]["mcu"]["l"]
        bk = self.extras["buck"]
        out.append(Instance(bk.name, bk, trans(bx - t, 0, tr["z0"] + 2 + mcu_l), "static"))
        if "cell" in self.extras:
            for k, (x, y, z) in enumerate(self.parts["battery_caddy"].meta["cells"]):
                out.append(Instance(f"cell_18650#{k}", self.extras["cell"], trans(x, y, z), "static"))
        # gimbal
        rz_pan = rot((0, 0, 1), pan)
        cam_m = rz_pan @ trans(z=sz.z_dome) @ rot((0, 1, 0), tilt)
        out.append(Instance("gimbal_pan_yoke", self.parts["gimbal_pan_yoke"], rz_pan, "gimbal"))
        out.append(Instance(self.get("tilt_servo").name, self.get("tilt_servo"), rz_pan, "gimbal"))
        out.append(Instance("gimbal_tilt_cradle", self.parts["gimbal_tilt_cradle"], cam_m, "gimbal"))
        out.append(Instance("camera_module", self.parts["camera_module"], cam_m, "gimbal"))
        # arms
        tip_z, tip_r = sz.tip(theta)
        th_deg = math.degrees(theta)
        (ln0, _), (lp0, _) = sz.lever_plate_y
        (rn0, _), (rp0, _) = sz.rocker_plate_y
        (lv0, _), _ = sz.lever_plate_y
        for i, phi in enumerate(sz.arm_angles()):
            grp = f"arm{i}"
            rz = rot((0, 0, 1), phi)
            lever = rz @ trans(sz.r_pivot) @ rot((0, 1, 0), th_deg)
            rocker = rz @ trans(tip_r, 0, tip_z)

            def add(name: str, part: Part, m: np.ndarray, *, grp: str = grp, i: int = i) -> None:
                out.append(Instance(f"{name}#{i}", part, m, grp))

            add("expansion_worm", self.parts["expansion_worm"], rz)
            add("bevel_pinion", self.parts["bevel_pinion"], rz)
            wl = sz.cad("worm_length_mm")
            rp_, rc_ = sz.bevel
            bp = 3 * sz.cad("bevel_module_mm")
            for k, x in enumerate((rc_ + bp / 2 + 0.3, sz.r_pivot + wl / 2 + 0.5)):
                add(f"worm_bearing_block_{k}", self.parts["worm_bearing_block"], rz @ trans(x, 0, sz.z_worm) @ rot((0, 1, 0), 90))
            add("lever_plate_n", self.parts["lever_plate"], lever @ trans(y=ln0))
            add("lever_plate_p", self.parts["lever_plate"], lever @ trans(y=lp0))
            add("worm_sector", self.parts["worm_sector"], lever)
            add("pivot_shaft", self.parts["pivot_shaft"], lever)
            add("rocker_plate_n", self.parts["rocker_plate"], rocker @ trans(y=rn0))
            add("rocker_plate_p", self.parts["rocker_plate"], rocker @ trans(y=rp0))
            ax = C_wheel.ax(sz)
            for k, z in enumerate((-sz.wheelbase / 2, sz.wheelbase / 2)):
                wm = rocker @ trans(ax, 0, z)
                add(f"wheel_hub_{k}", self.parts["wheel_hub"], wm)
                add(f"wheel_tread_{k}", self.parts["wheel_tread"], wm)
                add(f"wheel_axle_{k}", self.parts["wheel_axle"], wm)
                for j, yb in enumerate((rn0 + sz.s["rocker"]["plate_thickness_mm"] / 2, rp0 + sz.s["rocker"]["plate_thickness_mm"] / 2)):
                    add(f"bearing_{k}{j}", self.extras["bearing"], wm @ trans(y=yb))
            yb = sum(sz.belt_y) / 2
            for pname, (x, z, _) in C_wheel.pulleys(sz):
                add(f"pulley_{pname}", self.get("timing_pulley"), rocker @ trans(x, yb, z))
            for n in ("drive_belt", "belt_tensioner", "drive_motor_mount", "drive_stepper", "drive_worm_gearbox"):
                pt = self.get(n)
                add(pt.name, pt, rocker)
            tr_ = sz.s["rocker"]["plate_thickness_mm"]
            for side, y in (("n", rn0), ("p", rp0)):
                add(f"compliance_spring_retainer_{side}", self.parts["compliance_spring_retainer"], rocker @ trans(y=y))
                add(f"slide_bushing_{side}", self.parts["slide_bushing"], rocker @ trans(y=y))
            add("stub_pin_n", self.parts["stub_pin"], rocker @ trans(y=lv0))
            add("stub_pin_p", self.parts["stub_pin"], rocker @ trans(y=rp0))
            del tr_
        return out

    def check_quantities(self, inst: list[Instance]) -> list[str]:
        """Part.qty must equal the number of placed instances (BOM integrity)."""
        counts: dict[str, int] = {}
        for it in inst:
            counts[it.part.name] = counts.get(it.part.name, 0) + 1
        errs = []
        for name, pt in self.parts.items():
            n = counts.get(name, 0)
            if pt.qty != n:
                errs.append(f"{name}: qty {pt.qty} but {n} placed")
        return errs

    def assembly(self, theta: float, pan: float = 0.0, tilt: float = 0.0) -> cq.Assembly:
        """cq.Assembly (named, coloured) for STEP export."""
        assy = cq.Assembly(name=f"PIPEBOT-X_{self.sz.name}")
        for it in self.instances(theta, pan, tilt):
            assy.add(it.part.shape, name=it.name.replace("#", "_"), loc=to_loc(it.m), color=cq.Color(*it.part.color))
        return assy


def main() -> int:
    """CLI: export the assembled robot as STEP."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--size", default="M")
    ap.add_argument("--theta-frac", type=float, default=0.5, help="0 = min pipe, 1 = max pipe")
    ap.add_argument("--out", type=Path)
    a = ap.parse_args()
    r = Robot(a.size)
    th = r.sz.theta_min + a.theta_frac * (r.sz.theta_max - r.sz.theta_min)
    out = a.out or ROOT / "cad" / "exports" / a.size / "assembly.step"
    out.parent.mkdir(parents=True, exist_ok=True)
    r.assembly(th).export(str(out))
    print(f"wrote {out} (pipe {r.sz.pipe_for(th):.0f} mm)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
