"""Parameter access and derived geometry shared by every CAD module.

Everything here is computed from params/bot_params.yaml and calc/mechanism.py so the
CAD cannot drift from the design calcs.

Global frame: Z = pipe/robot axis (+Z toward the nose); z = 0 is the lever-pivot plane
(front face of the gearbox bulkhead). Arms A/B/C lie in radial planes at 0/120/240 deg.
Arm-local frame: X radial (out), Y tangential (+Y = belt side), Z axial.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "calc"))
import mechanism as mech  # noqa: E402

PARAMS_PATH = ROOT / "params" / "bot_params.yaml"


def load(path: Path = PARAMS_PATH) -> dict[str, Any]:
    """Load bot_params.yaml."""
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


@dataclass
class Size:
    """All dimensions for one robot size, with derived layout values."""

    name: str
    p: dict[str, Any] = field(repr=False)

    # ---- raw accessors -------------------------------------------------
    @property
    def s(self) -> dict[str, Any]:
        """Per-size block."""
        return self.p["sizes"][self.name]

    @property
    def c(self) -> dict[str, Any]:
        """Common block."""
        return self.p["common"]

    def cad(self, key: str) -> Any:
        """CAD constant, resolving per-size dicts."""
        v = self.p["cad"][key]
        return v[self.name] if isinstance(v, dict) and self.name in v else v

    def motor(self, which: str) -> dict[str, Any]:
        """Motor record for 'drive' or 'expansion'."""
        return self.p["motors"][self.s[which]["motor"]]

    # ---- body ----------------------------------------------------------
    @property
    def rb(self) -> float:
        """Body outer radius."""
        return self.s["body_dia_mm"] / 2

    @property
    def wall(self) -> float:
        """Pod shell wall."""
        return self.s["layout"]["wall_mm"]

    @property
    def rear(self) -> float:
        """Rear pod length (z from -rear to 0)."""
        return self.s["layout"]["rear_pod_mm"]

    @property
    def mid(self) -> float:
        """Mid (arm) zone length (z from 0 to mid)."""
        return self.s["layout"]["mid_zone_mm"]

    @property
    def joint(self) -> float:
        """Articulation joint length."""
        return self.s["layout"]["joint_mm"]

    @property
    def front(self) -> float:
        """Front pod length."""
        return self.s["layout"]["front_pod_mm"]

    @property
    def z_front0(self) -> float:
        """Rear face of the front pod."""
        return self.mid + self.joint

    @property
    def z_nose(self) -> float:
        """Tip of the dome."""
        return self.z_front0 + self.front

    @property
    def bh(self) -> float:
        """Bulkhead thickness."""
        return self.cad("bulkhead_mm")

    # ---- arm geometry ----------------------------------------------------
    @cached_property
    def geo(self) -> mech.ArmGeometry:
        """Planar arm geometry from calc/mechanism.py."""
        return mech.arm_geometry(self.p, self.name)

    @property
    def theta_min(self) -> float:
        """Lever angle at min pipe (rad)."""
        return mech.theta_for_pipe(self.geo, self.s["pipe_id_min_mm"])

    @property
    def theta_max(self) -> float:
        """Lever angle at max pipe (rad)."""
        return mech.theta_for_pipe(self.geo, self.s["pipe_id_max_mm"])

    def thetas(self) -> list[float]:
        """Stroke samples for interference checking (rad)."""
        return mech.theta_samples(self.theta_min, self.theta_max, self.cad("stroke_samples"))

    @property
    def r_pivot(self) -> float:
        """Lever pivot radius."""
        return self.s["lever"]["pivot_radius_mm"]

    @property
    def lever_len(self) -> float:
        """Lever length P->T."""
        return self.s["lever"]["length_mm"]

    @property
    def wheel_r(self) -> float:
        """Wheel outer radius."""
        return self.s["wheel_dia_mm"] / 2

    @property
    def wheelbase(self) -> float:
        """Rocker axle spacing."""
        return self.s["wheelbase_mm"]

    # ---- tangential stack (arm-local Y) ------------------------------------
    @property
    def pod_half(self) -> float:
        """Half width of the drive motor frame."""
        return self.motor("drive")["frame_mm"] / 2

    @property
    def centre_half(self) -> float:
        """Half width of the centre layer (pod / wheel / sector)."""
        return max(self.pod_half, self.s["wheel_width_mm"] / 2)

    @property
    def belt_y(self) -> tuple[float, float]:
        """Belt layer, +Y side."""
        y0 = self.centre_half + 0.5
        return y0, y0 + self.s["rocker"]["belt_width_mm"]

    @property
    def rocker_plate_y(self) -> tuple[tuple[float, float], tuple[float, float]]:
        """(-Y plate, +Y plate) spans."""
        t = self.s["rocker"]["plate_thickness_mm"]
        neg = (-(self.centre_half + 1.0 + t), -(self.centre_half + 1.0))
        y0 = self.belt_y[1] + 1.0
        return neg, (y0, y0 + t)

    @property
    def lever_plate_y(self) -> tuple[tuple[float, float], tuple[float, float]]:
        """(-Y plate, +Y plate) spans."""
        t = self.s["lever"]["plate_thickness_mm"]
        (n0, _), (_, p1) = self.rocker_plate_y
        return (n0 - 0.5 - t, n0 - 0.5), (p1 + 0.5, p1 + 0.5 + t)

    @property
    def lug_y(self) -> tuple[tuple[float, float], tuple[float, float]]:
        """Bulkhead pivot lugs, outside the lever plates."""
        t = self.cad("lug_thickness_mm")
        (n0, _), (_, p1) = self.lever_plate_y
        return (n0 - 0.5 - t, n0 - 0.5), (p1 + 0.5, p1 + 0.5 + t)

    # ---- expansion gearing -------------------------------------------------
    @property
    def sector_r(self) -> float:
        """Sector pitch radius."""
        e = self.s["expansion"]
        return e["sector_module_mm"] * e["sector_teeth"] / 2

    @property
    def worm_d(self) -> float:
        """Expansion worm pitch diameter."""
        return self.s["expansion"]["worm_pitch_dia_mm"]

    @property
    def z_worm(self) -> float:
        """Axial position of the expansion worm axes (behind the pivot plane)."""
        return -(self.sector_r + self.worm_d / 2)

    @property
    def bevel(self) -> tuple[float, float]:
        """(pinion pitch radius, crown pitch radius)."""
        m = self.cad("bevel_module_mm")
        zp = self.cad("bevel_pinion_teeth")
        rp = m * zp / 2
        return rp, rp * self.s["expansion"]["bevel_ratio"]

    @property
    def z_gearbox0(self) -> float:
        """Rear face of the gearbox housing."""
        m = self.s["expansion"]["sector_module_mm"]
        a = self.z_worm - self.worm_d / 2 - m - self.wall
        rp, _ = self.bevel
        b = self.z_worm - rp - 1.2 * self.cad("bevel_module_mm") * 3 - self.wall
        return min(a, b)

    @property
    def z_motor_face(self) -> float:
        """Expansion motor front face (planetary sits between it and the gearbox)."""
        return self.z_gearbox0 - self.cad("planetary_len_mm")

    @property
    def z_motor_back(self) -> float:
        """Expansion motor rear face."""
        return self.z_motor_face - self.motor("expansion")["length_mm"]

    @property
    def cap_len(self) -> float:
        """Rear cap length left over in the rear pod."""
        return self.z_motor_back + self.rear

    # ---- rocker / drive pod ------------------------------------------------
    @property
    def pod_drop(self) -> float:
        """Drive motor axis below the wheel axles."""
        return self.s["rocker"]["pod_drop_mm"]

    @property
    def pod_len(self) -> float:
        """Motor + worm gearbox length."""
        return self.motor("drive")["length_mm"] + self.cad("drive_gearbox_len_mm")

    @property
    def axle_d(self) -> float:
        """Wheel axle diameter."""
        return self.s["drive"]["axle_dia_mm"]

    @property
    def bearing(self) -> dict[str, float]:
        """Wheel bearing record."""
        return self.p["catalog"]["bearing"][self.s["drive"]["bearing"]]

    # ---- front pod ------------------------------------------------------------
    @property
    def dome_r(self) -> float:
        """Dome outer radius."""
        return self.cad("dome_outer_radius_mm")

    @property
    def z_dome(self) -> float:
        """Dome centre = camera tilt/pan centre."""
        return self.z_nose - self.dome_r

    def arm_angles(self) -> list[float]:
        """Arm plane angles (deg)."""
        n = self.c["arm_count"]
        return [i * self.c["arm_angle_deg"] for i in range(n)]

    def tip(self, theta: float) -> tuple[float, float]:
        """Rocker pivot (z, r) for lever angle theta."""
        return mech.tip(self.geo, theta)

    def pipe_for(self, theta: float) -> float:
        """Pipe ID gripped at theta."""
        return mech.pipe_for_theta(self.geo, theta)


def sizes(p: dict[str, Any] | None = None) -> list[Size]:
    """All configured sizes."""
    p = p or load()
    return [Size(n, p) for n in p["sizes"]]


def deg(rad: float) -> float:
    """Radians to degrees."""
    return math.degrees(rad)
