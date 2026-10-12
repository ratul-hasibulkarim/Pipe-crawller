"""Expansion-mechanism kinematics shared by the calcs, CAD and firmware LUT generator.

Each arm is a lever in its own radial plane. Coordinates are (z, r): z is axial
(+z toward the nose) measured from the lever pivot P, r is radial from the pipe axis.

    P = (0, pivot_r)        lever pivot on the front face of the rear bulkhead
    lever P -> T            length L, angle theta from the axis, T = wheel-rocker pivot
    worm sector             fixed to the lever at P, meshes a radial worm in the bulkhead

The three worms are driven from one central bevel gear, so all levers always share
the same theta (synchronous, self-centring). The worms are self-locking, so the
arms hold their position with the power off.

Why not a cone / leadscrew nut (revision 2)? The radial stroke is ~2x the body
radius on every size, and a wedge needs ramp length + travel = 2 * stroke / MA of
free axial space on one side of the follower, which does not fit (see docs/theory_of_operation.md).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

Params = dict[str, Any]


@dataclass(frozen=True)
class ArmGeometry:
    """Planar arm geometry for one robot size (mm)."""

    pivot_r: float
    lever_len: float
    rocker_offset: float
    wheel_radius: float
    wheelbase: float


def arm_geometry(p: Params, size: str) -> ArmGeometry:
    """Build the arm geometry for a size from bot_params."""
    s = p["sizes"][size]
    return ArmGeometry(
        pivot_r=s["lever"]["pivot_radius_mm"],
        lever_len=s["lever"]["length_mm"],
        rocker_offset=s["rocker"]["axle_offset_mm"],
        wheel_radius=s["wheel_dia_mm"] / 2,
        wheelbase=s["wheelbase_mm"],
    )


def wheel_centre_radius(g: ArmGeometry, theta: float) -> float:
    """Radius of the wheel axles for lever angle theta (rad)."""
    return g.pivot_r + g.lever_len * math.sin(theta) + g.rocker_offset


def contact_radius(g: ArmGeometry, theta: float) -> float:
    """Radius of the wheel-to-wall contact line for lever angle theta."""
    return wheel_centre_radius(g, theta) + g.wheel_radius


def theta_for_pipe(g: ArmGeometry, pipe_id_mm: float) -> float:
    """Lever angle (rad) that puts the wheels on a pipe wall of the given ID."""
    s = (pipe_id_mm / 2 - g.wheel_radius - g.rocker_offset - g.pivot_r) / g.lever_len
    if not -1.0 <= s <= 1.0:
        raise ValueError(f"pipe {pipe_id_mm} mm is outside the lever reach")
    return math.asin(s)


def stroke(p: Params, size: str) -> tuple[float, float]:
    """Lever stroke (theta_lo, theta_hi) in rad, both ends with working margin.

    Retracted: the wheels clear the minimum pipe by the insertion clearance (radial), so
    the robot slides in. Extended: the wheels reach the maximum pipe with the rocker slide
    fully compressed (2 x compliance travel), so the force servo can reach any grip target
    there. Without these margins the rated pipe range is only touched, not gripped.
    """
    g = arm_geometry(p, size)
    s, c = p["sizes"][size], p["common"]
    lo = theta_for_pipe(g, s["pipe_id_min_mm"] - 2 * c["insertion_clearance_mm"])
    hi = theta_for_pipe(g, s["pipe_id_max_mm"] + 4 * c["arm_compliance_travel_mm"])
    return lo, hi


def pipe_for_theta(g: ArmGeometry, theta: float) -> float:
    """Pipe ID (mm) gripped at lever angle theta."""
    return 2 * contact_radius(g, theta)


def tip(g: ArmGeometry, theta: float) -> tuple[float, float]:
    """Rocker pivot T (z, r)."""
    return g.lever_len * math.cos(theta), g.pivot_r + g.lever_len * math.sin(theta)


def lever_torque_nm(g: ArmGeometry, theta: float, radial_n: float, traction_n: float) -> float:
    """Torque at the lever pivot from the wall reaction.

    radial_n pushes the lever down (toward smaller theta) with arm L*cos(theta).
    traction_n is the axial wheel force (+ = robot driven toward the nose); it acts
    at the contact radius and also folds the lever. Both are carried by the worm.
    """
    arm_radial = g.lever_len * math.cos(theta) * 1e-3
    arm_axial = (contact_radius(g, theta) - g.pivot_r) * 1e-3
    return radial_n * arm_radial + traction_n * arm_axial


def wheel_z_span(g: ArmGeometry, theta: float) -> tuple[float, float]:
    """Axial extent (z_min, z_max) of the two wheels on the rocker."""
    tz, _ = tip(g, theta)
    half = g.wheelbase / 2 + g.wheel_radius
    return tz - half, tz + half


def theta_samples(theta_min: float, theta_max: float, n: int = 101) -> list[float]:
    """Evenly spaced lever angles over the stroke (rad)."""
    return [theta_min + (theta_max - theta_min) * i / (n - 1) for i in range(n)]


def diameter_lut(g: ArmGeometry, theta_min: float, theta_max: float, n: int = 33) -> list[tuple[float, float]]:
    """(pipe ID mm, lever angle deg from retracted) lookup table for the firmware."""
    return [(pipe_for_theta(g, th), math.degrees(th - theta_min)) for th in theta_samples(theta_min, theta_max, n)]
