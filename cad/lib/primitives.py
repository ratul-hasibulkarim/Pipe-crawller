"""Parametric primitives: tubes, plates, gears, bearings, motors, belts, boards.

Gears and worms are simplified (straight trapezoid teeth, ring-grooved worm). They carry
correct pitch geometry, envelope and mass; replace them with Toolbox/vendor gear models
for manufacturing drawings (docs/solidworks_guide.md).
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence

import cadquery as cq

Pt = tuple[float, float]


def tube(ro: float, ri: float, z0: float, z1: float) -> cq.Workplane:
    """Tube along Z from z0 to z1."""
    w = cq.Workplane("XY").workplane(offset=z0).circle(ro)
    if ri > 0:
        w = w.circle(ri)
    return w.extrude(z1 - z0)


def disc(r: float, z0: float, z1: float) -> cq.Workplane:
    """Solid cylinder along Z."""
    return tube(r, 0, z0, z1)


def box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> cq.Workplane:
    """Axis-aligned box from corner coordinates."""
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl_between(r: float, a: Sequence[float], b: Sequence[float]) -> cq.Workplane:
    """Cylinder of radius r from point a to point b."""
    va, vb = cq.Vector(*a), cq.Vector(*b)
    d = vb - va
    solid = cq.Solid.makeCylinder(r, d.Length, va, d.normalized())
    return cq.Workplane("XY").add(solid)


def hull2d(points: Iterable[Pt]) -> list[Pt]:
    """Convex hull (monotone chain), counter-clockwise."""
    pts = sorted(set((round(x, 6), round(y, 6)) for x, y in points))
    if len(pts) < 3:
        return pts

    def cross(o: Pt, a: Pt, b: Pt) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[Pt] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: list[Pt] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def circle_pts(cx: float, cy: float, r: float, n: int = 48) -> list[Pt]:
    """Points on a circle."""
    return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]


def xz_prism(outline: Sequence[Pt], y0: float, y1: float) -> cq.Workplane:
    """Extrude a closed (x, z) outline along Y from y0 to y1 (arm-plane plates)."""
    w = cq.Workplane("XY").polyline(list(outline)).close().extrude(y1 - y0)
    return w.rotate((0, 0, 0), (1, 0, 0), 90).translate((0, y1, 0))


def xz_holes(w: cq.Workplane, holes: Iterable[tuple[float, float, float]], y0: float, y1: float) -> cq.Workplane:
    """Cut round holes (x, z, dia) through a Y-thick plate."""
    for x, z, d in holes:
        w = w.cut(cyl_between(d / 2, (x, y0 - 1, z), (x, y1 + 1, z)))
    return w


def rounded_slot_outline(p0: Pt, p1: Pt, r: float, n: int = 24) -> list[Pt]:
    """Stadium outline around segment p0-p1 (x, z)."""
    return hull2d(circle_pts(*p0, r, n) + circle_pts(*p1, r, n))


def hull_plate(circles: Sequence[tuple[float, float, float]], y0: float, y1: float) -> cq.Workplane:
    """Plate = convex hull of circles (x, z, r) extruded along Y."""
    pts: list[Pt] = []
    for x, z, r in circles:
        pts += circle_pts(x, z, r, 32)
    return xz_prism(hull2d(pts), y0, y1)


def gear_outline(r_pitch: float, module: float, a0: float, a1: float, hub_r: float) -> list[Pt]:
    """Toothed sector outline in a plane (deg angles), simplified trapezoid teeth."""
    ra, rd = r_pitch + module, r_pitch - 1.25 * module
    pitch_ang = math.degrees(module * math.pi / r_pitch)
    n = max(1, int((a1 - a0) / pitch_ang))
    pts: list[Pt] = [(0.0, 0.0)] if hub_r <= 0 else []
    for i in range(n):
        b = a0 + i * pitch_ang
        for frac, rr in ((0.0, rd), (0.15, rd), (0.35, ra), (0.65, ra), (0.85, rd)):
            a = math.radians(b + frac * pitch_ang)
            pts.append((rr * math.cos(a), rr * math.sin(a)))
    a = math.radians(a0 + n * pitch_ang)
    pts.append((rd * math.cos(a), rd * math.sin(a)))
    if hub_r > 0:
        for k in range(17):
            a = math.radians(a0 + n * pitch_ang + k * (360 - (a1 - a0)) / 16)
            pts.append((hub_r * math.cos(a), hub_r * math.sin(a)))
    return pts


def spur_gear(r_pitch: float, module: float, face: float, bore: float) -> cq.Workplane:
    """Full spur gear along Z (simplified teeth), face from 0 to face."""
    outline = gear_outline(r_pitch, module, 0, 360, 0)[1:]
    w = cq.Workplane("XY").polyline(outline).close().extrude(face)
    if bore > 0:
        w = w.cut(disc(bore / 2, -1, face + 1))
    return w


def worm(d_pitch: float, module: float, length: float, shaft_d: float, shaft_len: float) -> cq.Workplane:
    """Worm along Z centred on z=0 (ring grooves at the axial pitch), with shaft."""
    ro, rr = d_pitch / 2 + module, d_pitch / 2 - 1.25 * module
    w = disc(ro, -length / 2, length / 2)
    pitch = math.pi * module
    n = int(length / pitch)
    for i in range(n):
        z = -length / 2 + (i + 0.5) * pitch
        w = w.cut(tube(ro + 1, rr, z - pitch * 0.25, z + pitch * 0.25))
    return w.union(disc(shaft_d / 2, -shaft_len / 2, shaft_len / 2))


def bevel(r_pitch: float, module: float, face: float, bore: float) -> cq.Workplane:
    """Bevel gear as a frustum along +Z (large end at z=0)."""
    r1 = r_pitch + module
    r2 = max(r1 - face, bore / 2 + 1)
    w = cq.Workplane("XY").add(cq.Solid.makeCone(r1, r2, face))
    if bore > 0:
        w = w.cut(disc(bore / 2, -1, face + 1))
    return w


def bearing(id_: float, od: float, w: float) -> cq.Workplane:
    """Ball bearing ring along Z, centred."""
    return tube(od / 2, id_ / 2, -w / 2, w / 2)


def stepper(frame: float, length: float, shaft_d: float, shaft_len: float) -> cq.Workplane:
    """Stepper motor along Z: front face at z=0, body toward -Z, shaft toward +Z."""
    ch = frame * 0.12
    body = box(-frame / 2, frame / 2, -frame / 2, frame / 2, -length, 0)
    body = body.edges("|Z").chamfer(ch)
    pilot = disc(frame * 0.39, 0, 2)
    shaft = disc(shaft_d / 2, 0, shaft_len)
    return body.union(pilot).union(shaft)


def pulley(pitch_d: float, width: float, bore: float) -> cq.Workplane:
    """Build a timing pulley along Z, centred, with flanges."""
    core = disc(pitch_d / 2, -width / 2, width / 2)
    fl = disc(pitch_d / 2 + 1.2, -width / 2 - 0.8, -width / 2).union(disc(pitch_d / 2 + 1.2, width / 2, width / 2 + 0.8))
    w = core.union(fl)
    return w.cut(disc(bore / 2, -width, width)) if bore > 0 else w


def belt_loop(circles: Sequence[tuple[float, float, float]], thick: float, y0: float, y1: float) -> cq.Workplane:
    """Belt around pulleys (x, z, pitch radius) in the X-Z plane, width along Y."""
    outer: list[Pt] = []
    inner: list[Pt] = []
    for x, z, r in circles:
        outer += circle_pts(x, z, r + thick / 2, 48)
        inner += circle_pts(x, z, r - thick / 2, 48)
    return xz_prism(hull2d(outer), y0, y1).cut(xz_prism(hull2d(inner), y0 - 1, y1 + 1))


def rbox(length: float, w: float, h: float, r: float) -> cq.Workplane:
    """Centred box with rounded vertical edges."""
    b = cq.Workplane("XY").box(length, w, h)
    return b.edges("|Z").fillet(r) if r > 0 else b


def torus_ring(r_major: float, r_minor: float) -> cq.Workplane:
    """Torus in the XY plane."""
    return cq.Workplane("XY").add(cq.Solid.makeTorus(r_major, r_minor))


def safe_fillet(w: cq.Workplane, selector: str, r: float) -> cq.Workplane:
    """Fillet edges if OCC can; geometry stays valid either way."""
    try:
        out = w.edges(selector).fillet(r)
        return out if out.val().isValid() else w
    except Exception:  # noqa: BLE001 - OCC raises generic StdFail errors
        return w
