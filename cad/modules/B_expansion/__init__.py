"""Module B - radial expansion: worm-sector levers driven by one central bevel gear.

Frames
  global      gearbox_bulkhead, bevel_crown, expansion motor, planetary
  arm-local   expansion_worm, bevel_pinion, worm_bearing_block (placed with Rz(arm angle))
  lever       lever_plate, worm_sector, pivot_shaft (origin at pivot P, lever along +Z;
              placed with Ry(theta), +X r_pivot, Rz(arm angle))

Part-name mapping to the original spec (R3.1): cone_nut -> bevel_crown + worm_sector,
arm_carriage -> lever_plate (+ rocker in C_wheel), rail_block -> worm_bearing_block,
leadscrew_bearing_block -> gearbox_bulkhead, return_spring_retainer ->
compliance_spring_retainer (C_wheel).
"""

from __future__ import annotations

import math

import cadquery as cq

from cad.lib import primitives as P
from cad.lib.params import Size
from cad.lib.part import COLORS, Part

MOD = "B_expansion"


def shaft_r(sz: Size) -> float:
    """Pivot shaft radius."""
    return sz.cad("pivot_shaft_dia_mm") / 2


def pivot_boss_r(sz: Size) -> float:
    """Lever / sector boss radius around the pivot."""
    return shaft_r(sz) + 3.0


def housing_front_z(sz: Size) -> float:
    """Front face of the gearbox housing (just behind the pivot bosses)."""
    return -(pivot_boss_r(sz) + 0.5)


def sector_span_deg(sz: Size) -> tuple[float, float]:
    """Toothed arc of the sector in the lever frame (deg from +X toward +Z)."""
    a0 = math.degrees(sz.theta_min) - 8 - 90
    a1 = math.degrees(sz.theta_max) + 8 - 90
    return a0, a1


def d_hole(r: float, y0: float, y1: float, x: float = 0.0, z: float = 0.0) -> cq.Workplane:
    """D-profile hole along Y (flat at 0.85 r)."""
    c = P.cyl_between(r, (x, y0, z), (x, y1, z))
    flat = P.box(x + 0.7 * r, x + r + 1, y0 - 1, y1 + 1, z - r - 1, z + r + 1)
    return c.cut(flat)


def worm_od(sz: Size) -> float:
    """Expansion worm outside diameter."""
    return sz.worm_d + 2 * sz.s["expansion"]["sector_module_mm"]


def gearbox_bulkhead(sz: Size) -> Part:
    """Gearbox housing + pivot lugs: holds the crown, 3 pinion/worm shafts, flooded with grease."""
    zf = housing_front_z(sz)
    z0 = sz.z_gearbox0
    w = P.disc(sz.rb, z0, zf)
    e = sz.s["expansion"]
    m = e["sector_module_mm"]
    rp, rc = sz.bevel
    mb = sz.cad("bevel_module_mm")
    bp = 3 * mb
    wl = sz.cad("worm_length_mm")
    shaft_d = max(3.0, sz.worm_d * 0.3)
    # central crown cavity
    crown_back = sz.z_worm - rp - 3.6 * mb
    w = w.cut(P.disc(rc + bp / 2 + 1.5, crown_back - 0.5, sz.z_worm + rp + mb + 0.5))
    # planetary output bore
    w = w.cut(P.disc(sz.motor("expansion")["shaft_mm"] + 2, z0 - 1, crown_back))
    slot_parts = []
    lug_parts = []
    a0, a1 = sector_span_deg(sz)
    dth = a1 - a0
    rs = sz.sector_r + m + 0.6
    for ang in sz.arm_angles():
        cut = cq.Workplane("XY")
        # worm + pinion channel (radial)
        cut = P.cyl_between(worm_od(sz) / 2 + 0.6, (sz.r_pivot - wl / 2 - 1, 0, sz.z_worm), (sz.r_pivot + wl / 2 + 1, 0, sz.z_worm))
        cut = cut.union(P.cyl_between(rp + mb + 0.8, (rc - bp / 2 - 1, 0, sz.z_worm), (rc + bp / 2 + 1, 0, sz.z_worm)))
        cut = cut.union(P.cyl_between(shaft_d / 2 + 2.0, (0, 0, sz.z_worm), (sz.r_pivot + wl / 2 + 4, 0, sz.z_worm)))
        # sector sweep slot (centre layer)
        pie = [(0.0, 0.0)] + [
            (rs * math.cos(math.radians(a)), rs * math.sin(math.radians(a)))
            for a in [a0 - dth + i * (2 * dth + 4) / 40 for i in range(41)]
        ]
        fw = e["sector_face_mm"] / 2 + 0.6
        cut = cut.union(P.xz_prism(pie, -fw, fw).translate((sz.r_pivot, 0, 0)))
        slot_parts.append(cut.rotate((0, 0, 0), (0, 0, 1), ang))
        # pivot lugs (outside the lever plates)
        rl = pivot_boss_r(sz) + 1.0
        for y0, y1 in sz.lug_y:
            outline = P.hull2d(P.circle_pts(sz.r_pivot, 0, rl, 32) + [(sz.r_pivot - rl, zf - 1), (sz.r_pivot + rl, zf - 1)])
            lug = P.xz_prism(outline, y0, y1)
            lug = lug.cut(P.cyl_between(shaft_r(sz) + 1.0, (sz.r_pivot, y0 - 1, 0), (sz.r_pivot, y1 + 1, 0)))
            lug_parts.append(lug.rotate((0, 0, 0), (0, 0, 1), ang))
    for c in slot_parts:
        w = w.cut(c)
    # lightening pockets between the arms (flooded with grease in service)
    r1 = rc + bp / 2 + 3.0
    r2 = sz.rb - sz.wall - 1.0
    rib = worm_od(sz) / 2 + 2 * sz.wall
    if r2 - r1 > 6:
        pz0, pz1 = z0 + sz.wall + 2.5, zf - sz.wall
        for ang in sz.arm_angles():
            mid = math.radians(ang + 60)
            half = math.radians(60) - math.atan2(rib, (r1 + r2) / 2)
            pts = [((r1) * math.cos(mid + half * (2 * i / 20 - 1)), (r1) * math.sin(mid + half * (2 * i / 20 - 1))) for i in range(21)]
            pts += [((r2) * math.cos(mid + half * (1 - 2 * i / 20)), (r2) * math.sin(mid + half * (1 - 2 * i / 20))) for i in range(21)]
            pocket = cq.Workplane("XY").polyline(pts).close().extrude(pz1 - pz0).translate((0, 0, pz0))
            w = w.cut(pocket)
    for lug in lug_parts:
        w = w.union(lug)
    w = w.cut(P.tube(sz.rb - sz.wall + 1.2, sz.rb - sz.wall - 1, z0 + 1, z0 + 2.5))
    f = sz.cad("fastener")
    return Part(
        "gearbox_bulkhead", MOD, w, "6061-T6",
        fasteners={f"{f}x8 SHCS (radial)": 6, f"{f}x10 SHCS (spine)": 4, "oil-impregnated bronze bushing (pivot)": 6},
        color=COLORS["alu"],
        note="Flooded gearbox (marine grease). Sector slots are open by design; dry pods are separately sealed.",
    )


def bevel_crown(sz: Size) -> Part:
    """Central bevel/face gear (replaces the spec's cone nut): drives all three pinions."""
    rp, rc = sz.bevel
    mb = sz.cad("bevel_module_mm")
    bp = 3 * mb
    zt = sz.z_worm - rp
    w = P.disc(rc + bp / 2, zt - 3.6 * mb, zt)
    # face teeth as radial ribs
    n = int(round(sz.cad("bevel_pinion_teeth") * sz.s["expansion"]["bevel_ratio"]))
    for k in range(n):
        rib = P.box(rc - bp / 2, rc + bp / 2, -mb * 0.6, mb * 0.6, zt, zt + mb).rotate((0, 0, 0), (0, 0, 1), 360 * k / n)
        w = w.union(rib)
    w = w.cut(P.disc(sz.motor("expansion")["shaft_mm"] / 2, zt - 4 * mb - 1, zt + 2))
    return Part("bevel_crown", MOD, w, "42CrMo4", color=COLORS["steel"], fasteners={"M3 set screw": 2},
                meta={"teeth": n})


def bevel_pinion(sz: Size) -> Part:
    """Pinion on the inner end of each worm shaft (arm-local frame, axis along X)."""
    rp, rc = sz.bevel
    mb = sz.cad("bevel_module_mm")
    bp = 3 * mb
    shaft_d = max(3.0, sz.worm_d * 0.3)
    w = P.cyl_between(rp + mb, (rc - bp / 2, 0, sz.z_worm), (rc + bp / 2, 0, sz.z_worm))
    w = w.cut(P.cyl_between(shaft_d / 2, (rc - bp, 0, sz.z_worm), (rc + bp, 0, sz.z_worm)))
    return Part("bevel_pinion", MOD, w, "42CrMo4", qty=3, color=COLORS["steel"])


def expansion_worm(sz: Size) -> Part:
    """Self-locking worm + shaft (arm-local frame, radial axis)."""
    e = sz.s["expansion"]
    wl = sz.cad("worm_length_mm")
    rp, rc = sz.bevel
    bp = 3 * sz.cad("bevel_module_mm")
    shaft_d = max(3.0, sz.worm_d * 0.3)
    x_in, x_out = rc - bp / 2 - 0.5, sz.r_pivot + wl / 2 + 3.5
    w = P.worm(sz.worm_d, e["sector_module_mm"], wl, shaft_d, 0.1)
    w = w.rotate((0, 0, 0), (0, 1, 0), 90).translate((sz.r_pivot, 0, sz.z_worm))
    w = w.union(P.cyl_between(shaft_d / 2, (x_in, 0, sz.z_worm), (x_out, 0, sz.z_worm)))
    return Part("expansion_worm", MOD, w, "42CrMo4", qty=3, color=COLORS["steel"],
                note=f"Single-start, lead angle {math.degrees(math.atan(e['sector_module_mm'] / sz.worm_d)):.2f} deg: self-locking.")


def worm_bearing_block(sz: Size) -> Part:
    """Flanged bronze bushing supporting each worm shaft (2 per shaft)."""
    shaft_d = max(3.0, sz.worm_d * 0.3)
    w = P.tube(shaft_d / 2 + 1.5, shaft_d / 2, 0, 4).union(P.tube(shaft_d / 2 + 3, shaft_d / 2, 0, 1))
    return Part("worm_bearing_block", MOD, w, "CuSn12", qty=6, color=COLORS["bronze"])


def worm_sector(sz: Size) -> Part:
    """Bronze gear sector keyed to the pivot shaft (lever frame)."""
    e = sz.s["expansion"]
    a0, a1 = sector_span_deg(sz)
    outline = P.gear_outline(sz.sector_r, e["sector_module_mm"], a0, a1, 0)
    fw = e["sector_face_mm"] / 2
    w = P.xz_prism(outline, -fw, fw)
    m = e["sector_module_mm"]
    r_in, r_out = pivot_boss_r(sz) + 2.0, sz.sector_r - 1.25 * m - 3 * m
    if r_out - r_in > 6:
        rm, rr = (r_in + r_out) / 2, (r_out - r_in) / 2 * 0.8
        span = a1 - a0
        n = max(1, int(span / math.degrees(2.6 * rr / rm)))
        for k in range(n):
            a = math.radians(a0 + span * (k + 0.5) / n)
            w = w.cut(P.cyl_between(rr, (rm * math.cos(a), -fw - 1, rm * math.sin(a)), (rm * math.cos(a), fw + 1, rm * math.sin(a))))
    (_, ln1), (lp0, _) = sz.lever_plate_y
    hub = P.cyl_between(pivot_boss_r(sz), (0, ln1 + 0.2, 0), (0, lp0 - 0.2, 0))
    w = w.union(hub)
    w = w.cut(d_hole(shaft_r(sz), ln1 - 1, lp0 + 1))
    return Part("worm_sector", MOD, w, "CuSn12", qty=3, color=COLORS["bronze"],
                note="Keyed (D-flat) on the pivot shaft; teeth hobbed to match the worm.")


def lever_plate(sz: Size) -> Part:
    """One side plate of a lever (lever frame, plate at y in [0, t]; two per arm)."""
    lv = sz.s["lever"]
    t, h = lv["plate_thickness_mm"], lv["plate_height_mm"]
    rb = pivot_boss_r(sz)
    outline = P.hull2d(P.circle_pts(0, 0, rb, 40) + P.circle_pts(0, sz.lever_len, h / 2, 40))
    w = P.xz_prism(outline, 0, t)
    w = w.cut(d_hole(shaft_r(sz), -1, t + 1))
    w = w.cut(P.cyl_between(sz.cad("pin_dia_mm") / 2, (0, -1, sz.lever_len), (0, t + 1, sz.lever_len)))
    # lightening slot in the middle third
    if h > 12:
        slot = P.rounded_slot_outline((0, sz.lever_len * 0.3), (0, sz.lever_len * 0.75), h * 0.18)
        w = w.cut(P.xz_prism(slot, -1, t + 1))
    return Part("lever_plate", MOD, w, "6061-T6", qty=6, color=COLORS["alu"],
                note="Waterjet 6061-T6 plate; D-hole keys it to the pivot shaft.")


def pivot_shaft(sz: Size) -> Part:
    """Keyed pivot shaft through lugs, lever plates and sector (lever frame, along Y)."""
    (l0, _), (_, l1) = sz.lug_y
    w = P.cyl_between(shaft_r(sz), (0, l0 - 1, 0), (0, l1 + 1, 0))
    w = w.cut(P.box(0.7 * shaft_r(sz), shaft_r(sz) + 1, l0 + 0.5, l1 - 0.5, -shaft_r(sz) - 1, shaft_r(sz) + 1))
    w = w.cut(P.cyl_between(0.55 * shaft_r(sz), (0, l0 - 2, 0), (0, l1 + 2, 0)))  # hollow: tau 16Td/(pi(d^4-di^4))
    return Part("pivot_shaft", MOD, w, "42CrMo4", qty=3, color=COLORS["steel"], fasteners={"E-clip": 2})


def expansion_motor(sz: Size) -> Part:
    """Purchased expansion stepper."""
    mt = sz.motor("expansion")
    w = P.stepper(mt["frame_mm"], mt["length_mm"], mt["shaft_mm"], 8).translate((0, 0, sz.z_motor_face))
    return Part(f"expansion_motor_{sz.s['expansion']['motor']}", "purchased", w, "steel/copper", purchased=True,
                mass_kg=mt["mass_kg"], cost=sz.p["catalog"]["motor_cost"][sz.s["expansion"]["motor"]],
                color=COLORS["motor"])


def planetary(sz: Size) -> Part:
    """Purchased inline planetary gearhead on the expansion motor."""
    fr = sz.motor("expansion")["frame_mm"]
    ln = sz.cad("planetary_len_mm")
    w = P.disc(fr * 0.45, sz.z_motor_face, sz.z_gearbox0)
    w = w.union(P.disc(sz.motor("expansion")["shaft_mm"] / 2 + 1, sz.z_gearbox0, sz.z_gearbox0 + 3))
    return Part(f"planetary_{sz.s['expansion']['planetary_ratio']}to1", "purchased", w, "steel", purchased=True,
                mass_kg=math.pi * (0.45 * fr) ** 2 * ln * 1e-3 * 7.85 * 0.5 / 1000,  # 50% solid steel
                cost=sz.p["catalog"]["planetary_gearhead_cost"][sz.name], color=COLORS["motor"])


def build(sz: Size) -> list[Part]:
    """All module B parts for a size."""
    return [
        gearbox_bulkhead(sz), bevel_crown(sz), bevel_pinion(sz), expansion_worm(sz), worm_bearing_block(sz),
        worm_sector(sz), lever_plate(sz), pivot_shaft(sz), expansion_motor(sz), planetary(sz),
    ]
