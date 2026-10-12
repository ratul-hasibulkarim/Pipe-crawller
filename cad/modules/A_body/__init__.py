"""Module A - central body: pods, spine, electronics tray, battery caddy, strain relief.

All parts are built in the global frame (they do not move).
"""

from __future__ import annotations

import math

import cadquery as cq

from cad.lib import primitives as P
from cad.lib.params import Size
from cad.lib.part import COLORS, Part
from cad.modules.D_gimbal import gimbal_rear_z

MOD = "A_body"


def body_rear_shell(sz: Size) -> Part:
    """Rear pod tube between the rear cap and the gearbox housing, with motor flange ring."""
    z0 = -sz.rear + sz.cap_len
    z1 = sz.z_gearbox0
    w = P.tube(sz.rb, sz.rb - sz.wall, z0, z1)
    # internal flange that locates the planetary gearhead face (motor hangs off it)
    fr = sz.motor("expansion")["frame_mm"]
    ring_ri = fr * 0.45 + 0.3  # planetary passes through; motor face bolts to this ring
    w = w.union(P.tube(sz.rb - sz.wall + 0.01, ring_ri, sz.z_motor_face, sz.z_motor_face + sz.wall))
    # O-ring grooves at both mating ends
    for z in (z0 + 2, z1 - 3):
        w = w.cut(P.tube(sz.rb - sz.wall + 1.2, sz.rb - sz.wall - 1, z, z + 1.5))
    f = sz.cad("fastener")
    return Part(
        "body_rear_shell", MOD, w, "6061-T6",
        fasteners={f"{f}x8 SHCS (radial)": 12},
        color=COLORS["shell"],
        note="Machined 6061 tube, hard-anodised. Bore holds the planetary flange ring.",
    )


def tether_strain_relief(sz: Size) -> Part:
    """Rear cap with cable gland boss, strain-relief cone and rear ToF window."""
    z0, z1 = -sz.rear, -sz.rear + sz.cap_len
    t_od = sz.c["tether_od_mm"]
    w = P.disc(sz.rb, z0, z1)
    w = w.cut(P.disc(sz.rb - sz.wall, z0 + sz.wall, z1 + 1))  # cup
    boss_r = t_od / 2 + 4
    cone_len = 6 * t_od
    cone = cq.Workplane("XY").add(cq.Solid.makeCone(boss_r, t_od / 2 + 1.2, cone_len)).mirror("XY").translate((0, 0, z0))
    w = w.union(cone)
    w = w.cut(P.disc(t_od / 2 + 0.3, z0 - cone_len - 1, z1 + 1))
    # rear ToF window (VL53L1X) offset from the gland
    off = (sz.rb - sz.wall) * 0.6
    w = w.cut(P.box(off - 4, off + 4, -3, 3, z0 - 1, z0 + sz.wall + 0.1))
    f = sz.cad("fastener")
    return Part(
        "tether_strain_relief", MOD, w, "6061-T6",
        fasteners={f"{f}x6 SHCS": 6, "split collet clamp (Kevlar strength member)": 1},
        color=COLORS["alu"],
        note="Tether load path: Kevlar member clamps in the collet to the cap, never to the PCB. Pull-test 200 N.",
    )


def body_mid_chassis(sz: Size) -> Part:
    """Spine tube (wire/battery conduit) + front bulkhead with the joint clevis seat."""
    lay = sz.s["layout"]
    sro = lay["spine_dia_mm"] / 2
    sri = sro - lay["spine_wall_mm"]
    z_start = -(sz.cad("pivot_shaft_dia_mm") / 2 + 3.5)  # = gearbox housing front face
    # the gear sectors swing inward behind z=0 at full extension: neck the spine there
    r_neck = min(sro, sz.r_pivot - sz.sector_r - sz.s["expansion"]["sector_module_mm"] - 1.0)
    if r_neck < sro:
        z_flare = 2.0 + (sro - r_neck)
        w = P.tube(r_neck, min(sri, r_neck - 2.0), z_start, 2.0)
        cone = cq.Workplane("XY").add(cq.Solid.makeCone(r_neck, sro, z_flare - 2.0)).translate((0, 0, 2.0))
        w = w.union(cone).union(P.tube(sro, sri, z_flare, sz.mid - sz.bh))
        w = w.cut(P.disc(min(sri, r_neck - 2.0), z_start - 1, z_flare + 0.1))
        w = w.cut(cq.Workplane("XY").add(cq.Solid.makeCone(r_neck - 2.0, sri, z_flare - 2.0)).translate((0, 0, 2.0)))
    else:
        w = P.tube(sro, sri, z_start, sz.mid - sz.bh)
    w = w.union(P.disc(sz.rb, sz.mid - sz.bh, sz.mid))
    w = w.cut(P.disc(min(sri, sz.rb * 0.25), sz.mid - sz.bh - 1, sz.mid + 1))  # wire pass-through
    rh = (sz.rb - sro) * 0.3
    if rh > 3:
        for k in range(6):
            a = math.radians(30 + 60 * k)
            rr = (sz.rb + sro) / 2
            w = w.cut(P.disc(rh, sz.mid - sz.bh - 1, sz.mid + 1).translate((rr * math.cos(a), rr * math.sin(a), 0)))
    return Part(
        "body_mid_chassis", MOD, w, "6061-T6",
        fasteners={f"{sz.cad('fastener')}x10 SHCS": 8},
        color=COLORS["alu"],
        note="Spine carries all inter-pod wiring (no cable crosses an arm joint).",
    )


def body_front_shell(sz: Size) -> Part:
    """Front pod: closed rear bulkhead, tube up to the LED ring seat."""
    z0 = sz.z_front0
    z1 = sz.z_dome - sz.cad("led_ring_thickness_mm")
    w = P.tube(sz.rb, sz.rb - sz.wall, z0, z1).union(P.disc(sz.rb, z0, z0 + sz.bh))
    w = w.cut(P.disc(sz.rb * 0.22, z0 - 1, z0 + sz.bh + 1))  # cable gland from the joint
    w = w.cut(P.tube(sz.rb - sz.wall + 1.2, sz.rb - sz.wall - 1, z1 - 3, z1 - 1.5))
    h = sz.cad("bumper_height_mm")
    w = w.cut(P.tube(sz.rb + 1, sz.rb - 1.5, z1 - 1.0 - h, z1 - 1.0))  # bumper groove
    return Part(
        "body_front_shell", MOD, w, "6061-T6",
        fasteners={f"{sz.cad('fastener')}x8 SHCS (radial)": 6},
        color=COLORS["shell"],
        note="Houses gimbal and compute board; dry compartment (IP68).",
    )


def electronics_tray(sz: Size) -> Part:
    """Snap-in carrier: compute board plane + MCU/driver plane, behind the gimbal."""
    ri = sz.rb - sz.wall - 0.5
    z0 = sz.z_front0 + sz.bh + 0.5
    z1 = gimbal_rear_z(sz) - 1.0
    board = sz.p["catalog"]["board"][sz.s["power"]["compute_board"]]
    t = sz.cad("min_wall_mm")
    # rails: two half-ring cradles joined by a spine plate
    w = P.tube(ri, ri - t, z0, z0 + t).union(P.tube(ri, ri - t, z1 - t, z1))
    cat = sz.p["catalog"]["board"]
    need = max(board["l"], cat["mcu"]["l"] + cat["buck"]["l"] + 1) + 2
    if z1 - z0 < need:
        raise ValueError(f"{sz.name}: electronics tray {z1 - z0:.1f} mm < {need:.1f} mm needed - lengthen front_pod_mm")
    plate_x = -min(board["h"] + 1.0, ri * 0.35)
    w = w.union(P.box(plate_x - t, plate_x, -ri * 0.8, ri * 0.8, z0, z1))
    w = w.intersect(P.disc(ri, z0 - 1, z1 + 1))
    return Part(
        "electronics_tray", MOD, w, "PA12-CF",
        fasteners={"M2.5x5 SHCS": 8, "M2.5 heat-set insert": 8},
        print_rot=((1, 0, 0), 0.0),
        color=COLORS["print"],
        meta={"board_x": plate_x, "z0": z0, "z1": z1},
    )


def battery_caddy(sz: Size) -> Part | None:
    """Pack holder: M = 4S1P (2x2) in the spine; L = 6S2P ring around the expansion motor; S = none (tether)."""
    cell = sz.p["catalog"]["cell_18650"]
    n_cells = sz.s["power"]["battery_series"] * sz.s["power"]["battery_parallel"]
    if n_cells <= 0:
        return None
    d, ln = cell["d"], cell["l"]
    t = 1.2
    if sz.name == "M":
        lay = sz.s["layout"]
        ri = lay["spine_dia_mm"] / 2 - lay["spine_wall_mm"] - 0.3
        z0 = sz.mid - sz.bh - ln - 2 * t - 4
        caps = P.disc(ri, z0, z0 + t).union(P.disc(ri, z0 + t + ln, z0 + 2 * t + ln))
        for sx in (-1, 1):
            for sy in (-1, 1):
                caps = caps.cut(P.disc(d / 2 + 0.2, z0 - 1, z0 + 2 * t + ln + 1).translate((sx * d / 2, sy * d / 2, 0)))
        return Part("battery_caddy", MOD, caps, "PA12-CF", print_rot=((1, 0, 0), 0.0), color=COLORS["print"],
                    meta={"cells": [(sx * d / 2, sy * d / 2, z0 + t) for sx in (-1, 1) for sy in (-1, 1)], "len": ln},
                    note="End caps only; cells wrapped in Kapton + fish paper inside the spine.")
    # L: ring of cells around the motor
    r_ring = sz.motor("expansion")["frame_mm"] / 2 * math.sqrt(2) + d / 2 + 1.5
    z0 = sz.z_motor_back + 2
    ro = sz.rb - sz.wall - 0.5
    ring = P.tube(ro, r_ring - d / 2 - t, z0, z0 + t).union(P.tube(ro, r_ring - d / 2 - t, z0 + t + ln, z0 + 2 * t + ln))
    cells = []
    for k in range(n_cells):
        a = math.radians(15 + 360 / n_cells * k)
        x, y = r_ring * math.cos(a), r_ring * math.sin(a)
        ring = ring.cut(P.disc(d / 2 + 0.2, z0 - 1, z0 + 2 * t + ln + 1).translate((x, y, 0)))
        cells.append((x, y, z0 + t))
    return Part("battery_caddy", MOD, ring, "PA12-CF", print_rot=((1, 0, 0), 0.0), color=COLORS["print"],
                meta={"cells": cells, "len": ln})


def build(sz: Size) -> list[Part]:
    """All module A parts for a size."""
    parts = [body_rear_shell(sz), tether_strain_relief(sz), body_mid_chassis(sz), body_front_shell(sz), electronics_tray(sz)]
    bc = battery_caddy(sz)
    if bc is not None:
        parts.append(bc)
    return parts
