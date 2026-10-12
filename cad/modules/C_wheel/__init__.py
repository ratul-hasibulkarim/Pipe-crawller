"""Module C - wheel rocker and drive pod.

Rocker frame: origin at the rocker pivot (stub pins on the lever tip), X radial, Y
tangential (+Y = belt side), Z axial. Wheel axles at X = axle_offset, Z = +/-wheelbase/2.
The rocker stays parallel to the pipe axis; it pitches about the stub pins and slides
+/- compliance travel in its slot (spring + hall sensor = grip-force measurement).
"""

from __future__ import annotations

import math

import cadquery as cq

from cad.lib import primitives as P
from cad.lib.params import Size
from cad.lib.part import COLORS, Part

MOD = "C_wheel"


def ax(sz: Size) -> float:
    """Axle height above the rocker pivot."""
    return sz.s["rocker"]["axle_offset_mm"]


def tread_t(sz: Size) -> float:
    """Silicone tread thickness at the crown."""
    return round(0.1 * sz.s["wheel_dia_mm"], 1)


def pod_x(sz: Size) -> float:
    """Drive motor axis X (rocker frame)."""
    return ax(sz) - sz.pod_drop


def gearbox_z(sz: Size) -> tuple[float, float]:
    """Worm gearbox z span (rocker frame); motor sits behind it."""
    z1 = sz.pod_len / 2
    return z1 - sz.cad("drive_gearbox_len_mm"), z1


def output_xz(sz: Size) -> tuple[float, float]:
    """Gearbox output shaft (Y axis) position."""
    g0, g1 = gearbox_z(sz)
    return pod_x(sz) + sz.pod_half * 0.35, (g0 + g1) / 2


def slot_span(sz: Size) -> tuple[float, float]:
    """Pivot slot X span in the rocker plate (stub-pin bushing + travel + spring)."""
    bush = sz.cad("pin_dia_mm") + 2
    trav = sz.c["arm_compliance_travel_mm"]
    return -bush / 2 - trav, bush / 2 + trav + 0.8 * bush


def rocker_plate(sz: Size) -> Part:
    """Rocker side plate (qty 2 per arm), built at y in [0, t]."""
    t = sz.s["rocker"]["plate_thickness_mm"]
    b = sz.bearing
    rb = b["od"] / 2 + 2.5
    hw = sz.wheelbase / 2
    xb = pod_x(sz) - sz.pod_half
    s0, s1 = slot_span(sz)
    pts = (
        P.circle_pts(ax(sz), -hw, rb, 32)
        + P.circle_pts(ax(sz), hw, rb, 32)
        + [(xb, -sz.pod_len / 2), (xb, sz.pod_len / 2)]
        + P.circle_pts(s1 + 2.5, 0, 3.5, 16)
    )
    w = P.xz_prism(P.hull2d(pts), 0, t)
    for z in (-hw, hw):
        w = w.cut(P.cyl_between(b["od"] / 2, (ax(sz), -1, z), (ax(sz), t + 1, z)))
    pin = sz.cad("pin_dia_mm") + 2
    w = w.cut(P.xz_prism(P.rounded_slot_outline((s0 + pin / 2, 0), (s1 - pin / 2, 0), pin / 2 + 0.1), -1, t + 1))
    # lightening windows between pod and axles
    win = P.rounded_slot_outline((pod_x(sz) + sz.pod_half * 0.2, -hw * 0.55), (pod_x(sz) + sz.pod_half * 0.2, -hw * 0.25), sz.pod_half * 0.3)
    if sz.pod_half > 12:
        w = w.cut(P.xz_prism(win, -1, t + 1)).cut(P.xz_prism([(x, -z) for x, z in win], -1, t + 1))
    f = sz.cad("fastener")
    return Part(
        "rocker_plate",
        MOD,
        w,
        "6061-T6",
        qty=6,
        color=COLORS["alu"],
        bearing_bores=[(0, 1, 0)],
        fasteners={f"{f}x6 SHCS (pod)": 2},
        note="Spec 'arm_carriage': carries both axles + the drive pod.",
    )


def wheel_hub(sz: Size) -> Part:
    """Rigid wheel hub (axis along Y, centred), D-bore, keyed rim for the overmould."""
    w_w = sz.s["wheel_width_mm"]
    r_h = sz.wheel_r - tread_t(sz)
    w = P.cyl_between(r_h, (0, -w_w / 2, 0), (0, w_w / 2, 0))
    for k in range(8):  # overmould keys
        a = 2 * math.pi * k / 8
        key = P.cyl_between(1.2, (r_h * math.cos(a), -w_w / 2 + 1, r_h * math.sin(a)), (r_h * math.cos(a), w_w / 2 - 1, r_h * math.sin(a)))
        w = w.cut(key)
    d = sz.axle_d
    bore = P.cyl_between(d / 2, (0, -w_w, 0), (0, w_w, 0)).cut(P.box(0.8 * d / 2, d, -w_w - 1, w_w + 1, -d, d))
    w = w.cut(bore)
    if r_h > 3 * d:
        for k in range(5):
            a = 2 * math.pi * k / 5
            rr = (r_h + d / 2 + 2) / 2
            w = w.cut(P.cyl_between((r_h - d / 2 - 4) / 2.6, (rr * math.cos(a), -w_w, rr * math.sin(a)), (rr * math.cos(a), w_w, rr * math.sin(a))))
    return Part(
        "wheel_hub",
        MOD,
        w,
        "PA12-CF",
        qty=6,
        color=COLORS["print"],
        bearing_bores=[],
        print_rot=((1, 0, 0), 90.0),
        note="Printed flat (axis vertical); D-bore, no supports.",
    )


def tread_profile(sz: Size, n: int = 24) -> list[tuple[float, float]]:
    """Crowned tread outline (radius, y) - single-line contact (rule 3)."""
    w_w = sz.s["wheel_width_mm"]
    rc = sz.s["wheel_crown_radius_mm"]
    r_h = sz.wheel_r - tread_t(sz)
    pts = [(r_h, -w_w / 2)]
    for i in range(n + 1):
        y = -w_w / 2 + w_w * i / n
        pts.append((sz.wheel_r - (rc - math.sqrt(rc * rc - y * y)), y))
    pts.append((r_h, w_w / 2))
    return pts


def wheel_tread(sz: Size) -> Part:
    """Silicone 60A crowned tread, cast in wheel_crown_mold (axis along Y)."""
    pts = tread_profile(sz)
    w = cq.Workplane("XY").polyline(pts).close().revolve(360, (0, 0, 0), (0, 1, 0))
    return Part(
        "wheel_tread",
        MOD,
        w,
        "silicone-60A",
        qty=6,
        color=COLORS["rubber"],
        note=f"Crown radius {sz.s['wheel_crown_radius_mm']} mm; mu dry {sz.c['wheel_dry_friction_coeff']}, wet {sz.c['wheel_wet_friction_coeff']}.",
    )


def axle_y(sz: Size) -> tuple[float, float]:
    """Axle y span (outer face of -Y rocker plate to outer face of +Y plate)."""
    (n0, _), (_, p1) = sz.rocker_plate_y
    return n0, p1


def wheel_axle(sz: Size) -> Part:
    """Hardened axle with bearing shoulders and D-flat (axis along Y, at origin)."""
    y0, y1 = axle_y(sz)
    d = sz.axle_d
    w = P.cyl_between(d / 2, (0, y0, 0), (0, y1, 0))
    w = w.union(P.cyl_between(d / 2 + 0.75, (0, -sz.s["wheel_width_mm"] / 2 - 0.5, 0), (0, -sz.s["wheel_width_mm"] / 2, 0)))
    return Part("wheel_axle", MOD, w, "42CrMo4", qty=6, color=COLORS["steel"], fasteners={"E-clip": 2})


def drive_motor_mount(sz: Size) -> Part:
    """Adapter plate between drive motor face and worm gearbox, slotted for belt tension."""
    fr = sz.motor("drive")["frame_mm"]
    g0, _ = gearbox_z(sz)
    t = 3.0
    w = P.box(-fr / 2, fr / 2, -fr / 2, fr / 2, 0, t).translate((pod_x(sz), 0, g0 - t))
    w = w.cut(P.disc(fr * 0.4 + 0.2, g0 - t - 1, g0 + 1).translate((pod_x(sz), 0, 0)))
    for sx in (-1, 1):
        slot = P.box(-1.0, 1.0, -2.5, 2.5, -1, t + 1).translate((pod_x(sz) + sx * fr * 0.35, fr * 0.35, g0 - t))
        w = w.cut(slot)
    return Part("drive_motor_mount", MOD, w, "6061-T6", qty=3, color=COLORS["alu"], fasteners={"M2x6 SHCS" if fr < 30 else "M3x8 SHCS": 4})


def drive_motor(sz: Size) -> Part:
    """Purchased drive stepper (rocker frame)."""
    mt = sz.motor("drive")
    g0, _ = gearbox_z(sz)
    face = g0 - 3.0
    w = P.stepper(mt["frame_mm"], min(mt["length_mm"], face + sz.pod_len / 2), mt["shaft_mm"], 3).translate((pod_x(sz), 0, face))
    return Part(
        f"drive_stepper_{sz.s['drive']['motor']}",
        "purchased",
        w,
        "steel/copper",
        qty=3,
        purchased=True,
        mass_kg=mt["mass_kg"],
        cost=sz.p["catalog"]["motor_cost"][sz.s["drive"]["motor"]],
        color=COLORS["motor"],
    )


def drive_gearbox(sz: Size) -> Part:
    """Purchased self-locking worm gearbox module, output shaft along +Y."""
    g0, g1 = gearbox_z(sz)
    h = sz.pod_half
    w = P.box(pod_x(sz) - h, pod_x(sz) + h, -h, h, g0, g1)
    w = w.edges("|Y").fillet(min(2.0, h * 0.2))
    xo, zo = output_xz(sz)
    w = w.union(P.cyl_between(sz.axle_d / 2, (xo, 0, zo), (xo, sz.belt_y[1], zo)))
    return Part(
        f"drive_worm_gearbox_{sz.s['drive']['worm_ratio']}to1",
        "purchased",
        w,
        "Zn/steel",
        qty=3,
        purchased=True,
        mass_kg=0.6 * w.val().Volume() * 1e-3 * 2.7 / 1000,
        cost=sz.p["catalog"]["worm_gearbox_cost"][sz.name],
        color=COLORS["motor"],
        note="Self-locking worm (lead angle 3 deg), rated torque in bot_params.",
    )


def pulleys(sz: Size) -> list[tuple[str, tuple[float, float, float]]]:
    """(name, (x, z, pitch radius)) of the three belt pulleys in the rocker frame."""
    rp = sz.cad("belt_pulley_pitch_dia_mm") / 2
    xo, zo = output_xz(sz)
    hw = sz.wheelbase / 2
    return [("rear", (ax(sz), -hw, rp)), ("front", (ax(sz), hw, rp)), ("out", (xo, zo, rp))]


def timing_pulley(sz: Size) -> Part:
    """Purchased GT2 pulley (axis Y, centred)."""
    bw = sz.s["rocker"]["belt_width_mm"]
    w = P.pulley(sz.cad("belt_pulley_pitch_dia_mm"), bw, sz.axle_d).rotate((0, 0, 0), (1, 0, 0), 90)
    return Part("timing_pulley_GT2", "purchased", w, "Al", qty=9, purchased=True, mass_kg=w.val().Volume() * 2.7e-6 * 0.7, cost=3.0, color=COLORS["alu"])


def drive_belt(sz: Size) -> Part:
    """Serpentine GT2 belt (rocker frame)."""
    y0, y1 = sz.belt_y
    w = P.belt_loop([c for _, c in pulleys(sz)], sz.cad("belt_thickness_mm"), y0, y1)
    length = sum(2 * math.pi * c[2] for _, c in pulleys(sz)) / 3 + 2 * sz.wheelbase  # approx
    return Part(
        "drive_belt_GT2",
        "purchased",
        w,
        "rubber/glass",
        qty=3,
        purchased=True,
        mass_kg=w.val().Volume() * 1.2e-6,
        cost=sz.p["catalog"]["gt2_belt_cost_per_m"] * length / 1000,
        color=COLORS["belt"],
        meta={"approx_length_mm": length},
    )


def belt_tensioner(sz: Size) -> Part:
    """Eccentric back-side idler mount on the +Y rocker plate (rocker frame)."""
    rp = sz.cad("belt_pulley_pitch_dia_mm") / 2
    th = sz.cad("belt_thickness_mm")
    ri = max(2.5, rp * 0.6)
    hw = sz.wheelbase / 2
    xo, zo = output_xz(sz)
    # tangent point on the lower run between rear pulley and output pulley
    zt = (-hw + zo) / 2
    xl = ax(sz) - rp + (xo - ax(sz)) * (zt + hw) / (zo + hw)
    xc = xl - th / 2 - ri - 0.3
    y0, y1 = sz.belt_y
    w = P.cyl_between(ri, (xc, y0, zt), (xc, y1, zt))
    w = w.union(P.cyl_between(ri * 0.55, (xc + 0.6, y1, zt), (xc + 0.6, y1 + 1.0, zt)))
    return Part("belt_tensioner", MOD, w, "POM", qty=3, color=COLORS["print"], fasteners={"M2x8 SHCS (eccentric)": 1})


def compliance_spring_retainer(sz: Size) -> Part:
    """Cap closing the top of the rocker slot; preloads the grip spring (qty 2/arm), y in [0, t]."""
    t = sz.s["rocker"]["plate_thickness_mm"]
    _, s1 = slot_span(sz)
    w = P.xz_prism(P.circle_pts(s1 + 2.5, 0, 3.0, 24), 0, t)
    w = w.cut(P.cyl_between(1.1, (s1 + 2.5, -1, 0), (s1 + 2.5, t + 1, 0)))
    return Part(
        "compliance_spring_retainer",
        MOD,
        w,
        "SS316",
        qty=6,
        color=COLORS["steel"],
        note="Spec 'return_spring_retainer'. Hall sensor (DRV5055) on the cap reads the stub-pin magnet.",
    )


def slide_bushing(sz: Size) -> Part:
    """Flanged POM bushing on the stub pin, rides in the rocker slot (y in [0, t])."""
    t = sz.s["rocker"]["plate_thickness_mm"]
    pin = sz.cad("pin_dia_mm")
    w = P.cyl_between(pin / 2 + 1, (0, 0, 0), (0, t, 0)).cut(P.cyl_between(pin / 2, (0, -1, 0), (0, t + 1, 0)))
    return Part("slide_bushing", MOD, w, "POM", qty=6, color=COLORS["print"])


def stub_pin(sz: Size) -> Part:
    """Stub pin: lever plate -> rocker slot (no centre-layer crossing). y in [0, span]."""
    (lv0, lv1), _ = sz.lever_plate_y
    (rn0, rn1), _ = sz.rocker_plate_y
    span = rn1 - lv0
    w = P.cyl_between(sz.cad("pin_dia_mm") / 2, (0, 0, 0), (0, span, 0))
    return Part("stub_pin", MOD, w, "42CrMo4", qty=6, color=COLORS["steel"], fasteners={"E-clip": 1})


def wheel_crown_mold(sz: Size) -> Part:
    """Two-part mould for casting the silicone tread around the hub (tooling, one half)."""
    m = 6.0
    r = sz.wheel_r + m
    w_w = sz.s["wheel_width_mm"]
    blk = P.box(-r, r, 0, w_w / 2 + m, -r, r)
    cav = cq.Workplane("XY").polyline(tread_profile(sz)).close().revolve(360, (0, 0, 0), (0, 1, 0))
    hub = P.cyl_between(sz.wheel_r - tread_t(sz), (0, -w_w, 0), (0, w_w, 0))
    blk = blk.cut(cav).cut(hub.intersect(P.box(-r, r, 0, w_w / 2 + m + 1, -r, r)))
    blk = blk.cut(P.cyl_between(2.0, (0, 0, r - 0.1), (0, 0, sz.wheel_r - 0.5)).translate((0, w_w / 4, 0)))  # sprue
    for sx in (-1, 1):
        blk = blk.cut(P.cyl_between(1.6, (sx * (r - 3), -1, r - 3), (sx * (r - 3), w_w / 2 + m + 1, r - 3)))
    return Part(
        "wheel_crown_mold",
        MOD,
        blk,
        "PETG",
        qty=0,
        color=COLORS["print"],
        print_rot=((1, 0, 0), -90.0),
        note="Tooling: print 2 halves, mirror one. Sprue + 2 dowel pins.",
    )


def build(sz: Size) -> list[Part]:
    """All module C parts for a size."""
    return [
        rocker_plate(sz),
        wheel_hub(sz),
        wheel_tread(sz),
        wheel_axle(sz),
        drive_motor_mount(sz),
        drive_motor(sz),
        drive_gearbox(sz),
        timing_pulley(sz),
        drive_belt(sz),
        belt_tensioner(sz),
        compliance_spring_retainer(sz),
        slide_bushing(sz),
        stub_pin(sz),
        wheel_crown_mold(sz),
    ]
