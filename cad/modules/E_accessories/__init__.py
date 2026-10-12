"""Module E - accessories: nose bumper, articulation joint, seals, range adapters, tether reel."""

from __future__ import annotations

import math

import cadquery as cq

from cad.lib import primitives as P
from cad.lib.params import Size
from cad.lib.part import COLORS, Part

MOD = "E_accessories"


def nose_bumper(sz: Size) -> Part:
    """Sacrificial silicone rim around the LED ring (protects LEDs from wall strikes)."""
    h = sz.cad("bumper_height_mm")
    z1 = sz.z_dome - sz.cad("led_ring_thickness_mm") - 1.0
    w = P.tube(sz.rb + 0.5, sz.rb - 1.5, z1 - h, z1)
    return Part(
        "nose_bumper",
        MOD,
        w,
        "silicone-60A",
        color=COLORS["rubber"],
        note="Sits in a 1.5 mm groove behind the LED ring, 0.5 mm proud: takes wall strikes without shadowing the 45 deg LEDs.",
    )


def joint_dims(sz: Size) -> dict[str, float]:
    """U-joint geometry."""
    a = sz.rb * 0.30
    return {"zc": sz.mid + sz.joint / 2, "arm": a, "pin": max(2.0, a * 0.25), "ear_t": max(2.5, a * 0.25), "ear_w": a * 0.9}


def articulation_yoke(sz: Size, front: bool) -> Part:
    """U-joint yoke: rear yoke on the mid-chassis bulkhead (ears +/-Y), front on the pod (ears +/-X)."""
    j = joint_dims(sz)
    zc = j["zc"]
    base_r = sz.rb * 0.55
    if front:
        base = P.disc(base_r, sz.z_front0 - 2.5, sz.z_front0)
    else:
        base = P.disc(base_r, sz.mid, sz.mid + 2.5)
    d_ear = j["arm"] + 0.6
    w = base
    for s in (-1, 1):
        zb = sz.z_front0 - 2.5 if front else sz.mid + 2.5
        outline = P.hull2d(P.circle_pts(0, zc, j["ear_w"] / 2, 24) + [(-j["ear_w"] / 2, zb), (j["ear_w"] / 2, zb)])
        ear = P.xz_prism(outline, d_ear, d_ear + j["ear_t"]) if s > 0 else P.xz_prism(outline, -d_ear - j["ear_t"], -d_ear)
        ear = ear.cut(P.cyl_between(j["pin"] / 2, (0, -d_ear - j["ear_t"] - 1, zc), (0, d_ear + j["ear_t"] + 1, zc)))
        if front:
            ear = ear.rotate((0, 0, zc), (0, 0, zc + 1), 90)
        w = w.union(ear)
    # cable pass-through (service loop runs beside the spider)
    w = w.cut(P.box(-base_r * 0.25, base_r * 0.25, base_r * 0.45, base_r * 0.85, sz.mid - 5, sz.z_front0 + 5))
    name = "articulation_yoke_front" if front else "articulation_yoke_rear"
    return Part(name, MOD, w, "SS316", color=COLORS["steel"], fasteners={f"{sz.cad('fastener')}x8 SHCS": 4})


def articulation_spider(sz: Size) -> Part:
    """U-joint cross (spider) with centring torsion springs (springs purchased)."""
    j = joint_dims(sz)
    a, pin = j["arm"], j["pin"]
    w = P.cyl_between(pin / 2, (0, -a - j["ear_t"] - 0.6, j["zc"]), (0, a + j["ear_t"] + 0.6, j["zc"]))
    w = w.union(P.cyl_between(pin / 2, (-a - j["ear_t"] - 0.6, 0, j["zc"]), (a + j["ear_t"] + 0.6, 0, j["zc"])))
    w = w.union(P.box(-pin, pin, -pin, pin, j["zc"] - pin, j["zc"] + pin))
    return Part(
        "articulation_spider",
        MOD,
        w,
        "SS316",
        color=COLORS["steel"],
        fasteners={"centring torsion spring": 2, "E-clip": 4},
        note="Joint range +/-45 deg, spring-centred.",
    )


def joint_boot(sz: Size) -> Part:
    """EPDM bellows over the joint (keeps grit out; cables run inside)."""
    ro = sz.rb * 0.85
    w = P.tube(ro, ro - 1.0, sz.mid, sz.z_front0)
    n = max(3, int(sz.joint / 6))
    for k in range(n):
        z = sz.mid + (k + 0.5) * sz.joint / n
        w = w.union(P.tube(ro + 1.5, ro - 1.0, z - 0.75, z + 0.75))
    return Part("joint_boot", MOD, w, "EPDM", color=COLORS["boot"], fasteners={"hose clamp": 2})


def service_hatch_gasket(sz: Size) -> Part:
    """O-ring for every pod joint (qty below); cross-section scales with size."""
    cs = {"S": 1.5, "M": 2.0, "L": 2.5}[sz.name]
    r = sz.rb - sz.wall + 0.3
    w = P.torus_ring(r, cs / 2)
    return Part(
        "service_hatch_gasket",
        MOD,
        w,
        "EPDM",
        qty=6,
        color=COLORS["rubber"],
        note=f"O-ring ID {2 * (r - cs / 2):.1f} x {cs} mm (cap, shell/gearbox, spine, front shell, dome, gland).",
    )


def pipe_size_adapter(sz: Size) -> Part:
    """Clip-on silicone tread sleeve: +2*t on the pipe-ID range top end (qty 6 per kit)."""
    t = sz.cad("tread_sleeve_mm")
    w_w = sz.s["wheel_width_mm"]
    w = P.cyl_between(sz.wheel_r + t, (0, -w_w / 2, 0), (0, w_w / 2, 0)).cut(P.cyl_between(sz.wheel_r - 0.5, (0, -w_w, 0), (0, w_w, 0)))
    lip = P.cyl_between(sz.wheel_r + t, (0, -w_w / 2 - 1.2, 0), (0, -w_w / 2, 0)).cut(P.cyl_between(sz.wheel_r - 1.5, (0, -w_w, 0), (0, w_w, 0)))
    return Part(
        "pipe_size_adapter",
        MOD,
        w.union(lip),
        "silicone-60A",
        qty=0,
        color=COLORS["rubber"],
        note=f"Range kit: extends max pipe ID by {2 * t} mm (to {sz.s['pipe_id_max_mm'] + 2 * t} mm). 6 per kit.",
    )


def tether_capacity_m(p: dict) -> float:
    """Tether length (m) that fits on the reel."""
    tr = p["tether_reel"]
    od = p["common"]["tether_od_mm"]
    area = (tr["flange_dia_mm"] / 2 - 3) ** 2 - (tr["core_dia_mm"] / 2) ** 2
    vol = math.pi * area * tr["width_mm"] * tr["packing_factor"]
    return vol / (math.pi * (od / 2) ** 2) / 1000


def tether_reel(p: dict) -> list[Part]:
    """Operator-side tether reel: spool, side frames, rewind motor, slip-ring housing (common to all sizes)."""
    tr = p["tether_reel"]
    rc, rf, wd, ft = tr["core_dia_mm"] / 2, tr["flange_dia_mm"] / 2, tr["width_mm"], tr["frame_thickness_mm"]
    spool = P.cyl_between(rc, (0, -wd / 2, 0), (0, wd / 2, 0))
    for s in (-1, 1):
        spool = spool.union(P.cyl_between(rf, (0, s * wd / 2, 0), (0, s * (wd / 2 + 4), 0)))
    spool = spool.cut(P.cyl_between(rc - 6, (0, -wd, 0), (0, wd, 0)))
    for k in range(6):
        a = 2 * math.pi * k / 6
        spool = spool.cut(P.cyl_between(rf * 0.18, (rf * 0.6 * math.cos(a), -wd, rf * 0.6 * math.sin(a)), (rf * 0.6 * math.cos(a), wd, rf * 0.6 * math.sin(a))))
    frame = cq.Workplane("XY")
    for s in (-1, 1):
        y0 = s * (wd / 2 + 8)
        plate = P.xz_prism(P.hull2d(P.circle_pts(0, 0, 30, 24) + [(-rf - 20, -rf - 30), (rf + 20, -rf - 30)]), min(y0, y0 + s * ft), max(y0, y0 + s * ft))
        frame = plate if s < 0 else frame.union(plate)
    frame = frame.union(P.box(-rf - 20, rf + 20, -wd / 2 - 8 - ft, wd / 2 + 8 + ft, -rf - 30 - ft, -rf - 30))
    axle = P.cyl_between(6, (0, -wd / 2 - 20, 0), (0, wd / 2 + 20, 0))
    motor = P.stepper(42.3, 48, 5, 10).rotate((0, 0, 0), (1, 0, 0), -90).translate((0, -wd / 2 - 8 - ft - 1, -rf + 10))
    slip = P.cyl_between(19, (0, wd / 2 + 8 + ft, 0), (0, wd / 2 + 8 + ft + 40, 0))
    cap = tether_capacity_m(p)
    return [
        Part("tether_reel_spool", MOD, spool, "PA12-CF", color=COLORS["print"], note=f"Capacity {cap:.0f} m of {p['common']['tether_od_mm']} mm tether."),
        Part("tether_reel_frame", MOD, frame, "6061-T6", color=COLORS["alu"], fasteners={"M4x12 SHCS": 12}),
        Part("tether_reel_axle", MOD, axle, "SS316", color=COLORS["steel"]),
        Part(
            "tether_reel_motor_NEMA17",
            "purchased",
            motor,
            "steel",
            purchased=True,
            mass_kg=0.36,
            cost=12,
            color=COLORS["motor"],
            note="Rewind motor through a spring-loaded slip clutch (limits pull on the tether to < tether rating).",
        ),
        Part(
            "tether_reel_slip_ring",
            "purchased",
            slip,
            "mixed",
            purchased=True,
            mass_kg=0.25,
            cost=120,
            color=COLORS["steel"],
            note="Power + Gigabit Ethernet slip ring (e.g. 2 power + 8 signal, 300 rpm).",
        ),
    ]


def build(sz: Size) -> list[Part]:
    """All per-robot module E parts."""
    return [
        nose_bumper(sz),
        articulation_yoke(sz, False),
        articulation_yoke(sz, True),
        articulation_spider(sz),
        joint_boot(sz),
        service_hatch_gasket(sz),
        pipe_size_adapter(sz),
    ]
