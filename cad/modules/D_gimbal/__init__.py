"""Module D - camera gimbal, dome window, LED ring.

Front-pod frame = global. Camera centre (pan/tilt centre) = dome centre (0, 0, z_dome).
Pan about Z (continuous: ring gear on the carrier, N20/GA25 gearmotor + AS5600, slip ring
on axis). Tilt about the yoke Y axis by a servo riding on the pan carrier.

Why a dome: tilt +/-90 deg looks straight at the pipe wall; behind a flat window the view
is cut off past ~50 deg and the LEDs reflect into the lens (R3, docs/real_world_issues.md).
LEDs and ToF sit in the annulus OUTSIDE the dome so their light never crosses the window.
"""

from __future__ import annotations

import math

import cadquery as cq

from cad.lib import primitives as P
from cad.lib.params import Size
from cad.lib.part import COLORS, Part

MOD = "D_gimbal"


def cam(sz: Size) -> dict[str, float]:
    """Camera module record."""
    return sz.p["catalog"]["camera_module"]


def sweep_r(sz: Size) -> float:
    """Radius of the sphere swept by camera + cradle about the gimbal centre."""
    c = cam(sz)
    return max(math.hypot(c["w"] / 2, c["h"] / 2, c["d"] / 2), math.hypot(c["w"] / 2 + 1.2, 5.5, c["d"] / 2 + 1.2)) + 0.5


def hub_off(sz: Size) -> float:
    """Dome centre to the yoke hub front face."""
    return sweep_r(sz) + 1.0


def tilt_servo(sz: Size) -> dict[str, float]:
    """Tilt servo record."""
    return sz.p["catalog"]["servo"][sz.cad("tilt_servo")]


def pan_drive(sz: Size) -> dict[str, float]:
    """Pan gearmotor record."""
    return sz.p["catalog"]["pan_drive"][sz.cad("pan_drive")]


def carrier_r(sz: Size) -> float:
    """Outer radius of the pan carrier tube (holds the tilt servo)."""
    sv = tilt_servo(sz)
    return math.hypot(sv["l"], sv["w"]) / 2 + 3.0


def z_hub(sz: Size) -> float:
    """Yoke hub front face."""
    return sz.z_dome - hub_off(sz)


def z_ring(sz: Size) -> float:
    """Pan ring-gear plane (rear end of the carrier)."""
    return z_hub(sz) - 3 - tilt_servo(sz)["h"] - 2


def pan_motor_xy(sz: Size) -> float:
    """Radial position of the static pan gearmotor axis (body alongside the carrier tube)."""
    return carrier_r(sz) + 2.0 + pan_drive(sz)["h"] / 2


def gimbal_rear_z(sz: Size) -> float:
    """Rearmost z used by the gimbal (slip ring behind the ring gear)."""
    return z_ring(sz) - 3.5 - sz.p["catalog"]["slip_ring"]["l"] - 1


def camera_window(sz: Size) -> Part:
    """Polycarbonate hemispherical dome + base flange."""
    rd, t = sz.dome_r, sz.cad("dome_wall_mm")
    sph = cq.Workplane("XY").sphere(rd).cut(cq.Workplane("XY").sphere(rd - t))
    sph = sph.intersect(P.disc(rd + 1, 0, rd + 1))
    w = sph.union(P.tube(rd + 3, rd - t, -2, 0)).translate((0, 0, sz.z_dome))
    return Part("camera_window", MOD, w, "PC", color=COLORS["clear"],
                note="Anti-fog: hydrophilic coating + 0.5 W heater trace on the flange; replaceable, O-ring sealed.")


def led_ring_mount(sz: Size) -> Part:
    """Annulus outside the dome: 4 white + 2 IR LEDs at 45 deg, front ToF window."""
    ri, ro = sz.dome_r + 3.2, sz.rb
    t = sz.cad("led_ring_thickness_mm")
    z1 = sz.z_dome
    w = P.tube(ro, ri, z1 - t, z1)
    rm = (ri + ro) / 2
    n_led = 6
    for k in range(n_led):
        a = 2 * math.pi * k / n_led
        c = (rm * math.cos(a), rm * math.sin(a), z1)
        d = (math.cos(a) * 0.707, math.sin(a) * 0.707, -0.707)
        pocket = P.cyl_between(min(2.0, (ro - ri) / 2 - 0.3), (c[0] - d[0] * 0.5, c[1] - d[1] * 0.5, c[2] + 0.5),
                               (c[0] + d[0] * 3, c[1] + d[1] * 3, c[2] + d[2] * 3))
        w = w.cut(pocket)
    a = math.radians(30)
    w = w.cut(P.box(-1.6, 1.6, -2.5, 2.5, z1 - 2, z1 + 1).translate((rm * math.cos(a), rm * math.sin(a), 0)))
    return Part("led_ring_mount", MOD, w, "PA12-CF", color=COLORS["print"], print_rot=((1, 0, 0), 0.0),
                fasteners={"M2x6 SHCS": 4}, meta={"leds_white": 4, "leds_ir": 2})


def gimbal_pan_yoke(sz: Size) -> Part:
    """Pan carrier: tilt-servo tube + ring gear + hub + two yoke arms (pan frame = global at pan 0)."""
    c = cam(sz)
    zh = z_hub(sz)
    rc = carrier_r(sz)
    ty = 2.0
    y_in = c["w"] / 2 + 1.5 + 0.5
    hub = P.disc(rc, zh - 3, zh)
    tube_ = P.tube(rc, rc - 1.5, z_ring(sz), zh - 3)
    ring = P.spur_gear(rc + 1.0, 0.5, 3.0, 0).translate((0, 0, z_ring(sz) - 3)).cut(P.disc(rc - 1.5, z_ring(sz) - 4, z_ring(sz) + 1))
    w = hub.union(tube_).union(ring)
    aw = 6.0
    for s in (-1, 1):
        y0, y1 = (y_in, y_in + ty) if s > 0 else (-y_in - ty, -y_in)
        arm = P.xz_prism(P.hull2d(P.circle_pts(0, sz.z_dome, aw / 2, 24) + [(-aw / 2, zh - 1), (aw / 2, zh - 1)]), y0, y1)
        arm = arm.cut(P.cyl_between(1.0, (0, y0 - 1, sz.z_dome), (0, y1 + 1, sz.z_dome)))
        w = w.union(arm)
    w = w.cut(P.disc(sz.p["catalog"]["slip_ring"]["d"] / 2 * 0.5, zh - 4, zh + 1))  # wire pass
    return Part("gimbal_pan_yoke", MOD, w, "PA12-CF", color=COLORS["print"], print_rot=((1, 0, 0), 0.0),
                bearing_bores=[(0, 0, 1)], fasteners={"M2x5 SHCS": 4, "M2 heat-set insert": 4},
                note="Continuous pan: ring gear (m0.5) driven by the static gearmotor; AS5600 magnet on the hub.")


def gimbal_tilt_cradle(sz: Size) -> Part:
    """Camera cradle with tilt pins and +/-90 deg hard-stop tab (camera frame, centred)."""
    c = cam(sz)
    t = 1.2
    back = P.box(-c["h"] / 2, c["h"] / 2, -c["w"] / 2 - t, c["w"] / 2 + t, -c["d"] / 2 - t, -c["d"] / 2)
    hx = 4.0  # side cheeks only near the tilt axis: keeps the swept sphere = camera corners
    sides = (P.box(-hx, hx, c["w"] / 2, c["w"] / 2 + t, -c["d"] / 2 - t, c["d"] / 2 * 0.6)
             .union(P.box(-hx, hx, -c["w"] / 2 - t, -c["w"] / 2, -c["d"] / 2 - t, c["d"] / 2 * 0.6)))
    pins = P.cyl_between(0.95, (0, -c["w"] / 2 - 2.0, 0), (0, c["w"] / 2 + 2.0, 0))
    stop = P.box(-hx - 1.5, -hx, c["w"] / 2, c["w"] / 2 + t, -1.0, 1.0)
    w = back.union(sides).union(pins).union(stop)
    return Part("gimbal_tilt_cradle", MOD, w, "PA12-CF", color=COLORS["print"], print_rot=((1, 0, 0), 0.0),
                fasteners={"M2x4 SHCS": 4})


def camera(sz: Size) -> Part:
    """Purchased camera module (camera frame, lens toward +Z)."""
    c = cam(sz)
    w = P.box(-c["h"] / 2, c["h"] / 2, -c["w"] / 2, c["w"] / 2, -c["d"] / 2, c["d"] / 2 - 4)
    w = w.union(P.disc(4.5, c["d"] / 2 - 4, c["d"] / 2))
    return Part("camera_module", "purchased", w, "PCB", purchased=True, mass_kg=c["mass"], cost=c["cost"], color=COLORS["pcb"])


def tilt_servo_part(sz: Size) -> Part:
    """Purchased tilt servo inside the pan carrier (rotates with pan)."""
    sv = tilt_servo(sz)
    zh = z_hub(sz)
    w = P.box(-sv["l"] / 2, sv["l"] / 2, -sv["w"] / 2, sv["w"] / 2, zh - 3 - sv["h"], zh - 3)
    return Part(f"tilt_servo_{sz.cad('tilt_servo')}", "purchased", w, "plastic", purchased=True, mass_kg=sv["mass"],
                cost=sv["cost"], color=COLORS["motor"])


def pan_motor(sz: Size) -> Part:
    """Purchased static pan gearmotor alongside the carrier; pinion meshes the ring gear."""
    pd = pan_drive(sz)
    x = pan_motor_xy(sz)
    z0 = z_ring(sz) + 0.5
    w = P.box(x - pd["h"] / 2, x + pd["h"] / 2, -pd["w"] / 2, pd["w"] / 2, z0, z0 + pd["l"])
    w = w.union(P.spur_gear(pan_motor_xy(sz) - carrier_r(sz) - 1.0 - 0.5, 0.5, 3.0, 0).translate((x, 0, z0 - 3.5)))
    return Part(f"pan_gearmotor_{sz.cad('pan_drive')}", "purchased", w, "steel", purchased=True, mass_kg=pd["mass"],
                cost=pd["cost"], color=COLORS["motor"])


def slip_ring(sz: Size) -> Part:
    """Purchased 6-wire slip ring on the pan axis."""
    sr = sz.p["catalog"]["slip_ring"]
    z1 = z_ring(sz) - 3.5
    w = P.disc(sr["d"] / 2, z1 - sr["l"], z1)
    return Part("slip_ring_6w", "purchased", w, "mixed", purchased=True, mass_kg=sr["mass"], cost=sr["cost"], color=COLORS["steel"])


def build(sz: Size) -> list[Part]:
    """All module D parts for a size."""
    return [camera_window(sz), led_ring_mount(sz), gimbal_pan_yoke(sz), gimbal_tilt_cradle(sz), camera(sz),
            tilt_servo_part(sz), pan_motor(sz), slip_ring(sz)]
