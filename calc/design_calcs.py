"""PIPEBOT-X design calculations.

Reads params/bot_params.yaml, computes every derived quantity (grip force, self-locking,
expansion torque, gear stress, speeds, deflection, packaging, elbow fit, power) and asserts
each against its requirement. Exits non-zero if any check fails.

Run:  python3 calc/design_calcs.py [--params path/to/bot_params.yaml] [--json out.json]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechanism as mech  # noqa: E402

Params = dict[str, Any]
DEFAULT_PARAMS = Path(__file__).resolve().parent.parent / "params" / "bot_params.yaml"


@dataclass(frozen=True)
class Check:
    """One requirement: `value` must be >= (or <=) `limit`."""

    name: str
    value: float
    limit: float
    kind: str  # ">=" or "<="
    unit: str

    @property
    def passed(self) -> bool:
        """True when the requirement is met."""
        return self.value >= self.limit if self.kind == ">=" else self.value <= self.limit


@dataclass(frozen=True)
class ThreadMetrics:
    """Efficiency and self-locking of a screw or worm thread."""

    lead_angle_deg: float
    effective_mu: float
    efficiency: float
    self_lock_margin: float


def load_params(path: Path = DEFAULT_PARAMS) -> Params:
    """Load the parameter YAML."""
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def required_total_normal_force(mass_kg: float, incline_deg: float, mu: float, safety_factor: float, g: float) -> float:
    """Total wheel normal force N_total >= SF*m*g*sin(incline)/mu to hold in place."""
    return safety_factor * mass_kg * g * math.sin(math.radians(incline_deg)) / mu


def thread_metrics(lead_angle_deg: float, mu: float, flank_half_angle_deg: float) -> ThreadMetrics:
    """Forward efficiency and self-lock margin of a thread (screw or worm)."""
    lam = math.radians(lead_angle_deg)
    mu_eff = mu / math.cos(math.radians(flank_half_angle_deg))
    phi = math.atan(mu_eff)
    return ThreadMetrics(
        lead_angle_deg=lead_angle_deg,
        effective_mu=mu_eff,
        efficiency=math.tan(lam) / math.tan(lam + phi),
        self_lock_margin=mu_eff / math.tan(lam),
    )


def screw_lead_angle_deg(lead_mm: float, dia_mm: float) -> float:
    """Lead angle of a screw/worm thread measured at the given diameter."""
    return math.degrees(math.atan(lead_mm / (math.pi * dia_mm)))


def lewis_stress_mpa(tangential_n: float, face_mm: float, module_mm: float, form_factor: float) -> float:
    """Lewis tooth-root bending stress."""
    return tangential_n / (face_mm * module_mm * form_factor)


def max_rigid_length_mm(bend_r: float, pipe_r: float, body_r: float, clearance: float) -> float:
    """Longest rigid cylinder (radius body_r) that passes a 90 deg bend of centreline radius bend_r.

    The body chord sits against the inner wall at its middle and touches the outer wall
    with both end corners: (R - Rp + 2*Rb + c)^2 + (L/2)^2 <= (R + Rp - c)^2.
    """
    a = bend_r - pipe_r + 2 * body_r + clearance
    b = bend_r + pipe_r - clearance
    return 2 * math.sqrt(max(b * b - a * a, 0.0))


def motor_copper_w(motor: dict[str, float], current_fraction: float) -> float:
    """Two-phase stepper winding loss at a fraction of rated current."""
    return 2 * motor["winding_ohm"] * (current_fraction * motor["rated_current_a"]) ** 2


def tether_robot_voltage(v_supply: float, power_w: float, loop_ohm: float) -> float:
    """Voltage at the robot for a constant-power load at the end of a resistive tether."""
    disc = v_supply * v_supply - 4 * power_w * loop_ohm
    return (v_supply + math.sqrt(disc)) / 2 if disc >= 0 else 0.0


def hold_incline_table(p: Params, size: str) -> list[tuple[float, float, float]]:
    """(incline deg, required N_total, required N per arm) for each hold angle."""
    c, s = p["common"], p["sizes"][size]
    rows = []
    for angle in c["hold_incline_deg"]:
        n_tot = required_total_normal_force(s["mass_target_kg"], angle, c["wheel_wet_friction_coeff"], c["grip_safety_factor"], c["gravity_m_s2"])
        rows.append((float(angle), n_tot, n_tot / c["arm_count"]))
    return rows


def expansion_ratio(s: dict[str, Any]) -> float:
    """Motor turns per lever-sector turn."""
    e = s["expansion"]
    return e["planetary_ratio"] * e["bevel_ratio"] * e["sector_teeth"]


def expansion_worm(p: Params, size: str) -> ThreadMetrics:
    """Self-locking metrics of the expansion worm (20 deg normal pressure angle)."""
    e = p["sizes"][size]["expansion"]
    w = p["common"]["expansion_worm"]
    lam = math.degrees(math.atan(w["starts"] * e["sector_module_mm"] / e["worm_pitch_dia_mm"]))
    return thread_metrics(lam, w["friction_coeff"], 20.0)


def arm_half_width_mm(p: Params, size: str) -> float:
    """Tangential half-width of an arm on its belt side: pod, belt, rocker plate, lever plate (nested forks)."""
    s = p["sizes"][size]
    pod = p["motors"][s["drive"]["motor"]]["frame_mm"]
    belt_side = pod / 2 + 0.5 + s["rocker"]["belt_width_mm"]
    return belt_side + 1.0 + s["rocker"]["plate_thickness_mm"] + 0.5 + s["lever"]["plate_thickness_mm"]


def evaluate_size(p: Params, size: str) -> tuple[dict[str, float], list[Check]]:
    """Compute all derived values and checks for one robot size."""
    c, s = p["common"], p["sizes"][size]
    worm, lim, pw, rej = c["drive_worm"], c["motor_limits"], c["power"], c["rejected"]
    g, n_arms = c["gravity_m_s2"], c["arm_count"]
    m, mu_w, mu_d = s["mass_target_kg"], c["wheel_wet_friction_coeff"], c["wheel_dry_friction_coeff"]
    weight = m * g
    d: dict[str, float] = {"weight_n": weight}
    chk: list[Check] = []

    # ---- Grip -----------------------------------------------------------
    n_tot = required_total_normal_force(m, 90, mu_w, c["grip_safety_factor"], g)
    f_arm = n_tot / n_arms
    f_max = s["max_grip_force_per_arm_n"]
    comp, k_arm = c["arm_compliance_travel_mm"], s["rocker"]["spring_rate_n_mm"]
    d.update(n_total_vertical_n=n_tot, grip_per_arm_n=f_arm)
    chk.append(Check("working grip per arm <= arm max", f_arm, f_max, "<=", "N"))
    chk.append(Check("grip at full spring compression <= arm max", f_arm + k_arm * comp, f_max, "<=", "N"))
    sf_min = mu_w * n_arms * (f_arm - k_arm * comp) / weight
    d["sf_min_transient"] = sf_min
    chk.append(Check("vertical wet SF at worst spring excursion", sf_min, c["min_transient_safety_factor"], ">=", "-"))
    d["wet_sf_at_max_grip"] = mu_w * n_arms * f_max / weight
    t1_margin = mu_d * n_arms * f_max / (c["hold_test_load_multiple"] * weight)
    d["t1_dry_margin"] = t1_margin
    chk.append(Check("T1 dry hold margin at max grip", t1_margin, c["hold_test_min_margin"], ">=", "-"))
    r_min = s["pipe_id_min_mm"] / 2 * 1e-3
    skew = math.sin(math.radians(c["wheel_alignment_error_deg"]))
    roll_demand = weight * (s["cg_offset_mm"] * 1e-3 + skew * r_min)
    roll_cap = mu_w * n_tot * r_min
    chk.append(Check("roll-torque capacity / demand", roll_cap / roll_demand, c["grip_safety_factor"], ">=", "-"))

    # ---- Expansion mechanism (worm-sector levers) ---------------------------
    geo = mech.arm_geometry(p, size)
    th0 = mech.theta_for_pipe(geo, s["pipe_id_min_mm"])
    th1 = mech.theta_for_pipe(geo, s["pipe_id_max_mm"])
    lo, hi = c["lever_angle_limits_deg"]
    d.update(theta_min_deg=math.degrees(th0), theta_max_deg=math.degrees(th1))
    chk.append(Check("lever angle at min pipe >= limit", math.degrees(th0), lo, ">=", "deg"))
    chk.append(Check("lever angle at max pipe <= limit", math.degrees(th1), hi, "<=", "deg"))
    samples = mech.theta_samples(th0, th1)
    tq = [mech.lever_torque_nm(geo, th, f_max, mu_d * f_max) for th in samples]
    t_lever = max(tq)
    e = s["expansion"]
    ew = c["expansion_worm"]
    wm_e = expansion_worm(p, size)
    r_sector = e["sector_module_mm"] * e["sector_teeth"] / 2
    f_tooth = t_lever / (r_sector * 1e-3)
    sigma = lewis_stress_mpa(f_tooth, e["sector_face_mm"], e["sector_module_mm"], ew["lewis_form_factor"])
    exp_m = p["motors"][e["motor"]]
    ratio = expansion_ratio(s)
    eta_chain = wm_e.efficiency * ew["bevel_efficiency"] * ew["planetary_efficiency"]
    t_motor = n_arms * t_lever / (ratio * eta_chain)
    avail = exp_m["holding_torque_nm"] * lim["squeeze_torque_fraction"]
    sweep_rpm = (th1 - th0) / (2 * math.pi) * ratio / c["sweep_time_s"] * 60
    rail_v = s["power"]["motor_rail_v"]

    def rpm_limit(mt: dict[str, Any]) -> float:
        return (mt["max_rpm_24v"] if rail_v >= 24 else mt["max_rpm"]) * lim["rpm_utilization"]

    steps_rad = lim["steps_per_rev"] * lim["microsteps"] * ratio / (2 * math.pi)
    d.update(
        lever_torque_max_nm=t_lever,
        expansion_worm_lead_deg=wm_e.lead_angle_deg,
        expansion_worm_lock_margin=wm_e.self_lock_margin,
        sector_pitch_radius_mm=r_sector,
        sector_tooth_force_n=f_tooth,
        sector_stress_mpa=sigma,
        expansion_ratio=ratio,
        expansion_motor_torque_nm=t_motor,
        sweep_motor_rpm=sweep_rpm,
        steps_per_mm_dia_at_min=steps_rad / (2 * geo.lever_len * math.cos(th0)),
        steps_per_mm_dia_at_max=steps_rad / (2 * geo.lever_len * math.cos(th1)),
    )
    chk.append(Check("expansion worm self-lock margin", wm_e.self_lock_margin, ew["min_self_lock_margin"], ">=", "-"))
    chk.append(Check("sector tooth bending stress (Lewis)", sigma, ew["sector_allow_stress_mpa"], "<=", "MPa"))
    chk.append(Check("expansion motor torque margin", avail / t_motor, lim["min_torque_margin"], ">=", "-"))
    chk.append(Check("full-range sweep rpm", sweep_rpm, rpm_limit(exp_m), "<=", "rpm"))

    # ---- Lever structure ---------------------------------------------------
    st, lv = c["arm_structure"], s["lever"]
    t_pl, h_pl = lv["plate_thickness_mm"], lv["plate_height_mm"]
    i_lever = 2 * t_pl * h_pl**3 / 12
    sigma_lever = t_lever * 1e3 * (h_pl / 2) / i_lever
    defl = f_max * geo.lever_len**3 / (3 * st["e_modulus_gpa"] * 1e3 * i_lever)
    defl_print = defl * st["e_modulus_gpa"] / st["printed_e_modulus_gpa"]
    d.update(lever_stress_mpa=sigma_lever, lever_deflection_mm=defl, lever_deflection_printed_mm=defl_print)
    chk.append(Check(f"lever bending stress ({st['material']})", sigma_lever, st["yield_mpa"] / st["min_yield_sf"], "<=", "MPa"))
    chk.append(Check("lever tip deflection", defl, st["max_wheel_deflection_mm"], "<=", "mm"))

    # ---- Packaging (planar) -------------------------------------------------
    lay = s["layout"]
    rb = s["body_dia_mm"] / 2
    drv = p["motors"][s["drive"]["motor"]]
    pod_half = drv["frame_mm"] / 2
    clr = c["pod_wheel_clearance_mm"]
    worst_axial = math.inf
    for th in samples:
        r_in = mech.wheel_centre_radius(geo, th) - geo.wheel_radius
        if r_in < rb + clr:
            z0, z1 = mech.wheel_z_span(geo, th)
            worst_axial = min(worst_axial, z0, lay["mid_zone_mm"] - z1)
    d["wheel_to_pod_axial_clearance_mm"] = worst_axial
    chk.append(Check("wheels clear the pods over the stroke", worst_axial, 0.0, ">=", "mm"))
    pod_top_gap = geo.wheel_radius + s["rocker"]["pod_drop_mm"] - pod_half
    d["pod_wall_gap_mm"] = pod_top_gap
    chk.append(Check("drive pod clears wall by a weld bead", pod_top_gap, c["weld_bead_mm"], ">=", "mm"))
    w_half = arm_half_width_mm(p, size)
    r_low = min(mech.wheel_centre_radius(geo, th0) - s["rocker"]["pod_drop_mm"] - pod_half, geo.pivot_r - h_pl / 2)
    r_need = w_half / math.tan(math.radians(55))
    d.update(arm_half_width_mm=w_half, arm_lowest_radius_mm=r_low)
    chk.append(Check("arms clear each other at 120 deg (lowest part radius)", r_low, r_need, ">=", "mm"))
    chk.append(Check("pods clear the spine when retracted", r_low, lay["spine_dia_mm"] / 2 + 2, ">=", "mm"))
    seg_rear = lay["rear_pod_mm"] + lay["mid_zone_mm"]
    seg_front = lay["front_pod_mm"]
    bend_r = c["bend_radius_over_pipe_id"] * s["pipe_id_min_mm"]
    l_allow = max_rigid_length_mm(bend_r, s["pipe_id_min_mm"] / 2, rb, c["elbow_clearance_mm"])
    joint_need = math.degrees((seg_rear / 2 + lay["joint_mm"] + seg_front / 2) / bend_r)
    d.update(bend_radius_mm=bend_r, rigid_rear_segment_mm=seg_rear, rigid_front_segment_mm=seg_front, elbow_max_rigid_length_mm=l_allow)
    d["one_piece_length_mm"] = seg_rear + lay["joint_mm"] + seg_front
    chk.append(Check("rear segment passes 90 deg elbow at min pipe", seg_rear, l_allow, "<=", "mm"))
    chk.append(Check("front segment passes 90 deg elbow at min pipe", seg_front, l_allow, "<=", "mm"))
    chk.append(Check("articulation joint angle needed in the bend", joint_need, lay["joint_max_angle_deg"], "<=", "deg"))

    # ---- Drive train -------------------------------------------------------
    wm = thread_metrics(worm["lead_angle_deg"], worm["friction_coeff"], 0.0)
    dr = s["drive"]
    r_wheel = s["wheel_dia_mm"] / 2 * 1e-3
    wpm = c["wheels_per_arm"]  # one motor per arm
    traction = mu_d * (f_max / c["wheels_per_arm"]) * r_wheel * wpm
    stall_at_wheel = drv["holding_torque_nm"] * dr["worm_ratio"] * wm.efficiency
    hold_load = c["hold_test_load_multiple"] * weight / (n_arms * c["wheels_per_arm"]) * r_wheel * wpm
    climb = weight * r_wheel / n_arms
    rpm_max = s["max_speed_mm_s"] * 1e-3 / (math.pi * s["wheel_dia_mm"] * 1e-3) * 60 * dr["worm_ratio"]
    d.update(worm_efficiency=wm.efficiency, traction_limit_nm=traction, drive_motor_rpm_max=rpm_max)
    chk.append(Check("drive worm self-lock margin", wm.self_lock_margin, worm["min_self_lock_margin"], ">=", "-"))
    chk.append(Check("wheel slips before motor stalls", stall_at_wheel / traction, 1.0, ">=", "-"))
    chk.append(Check("drive worm rating / traction limit", dr["worm_rated_torque_nm"] / traction, worm["min_rating_margin"], ">=", "-"))
    chk.append(Check("drive worm rating / T1 hold load", dr["worm_rated_torque_nm"] / hold_load, worm["min_rating_margin"], ">=", "-"))
    climb_avail = drv["holding_torque_nm"] * lim["speed_torque_fraction"] * dr["worm_ratio"] * wm.efficiency
    chk.append(Check("climb torque margin at max speed", climb_avail / climb, 2.0, ">=", "-"))
    chk.append(Check("drive motor rpm at max speed", rpm_max, rpm_limit(drv), "<=", "rpm"))
    drv_lim = p["stepper_drivers"][s["power"]["stepper_driver"]]
    i_need = max(drv["rated_current_a"], exp_m["rated_current_a"]) * pw["motor_run_current_fraction"]
    chk.append(Check(f"motor run current within {s['power']['stepper_driver']} rating", i_need, drv_lim["max_rms_a"], "<=", "A"))
    chk.append(Check("motor rail within driver voltage", rail_v, drv_lim["max_v"], "<=", "V"))

    # ---- Power -------------------------------------------------------------
    sp = s["power"]
    p_run = 3 * motor_copper_w(drv, pw["motor_run_current_fraction"])
    p_mech = weight * s["max_speed_mm_s"] * 1e-3 / (wm.efficiency * lim["drive_motor_efficiency"])
    p_hold = 3 * motor_copper_w(drv, pw["motor_hold_current_fraction"])
    p_motors = pw["crawl_duty"] * (p_run + p_mech) + (1 - pw["crawl_duty"]) * p_hold
    p_base = sp["compute_avg_w"] + pw["camera_w"] + pw["mcu_sensors_w"] + pw["led_white_w_max"] * pw["led_avg_duty"]
    p_avg = p_base + pw["gimbal_avg_w"] + p_motors
    motors_peak = 3 * motor_copper_w(drv, 1.0) + motor_copper_w(exp_m, 1.0) + p_mech
    p12 = motors_peak + pw["led_white_w_max"] + pw["led_ir_w_max"] + pw["gimbal_peak_w"]
    p5 = sp["compute_peak_w"] + pw["camera_w"] + pw["mcu_sensors_w"]
    i12, i5 = p12 / rail_v / pw["driver_efficiency"], p5 / pw["rail_logic_v"]
    p_peak_in = (p12 / pw["driver_efficiency"] + p5) / pw["converter_efficiency"]
    loop = pw["tether_conductor_ohm_per_m"] * max(c["tether_length_m"])
    v_robot = tether_robot_voltage(pw["tether_v"], p_peak_in, loop)
    d.update(avg_power_w=p_avg, peak_input_power_w=p_peak_in, rail_motor_a=i12, rail_5v_a=i5, tether_v_at_robot=v_robot)
    if sp["battery_series"] > 0:
        wh = sp["battery_series"] * pw["cell_nominal_v"] * sp["battery_parallel"] * sp["cell_ah"]
        runtime = wh * pw["battery_usable_fraction"] * pw["converter_efficiency"] / p_avg * 60
        d.update(battery_wh=wh, runtime_min=runtime)
        chk.append(Check("battery runtime", runtime, c["min_runtime_min"], ">=", "min"))
    chk.append(Check(f"{rail_v:.0f} V motor buck rating / peak load", sp["buck_motor_a"] / i12, pw["rail_margin"], ">=", "-"))
    chk.append(Check("5 V buck rating / peak load", sp["buck_5v_a"] / i5, pw["rail_margin"], ">=", "-"))
    chk.append(Check("robot voltage at peak load on longest tether", v_robot, pw["tether_min_v_at_robot"], ">=", "V"))
    d["tether_min_break_n"] = c["tether_break_load_factor"] * weight

    # ---- Evidence for rejected original-spec choices -----------------------
    stroke = (s["pipe_id_max_mm"] - s["pipe_id_min_mm"]) / 2
    ma_cone = math.tan(math.radians(rej["cone_half_angle_deg"]))
    belt_hold = drv["detent_torque_nm"] * rej["drive_belt_ratio"] * rej["drive_belt_efficiency"]
    d.update(
        rej_radial_stroke_over_body_radius=stroke / rb,
        rej_wedge_axial_space_mm=2 * stroke / ma_cone,
        rej_belt_detent_hold_ratio=belt_hold / hold_load,
        rej_nominal_dia_lead_angle_deg=screw_lead_angle_deg(rej["leadscrew_lead_mm"], rej["leadscrew_nominal_dia_mm"]),
        rej_pitch_dia_lead_angle_deg=screw_lead_angle_deg(rej["leadscrew_lead_mm"], rej["leadscrew_pitch_dia_mm"]),
    )
    return d, chk


def format_report(p: Params, size: str, d: dict[str, float], checks: list[Check]) -> str:
    """Render one size's results as text."""
    s = p["sizes"][size]
    out = [f"=== Size {size}: pipe {s['pipe_id_min_mm']}-{s['pipe_id_max_mm']} mm, {s['mass_target_kg']} kg ==="]
    out.append("Hold requirement (wet, SF applied) by pipe incline:")
    for ang, n_tot, n_arm in hold_incline_table(p, size):
        out.append(f"  {ang:5.0f} deg  N_total {n_tot:7.1f} N  per arm {n_arm:6.1f} N (max {s['max_grip_force_per_arm_n']} N)")
    out.append("Derived values:")
    for key, val in d.items():
        out.append(f"  {key:38s} {val:12.4f}")
    out.append("Checks:")
    for ch in checks:
        tag = "PASS" if ch.passed else "FAIL"
        out.append(f"  [{tag}] {ch.name:56s} {ch.value:10.3f} {ch.kind} {ch.limit:<9.3f} {ch.unit}")
    return "\n".join(out)


def main(argv: list[str] | None = None) -> int:
    """CLI entry point; returns 1 if any check fails."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--params", type=Path, default=DEFAULT_PARAMS)
    ap.add_argument("--json", type=Path, help="also write all derived values and checks as JSON")
    args = ap.parse_args(argv)
    p = load_params(args.params)
    failed = 0
    report: dict[str, Any] = {}
    for size in p["sizes"]:
        d, checks = evaluate_size(p, size)
        print(format_report(p, size, d, checks), end="\n\n")
        failed += sum(not ch.passed for ch in checks)
        report[size] = {"derived": d, "checks": [asdict(ch) | {"passed": ch.passed} for ch in checks]}
    print("Design-review evidence (original spec, rejected):")
    for size in p["sizes"]:
        d = report[size]["derived"]
        print(
            f"  {size}: radial stroke = {d['rej_radial_stroke_over_body_radius']:.2f} x body radius | "
            f"40deg cone needs {d['rej_wedge_axial_space_mm']:.0f} mm free axial space | "
            f"belt+detent hold = {d['rej_belt_detent_hold_ratio'] * 100:.1f}% of T1 | "
            f"Tr8x1.5 lead angle {d['rej_nominal_dia_lead_angle_deg']:.2f} (nominal) vs "
            f"{d['rej_pitch_dia_lead_angle_deg']:.2f} deg (pitch)"
        )
    if args.json:
        args.json.write_text(json.dumps(report, indent=2))
    print(f"\n{'ALL CHECKS PASSED' if not failed else f'{failed} CHECK(S) FAILED'}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
