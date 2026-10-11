"""PIPEBOT-X design calculations.

Reads params/bot_params.yaml, computes every derived quantity (grip force,
self-locking, leadscrew torque, speeds, deflection, power) and asserts each
against its requirement. Exits non-zero if any check fails.

Run:  python3 calc/design_calcs.py [--params path/to/bot_params.yaml]
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

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
    """Lead angle of a screw thread measured at the given diameter."""
    return math.degrees(math.atan(lead_mm / (math.pi * dia_mm)))


def cone_axial_force(radial_per_arm_n: float, arms: int, half_angle_deg: float, mu: float) -> float:
    """Axial cone thrust needed for `radial_per_arm_n` on every arm (wedge + friction)."""
    return arms * radial_per_arm_n * math.tan(math.radians(half_angle_deg) + math.atan(mu))


def leadscrew_torque_nm(axial_n: float, lead_mm: float, eta_screw: float, eta_bearing: float) -> float:
    """Screw torque T = F*p/(2*pi*eta) including the thrust bearing."""
    return axial_n * lead_mm * 1e-3 / (2 * math.pi * eta_screw * eta_bearing)


def euler_buckling_n(e_gpa: float, minor_dia_mm: float, length_mm: float, k: float) -> float:
    """Euler critical load of a round screw core."""
    inertia = math.pi * minor_dia_mm**4 / 64
    return math.pi**2 * e_gpa * 1e3 * inertia / (k * length_mm) ** 2


def cantilever_deflection_mm(force_n: float, length_mm: float, e_gpa: float, b_mm: float, h_mm: float) -> float:
    """Tip deflection of a rectangular cantilever, h in the bending direction."""
    inertia = b_mm * h_mm**3 / 12
    return force_n * length_mm**3 / (3 * e_gpa * 1e3 * inertia)


def sag_mm(chord_mm: float, bend_radius_mm: float) -> float:
    """Sagitta of a rigid body of length `chord_mm` in a bend of centreline radius R."""
    return bend_radius_mm - math.sqrt(bend_radius_mm**2 - (chord_mm / 2) ** 2)


def motor_copper_w(motor: dict[str, float], current_fraction: float) -> float:
    """Two-phase stepper winding loss at a fraction of rated current."""
    return 2 * motor["winding_ohm"] * (current_fraction * motor["rated_current_a"]) ** 2


def hold_incline_table(p: Params, size: str) -> list[tuple[float, float, float]]:
    """(incline deg, required N_total, required N per arm) for each hold angle."""
    c, s = p["common"], p["sizes"][size]
    rows = []
    for angle in c["hold_incline_deg"]:
        n_tot = required_total_normal_force(
            s["mass_target_kg"],
            angle,
            c["wheel_wet_friction_coeff"],
            c["grip_safety_factor"],
            c["gravity_m_s2"],
        )
        rows.append((float(angle), n_tot, n_tot / c["arm_count"]))
    return rows


def evaluate_size(p: Params, size: str) -> tuple[dict[str, float], list[Check]]:
    """Compute all derived values and checks for one robot size."""
    c, s = p["common"], p["sizes"][size]
    ls, worm, lim = c["leadscrew"], c["drive_worm"], c["motor_limits"]
    pw, rej = c["power"], c["rejected"]
    g, n_arms = c["gravity_m_s2"], c["arm_count"]
    m, mu_w, mu_d = s["mass_target_kg"], c["wheel_wet_friction_coeff"], c["wheel_dry_friction_coeff"]
    weight = m * g
    d: dict[str, float] = {"weight_n": weight}
    chk: list[Check] = []

    # ---- Grip -----------------------------------------------------------
    n_tot = required_total_normal_force(m, 90, mu_w, c["grip_safety_factor"], g)
    f_arm = n_tot / n_arms
    f_max = s["max_grip_force_per_arm_n"]
    comp = c["arm_compliance_travel_mm"]
    k_arm = s["arm_spring_rate_n_mm"]
    d.update(n_total_vertical_n=n_tot, grip_per_arm_n=f_arm)
    chk.append(Check("working grip per arm <= arm max", f_arm, f_max, "<=", "N"))
    chk.append(Check("grip at full spring extension <= arm max", f_arm + k_arm * comp, f_max, "<=", "N"))
    sf_min = mu_w * n_arms * (f_arm - k_arm * comp) / weight
    d["sf_min_transient"] = sf_min
    chk.append(Check("vertical wet SF at worst spring excursion", sf_min, c["min_transient_safety_factor"], ">=", "-"))
    wet_sf_max = mu_w * n_arms * f_max / weight
    d["wet_sf_at_max_grip"] = wet_sf_max
    t1_load = c["hold_test_load_multiple"] * weight
    t1_margin = mu_d * n_arms * f_max / t1_load
    d["t1_dry_margin"] = t1_margin
    chk.append(Check("T1 dry hold margin at max grip", t1_margin, c["hold_test_min_margin"], ">=", "-"))
    r_min = s["pipe_id_min_mm"] / 2 * 1e-3
    roll_demand = weight * (s["cg_offset_mm"] * 1e-3 + math.sin(math.radians(c["wheel_alignment_error_deg"])) * r_min)
    roll_cap = mu_w * n_tot * r_min
    d.update(roll_demand_nm=roll_demand, roll_capacity_nm=roll_cap)
    chk.append(Check("roll-torque capacity / demand", roll_cap / roll_demand, c["grip_safety_factor"], ">=", "-"))

    # ---- Expansion mechanism ---------------------------------------------
    exp_m = p["motors"][s["expansion_motor"]]
    lam_deg = screw_lead_angle_deg(ls["lead_mm"], ls["pitch_dia_mm"])
    scr = thread_metrics(lam_deg, ls["friction_coeff"], ls["flank_half_angle_deg"])
    stroke_r = (s["pipe_id_max_mm"] - s["pipe_id_min_mm"]) / 2
    tan_a = math.tan(math.radians(c["cone"]["half_angle_deg"]))
    travel = stroke_r / tan_a
    f_ax = cone_axial_force(
        f_max + s["return_spring_force_n"], n_arms, c["cone"]["half_angle_deg"], c["cone"]["friction_coeff"]
    )
    torque = leadscrew_torque_nm(f_ax, ls["lead_mm"], scr.efficiency, ls["thrust_bearing_efficiency"])
    avail = exp_m["holding_torque_nm"] * lim["squeeze_torque_fraction"]
    sweep_rpm = travel / ls["lead_mm"] / c["sweep_time_s"] * 60
    screw_len = travel + ls["nut_length_mm"] + 2 * ls["end_margin_mm"]
    p_cr = euler_buckling_n(ls["e_modulus_gpa"], ls["minor_dia_mm"], s["chassis_length_mm"], ls["buckling_k"])
    steps_rev = lim["steps_per_rev"] * lim["microsteps"]
    steps_axial = steps_rev / ls["lead_mm"]
    d.update(
        lead_angle_deg=lam_deg,
        screw_efficiency=scr.efficiency,
        screw_lock_margin=scr.self_lock_margin,
        radial_stroke_mm=stroke_r,
        axial_travel_mm=travel,
        cone_axial_force_n=f_ax,
        screw_torque_nm=torque,
        sweep_motor_rpm=sweep_rpm,
        screw_length_mm=screw_len,
        buckling_load_n=p_cr,
        steps_per_mm_axial=steps_axial,
        steps_per_mm_radial=steps_axial / tan_a,
        steps_per_mm_diameter=steps_axial / tan_a / 2,
    )
    chk.append(Check("leadscrew self-lock margin", scr.self_lock_margin, ls["min_self_lock_margin"], ">=", "-"))
    chk.append(Check("final-squeeze torque margin (usable / required)", avail / torque, 1.5, ">=", "-"))
    f_sweep = cone_axial_force(
        s["return_spring_force_n"], n_arms, c["cone"]["half_angle_deg"], c["cone"]["friction_coeff"]
    )
    t_sweep = leadscrew_torque_nm(f_sweep, ls["lead_mm"], scr.efficiency, ls["thrust_bearing_efficiency"])
    chk.append(
        Check(
            "sweep torque margin at speed",
            exp_m["holding_torque_nm"] * lim["drive_speed_torque_fraction"] / t_sweep,
            1.5,
            ">=",
            "-",
        )
    )
    chk.append(
        Check(
            "full-range sweep rpm <= utilisation limit",
            sweep_rpm,
            exp_m["max_rpm"] * lim["rpm_utilization"],
            "<=",
            "rpm",
        )
    )
    chk.append(Check("leadscrew length <= chassis length", screw_len, s["chassis_length_mm"], "<=", "mm"))
    chk.append(Check("leadscrew buckling SF", p_cr / f_ax, ls["min_buckling_sf"], ">=", "-"))
    wheel_inner = s["pipe_id_min_mm"] / 2 - s["wheel_dia_mm"]
    chk.append(
        Check(
            "wheel inner edge radius at min pipe >= screw + clearance",
            wheel_inner,
            ls["nominal_dia_mm"] / 2 + c["packaging_clearance_mm"],
            ">=",
            "mm",
        )
    )

    # ---- Drive train -----------------------------------------------------
    drv = p["motors"][s["drive_motor"]]
    wm = thread_metrics(worm["lead_angle_deg"], worm["friction_coeff"], 0.0)
    r_wheel = s["wheel_dia_mm"] / 2 * 1e-3
    wheels_per_motor = n_arms * c["wheels_per_arm"] / n_arms
    traction = mu_d * (f_max / c["wheels_per_arm"]) * r_wheel * wheels_per_motor
    stall_at_wheel = drv["holding_torque_nm"] * worm["ratio"] * wm.efficiency
    hold_load = c["hold_test_load_multiple"] * weight / (n_arms * c["wheels_per_arm"]) * r_wheel * wheels_per_motor
    climb = weight * r_wheel / n_arms
    rpm_max = c["max_speed_mm_s"] * 1e-3 / (math.pi * s["wheel_dia_mm"] * 1e-3) * 60 * worm["ratio"]
    d.update(
        worm_efficiency=wm.efficiency,
        worm_lock_margin=wm.self_lock_margin,
        traction_limit_nm=traction,
        motor_stall_at_wheel_nm=stall_at_wheel,
        hold_load_nm=hold_load,
        climb_torque_nm=climb,
        drive_motor_rpm_max=rpm_max,
    )
    chk.append(Check("drive worm self-lock margin", wm.self_lock_margin, worm["min_self_lock_margin"], ">=", "-"))
    chk.append(Check("wheel slips before motor stalls (stall/traction)", stall_at_wheel / traction, 1.0, ">=", "-"))
    chk.append(
        Check(
            "gearbox rating / traction limit",
            s["drive_worm_rated_torque_nm"] / traction,
            worm["min_rating_margin"],
            ">=",
            "-",
        )
    )
    chk.append(
        Check(
            "gearbox rating / T1 power-off hold load",
            s["drive_worm_rated_torque_nm"] / hold_load,
            worm["min_rating_margin"],
            ">=",
            "-",
        )
    )
    chk.append(
        Check(
            "climb torque margin at max speed",
            drv["holding_torque_nm"] * lim["drive_speed_torque_fraction"] * worm["ratio"] * wm.efficiency / climb,
            2.0,
            ">=",
            "-",
        )
    )
    chk.append(Check("drive motor rpm at max speed", rpm_max, drv["max_rpm"] * lim["rpm_utilization"], "<=", "rpm"))

    # ---- Structure & geometry --------------------------------------------
    st = c["arm_structure"]
    defl = cantilever_deflection_mm(
        f_max, s["arm_cantilever_mm"], st["e_modulus_gpa"], s["arm_section_b_mm"], s["arm_section_h_mm"]
    )
    defl_print = cantilever_deflection_mm(
        f_max, s["arm_cantilever_mm"], st["printed_e_modulus_gpa"], s["arm_section_b_mm"], s["arm_section_h_mm"]
    )
    d.update(arm_deflection_mm=defl, arm_deflection_printed_mm=defl_print)
    chk.append(Check(f"arm wheel deflection ({st['material']})", defl, st["max_wheel_deflection_mm"], "<=", "mm"))
    clearance = s["pipe_id_min_mm"] / 2 - s["body_dia_mm"] / 2
    elbow = sag_mm(s["chassis_length_mm"], c["max_pipe_bend_radius_mm"]) + comp
    d["elbow_sag_mm"] = elbow
    chk.append(Check("90 deg elbow: body sag + compliance <= clearance", elbow, clearance, "<=", "mm"))

    # ---- Power ------------------------------------------------------------
    v_bat, ah = pw["battery_nominal_v"], s["battery_capacity_ah"]
    wh = v_bat * ah
    p_run = 3 * motor_copper_w(drv, pw["motor_run_current_fraction"])
    p_mech = weight * c["max_speed_mm_s"] * 1e-3 / (wm.efficiency * lim["drive_motor_efficiency"])
    p_hold = 3 * motor_copper_w(drv, pw["motor_hold_current_fraction"])
    p_motors = pw["crawl_duty"] * (p_run + p_mech) + (1 - pw["crawl_duty"]) * p_hold
    p_base = (
        s["compute_avg_w"]
        + pw["camera_w"]
        + pw["mcu_sensors_w"]
        + pw["led_white_w_max"] * pw["led_avg_duty"]
        + pw["gimbal_avg_w"]
    )
    p_avg = p_base + p_motors
    runtime = wh * pw["battery_usable_fraction"] * pw["converter_efficiency"] / p_avg * 60
    motors_peak = 3 * motor_copper_w(drv, 1.0) + motor_copper_w(exp_m, 1.0)
    i12 = (
        (motors_peak + pw["led_white_w_max"] + pw["led_ir_w_max"] + pw["gimbal_peak_w"])
        / pw["rail_motor_v"]
        / pw["driver_efficiency"]
    )
    i5 = (s["compute_peak_w"] + pw["camera_w"] + pw["mcu_sensors_w"]) / pw["rail_logic_v"]
    d.update(battery_wh=wh, avg_power_w=p_avg, runtime_min=runtime, rail_12v_a=i12, rail_5v_a=i5)
    chk.append(Check("battery runtime", runtime, c["min_runtime_min"], ">=", "min"))
    chk.append(Check("12 V buck rating / peak load", s["buck_12v_a"] / i12, pw["rail_margin"], ">=", "-"))
    chk.append(Check("5 V buck rating / peak load", s["buck_5v_a"] / i5, pw["rail_margin"], ">=", "-"))

    # ---- Evidence for rejected original-spec choices -----------------------
    orig_rpm = sweep_rpm * rej["expansion_worm_ratio"]
    belt_hold = drv["detent_torque_nm"] * rej["drive_belt_ratio"] * rej["drive_belt_efficiency"]
    ts = thread_metrics(
        screw_lead_angle_deg(ls["lead_mm"], rej["leadscrew_nominal_dia_lead_angle_mm"]),
        ls["friction_coeff"],
        ls["flank_half_angle_deg"],
    )
    cur_step = exp_m["rated_current_a"] / rej["stepper_current_steps"]
    kt = exp_m["holding_torque_nm"] / exp_m["rated_current_a"]
    d.update(
        rej_worm_expansion_rpm=orig_rpm,
        rej_belt_detent_hold_ratio=belt_hold / hold_load,
        rej_nominal_dia_lead_angle_deg=ts.lead_angle_deg,
        current_setpoint_force_step_n=cur_step
        * kt
        * 2
        * math.pi
        * scr.efficiency
        * ls["thrust_bearing_efficiency"]
        / (ls["lead_mm"] * 1e-3)
        / (n_arms * math.tan(math.radians(c["cone"]["half_angle_deg"]) + math.atan(c["cone"]["friction_coeff"]))),
    )
    return d, chk


def format_report(p: Params, size: str, d: dict[str, float], checks: list[Check]) -> str:
    """Render one size's results as text."""
    s = p["sizes"][size]
    out = [f"=== Size {size}: pipe {s['pipe_id_min_mm']}-{s['pipe_id_max_mm']} mm, {s['mass_target_kg']} kg ==="]
    out.append("Hold requirement (wet, SF applied) by pipe incline:")
    for ang, n_tot, n_arm in hold_incline_table(p, size):
        out.append(
            f"  {ang:5.0f} deg  N_total {n_tot:7.1f} N   per arm {n_arm:6.1f} N (max {s['max_grip_force_per_arm_n']} N)"
        )
    out.append("Derived values:")
    for key, val in d.items():
        out.append(f"  {key:34s} {val:12.4f}")
    out.append("Checks:")
    for ch in checks:
        tag = "PASS" if ch.passed else "FAIL"
        out.append(f"  [{tag}] {ch.name:58s} {ch.value:10.3f} {ch.kind} {ch.limit:<9.3f} {ch.unit}")
    return "\n".join(out)


def main(argv: list[str] | None = None) -> int:
    """CLI entry point; returns 1 if any check fails."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--params", type=Path, default=DEFAULT_PARAMS)
    args = ap.parse_args(argv)
    p = load_params(args.params)
    failed = 0
    for size in p["sizes"]:
        d, checks = evaluate_size(p, size)
        print(format_report(p, size, d, checks), end="\n\n")
        failed += sum(not ch.passed for ch in checks)
    print("Design-review evidence (original spec, rejected):")
    for size in p["sizes"]:
        d, _ = evaluate_size(p, size)
        print(
            f"  {size}: 1:20 worm on expansion needs {d['rej_worm_expansion_rpm']:7.0f} rpm | "
            f"belt+detent power-off hold = {d['rej_belt_detent_hold_ratio'] * 100:4.1f}% of T1 load | "
            f"nominal-dia lead angle {d['rej_nominal_dia_lead_angle_deg']:.2f} vs true {d['lead_angle_deg']:.2f} deg | "
            f"current-setpoint force step {d['current_setpoint_force_step_n']:.2f} N/arm"
        )
    print(f"\n{'ALL CHECKS PASSED' if not failed else f'{failed} CHECK(S) FAILED'}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
