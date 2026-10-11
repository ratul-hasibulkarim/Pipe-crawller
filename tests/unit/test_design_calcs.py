"""Unit tests for calc/design_calcs.py."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "calc"))

import design_calcs as dc  # noqa: E402

P = dc.load_params()


def test_grip_force_formula() -> None:
    """N_total = SF*m*g/mu at 90 deg; zero at horizontal."""
    assert math.isclose(dc.required_total_normal_force(1.0, 90, 0.5, 2.0, 10.0), 40.0)
    assert dc.required_total_normal_force(1.0, 0, 0.5, 2.0, 10.0) < 1e-9


def test_leadscrew_self_locking() -> None:
    """Tr8x1.5 uses the pitch diameter and must self-lock with margin."""
    ls = P["common"]["leadscrew"]
    lam = dc.screw_lead_angle_deg(ls["lead_mm"], ls["pitch_dia_mm"])
    m = dc.thread_metrics(lam, ls["friction_coeff"], ls["flank_half_angle_deg"])
    assert math.isclose(lam, 3.768, abs_tol=0.01)
    assert m.self_lock_margin >= ls["min_self_lock_margin"]
    assert 0.2 < m.efficiency < 0.4


def test_all_size_checks_pass() -> None:
    """Every requirement passes for every size in the YAML."""
    for size in P["sizes"]:
        _, checks = dc.evaluate_size(P, size)
        assert [c.name for c in checks if not c.passed] == []


def test_checks_detect_failure() -> None:
    """A weakened design must be flagged, not silently accepted."""
    bad = dc.load_params()
    bad["common"]["wheel_wet_friction_coeff"] = 0.2
    _, checks = dc.evaluate_size(bad, "S")
    assert any(not c.passed for c in checks)


def test_steps_per_mm() -> None:
    """3200 usteps/rev over a 1.5 mm lead."""
    d, _ = dc.evaluate_size(P, "S")
    assert math.isclose(d["steps_per_mm_axial"], 3200 / 1.5)
