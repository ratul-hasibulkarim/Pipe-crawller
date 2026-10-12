"""Unit tests for calc/design_calcs.py."""

import copy
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


def test_thread_metrics_self_lock() -> None:
    """A 3.8 deg lead with mu 0.15 self-locks; a 15 deg lead does not."""
    assert dc.thread_metrics(3.8, 0.15, 15.0).self_lock_margin > 1.0
    assert dc.thread_metrics(15.0, 0.15, 15.0).self_lock_margin < 1.0


def test_all_size_checks_pass() -> None:
    """Every requirement passes for every size in the YAML."""
    for size in P["sizes"]:
        _, checks = dc.evaluate_size(P, size)
        assert [c.name for c in checks if not c.passed] == []


def test_checks_detect_wet_friction_failure() -> None:
    """A slippery wall must be flagged, not silently accepted."""
    bad = copy.deepcopy(P)
    bad["common"]["wheel_wet_friction_coeff"] = 0.1
    _, checks = dc.evaluate_size(bad, "M")
    assert any(not c.passed for c in checks)


def test_expansion_worms_self_lock_every_size() -> None:
    """Arms must hold with the power off (worm lock), on every size."""
    for size in P["sizes"]:
        assert dc.expansion_worm(P, size).self_lock_margin > 1.0


def test_rejected_cone_design_numbers() -> None:
    """The cone/leadscrew design stays rejected: radial stroke exceeds the body radius."""
    for size in P["sizes"]:
        d, _ = dc.evaluate_size(P, size)
        assert d["rej_radial_stroke_over_body_radius"] > 1.0


def test_vertical_hold_has_margin() -> None:
    """Wet safety factor at max grip is at least 1.5 on every size."""
    for size in P["sizes"]:
        d, _ = dc.evaluate_size(P, size)
        assert d["wet_sf_at_max_grip"] >= 1.5
