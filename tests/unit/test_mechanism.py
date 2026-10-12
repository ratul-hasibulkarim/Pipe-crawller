"""Unit tests for calc/mechanism.py (lever kinematics shared by CAD and firmware LUT)."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "calc"))

import design_calcs as dc  # noqa: E402
import mechanism as mech  # noqa: E402

P = dc.load_params()


@pytest.mark.parametrize("size", list(P["sizes"]))
def test_theta_pipe_roundtrip(size: str) -> None:
    """pipe_for_theta(theta_for_pipe(D)) == D across the rated range."""
    g = mech.arm_geometry(P, size)
    lo, hi = P["sizes"][size]["pipe_id_min_mm"], P["sizes"][size]["pipe_id_max_mm"]
    for d in (lo, (lo + hi) / 2, hi):
        assert math.isclose(mech.pipe_for_theta(g, mech.theta_for_pipe(g, d)), d, abs_tol=1e-6)


@pytest.mark.parametrize("size", list(P["sizes"]))
def test_lut_monotonic(size: str) -> None:
    """The firmware diameter LUT is strictly increasing in both columns."""
    g = mech.arm_geometry(P, size)
    lo, hi = P["sizes"][size]["pipe_id_min_mm"], P["sizes"][size]["pipe_id_max_mm"]
    lut = mech.diameter_lut(g, mech.theta_for_pipe(g, lo - 5), mech.theta_for_pipe(g, hi + 5))
    assert all(b[0] > a[0] and b[1] > a[1] for a, b in zip(lut, lut[1:], strict=False))
    assert lut[0][1] == 0.0


def test_out_of_reach_raises() -> None:
    """A pipe the lever cannot reach is an error, not a silent clamp."""
    g = mech.arm_geometry(P, "M")
    with pytest.raises(ValueError):
        mech.theta_for_pipe(g, 10_000)


def test_lever_torque_grows_with_load() -> None:
    """More wall force or traction means more worm torque."""
    g = mech.arm_geometry(P, "M")
    th = mech.theta_for_pipe(g, 200)
    assert mech.lever_torque_nm(g, th, 100, 0) > mech.lever_torque_nm(g, th, 50, 0) > 0
    assert mech.lever_torque_nm(g, th, 50, 20) > mech.lever_torque_nm(g, th, 50, 0)
