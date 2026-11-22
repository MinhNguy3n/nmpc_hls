"""Offline tests of the C1-C6 analysis on synthetic trajectories."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bringup_analysis as ba  # noqa: E402


def arc(v: float, kappa: float, duration: float, dt: float = 0.01, x0=0.0, y0=0.0, yaw0=0.0):
    t = np.arange(0.0, duration, dt)
    yaw = yaw0 + v * kappa * t
    if kappa == 0.0:
        x, y = x0 + v * t * math.cos(yaw0), y0 + v * t * math.sin(yaw0)
    else:
        x = x0 + (np.sin(yaw) - math.sin(yaw0)) / kappa
        y = y0 - (np.cos(yaw) - math.cos(yaw0)) / kappa
    return t, x, y, np.arctan2(np.sin(yaw), np.cos(yaw))  # wrapped like the truth


def test_drift_of_resting_and_creeping_poses():
    t = np.linspace(0, 10, 1001)
    still = ba.drift(t, np.full_like(t, 1.0), np.full_like(t, 2.0), np.full_like(t, 3.1))
    assert still["max_xy_m"] == 0.0 and still["max_yaw_rad"] == 0.0
    creep = ba.drift(t, 1.0 + 1e-4 * t, np.zeros_like(t), np.zeros_like(t))
    assert creep["max_xy_m"] == pytest.approx(1e-3)


def test_straight_line_speed_and_heading():
    t, x, y, yaw = arc(1.2, 0.0, 5.0, yaw0=0.7)
    r = ba.straight_line(t, x, y, yaw)
    assert r["speed_mps"] == pytest.approx(1.2, rel=1e-9)
    assert abs(r["heading_change_rad"]) < 1e-12 and r["lateral_max_m"] < 1e-9


@pytest.mark.parametrize("kappa", [1.0, -1.0, 0.5, -2.0])
@pytest.mark.parametrize("v", [0.5, -0.5])
def test_circle_fit_radius_sign_and_wrap(kappa, v):
    """Full circles cross the +-pi wrap of the yaw; the fit and the turn sign must not care."""
    delta = math.atan(0.275 * kappa)
    t, x, y, yaw = arc(v, kappa, 1.25 * 2 * math.pi / abs(v * kappa), x0=3.0, y0=-1.0, yaw0=2.0)
    r = ba.circle(t, x, y, yaw, 0.275, delta)
    assert r["radius_fit_m"] == pytest.approx(1.0 / abs(kappa), rel=1e-9)
    assert abs(r["radius_rel_err"]) < 1e-9 and r["fit_rms_m"] < 1e-9
    assert r["turn_sign_ok"]
    assert abs(r["lateral_speed_mean"]) < 1e-3  # no slip on the ideal arc (gradient error only)


def test_circle_rear_axle_speed_error():
    t, x, y, yaw = arc(0.52, 1.0, 10.0)  # runs 4 % faster than the 0.5 m/s command
    r = ba.circle(t, x, y, yaw, 0.275, math.atan(0.275), speed_cmd=0.5)
    assert r["speed_rel_err"] == pytest.approx(0.04, abs=1e-4)  # end-point differences
    assert math.isnan(ba.circle(t, x, y, yaw, 0.275, math.atan(0.275))["speed_rel_err"])


def test_circle_detects_wrong_turn_direction():
    t, x, y, yaw = arc(0.5, -1.0, 8.0)
    assert not ba.circle(t, x, y, yaw, 0.275, +math.atan(0.275))["turn_sign_ok"]


def test_lateral_speed_detects_slip():
    t, x, y, yaw = arc(1.0, 0.0, 2.0)
    y = y + 0.1 * t  # crabbing sideways at 0.1 m/s while heading along x
    _, v_lat, _ = ba.body_velocities(t, x, y, yaw)
    assert np.mean(v_lat) == pytest.approx(0.1, rel=1e-6)


def test_rise_time_first_order():
    t = np.linspace(0, 1, 100001)
    tau = 0.02
    value = 0.4 * (1 - np.exp(-t / tau))
    assert ba.rise_time(t, value, 0.0, 0.4) == pytest.approx(tau * math.log(9), rel=1e-3)
    assert math.isnan(ba.rise_time(t, 0.5 * value, 0.0, 0.4))  # never reaches 90 %
    # a negative step works the same way
    assert ba.rise_time(t, -value, 0.0, -0.4) == pytest.approx(tau * math.log(9), rel=1e-3)


def test_rate_and_rtf():
    s = np.arange(0, 2, 0.005)
    r = ba.rate(s)
    assert r["rate_hz"] == pytest.approx(200.0) and r["max_gap_s"] == pytest.approx(0.005)
    gap = np.delete(s, [100, 101])
    assert ba.rate(gap)["max_gap_s"] == pytest.approx(0.015)
    wall = np.linspace(0, 10, 50)
    assert ba.real_time_factor(0.8 * wall + 3.0, wall) == pytest.approx(0.8)
