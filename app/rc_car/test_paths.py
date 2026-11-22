"""Offline tests of the reference paths: inside the geofence, peak curvature <= 70 % of kappa_max,
sizes, smoothness, and the fixed tuning path."""

from __future__ import annotations

import math

import numpy as np
import pytest

import paths
import vehicle_params as vp

G = vp.load_geometry()
AREA = paths.load_area()


@pytest.mark.parametrize("name", paths.ALL)
def test_inside_geofence_and_feasible(name):
    p = paths.build(name, AREA, kappa_max=G.kappa_max)
    assert paths.inside(p, AREA)
    assert np.max(np.abs(p.kappa)) <= paths.PEAK_FRACTION * G.kappa_max
    assert np.allclose(np.diff(p.s), 0.01, atol=1e-9)
    step = np.hypot(np.diff(p.x), np.diff(p.y))
    assert np.allclose(step, 0.01, rtol=2e-3)  # arc-length spacing
    assert p.length > 3.0


def test_the_lab_area_and_the_measured_steering_limit():
    assert (AREA.length, AREA.width, AREA.margin) == (10.0, 10.0, 1.0) and AREA.usable == (8.0, 8.0)
    assert G.delta_max == 0.356 and G.front_track == G.rear_track == 0.30
    assert paths.PEAK_FRACTION * G.kappa_max == pytest.approx(0.9465, abs=1e-3)  # R >= 1.06 m


def test_circle_radius_and_laps():
    p = paths.build("circle", AREA)
    r = np.hypot(p.x, p.y)
    assert np.allclose(r, 1.5, rtol=1e-4) and r[0] >= 1.5 * G.r_min
    assert p.length == pytest.approx(2 * 2 * math.pi * 1.5, rel=1e-3)
    assert p.start == pytest.approx((0.0, -1.5, 0.0), abs=1e-6)
    assert np.all(p.kappa > 0) and np.allclose(p.kappa, 1 / 1.5, rtol=1e-4)  # counter-clockwise


def test_figure_eight_is_a_smooth_lemniscate():
    p = paths.build("figure_eight", AREA)
    assert p.kappa.min() < -0.5 and p.kappa.max() > 0.5  # turns both ways
    assert np.max(np.abs(p.kappa)) == pytest.approx(3 / 3.5, rel=2e-3)  # 3 / a at the tips
    assert math.hypot(p.x[-1] - p.x[0], p.y[-1] - p.y[0]) < 0.02
    assert np.max(np.abs(np.diff(p.kappa))) < 0.01  # no curvature jumps
    assert abs(p.x[0]) < 1e-9 and abs(p.y[0]) < 1e-9  # starts at the crossing


def test_arcs_and_lines_jumps_but_is_continuous():
    p = paths.build("arcs_and_lines", AREA)
    assert set(np.round(np.unique(np.round(p.kappa, 6)), 3)) == {-0.833, 0.0, 0.833}  # 0 and +-1/1.2
    assert math.hypot(p.x[-1] - p.x[0], p.y[-1] - p.y[0]) < 0.02
    assert np.max(np.hypot(np.diff(p.x), np.diff(p.y))) <= 0.01 + 1e-9
    dyaw = np.angle(np.exp(1j * np.diff(p.yaw)))
    assert np.max(np.abs(dyaw)) <= 0.01 * np.max(np.abs(p.kappa)) + 1e-9
    assert np.allclose(np.unwrap(p.yaw) - p.yaw[0], np.concatenate([[0], np.cumsum(p.kappa[1:] * 0.01)]), atol=0.02)


def test_lane_change_offset_and_straight_ends():
    p = paths.build("lane_change", AREA)
    assert p.y[-1] - p.y[0] == pytest.approx(1.0)
    assert np.all(np.abs(p.yaw[:20]) < 1e-6) and np.all(np.abs(p.yaw[-20:]) < 1e-6)


def test_tuning_path_is_fixed_and_not_evaluated():
    a = paths.build("s_curve", AREA)
    b = paths.build("s_curve", paths.Area(6.0, 4.0, 0.5))  # the area the baseline was tuned in
    assert np.array_equal(a.x, b.x) and np.array_equal(a.y, b.y)
    assert a.start == pytest.approx((-2.5, 0.0, math.atan(0.3 * 2 * math.pi / 3.6)), abs=1e-3)
    assert not set(paths.TUNING) & set(paths.EVALUATION + paths.STRESS)


def test_paths_fit_other_areas():
    for area in (paths.Area(12.0, 8.0, 1.0), paths.Area(10.0, 12.0, 1.0)):  # usable length >= 8 m (the lemniscate)
        for name in paths.ALL:
            assert paths.inside(paths.build(name, area, kappa_max=G.kappa_max), area)


def test_too_small_area_is_refused():
    small = paths.Area(4.0, 3.0, 0.5)
    for name in ("arcs_and_lines", "figure_eight"):
        with pytest.raises(ValueError):
            paths.build(name, small, kappa_max=G.kappa_max)
