"""Tests for the Ackermann reference model (the review's "Validation proposal").

Run: python3 -m pytest app/ackermann -q

Every vehicle number below is a PLACEHOLDER chosen only to exercise the equations. None is a
property of any real vehicle; replace them with values from the parameter-source table.
"""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ackermann_model as am  # noqa: E402
import hw_estimates as hw  # noqa: E402


# ---- PLACEHOLDERS (not vehicle data) ----------------------------------------------------------
PLACEHOLDER_GEOMETRY = am.AckermannGeometry(wheelbase=1.0, front_track=0.5, rear_track=0.5)
PLACEHOLDER_DELTA_MAX = 0.5  # rad
PLACEHOLDER_DELTA_RATE_MAX = 1.0  # rad/s
PLACEHOLDER_LIMITS = am.AckermannLimits(
    v_min=0.0,
    v_max=2.0,
    kappa_max=am.kappa_limit_from_steering(PLACEHOLDER_DELTA_MAX, PLACEHOLDER_GEOMETRY.wheelbase),
    accel_max=1.0,
    decel_max=2.0,
    kappa_rate_max=am.kappa_rate_limit_from_steering_rate(PLACEHOLDER_DELTA_RATE_MAX,
                                                          PLACEHOLDER_GEOMETRY.wheelbase),
    lat_accel_max=1.5,
)
PLACEHOLDER_REVERSE_LIMITS = am.AckermannLimits(
    v_min=-1.0, v_max=2.0, kappa_max=PLACEHOLDER_LIMITS.kappa_max, accel_max=1.0, decel_max=2.0,
    kappa_rate_max=PLACEHOLDER_LIMITS.kappa_rate_max, lat_accel_max=None,
)
PLACEHOLDER_WEIGHTS = am.CostWeights(q_pos=100.0, q_psi=10.0, q_v=1.0, r_kappa=0.1, s_dv=1.0,
                                     s_dkappa=1.0, qf_pos=1000.0, qf_psi=100.0)
TS = 0.05  # s, the current config.hpp _Ts; the target vehicle's period is still open
SAMPLE_TIMES = (0.02, 0.05, 0.1)  # s, placeholder range
V_RANGE = 2.0  # m/s, placeholder |v| range
KAPPA_RANGE = PLACEHOLDER_LIMITS.kappa_max  # 1/m, tan(0.5) / 1.0 = 0.546
# ------------------------------------------------------------------------------------------------

METHODS = ("arc", "chord", "rk4", "midpoint", "euler")


# 1. Zero steering -> straight line; zero speed -> pose unchanged ---------------------------------

@pytest.mark.parametrize("method", METHODS)
@pytest.mark.parametrize("psi", [-3.0, -1.0, 0.0, 0.7, 2.5])
def test_zero_steering_is_straight_line(method: str, psi: float) -> None:
    states = am.rollout([1.0, -2.0, psi], [[1.5, 0.0]], TS, horizon=40, method=method)
    distance = 1.5 * TS * np.arange(41)
    np.testing.assert_allclose(states[:, 0], 1.0 + distance * math.cos(psi), atol=1e-12)
    np.testing.assert_allclose(states[:, 1], -2.0 + distance * math.sin(psi), atol=1e-12)
    np.testing.assert_allclose(states[:, 2], psi, atol=1e-15)


@pytest.mark.parametrize("method", METHODS)
@pytest.mark.parametrize("kappa", [0.0, 0.3, -0.3, 1e6, -1e-300])
def test_zero_speed_leaves_pose_unchanged(method: str, kappa: float) -> None:
    start = np.array([3.0, 4.0, 1.2])
    np.testing.assert_array_equal(am.step(start, [0.0, kappa], TS, method), start)


# 2. Yaw sign under the convention (x forward, y left, psi CCW, positive kappa/delta = left) ------

@pytest.mark.parametrize("method", METHODS)
def test_yaw_sign_follows_steering_sign(method: str) -> None:
    origin = [0.0, 0.0, 0.0]
    left = am.step(origin, [1.0, am.kappa_from_delta(0.3, 1.0)], TS, method)
    right = am.step(origin, [1.0, am.kappa_from_delta(-0.3, 1.0)], TS, method)
    assert left[2] > 0.0 and right[2] < 0.0
    if method != "euler":  # Euler moves along the initial heading only
        assert left[1] > 0.0 and right[1] < 0.0
    np.testing.assert_allclose(left * [1, -1, -1], right, atol=1e-15)  # mirror symmetry
    reverse_left = am.step(origin, [-1.0, 0.4], TS, method)
    assert reverse_left[2] < 0.0  # reversing with left steering turns the heading clockwise
    assert am.continuous_dynamics(origin, [1.0, 0.4])[2] > 0.0
    assert math.copysign(1.0, am.delta_from_kappa(-0.2, 1.0)) == -1.0


# 3. Constant speed/steering traces the predicted circle ----------------------------------------

@pytest.mark.parametrize("v, kappa", [(1.0, 0.5), (1.0, -0.5), (-0.8, 0.25), (2.0, 1e-3)])
def test_constant_inputs_trace_circle(v: float, kappa: float) -> None:
    radius = 1.0 / kappa
    centre = np.array([0.0, radius])  # ICR at y_b = R from the origin pose (0, 0, 0)
    for method, tolerance in (("arc", 1e-12), ("chord", 1e-12), ("rk4", 1e-9), ("midpoint", 1e-4)):
        states = am.rollout([0.0, 0.0, 0.0], [[v, kappa]], TS, horizon=200, method=method, wrap=False)
        distance_to_centre = np.hypot(states[:, 0] - centre[0], states[:, 1] - centre[1])
        np.testing.assert_allclose(distance_to_centre, abs(radius), rtol=tolerance, atol=tolerance)
        np.testing.assert_allclose(states[:, 2], v * kappa * TS * np.arange(201), atol=1e-12)
    # A full revolution with the exact arc returns to the start pose.
    steps = 360
    full_turn = am.rollout([0, 0, 0], [[2 * math.pi / (kappa * TS * steps) * math.copysign(1, v), kappa]],
                           TS, horizon=steps, method="chord")
    if abs(kappa) > 0.1:
        np.testing.assert_allclose(full_turn[-1], [0.0, 0.0, 0.0], atol=1e-9)


# 4. Ackermann inner/outer wheel angles -----------------------------------------------------------

@pytest.mark.parametrize("radius", [1.0, 2.5, 10.0, 100.0])
def test_inner_outer_ordering_left_and_right(radius: float) -> None:
    L, W = PLACEHOLDER_GEOMETRY.wheelbase, PLACEHOLDER_GEOMETRY.front_track
    left_turn = am.wheel_steering_angles(1.0 / radius, L, W)
    right_turn = am.wheel_steering_angles(-1.0 / radius, L, W)
    bicycle = am.delta_from_kappa(1.0 / radius, L)
    # Left turn: both positive, left (inner) > bicycle > right (outer); review's formula.
    assert left_turn[0] > bicycle > left_turn[1] > 0.0
    assert math.tan(left_turn[0]) == pytest.approx(L / (radius - W / 2), rel=1e-12)
    assert math.tan(left_turn[1]) == pytest.approx(L / (radius + W / 2), rel=1e-12)
    # Right turn: mirror image, right wheel is inner.
    assert right_turn[1] < -bicycle < right_turn[0] < 0.0
    np.testing.assert_allclose(right_turn, [-left_turn[1], -left_turn[0]], atol=1e-15)
    inner, outer = am.inner_outer_angles(-1.0 / radius, L, W)
    assert (inner, outer) == (right_turn[1], right_turn[0]) and abs(inner) > abs(outer)
    # Each front wheel points perpendicular to the ray from the ICR (0, R).
    for kappa, angles in ((1.0 / radius, left_turn), (-1.0 / radius, right_turn)):
        for y_wheel, angle in zip((W / 2, -W / 2), angles):
            ray = np.array([L, y_wheel - 1.0 / kappa])
            assert abs(ray @ [math.cos(angle), math.sin(angle)]) < 1e-12 * np.linalg.norm(ray)


def test_wheel_angles_converge_to_bicycle_as_track_shrinks() -> None:
    kappa, L = 0.4, PLACEHOLDER_GEOMETRY.wheelbase
    bicycle = am.delta_from_kappa(kappa, L)
    previous = math.inf
    for track in (1.0, 0.1, 0.01, 1e-3, 1e-6):
        left, right = am.wheel_steering_angles(kappa, L, track)
        error = max(abs(left - bicycle), abs(right - bicycle))
        assert error < previous and error <= 0.5 * track  # O(W) convergence
        previous = error
    assert am.wheel_steering_angles(kappa, L, 0.0) == (bicycle, bicycle)
    assert am.wheel_steering_angles(0.0, L, 0.5) == (0.0, 0.0)
    with pytest.raises(ValueError):
        am.wheel_steering_angles(4.0, L, 0.5)  # ICR inside the track (|kappa| W/2 = 1)


def test_rear_wheel_speeds_match_circle() -> None:
    left, right = am.rear_wheel_speeds(1.0, 0.5, 0.5)
    assert left == pytest.approx(1.0 * (2.0 - 0.25) / 2.0) and right == pytest.approx(1.0 * (2.0 + 0.25) / 2.0)


# 5. Exact arc vs integrated versions over the (placeholder) Ts, speed and steering ranges ------

def test_exact_arc_vs_rk4_and_midpoint_within_stated_tolerance() -> None:
    """Per step, RK4 reduces to Simpson's rule here: |error| <= s dpsi^4 / 2880; midpoint
    errs by s (1 - sinc(dpsi/2)) <= s dpsi^2 / 24 (s = |v| Ts, dpsi = v kappa Ts)."""
    rng = np.random.default_rng(1)
    worst = {"rk4": 0.0, "midpoint": 0.0}
    for _ in range(3000):
        ts = float(rng.choice(SAMPLE_TIMES))
        v = rng.uniform(-V_RANGE, V_RANGE)
        kappa = rng.uniform(-KAPPA_RANGE, KAPPA_RANGE)
        state = np.array([rng.uniform(-50, 50), rng.uniform(-50, 50), rng.uniform(-math.pi, math.pi)])
        exact = am.step_exact_arc(state, [v, kappa], ts)
        np.testing.assert_allclose(am.step_exact_arc_chord(state, [v, kappa], ts), exact, atol=1e-12)
        s, dpsi = abs(v) * ts, abs(v * kappa * ts)
        for method, bound in (("rk4", s * dpsi**4 / 2880), ("midpoint", s * dpsi**2 / 24)):
            error = float(np.hypot(*(am.step(state, [v, kappa], ts, method)[:2] - exact[:2])))
            assert error <= 1.001 * bound + 1e-12
            worst[method] = max(worst[method], error)
    # Stated tolerances for this placeholder range (max dpsi = 0.109 rad, s <= 0.2 m):
    assert worst["rk4"] < 1e-7 and worst["midpoint"] < 1e-3


def test_horizon_prediction_error_rk4_vs_exact() -> None:
    controls = [[1.0, 0.3], [1.5, -0.2], [2.0, 0.5], [0.5, -0.5]]
    exact = am.rollout([0, 0, 0.3], controls, TS, horizon=25, method="chord", wrap=False)
    rk4 = am.rollout([0, 0, 0.3], controls, TS, horizon=25, method="rk4", wrap=False)
    assert np.max(np.hypot(*(exact[:, :2] - rk4[:, :2]).T)) < 1e-8


# 6. Near-zero curvature: continuous and finite -------------------------------------------------

@pytest.mark.parametrize("psi", [0.0, 0.9, -2.2, 3.1])
def test_near_zero_curvature_is_continuous_and_finite(psi: float) -> None:
    v = 1.3
    s = v * TS
    eps = am.DPSI_EPS
    # Curvatures whose dpsi spans the threshold from both sides and both signs.
    dpsis = np.concatenate([np.geomspace(1e-3 * eps, 1e3 * eps, 301), [eps, eps * (1 + 1e-12)]])
    for sign in (1.0, -1.0):
        arc = np.array([am.step_exact_arc([0, 0, psi], [v, sign * d / s], TS) for d in dpsis])
        chord = np.array([am.step_exact_arc_chord([0, 0, psi], [v, sign * d / s], TS) for d in dpsis])
        assert np.all(np.isfinite(arc)) and np.all(np.isfinite(chord))
        assert np.max(np.abs(arc - chord)) <= 2e-8 * s  # jump at the threshold is below 2e-8 s
        # Adjacent samples move no more than the curvature change itself explains (s ddpsi / 2)
        # plus the 2e-8 s branch/rounding allowance on each side.
        smooth_bound = 0.5 * s * np.max(np.diff(dpsis[:301])) + 4e-8 * s
        assert np.max(np.abs(np.diff(arc[:301, :2], axis=0))) <= smooth_bound
    for kappa in (0.0, -0.0, 5e-324, -5e-324, 1e-300, 1e-200):
        result = am.step_exact_arc([0, 0, psi], [v, kappa], TS)
        assert np.all(np.isfinite(result))
        np.testing.assert_allclose(result, [s * math.cos(psi), s * math.sin(psi), am.wrap_angle(psi)], atol=1e-15)
    assert am.sinc(0.0) == 1.0
    assert am.sinc(0.99e-4) == pytest.approx(math.sin(0.99e-4) / 0.99e-4, rel=1e-15)
    assert am.sinc(1.01e-4) == pytest.approx(1 - 1.01e-4**2 / 6, rel=1e-15)


# 7. Heading wrap across -pi/+pi ----------------------------------------------------------------

def test_wrap_angle_and_heading_error_across_pi() -> None:
    assert am.wrap_angle(math.pi) == pytest.approx(-math.pi)
    assert am.wrap_angle(-math.pi) == pytest.approx(-math.pi)
    assert am.wrap_angle(3 * math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert am.wrap_angle(-7.0) == pytest.approx(-7.0 + 2 * math.pi)
    angles = np.linspace(-20, 20, 4001)
    wrapped = am.wrap_angle(angles)
    assert np.all((wrapped >= -math.pi) & (wrapped < math.pi))
    np.testing.assert_allclose(np.sin(wrapped), np.sin(angles), atol=1e-12)
    np.testing.assert_allclose(np.cos(wrapped), np.cos(angles), atol=1e-12)
    assert am.angle_error(math.pi - 0.1, -math.pi + 0.1) == pytest.approx(-0.2)
    assert am.angle_error(-math.pi + 0.1, math.pi - 0.1) == pytest.approx(0.2)
    assert am.unwrap_near(-math.pi + 0.1, center=math.pi - 0.05) == pytest.approx(math.pi + 0.1)


def test_step_across_pi_and_cost_invariance() -> None:
    # Turn left through psi = +pi; the stored heading wraps to -pi.., the error stays small.
    states = am.rollout([0, 0, math.pi - 0.05], [[1.0, 0.5]], TS, horizon=10, method="chord")
    assert np.all((states[:, 2] >= -math.pi) & (states[:, 2] < math.pi))
    assert states[-1, 2] < 0.0
    unwrapped = am.rollout([0, 0, math.pi - 0.05], [[1.0, 0.5]], TS, horizon=10, method="chord", wrap=False)
    np.testing.assert_allclose(am.angle_error(states[:, 2], unwrapped[:, 2]), 0.0, atol=1e-12)
    controls = am.expand_horizon([[1.0, 0.5]], 10)
    cost_wrapped = am.mpc_cost(states, controls, [1.0, 0.5], unwrapped[1:], PLACEHOLDER_WEIGHTS)
    cost_shifted = am.mpc_cost(states, controls, [1.0, 0.5], unwrapped[1:] + [0, 0, 2 * math.pi],
                               PLACEHOLDER_WEIGHTS)
    assert cost_wrapped["heading"] == pytest.approx(0.0, abs=1e-20)
    assert cost_shifted["heading"] == pytest.approx(0.0, abs=1e-20)
    assert cost_wrapped["total"] == pytest.approx(cost_shifted["total"])


# 8. Generated feasible controls never exceed the bounds ----------------------------------------

@pytest.mark.parametrize("limits", [PLACEHOLDER_LIMITS, PLACEHOLDER_REVERSE_LIMITS])
def test_projected_controls_never_exceed_bounds(limits: am.AckermannLimits) -> None:
    rng = np.random.default_rng(2)
    for trial in range(400):
        u_prev = am.project_controls([[rng.uniform(-3, 3), rng.uniform(-1, 1)]], [0.0, 0.0], limits, TS)[0]
        raw = rng.normal(0.0, 3.0, size=(25, 2)) * [1.0, 0.5]
        if trial % 7 == 0:
            raw[rng.integers(25)] = [np.nan, 1e9]  # garbage from the optimizer
        feasible = am.project_controls(raw, u_prev, limits, TS)
        report = am.check_controls(feasible, u_prev, limits, TS, PLACEHOLDER_GEOMETRY, PLACEHOLDER_DELTA_MAX)
        assert report["ok"], report
        # Steering angle and steering rate hold in delta, through the conservative kappa bounds.
        delta = am.delta_from_kappa(np.concatenate([[u_prev[1]], feasible[:, 1]]), PLACEHOLDER_GEOMETRY.wheelbase)
        assert np.max(np.abs(delta)) <= PLACEHOLDER_DELTA_MAX + 1e-12
        assert np.max(np.abs(np.diff(delta))) <= PLACEHOLDER_DELTA_RATE_MAX * TS + 1e-12
        # The candidates are feasible by construction too.
        for candidate in (am.hold_candidate(u_prev, 25), am.brake_candidate(u_prev, 25, limits, TS)):
            assert am.check_controls(candidate, u_prev, limits, TS)["ok"]
        # Next period: feasible[0] was applied, so the shifted plan is checked from it.
        assert am.check_controls(am.shift_warm_start(feasible), feasible[0], limits, TS)["ok"]


def test_shifted_warm_start_is_feasible_from_applied_input() -> None:
    rng = np.random.default_rng(3)
    u_prev = np.array([1.0, 0.1])
    plan = am.project_controls(rng.normal(size=(25, 2)), u_prev, PLACEHOLDER_LIMITS, TS)
    shifted = am.shift_warm_start(plan)
    assert am.check_controls(shifted, plan[0], PLACEHOLDER_LIMITS, TS)["ok"]
    assert am.brake_candidate(plan[0], 25, PLACEHOLDER_LIMITS, TS)[-1, 0] == 0.0
    # A known-feasible candidate that is not zero control: the reference feed-forward.
    reference = am.rollout([0, 0, 0], [[1.2, 0.3]], TS, horizon=25, method="chord")[1:]
    feedforward = am.feedforward_from_reference(reference, [0, 0, 0], TS)
    np.testing.assert_allclose(feedforward, np.tile([1.2, 0.3], (25, 1)), atol=1e-9)
    projected = am.project_controls(feedforward, u_prev, PLACEHOLDER_LIMITS, TS)
    assert am.check_controls(projected, u_prev, PLACEHOLDER_LIMITS, TS)["ok"]


def test_check_controls_reports_each_violation_separately() -> None:
    report = am.check_controls([[2.5, 0.0], [2.5, 0.9]], [2.0, 0.0], PLACEHOLDER_LIMITS, TS,
                               PLACEHOLDER_GEOMETRY, PLACEHOLDER_DELTA_MAX)
    assert not report["ok"]
    assert report["speed"] == pytest.approx(0.5)
    assert report["accel"] == pytest.approx(0.5 / TS - 1.0)
    assert report["kappa"] == pytest.approx(0.9 - PLACEHOLDER_LIMITS.kappa_max)
    assert report["kappa_rate"] > 0.0 and report["lat_accel"] == pytest.approx(2.5**2 * 0.9 - 1.5)
    assert report["steering"] == pytest.approx(math.atan(0.9) - PLACEHOLDER_DELTA_MAX)
    assert report["decel"] == 0.0


def test_normalization_round_trip_and_slew() -> None:
    controls = np.array([[0.0, -PLACEHOLDER_LIMITS.kappa_max], [2.0, PLACEHOLDER_LIMITS.kappa_max], [1.0, 0.0]])
    normalized = am.normalize_controls(controls, PLACEHOLDER_LIMITS, scale=100.0)
    np.testing.assert_allclose(normalized, [[-100, -100], [100, 100], [0, 0]], atol=1e-12)
    np.testing.assert_allclose(am.denormalize_controls(normalized, PLACEHOLDER_LIMITS, 100.0), controls, atol=1e-12)
    du = am.normalized_slew_limits(PLACEHOLDER_LIMITS, TS, scale=100.0)
    np.testing.assert_allclose(du, [100 * 1.0 * TS / 1.0, 100 * PLACEHOLDER_LIMITS.kappa_rate_max * TS
                                    / PLACEHOLDER_LIMITS.kappa_max])


def test_expand_horizon_holds_last_input() -> None:
    expanded = am.expand_horizon([[1, 0.1], [2, 0.2]], 5)
    np.testing.assert_array_equal(expanded, [[1, 0.1], [2, 0.2], [2, 0.2], [2, 0.2], [2, 0.2]])
    with pytest.raises(ValueError):
        am.expand_horizon(np.zeros((6, 2)), 5)


# Hardware-arithmetic emulation and budget sanity ------------------------------------------------

def test_table_sine_emulation_error_and_budget_calibration() -> None:
    angles = np.linspace(-math.pi, math.pi, 2001)
    error = max(abs(float(hw.local_sin(np.float16(a))) - math.sin(a)) for a in angles)
    assert error < 6e-3  # half-step of the 1024-entry table plus half-precision index
    low, high = hw.solve_ms(3, 24, 48977, 100)
    assert low == pytest.approx(38.1, abs=0.1) and high == pytest.approx(39.4, abs=0.1)
