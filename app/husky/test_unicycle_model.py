"""pytest tests of app/husky/unicycle_model.py (design note docs/husky_a300/unicycle_mpc_pso_model.md).

Run: /usr/bin/python3 -m pytest app/husky -q -p no:cacheprovider

Every operating-envelope number that is not a documented robot value is a PLACEHOLDER_ constant.
Documented values come from unicycle_model.PARAMETER_SETS (each with its source).
"""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import unicycle_model as um  # noqa: E402

TS = 0.05  # s: the existing solver period (config.hpp _Ts) and 20 Hz Create 3 odometry
METHODS = ("chord", "chord_poly", "arc", "rk4", "midpoint", "euler")

# PLACEHOLDER_ operating-envelope values (not robot properties; chosen only to exercise the code).
PLACEHOLDER_A300_SIDE_SPEED_MAX = 1.2  # m/s, unknown motor/driver side limit
PLACEHOLDER_A300_SIDE_ACCEL_MAX = 2.0  # m/s^2, unknown
PLACEHOLDER_A300_V_ENVELOPE = 1.0  # m/s, e.g. the 1.0 m/s off-road payload limit as an envelope
PLACEHOLDER_A300_OMEGA_ENVELOPE = 1.0  # rad/s
PLACEHOLDER_TB4_SIDE_SPEED_MAX = 0.306  # m/s, if the documented body limit also bounds each wheel


def a300_documented() -> um.UnicycleLimits:
    """Controller defaults only (reverse allowed, no side limits: those are unknown)."""
    return um.limits_from_parameter_set("husky_a300")


def a300_with_placeholders() -> um.UnicycleLimits:
    return um.limits_from_parameter_set(
        "husky_a300", v_min=-PLACEHOLDER_A300_V_ENVELOPE, v_max=PLACEHOLDER_A300_V_ENVELOPE,
        omega_max=PLACEHOLDER_A300_OMEGA_ENVELOPE, side_speed_max=PLACEHOLDER_A300_SIDE_SPEED_MAX,
        side_accel_max=PLACEHOLDER_A300_SIDE_ACCEL_MAX)


def tb4_with_placeholders() -> um.UnicycleLimits:
    return um.limits_from_parameter_set("turtlebot4_create3", side_speed_max=PLACEHOLDER_TB4_SIDE_SPEED_MAX)


ALL_LIMITS = {"a300_documented": a300_documented, "a300_placeholders": a300_with_placeholders,
              "tb4_placeholders": tb4_with_placeholders}


# ----------------------------------------------------------------------------------------------
# Parameter sets
# ----------------------------------------------------------------------------------------------


def test_parameter_sets_are_sourced_or_placeholder():
    for robot, params in um.PARAMETER_SETS.items():
        for name, item in params.items():
            assert item.source, (robot, name)
            if item.value is None:
                assert "PLACEHOLDER_" in item.source, (robot, name)
            else:
                assert item.source.startswith("http"), (robot, name)


def test_unknown_parameters_must_be_supplied():
    with pytest.raises(ValueError):
        um.limits_from_parameter_set("husky_a300", omega_max=None)
    limits = a300_documented()
    assert limits.side_speed_max is None and limits.side_accel_max is None
    assert limits.track == pytest.approx(0.562 * 1.75)


# ----------------------------------------------------------------------------------------------
# Motion primitives
# ----------------------------------------------------------------------------------------------


@pytest.mark.parametrize("method", METHODS)
def test_straight_motion(method):
    for psi in (0.0, 0.7, -2.5, math.pi):
        for v in (0.3, -0.2):
            state = um.step([1.0, -2.0, psi], [v, 0.0], TS, method)
            assert state[0] == pytest.approx(1.0 + v * TS * math.cos(psi), abs=1e-15)
            assert state[1] == pytest.approx(-2.0 + v * TS * math.sin(psi), abs=1e-15)
            assert state[2] == pytest.approx(um.wrap_angle(psi), abs=0)


@pytest.mark.parametrize("method", METHODS)
def test_zero_motion_leaves_pose_unchanged(method):
    start = np.array([3.0, 4.0, -1.0])
    assert np.array_equal(um.step(start, [0.0, 0.0], TS, method), start)


@pytest.mark.parametrize("method", METHODS)
def test_in_place_turn_and_yaw_sign(method):
    start = np.array([0.5, -0.5, 0.3])
    for omega in (1.9, -1.9, 1e-12):
        end = um.step(start, [0.0, omega], TS, method)
        assert end[0] == start[0] and end[1] == start[1]  # no translation at all
        assert end[2] - start[2] == pytest.approx(omega * TS, rel=1e-12)
        assert math.copysign(1.0, end[2] - start[2]) == math.copysign(1.0, omega)
    # Positive angular.z is counter-clockwise (REP-103): driving forward while turning left
    # moves the robot to +y from heading 0.
    state = um.rollout([0.0, 0.0, 0.0], [[0.3, 1.0]], TS, horizon=10)
    assert state[-1, 1] > 0.0 and state[-1, 2] > 0.0


def test_constant_arc_radius():
    for v, omega in ((0.3, 1.0), (1.5, -0.5), (-0.4, 0.8), (0.306, 1.9)):
        radius = v / omega
        centre = np.array([0.0, radius])  # ICR from heading 0 at the origin
        for method, tol in (("chord", 1e-12), ("arc", 1e-12), ("rk4", 1e-8), ("midpoint", 1e-3)):
            states = um.rollout([0.0, 0.0, 0.0], [[v, omega]], TS, horizon=200, method=method)
            distance = np.hypot(states[:, 0] - centre[0], states[:, 1] - centre[1])
            assert np.max(np.abs(distance - abs(radius))) <= tol * max(1.0, abs(radius)), method
    # A full revolution returns to the start (exact methods).
    omega = 2.0 * math.pi / (40 * TS)
    end = um.rollout([1.0, 2.0, 0.5], [[0.5, omega]], TS, horizon=40)[-1]
    assert np.allclose(end, [1.0, 2.0, 0.5], atol=1e-12)


def test_omega_to_zero_continuity_and_finiteness():
    psi, v = 0.4, 1.3
    straight = um.step([0.0, 0.0, psi], [v, 0.0], TS)
    for omega in [0.0, -0.0, 5e-324, 1e-300, 1e-20, 1e-12, 1e-9, um.DPSI_EPS / TS, 2 * um.DPSI_EPS / TS, 1e-6]:
        for sign in (1.0, -1.0):
            chord = um.step([0.0, 0.0, psi], [v, sign * omega], TS, "chord")
            arc = um.step([0.0, 0.0, psi], [v, sign * omega], TS, "arc")
            assert np.all(np.isfinite(chord)) and np.all(np.isfinite(arc))
            # Lateral offset of an arc is |s| |dpsi| / 2: below the bound, the step equals the line.
            bound = abs(v * TS) * abs(omega * TS) / 2 + 1e-15
            assert np.hypot(*(chord[:2] - straight[:2])) <= bound
            assert np.hypot(*(arc[:2] - chord[:2])) <= 2e-8 * abs(v * TS) + 1e-15
    omegas = np.linspace(-1e-6, 1e-6, 2001)
    xs = np.array([um.step([0.0, 0.0, psi], [v, w], TS, "chord")[:2] for w in omegas])
    steps = np.diff(xs, axis=0)  # equal omega increments -> equal position increments (no jump at 0)
    assert np.max(np.ptp(steps, axis=0)) < 1e-15 and np.max(np.abs(steps)) < 2e-12


def test_chord_vs_rk4_and_other_integrators_within_stated_tolerance():
    """Stated per-step tolerances over |v| <= 2 m/s, |omega| <= 2 rad/s (A300 controller limits)
    and Ts in {0.02, 0.05, 0.1} s: RK4 <= |s| dpsi^4/2880 (<= 1.2e-7 m; <= 3.5e-9 m at Ts = 0.05),
    midpoint <= |s| dpsi^2/24 (<= 1e-3 m), Euler <= |s| |dpsi|/2, chord = literal arc to 1e-12 m."""
    rng = np.random.default_rng(1)
    worst = {"rk4": 0.0, "midpoint": 0.0}
    for _ in range(3000):
        ts = float(rng.choice([0.02, 0.05, 0.1]))
        u = np.array([rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0)])
        start = np.array([rng.uniform(-50, 50), rng.uniform(-50, 50), rng.uniform(-math.pi, math.pi)])
        exact = um.step_chord(start, u, ts)
        s, dpsi = abs(u[0] * ts), abs(u[1] * ts)
        errors = {m: float(np.hypot(*(um.step(start, u, ts, m)[:2] - exact[:2]))) for m in METHODS}
        assert errors["arc"] <= 1e-12
        assert errors["chord_poly"] <= s * (dpsi / 2) ** 4 / 120 + 1e-13  # + rounding at 50 m
        assert errors["rk4"] <= s * dpsi ** 4 / 2880 + 1e-13
        assert errors["midpoint"] <= s * dpsi ** 2 / 24 + 1e-13
        assert errors["euler"] <= s * dpsi / 2 + 1e-13
        assert errors["rk4"] <= (1.2e-7 if ts > 0.05 else 3.5e-9) and errors["midpoint"] <= 1e-3
        for m in worst:
            worst[m] = max(worst[m], errors[m])
        assert um.step_rk4(start, u, ts)[2] == pytest.approx(exact[2], abs=1e-12)
    assert worst["rk4"] < worst["midpoint"]


def test_horizon_rk4_vs_chord_over_nh_25():
    rng = np.random.default_rng(2)
    for _ in range(200):
        controls = np.column_stack([rng.uniform(0, 2.0, 25), rng.uniform(-2.0, 2.0, 25)])
        start = [rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-math.pi, math.pi)]
        a = um.rollout(start, controls, TS, method="chord")
        b = um.rollout(start, controls, TS, method="rk4")
        assert np.max(np.hypot(a[:, 0] - b[:, 0], a[:, 1] - b[:, 1])) <= 25 * 1e-7


def test_hardware_sinc_approximation_is_separate_and_bounded():
    """Exact reference (sin(a)/a) vs the division-free hardware polynomial 1 - a^2/6."""
    for a in np.linspace(-0.49, 0.49, 981):
        error = um.sinc(a) - um.sinc_poly2(a)  # the polynomial underestimates the chord
        assert -1e-16 <= error <= um.sinc_poly2_error_bound(a) + 1e-16
        assert um.sinc_poly2_error_bound(a) < 4.9e-4  # below the half spacing near 1
    worst_relative = um.sinc_poly2_error_bound(um.SINC_POLY_INTENDED_ABS_DPSI / 2)
    assert worst_relative <= 5.3e-8
    rng = np.random.default_rng(10)
    worst = 0.0
    for _ in range(2000):
        u = np.array([rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0)])  # |omega Ts| <= 0.1
        start = np.array([rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-math.pi, math.pi)])
        exact, poly = um.step_chord(start, u, TS), um.step_chord_poly(start, u, TS)
        error = float(np.hypot(*(poly[:2] - exact[:2])))
        assert error <= abs(u[0] * TS) * um.sinc_poly2_error_bound(u[1] * TS / 2) + 1e-15
        assert poly[2] == exact[2]
        worst = max(worst, error)
    assert worst <= 0.1 * 5.3e-8  # <= 5.3e-9 m per step at A300 controller limits


# ----------------------------------------------------------------------------------------------
# Wheel-side mapping and saturation
# ----------------------------------------------------------------------------------------------


def test_wheel_side_mapping_round_trip_and_manual_equations():
    rng = np.random.default_rng(3)
    for track in (um.parameter("husky_a300", "track_kinematic").value,
                  um.parameter("turtlebot4_create3", "track_kinematic").value):
        v, w = rng.uniform(-2, 2, 500), rng.uniform(-2, 2, 500)
        left, right = um.body_to_sides(v, w, track)
        back_v, back_w = um.sides_to_body(left, right, track)
        assert np.allclose(back_v, v, atol=1e-15) and np.allclose(back_w, w, atol=1e-14)
        # A300 manual: v = (v_r + v_l)/2, omega = (v_r - v_l)/W
        assert np.allclose((right + left) / 2, v) and np.allclose((right - left) / track, w)
    left, right = um.body_to_sides(0.0, 1.0, 0.5)
    assert right > 0 > left  # CCW in-place turn: right side forward
    wl, wr = um.wheel_angular_speeds(0.5, 0.0, 0.562, 0.1625)
    assert wl == wr == pytest.approx(0.5 / 0.1625)


def test_saturation_preserves_turn_direction_and_curvature():
    rng = np.random.default_rng(4)
    track, limit = 0.233, PLACEHOLDER_TB4_SIDE_SPEED_MAX
    for _ in range(2000):
        v, w = rng.uniform(-1.0, 1.0), rng.uniform(-6.0, 6.0)
        sv, sw = um.saturate_sides(v, w, track, limit, "scale")
        left, right = um.body_to_sides(sv, sw, track)
        assert max(abs(left), abs(right)) <= limit + 1e-12
        assert np.sign(sw) == np.sign(w) and np.sign(sv) == np.sign(v)
        if abs(v) > 1e-9:
            assert sw / sv == pytest.approx(w / v, rel=1e-12)  # same turn radius
        if max(abs(x) for x in um.body_to_sides(v, w, track)) <= limit:
            assert (sv, sw) == (v, w)
    # Clipping each side separately can change the radius (shown, not proposed).
    cv, cw = um.saturate_sides(0.3, 1.5, track, limit, "clip")
    assert cw / cv != pytest.approx(1.5 / 0.3)


# ----------------------------------------------------------------------------------------------
# Feasible control generation
# ----------------------------------------------------------------------------------------------


@pytest.mark.parametrize("name", sorted(ALL_LIMITS))
def test_projected_controls_never_exceed_bounds(name):
    limits = ALL_LIMITS[name]()
    rng = np.random.default_rng(5)
    for trial in range(300):
        u_prev = um.project_stage(rng.uniform(-3, 3, 2), [0.0, 0.0], limits, TS)
        raw = rng.uniform(-5, 5, (25, 2))
        if trial % 10 == 0:
            raw[3] = [np.nan, 1e9]
        projected = um.project_controls(raw, u_prev, limits, TS)
        report = um.check_controls(projected, u_prev, limits, TS)
        assert report["ok"], (name, report)
        assert report["finite"]


@pytest.mark.parametrize("name", sorted(ALL_LIMITS))
def test_candidates_are_feasible(name):
    limits = ALL_LIMITS[name]()
    rng = np.random.default_rng(6)
    for _ in range(100):
        u_prev = um.project_stage(rng.uniform(-3, 3, 2), [0.0, 0.0], limits, TS)
        for _ in range(int(rng.integers(0, 30))):  # reach a realistic, still feasible, u_prev
            u_prev = um.project_stage(rng.uniform(-3, 3, 2), u_prev, limits, TS)
        stop = um.stop_candidate(u_prev, 25, limits, TS)
        turn = um.in_place_turn_candidate(u_prev, rng.uniform(-math.pi, math.pi), 25, limits, TS)
        hold = um.hold_candidate(u_prev, 25)
        for candidate in (stop, turn, hold):
            assert um.check_controls(candidate, u_prev, limits, TS)["ok"]
        assert np.allclose(stop[-1], 0.0)  # 25 x 50 ms is enough to stop from any documented speed
        assert np.all(np.abs(stop[:, 0]) <= abs(u_prev[0]) + 1e-12)
        assert abs(turn[-1, 0]) <= 1e-12


def test_stop_from_documented_a300_maximum_takes_decel_limited_time():
    limits = a300_documented()
    stop = um.stop_candidate([2.0, 0.0], 25, limits, TS)
    first_zero = int(np.argmax(np.abs(stop[:, 0]) < 1e-12))
    assert first_zero + 1 == math.ceil(2.0 / (limits.decel_max * TS) - 1e-9)  # 10 stages = 0.5 s


def test_in_place_turn_reaches_heading():
    limits = tb4_with_placeholders()
    heading_error = 2.5
    turn = um.in_place_turn_candidate([0.0, 0.0], heading_error, 40, limits, TS)
    states = um.rollout([0.0, 0.0, 0.0], turn, TS, method="chord", wrap=False)
    assert np.allclose(states[:, :2], 0.0)
    assert abs(states[-1, 2] - heading_error) < 0.05


def test_shifted_warm_start_is_feasible():
    limits = a300_with_placeholders()
    rng = np.random.default_rng(7)
    for _ in range(200):
        u_prev = um.project_stage(rng.uniform(-1, 1, 2), [0.0, 0.0], limits, TS)
        plan = um.project_controls(rng.uniform(-2, 2, (25, 2)), u_prev, limits, TS)
        shifted = um.shift_warm_start(plan)
        assert um.check_controls(shifted, plan[0], limits, TS)["ok"]


def test_check_controls_reports_each_violation_separately():
    limits = a300_with_placeholders()
    bad = np.array([[1.5, 0.0], [1.5, 1.5], [0.0, -1.5]])
    report = um.check_controls(bad, [0.0, 0.0], limits, TS)
    assert not report["ok"]
    assert report["speed"] == pytest.approx(0.5)
    assert report["omega"] == pytest.approx(0.5)
    assert report["linear_slew"] > 0 and report["angular_slew"] > 0
    assert report["side_speed"] > 0 and report["side_accel"] > 0
    assert um.check_controls([[np.nan, 0.0]], [0.0, 0.0], limits, TS) == {"finite": False, "ok": False}


def test_feedforward_inverts_the_chord_step():
    rng = np.random.default_rng(8)
    controls = np.column_stack([rng.uniform(-1, 1, 25), rng.uniform(-2, 2, 25)])
    controls[5] = [0.0, 1.0]  # in-place turn
    controls[6] = [0.4, 0.0]  # straight
    start = np.array([1.0, -1.0, 3.0])
    reference = um.rollout(start, controls, TS)[1:]
    assert np.allclose(um.feedforward_from_reference(reference, start, TS), controls, atol=1e-9)


def test_normalization_round_trip_and_slew():
    limits = a300_with_placeholders()
    controls = np.array([[-1.0, -1.0], [0.0, 0.0], [1.0, 1.0], [0.25, -0.5]])
    z = um.normalize_controls(controls, limits)
    assert np.allclose(z[[0, 2]], [[-100, -100], [100, 100]])
    assert np.allclose(um.denormalize_controls(z, limits), controls)
    du = um.normalized_slew_limits(limits, TS)
    # min(accel 2, decel 4) * Ts / half-range 1 * 100 and 4 rad/s^2 * Ts / 1 rad/s * 100
    assert np.allclose(du, [10.0, 20.0])
    v_box, w_box = um.inscribed_box(limits, 0.5)
    assert v_box + w_box * limits.half_track <= limits.side_speed_max + 1e-12


# ----------------------------------------------------------------------------------------------
# Angles
# ----------------------------------------------------------------------------------------------


def test_wrap_across_pi():
    assert um.wrap_angle(math.pi) == math.pi
    assert um.wrap_angle(-math.pi) == pytest.approx(math.pi)
    assert um.wrap_angle(3 * math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    for angle in np.linspace(-math.pi + 1e-9, math.pi, 101):
        assert um.wrap_angle(angle) == angle  # idempotent, bit-exact
    # Residual across the cut: 179 deg vs -179 deg is 2 deg, not 358 deg.
    a, b = math.radians(179.0), math.radians(-179.0)
    assert um.angle_error(a, b) == pytest.approx(math.radians(-2.0))
    assert um.angle_error(b, a) == pytest.approx(math.radians(2.0))
    # The HLS one-conditional residual (commit 8c61f1b) agrees for |x_hat - ref| < 3 pi.
    rng = np.random.default_rng(9)
    for _ in range(5000):
        ref = rng.uniform(-math.pi, math.pi)
        x_hat = ref + rng.uniform(-3 * math.pi + 1e-9, 3 * math.pi - 1e-9)
        assert um.hls_angle_residual(x_hat, ref) == pytest.approx(um.angle_error(x_hat, ref), abs=1e-12)
    assert abs(um.hls_angle_residual(4 * math.pi, 0.0)) > math.pi  # outside the contract


def test_step_across_pi_and_cost_invariance():
    start = np.array([0.0, 0.0, math.pi - 0.01])
    end = um.step(start, [0.5, 1.0], TS)
    assert -math.pi < end[2] < -math.pi + 0.05  # wrapped into (-pi, pi]
    unwrapped = um.step(start, [0.5, 1.0], TS, wrap=False)
    assert np.allclose(end[:2], unwrapped[:2]) and um.angle_error(end[2], unwrapped[2]) == pytest.approx(0, abs=1e-15)
    weights = um.CostWeights(q_pos=1.0, q_psi=3.0, qf_pos=10.0, qf_psi=30.0, q_v=0.0, r_omega=0.1)
    controls = np.tile([0.5, 1.0], (25, 1))
    states_w = um.rollout(start, controls, TS)
    states_u = um.rollout(start, controls, TS, wrap=False)
    reference = states_w[1:].copy()
    reference[:, 2] += 0.01
    costs = [um.mpc_cost(s, controls, [0.5, 1.0], reference, weights, hls_residual=h)["total"]
             for s in (states_w, states_u) for h in (False, True)]
    assert np.allclose(costs, costs[0], rtol=1e-12)


def test_local_frame_keeps_residual_contract():
    state0 = np.array([10.0, -3.0, 3.0])
    controls = np.tile([0.3, 1.9], (25, 1))  # turns 2.375 rad over the horizon, crossing +-pi
    world = um.rollout(state0, controls, TS)[1:]
    local = um.to_local_frame(state0, world)
    local_pred = um.rollout([0.0, 0.0, 0.0], controls, TS, wrap=False)[1:]
    assert np.allclose(local, local_pred, atol=1e-12)
    assert np.max(np.abs(local[:, 2])) < 3 * math.pi


def test_yaw_from_quaternion():
    for yaw in (0.0, 1.0, -2.0, math.pi):
        q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        assert um.angle_error(um.yaw_from_quaternion(*q), yaw) == pytest.approx(0.0, abs=1e-12)


# ----------------------------------------------------------------------------------------------
# State estimate (measured twist to the tick, applied command to actuation)
# ----------------------------------------------------------------------------------------------


def test_propagate_matches_chord_rollout_and_handles_segments():
    start = np.array([1.0, 2.0, 3.0])
    one = um.propagate_twist_segments(start, [[0.0, 0.4, 0.8]], 0.0, 0.05)
    assert np.allclose(one, um.step_chord(start, [0.4, 0.8], 0.05), atol=1e-15)
    controls = [[0.3, 0.5], [0.2, -0.4], [0.0, 1.0]]
    segments = [[0.05 * k, v, w] for k, (v, w) in enumerate(controls)]
    pieced = um.propagate_twist_segments(start, segments, 0.0, 0.15)
    assert np.allclose(pieced, um.rollout(start, controls, 0.05)[-1], atol=1e-12)
    # Splitting a constant twist at arbitrary times is exact (chord steps compose).
    split = um.propagate_twist_segments(start, [[0.0, 0.4, 0.8], [0.013, 0.4, 0.8], [0.031, 0.4, 0.8]], 0.0, 0.05)
    assert np.allclose(split, one, atol=1e-14)
    assert np.array_equal(um.propagate_twist_segments(start, [[0.0, 1.0, 1.0]], 0.2, 0.2), start)  # zero length
    with pytest.raises(ValueError):
        um.propagate_twist_segments(start, [[0.1, 0, 0], [0.0, 0, 0]], 0.0, 0.2)


def test_state_estimate_uses_measured_twist_then_applied_command():
    pose, stamp, tick = np.array([0.0, 0.0, 0.0]), 10.000, 10.020
    measured = [0.25, 0.1]  # odometry twist (realized), differs from the command below
    applied = [[10.020, 0.30, 0.0]]  # cmd_vel_out (applied) from the tick on
    at_tick, at_act = um.estimate_state_at_actuation(pose, measured, stamp, tick, tick + 0.05, applied, 0.06)
    assert np.allclose(at_tick, um.step_chord(pose, measured, 0.020))
    assert np.allclose(at_act, um.step_chord(at_tick, [0.30, 0.0], 0.05))
    with pytest.raises(ValueError, match="stale"):
        um.estimate_state_at_actuation(pose, measured, 9.90, tick, tick + 0.05, applied, 0.06)
    with pytest.raises(ValueError, match="future"):
        um.estimate_state_at_actuation(pose, measured, 10.03, tick, tick + 0.05, applied, 0.06)
    # Zero age: the tick estimate is the odometry pose itself.
    same, _ = um.estimate_state_at_actuation(pose, measured, tick, tick, tick + 0.05, applied, 0.06)
    assert np.array_equal(same, pose)
