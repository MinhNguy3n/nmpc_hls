"""Double-precision reference model of an Ackermann-steered, car-like vehicle for MPC-PSO.

This is an independent software reference for the proposal in
docs/ackermann/ackermann_mpc_pso_model.md. It is not wired into the HLS solver, the overlay or
any vehicle. Every vehicle parameter is a required argument: this module holds no vehicle values.
Tests and examples pass clearly labelled placeholders.

Conventions (see the design note, section 2):
- World frame: right-handed, x and y in metres, z up. Heading psi (rad) is measured from +x,
  counter-clockwise positive (CCW).
- Body frame at the rear-axle midpoint: x_b forward, y_b left.
- Signed speed v (m/s) along x_b; v < 0 is reverse.
- Signed path curvature kappa (1/m) and virtual (bicycle) steering angle delta (rad):
  positive = steering to the left, so the instantaneous centre of rotation (ICR) is at
  y_b = R = 1/kappa > 0. kappa = tan(delta) / L.
- State [x, y, psi]; control [v, kappa]. psidot = v * kappa, so reversing with left steering
  turns the heading clockwise, on the same circle.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Callable

import numpy as np


NX = 3  # [x (m), y (m), psi (rad)]
NU = 2  # [v (m/s), kappa (1/m)]

# Near-zero-turn threshold for step_exact_arc, on |dpsi| = |v * kappa * Ts| (rad). Below it the
# literal arc formula would divide by a tiny curvature, so the straight-line limit is used. In
# double precision the straight-line branch errs by about s * |dpsi| / 2 (s = |v| Ts) and the
# division branch by about s * c * eps / |dpsi| (cancellation in sin(psi + dpsi) - sin(psi),
# eps = 2.2e-16). The two balance near |dpsi| = 2e-8: a 20,000-sample random scan around the
# threshold gives a worst position error of 1.1e-8 * s against the chord form (3e-8: 1.5e-8 * s;
# 5e-9: 4.6e-8 * s), so the jump across the threshold is below ~2e-8 * s.
DPSI_EPS = 2e-8

# Below this |a| the chord factor sin(a)/a uses its Taylor series; the truncation error
# |a|^6 / 5040 < 2e-28 is far below double rounding, so the switch is invisible.
_SINC_SERIES_LIMIT = 1e-4


# ----------------------------------------------------------------------------------------------
# Parameters (all supplied by the caller; none are defaults)
# ----------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class AckermannGeometry:
    """Vehicle geometry. Obtain every value from the vehicle (see the parameter-source table).

    wheelbase: L, rear axle to front axle (m).
    front_track: W_f, distance between the front steering pivots (m).
    rear_track: W_r, distance between the rear wheel contact centres (m).
    """

    wheelbase: float
    front_track: float
    rear_track: float

    def __post_init__(self) -> None:
        if not (math.isfinite(self.wheelbase) and self.wheelbase > 0.0):
            raise ValueError("wheelbase must be finite and positive")
        for name in ("front_track", "rear_track"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f"{name} must be finite and non-negative")


@dataclass(frozen=True)
class AckermannLimits:
    """Control limits in physical units. Obtain every value from the vehicle.

    v_min, v_max: signed speed bounds (m/s). v_min = 0 means forward only (no reverse).
    kappa_max: symmetric curvature bound |kappa| <= kappa_max (1/m), normally
        tan(delta_max) / L from the steering travel (kappa_limit_from_steering).
    accel_max: largest allowed increase rate of v (m/s^2, > 0).
    decel_max: largest allowed decrease rate of v (m/s^2, > 0, given as a magnitude).
    kappa_rate_max: largest |dkappa/dt| (1/(m s)), e.g. from the steering rate via
        kappa_rate_limit_from_steering_rate.
    lat_accel_max: optional bound on v^2 |kappa| (m/s^2); None disables the check.
    """

    v_min: float
    v_max: float
    kappa_max: float
    accel_max: float
    decel_max: float
    kappa_rate_max: float
    lat_accel_max: float | None = None

    def __post_init__(self) -> None:
        values = [self.v_min, self.v_max, self.kappa_max, self.accel_max, self.decel_max,
                  self.kappa_rate_max]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("limits must be finite")
        if not self.v_min < self.v_max:
            raise ValueError("v_min must be below v_max")
        if not self.v_min <= 0.0 <= self.v_max:
            # The brake candidate and the stop/watchdog policy need v = 0 to be admissible.
            raise ValueError("the speed range must contain 0")
        for name in ("kappa_max", "accel_max", "decel_max", "kappa_rate_max"):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.lat_accel_max is not None and not (
            math.isfinite(self.lat_accel_max) and self.lat_accel_max > 0.0
        ):
            raise ValueError("lat_accel_max must be positive and finite, or None")


@dataclass(frozen=True)
class CostWeights:
    """MPC cost weights; each is 1 / (acceptable error)^2 in the stated unit.

    q_pos (1/m^2), q_psi (1/rad^2): stage tracking of position and heading.
    q_v (s^2/m^2): speed tracking against the reference speed (path progress).
    r_kappa (m^2): curvature effort about the reference curvature.
    s_dv (s^2/m^2), s_dkappa (m^2): per-sample control increments (including from u_prev).
    qf_pos (1/m^2), qf_psi (1/rad^2): terminal pose.
    """

    q_pos: float
    q_psi: float
    q_v: float
    r_kappa: float
    s_dv: float
    s_dkappa: float
    qf_pos: float
    qf_psi: float


# ----------------------------------------------------------------------------------------------
# Angles and steering geometry
# ----------------------------------------------------------------------------------------------


def wrap_angle(angle: Any) -> Any:
    """Wrap to [-pi, pi). Works on scalars (returns float) and arrays.

    Angles already in [-pi, pi) are returned bit-exactly, so wrapping is idempotent.
    """
    values = np.asarray(angle, dtype=np.float64)
    shifted = np.mod(values + np.pi, 2.0 * np.pi) - np.pi
    shifted = np.where(shifted >= np.pi, shifted - 2.0 * np.pi, shifted)  # rounding guard
    wrapped = np.where((values >= -np.pi) & (values < np.pi), values, shifted)
    return float(wrapped) if np.ndim(wrapped) == 0 else wrapped


def angle_error(psi: Any, psi_ref: Any) -> Any:
    """Heading residual wrap(psi - psi_ref) in [-pi, pi)."""
    return wrap_angle(np.asarray(psi, dtype=np.float64) - np.asarray(psi_ref, dtype=np.float64))


def unwrap_near(angle: Any, center: float) -> Any:
    """Shift angle by a multiple of 2 pi so it lies within [center - pi, center + pi)."""
    return center + angle_error(angle, center)


def kappa_from_delta(delta: Any, wheelbase: float) -> Any:
    """Bicycle curvature kappa = tan(delta) / L for |delta| < pi/2."""
    delta_array = np.asarray(delta, dtype=np.float64)
    if np.any(np.abs(delta_array) >= np.pi / 2):
        raise ValueError("|delta| must be below pi/2")
    result = np.tan(delta_array) / wheelbase
    return float(result) if np.ndim(result) == 0 else result


def delta_from_kappa(kappa: Any, wheelbase: float) -> Any:
    """Bicycle steering angle delta = atan(L kappa); the command-boundary inverse map."""
    result = np.arctan(wheelbase * np.asarray(kappa, dtype=np.float64))
    return float(result) if np.ndim(result) == 0 else result


def kappa_limit_from_steering(delta_max: float, wheelbase: float) -> float:
    """|kappa| <= tan(delta_max) / L, the curvature bound equivalent to |delta| <= delta_max."""
    return float(kappa_from_delta(delta_max, wheelbase))


def kappa_rate_limit_from_steering_rate(delta_rate_max: float, wheelbase: float) -> float:
    """A curvature-rate bound that guarantees |d delta / dt| <= delta_rate_max.

    d delta / d kappa = L / (1 + L^2 kappa^2) <= L, so |dkappa/dt| <= delta_rate_max / L implies
    the steering-rate limit at every curvature. It is exact at delta = 0 and conservative by
    1 + tan^2(delta) elsewhere (1.33 at 30 degrees).
    """
    return delta_rate_max / wheelbase


def wheel_steering_angles(kappa: float, wheelbase: float, front_track: float) -> tuple[float, float]:
    """Signed ideal-Ackermann steering angles (left, right) of the front wheels.

    A wheel at body position (L, y_w) rotating about the ICR (0, 1/kappa) must point along
    (1 - kappa y_w, L kappa), so tan(delta_w) = L kappa / (1 - kappa y_w) with y_w = +W_f/2
    (left) or -W_f/2 (right). This is the review's tan(delta) = L / (R -+ W_f/2) multiplied
    through by kappa, valid for either sign and finite at kappa = 0. For kappa > 0 (left turn)
    the left wheel is the inner one; for kappa < 0 the right wheel is. Requires |kappa| W_f/2 < 1
    (ICR outside the front track); otherwise the inner wheel would need |delta| >= 90 degrees.
    Steering-pivot offset (scrub radius) and steering-linkage error are neglected.
    """
    if not math.isfinite(kappa):
        raise ValueError("kappa must be finite")
    half_track = 0.5 * front_track
    if abs(kappa) * half_track >= 1.0:
        raise ValueError("|kappa| * W_f / 2 must be below 1 (ICR inside the front track)")
    left = math.atan2(wheelbase * kappa, 1.0 - kappa * half_track)
    right = math.atan2(wheelbase * kappa, 1.0 + kappa * half_track)
    return left, right


def inner_outer_angles(kappa: float, wheelbase: float, front_track: float) -> tuple[float, float]:
    """(inner, outer) signed wheel angles; the inner wheel is on the side of the ICR."""
    left, right = wheel_steering_angles(kappa, wheelbase, front_track)
    return (left, right) if kappa >= 0.0 else (right, left)


def rear_wheel_speeds(v: float, kappa: float, rear_track: float) -> tuple[float, float]:
    """No-slip rear wheel ground speeds (left, right): v (1 -+ kappa W_r / 2).

    Useful to check the model against rear wheel encoders during replay.
    """
    return v * (1.0 - 0.5 * kappa * rear_track), v * (1.0 + 0.5 * kappa * rear_track)


# ----------------------------------------------------------------------------------------------
# Continuous dynamics and discrete steps (control held constant over one sample)
# ----------------------------------------------------------------------------------------------


def _state_control(state: Any, control: Any) -> tuple[np.ndarray, np.ndarray]:
    state_array = np.asarray(state, dtype=np.float64)
    control_array = np.asarray(control, dtype=np.float64)
    if state_array.shape != (NX,):
        raise ValueError(f"state must have shape ({NX},), got {state_array.shape}")
    if control_array.shape != (NU,):
        raise ValueError(f"control must have shape ({NU},), got {control_array.shape}")
    if not (np.all(np.isfinite(state_array)) and np.all(np.isfinite(control_array))):
        raise ValueError("state and control must be finite")
    return state_array, control_array


def _check_ts(sample_time: float) -> None:
    if not (math.isfinite(sample_time) and sample_time > 0.0):
        raise ValueError("sample_time must be finite and positive")


def continuous_dynamics(state: Any, control: Any) -> np.ndarray:
    """[xdot, ydot, psidot] = [v cos psi, v sin psi, v kappa] about the rear-axle midpoint."""
    state_array, control_array = _state_control(state, control)
    psi = state_array[2]
    v, kappa = control_array
    return np.array([v * math.cos(psi), v * math.sin(psi), v * kappa])


def continuous_dynamics_delta(state: Any, v: float, delta: float, wheelbase: float) -> np.ndarray:
    """Same model with the steering angle as input: psidot = (v / L) tan(delta)."""
    return continuous_dynamics(state, [v, kappa_from_delta(delta, wheelbase)])


def _finish(x: float, y: float, psi: float, wrap: bool) -> np.ndarray:
    return np.array([x, y, wrap_angle(psi) if wrap else psi])


def step_exact_arc(
    state: Any, control: Any, sample_time: float, dpsi_eps: float = DPSI_EPS, wrap: bool = True
) -> np.ndarray:
    """Exact arc update in the review's form, with the straight-line limit for |dpsi| <= dpsi_eps.

    x+ = x + (sin(psi + dpsi) - sin(psi)) / kappa,  y+ = y + (cos(psi) - cos(psi + dpsi)) / kappa,
    dpsi = v kappa Ts. The threshold is on |dpsi|, which also covers v = 0 for any kappa.
    """
    _check_ts(sample_time)
    state_array, control_array = _state_control(state, control)
    x, y, psi = state_array
    v, kappa = control_array
    dpsi = v * kappa * sample_time
    if abs(dpsi) <= dpsi_eps:
        distance = v * sample_time
        return _finish(x + distance * math.cos(psi), y + distance * math.sin(psi), psi + dpsi, wrap)
    return _finish(
        x + (math.sin(psi + dpsi) - math.sin(psi)) / kappa,
        y + (math.cos(psi) - math.cos(psi + dpsi)) / kappa,
        psi + dpsi,
        wrap,
    )


def sinc(a: float) -> float:
    """sin(a) / a with sinc(0) = 1, accurate for all finite a (no cancellation)."""
    if abs(a) < _SINC_SERIES_LIMIT:
        a2 = a * a
        return 1.0 - a2 / 6.0 + a2 * a2 / 120.0
    return math.sin(a) / a


def step_exact_arc_chord(state: Any, control: Any, sample_time: float, wrap: bool = True) -> np.ndarray:
    """The same exact arc, written as a chord: no division by kappa and no near-zero branch.

    sin(psi + dpsi) - sin(psi) = 2 sin(dpsi/2) cos(psi + dpsi/2), so with s = v Ts:
    x+ = x + s sinc(dpsi/2) cos(psi + dpsi/2),  y+ = y + s sinc(dpsi/2) sin(psi + dpsi/2).
    This is the form proposed for hardware (one sin/cos pair, one short polynomial, no divide).
    """
    _check_ts(sample_time)
    state_array, control_array = _state_control(state, control)
    x, y, psi = state_array
    v, kappa = control_array
    distance = v * sample_time
    dpsi = distance * kappa
    half = 0.5 * dpsi
    chord = distance * sinc(half)
    return _finish(x + chord * math.cos(psi + half), y + chord * math.sin(psi + half), psi + dpsi, wrap)


def step_euler(state: Any, control: Any, sample_time: float, wrap: bool = True) -> np.ndarray:
    _check_ts(sample_time)
    state_array, _ = _state_control(state, control)
    result = state_array + sample_time * continuous_dynamics(state_array, control)
    return _finish(result[0], result[1], result[2], wrap)


def step_midpoint(state: Any, control: Any, sample_time: float, wrap: bool = True) -> np.ndarray:
    """Explicit midpoint (RK2): k2 = f(x + Ts/2 k1), x+ = x + Ts k2."""
    _check_ts(sample_time)
    state_array, _ = _state_control(state, control)
    k1 = continuous_dynamics(state_array, control)
    k2 = continuous_dynamics(state_array + 0.5 * sample_time * k1, control)
    result = state_array + sample_time * k2
    return _finish(result[0], result[1], result[2], wrap)


def step_rk4(state: Any, control: Any, sample_time: float, wrap: bool = True) -> np.ndarray:
    """Classical RK4, the scheme of System::one_step_prediction in hls_system.hpp."""
    _check_ts(sample_time)
    state_array, _ = _state_control(state, control)
    k1 = continuous_dynamics(state_array, control)
    k2 = continuous_dynamics(state_array + 0.5 * sample_time * k1, control)
    k3 = continuous_dynamics(state_array + 0.5 * sample_time * k2, control)
    k4 = continuous_dynamics(state_array + sample_time * k3, control)
    result = state_array + sample_time / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    return _finish(result[0], result[1], result[2], wrap)


STEP_METHODS: dict[str, Callable[..., np.ndarray]] = {
    "arc": step_exact_arc,
    "chord": step_exact_arc_chord,
    "rk4": step_rk4,
    "midpoint": step_midpoint,
    "euler": step_euler,
}


def step(state: Any, control: Any, sample_time: float, method: str = "chord", wrap: bool = True) -> np.ndarray:
    if method not in STEP_METHODS:
        raise ValueError(f"unknown method {method!r}; choose from {sorted(STEP_METHODS)}")
    return STEP_METHODS[method](state, control, sample_time, wrap=wrap)


def expand_horizon(controls: Any, horizon: int) -> np.ndarray:
    """Nu x 2 control-horizon sequence -> Nh x 2 with the last input held (Nu <= Nh).

    Matches uu_loop/get_next_uu in hls_system.hpp.
    """
    control_array = np.asarray(controls, dtype=np.float64).reshape(-1, NU)
    if not 1 <= len(control_array) <= horizon:
        raise ValueError("need 1 <= Nu <= Nh")
    tail = np.repeat(control_array[-1:], horizon - len(control_array), axis=0)
    return np.vstack([control_array, tail])


def rollout(
    state0: Any, controls: Any, sample_time: float, horizon: int | None = None,
    method: str = "chord", wrap: bool = True,
) -> np.ndarray:
    """Predicted states, shape (Nh + 1, 3); row 0 is state0. Tail beyond Nu holds the last input."""
    control_array = np.asarray(controls, dtype=np.float64).reshape(-1, NU)
    full = expand_horizon(control_array, len(control_array) if horizon is None else horizon)
    states = np.empty((len(full) + 1, NX))
    states[0] = np.asarray(state0, dtype=np.float64)
    for index, control in enumerate(full):
        states[index + 1] = step(states[index], control, sample_time, method, wrap)
    return states


# ----------------------------------------------------------------------------------------------
# Constraints, projection and candidate sequences
# ----------------------------------------------------------------------------------------------


def check_controls(
    controls: Any, u_prev: Any, limits: AckermannLimits, sample_time: float,
    geometry: AckermannGeometry | None = None, delta_max: float | None = None, tol: float = 1e-9,
) -> dict[str, Any]:
    """Largest violation of each control constraint (0 = satisfied), reported separately.

    Increments include the step from u_prev (the command applied in the previous period) to
    controls[0]. The steering-angle check runs only when geometry and delta_max are given.
    """
    _check_ts(sample_time)
    sequence = np.asarray(controls, dtype=np.float64).reshape(-1, NU)
    previous = np.asarray(u_prev, dtype=np.float64).reshape(NU)
    v, kappa = sequence[:, 0], sequence[:, 1]
    dv = np.diff(np.concatenate([[previous[0]], v]))
    dkappa = np.diff(np.concatenate([[previous[1]], kappa]))
    violations = {
        "speed": float(max(0.0, np.max(v - limits.v_max), np.max(limits.v_min - v))),
        "kappa": float(max(0.0, np.max(np.abs(kappa) - limits.kappa_max))),
        "accel": float(max(0.0, np.max(dv / sample_time - limits.accel_max))),
        "decel": float(max(0.0, np.max(-dv / sample_time - limits.decel_max))),
        "kappa_rate": float(max(0.0, np.max(np.abs(dkappa) / sample_time - limits.kappa_rate_max))),
        "lat_accel": None if limits.lat_accel_max is None else float(
            max(0.0, np.max(v * v * np.abs(kappa) - limits.lat_accel_max))),
        "steering": None,
        "finite": bool(np.all(np.isfinite(sequence))),
    }
    if geometry is not None and delta_max is not None:
        delta = np.abs(np.arctan(geometry.wheelbase * kappa))
        violations["steering"] = float(max(0.0, np.max(delta - delta_max)))
    amounts = [value for key, value in violations.items() if key != "finite" and value is not None]
    violations["ok"] = violations["finite"] and all(amount <= tol for amount in amounts)
    return violations


def _slew_steps(limits: AckermannLimits, sample_time: float) -> tuple[float, float, float]:
    return limits.accel_max * sample_time, limits.decel_max * sample_time, limits.kappa_rate_max * sample_time


def project_controls(
    raw_controls: Any, u_prev: Any, limits: AckermannLimits, sample_time: float
) -> np.ndarray:
    """Sequential projection onto the feasible set; the result always passes check_controls.

    Stage by stage, v is clamped to the box, the slew window from the previous stage and, if a
    lateral limit is set, |v| <= sqrt(a_y / max(|kappa_prev| - dkappa, 0)). That cap keeps the
    curvature window below non-empty. kappa is then clamped to the intersection of the box, the
    slew window and |kappa| <= a_y / v^2. All three intervals pairwise intersect, so (Helly, 1-D)
    the intersection is non-empty whenever u_prev is feasible. u_prev must be feasible.
    """
    _check_ts(sample_time)
    raw = np.asarray(raw_controls, dtype=np.float64).reshape(-1, NU)
    previous = np.asarray(u_prev, dtype=np.float64).reshape(NU)
    if not check_controls(previous[None, :], previous, limits, sample_time)["ok"]:
        raise ValueError("u_prev violates the limits; use a brake/stop fallback instead")
    dv_up, dv_down, dkappa = _slew_steps(limits, sample_time)
    lateral = limits.lat_accel_max
    result = np.empty_like(raw)
    v_prev, kappa_prev = previous
    for index, (v_raw, kappa_raw) in enumerate(np.nan_to_num(raw, nan=0.0)):
        v_low = max(limits.v_min, v_prev - dv_down)
        v_high = min(limits.v_max, v_prev + dv_up)
        if lateral is not None:
            reachable = abs(kappa_prev) - dkappa
            if reachable > 0.0:
                cap = math.sqrt(lateral / reachable)
                v_low, v_high = max(v_low, -cap), min(v_high, cap)
        v = min(max(v_raw, v_low), v_high)
        k_low = max(-limits.kappa_max, kappa_prev - dkappa)
        k_high = min(limits.kappa_max, kappa_prev + dkappa)
        if lateral is not None and v != 0.0:
            k_cap = lateral / (v * v)
            k_low, k_high = max(k_low, -k_cap), min(k_high, k_cap)
        if k_low > k_high:  # only by rounding at the lateral cap
            k_low = k_high = 0.5 * (k_low + k_high)
        kappa = min(max(kappa_raw, k_low), k_high)
        result[index] = v, kappa
        v_prev, kappa_prev = v, kappa
    return result


def shift_warm_start(previous_solution: Any) -> np.ndarray:
    """Drop the applied first input and repeat the last: [u1 .. u_{Nu-1}, u_{Nu-1}].

    If the previous sequence was feasible from its own u_prev and its first input was the one
    applied (the new u_prev), the shifted sequence is feasible without projection.
    """
    sequence = np.asarray(previous_solution, dtype=np.float64).reshape(-1, NU)
    return np.vstack([sequence[1:], sequence[-1:]])


def hold_candidate(u_prev: Any, control_horizon: int) -> np.ndarray:
    """Hold the current command; feasible whenever u_prev is (zero increments)."""
    return np.repeat(np.asarray(u_prev, dtype=np.float64).reshape(1, NU), control_horizon, axis=0)


def brake_candidate(u_prev: Any, control_horizon: int, limits: AckermannLimits, sample_time: float) -> np.ndarray:
    """Ramp v to 0 at the decel (or accel, when reversing) limit with kappa held.

    Feasible whenever u_prev is: |v| only shrinks, so v^2 |kappa| only shrinks, and kappa does
    not change. It stops on the current circle; it is not a collision-free guarantee.
    """
    previous = np.asarray(u_prev, dtype=np.float64).reshape(NU)
    dv_up, dv_down, _ = _slew_steps(limits, sample_time)
    result = np.empty((control_horizon, NU))
    v = previous[0]
    for index in range(control_horizon):
        v = max(v - dv_down, 0.0) if v > 0.0 else min(v + dv_up, 0.0)
        result[index] = v, previous[1]
    return result


def feedforward_from_reference(reference_states: Any, state0: Any, sample_time: float) -> np.ndarray:
    """Constant-arc controls that move exactly through consecutive reference poses.

    Inverts the chord step: dpsi = wrap(psi_{k+1} - psi_k), chord c = |p_{k+1} - p_k|,
    s = c / sinc(dpsi/2), v = +-s / Ts (sign from the chord direction relative to psi_k +
    dpsi/2), kappa = dpsi / (v Ts) (0 when s = 0). Only exact if the reference poses are
    themselves arc-consistent; project the result before use.
    """
    _check_ts(sample_time)
    poses = np.vstack([np.asarray(state0, dtype=np.float64).reshape(1, NX),
                       np.asarray(reference_states, dtype=np.float64).reshape(-1, NX)])
    controls = np.zeros((len(poses) - 1, NU))
    for index in range(len(poses) - 1):
        start, end = poses[index], poses[index + 1]
        dpsi = angle_error(end[2], start[2])
        delta_position = end[:2] - start[:2]
        chord = float(np.hypot(*delta_position))
        if chord == 0.0:
            continue
        direction = start[2] + 0.5 * dpsi
        along = delta_position[0] * math.cos(direction) + delta_position[1] * math.sin(direction)
        arc = chord / sinc(0.5 * dpsi)
        signed_arc = arc if along >= 0.0 else -arc
        controls[index] = signed_arc / sample_time, dpsi / signed_arc
    return controls


# ----------------------------------------------------------------------------------------------
# Normalized decision variables for PSO
# ----------------------------------------------------------------------------------------------


def normalize_controls(controls: Any, limits: AckermannLimits, scale: float = 1.0) -> np.ndarray:
    """Physical [v, kappa] -> [-scale, scale] per dimension (scale = 100 matches the quadrotor ABI)."""
    sequence = np.asarray(controls, dtype=np.float64).reshape(-1, NU)
    center = 0.5 * (limits.v_max + limits.v_min)
    half_range = 0.5 * (limits.v_max - limits.v_min)
    return np.column_stack([(sequence[:, 0] - center) / half_range,
                            sequence[:, 1] / limits.kappa_max]) * scale


def denormalize_controls(normalized: Any, limits: AckermannLimits, scale: float = 1.0) -> np.ndarray:
    sequence = np.asarray(normalized, dtype=np.float64).reshape(-1, NU) / scale
    center = 0.5 * (limits.v_max + limits.v_min)
    half_range = 0.5 * (limits.v_max - limits.v_min)
    return np.column_stack([center + half_range * sequence[:, 0], limits.kappa_max * sequence[:, 1]])


def normalized_slew_limits(limits: AckermannLimits, sample_time: float, scale: float = 1.0) -> np.ndarray:
    """Symmetric per-sample increment bounds in normalized units, for the HLS du_max box.

    The HLS projection is symmetric, so v uses min(accel, decel), which is conservative.
    """
    dv_up, dv_down, dkappa = _slew_steps(limits, sample_time)
    half_range = 0.5 * (limits.v_max - limits.v_min)
    return np.array([min(dv_up, dv_down) / half_range, dkappa / limits.kappa_max]) * scale


# ----------------------------------------------------------------------------------------------
# Cost (constraint violations are reported by check_controls, not folded in)
# ----------------------------------------------------------------------------------------------


def mpc_cost(
    states: Any, controls: Any, u_prev: Any, reference_states: Any, weights: CostWeights,
    reference_controls: Any | None = None,
) -> dict[str, float]:
    """Dimensionless cost of a predicted trajectory, split into its terms.

    states: (Nh + 1) x 3 from rollout (row 0 = current state, not penalized).
    controls: Nh x 2 (already expanded with the held tail).
    reference_states: Nh x 3 references for states[1:]; reference_controls: Nh x 2 or None (zero).
    Stage k = 1..Nh-1 uses q_*; stage Nh uses qf_* (as hls_system.hpp uses Qf at i = N - 1).
    """
    predicted = np.asarray(states, dtype=np.float64)[1:]
    sequence = np.asarray(controls, dtype=np.float64).reshape(-1, NU)
    reference = np.asarray(reference_states, dtype=np.float64).reshape(-1, NX)
    if not (len(predicted) == len(sequence) == len(reference)):
        raise ValueError("states[1:], controls and reference_states must have Nh rows")
    reference_u = np.zeros_like(sequence) if reference_controls is None else np.asarray(
        reference_controls, dtype=np.float64).reshape(-1, NU)
    position_sq = np.sum((predicted[:, :2] - reference[:, :2]) ** 2, axis=1)
    heading_sq = angle_error(predicted[:, 2], reference[:, 2]) ** 2
    increments = np.diff(np.vstack([np.asarray(u_prev, dtype=np.float64).reshape(1, NU), sequence]), axis=0)
    terms = {
        "position": float(weights.q_pos * np.sum(position_sq[:-1])),
        "heading": float(weights.q_psi * np.sum(heading_sq[:-1])),
        "speed": float(weights.q_v * np.sum((sequence[:, 0] - reference_u[:, 0]) ** 2)),
        "curvature": float(weights.r_kappa * np.sum((sequence[:, 1] - reference_u[:, 1]) ** 2)),
        "increments": float(np.sum(weights.s_dv * increments[:, 0] ** 2
                                   + weights.s_dkappa * increments[:, 1] ** 2)),
        "terminal": float(weights.qf_pos * position_sq[-1] + weights.qf_psi * heading_sq[-1]),
    }
    terms["total"] = float(sum(terms.values()))
    return terms
