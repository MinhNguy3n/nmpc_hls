"""Double-precision reference model of a unicycle / skid-steer / differential-drive robot for MPC-PSO.

Independent software reference for docs/husky_a300/unicycle_mpc_pso_model.md. Targets: Clearpath
Husky A300 (4-wheel skid steer); the TurtleBot 4 (iRobot Create 3, differential drive) parameter
set is kept for a later, separate scope. It is not wired into the HLS solver, the overlay, ROS or
any robot, and it sends no commands.

Exact reference vs hardware approximation: step_chord is the exact zero-order-hold step; its chord
factor sinc(a) = sin(a)/a is evaluated with a division (or its series near 0). The hardware
variants replace it with the division-free polynomial sinc_poly2(a) = 1 - a^2/6, which underestimates it:
0 <= sinc(a) - sinc_poly2(a) <= a^4/120 (step_chord_poly reproduces that approximation in double).

Reuse and attribution: the chord (exact-arc) step, the near-zero analysis, the warm-start /
fallback candidates, the per-dimension normalization, the separate constraint report and the cost
split are adapted from the Ackermann proposal (app/ackermann/ackermann_model.py in worktree
agent-a3984fccbf29de483, 2026-10-25). The Ackermann steering geometry, steering and curvature-rate
limits and the lateral-acceleration projection are dropped; the wheel-side (skid-steer) limits are
new.

Conventions (REP-103 / ISO 8855, z up; see the design note section 2):
- odom frame: right-handed, x and y in metres, z up. Heading psi (rad) from +x, counter-clockwise
  positive. Wrapped headings lie in (-pi, pi], the range of the HLS cost residual (commit 8c61f1b)
  and of ROS angles::normalize_angle.
- body frame (base_link) at the robot centre: x forward, y left. For the A300 diff_4wd URDF the
  chassis is centred on base_link; for the Create 3 base_link is "at the center of rotation".
- state [x, y, psi]; control [v, omega] = TwistStamped twist.linear.x (m/s), twist.angular.z (rad/s).
- wheel sides: v_left = v - omega*W/2, v_right = v + omega*W/2 (W = track used for the mapping).

Every number below that describes a robot is either taken from a cited official source (see
PARAMETER_SETS) or is None and labelled PLACEHOLDER_ (query on the robot / identify by experiment).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Callable

import numpy as np


NX = 3  # [x (m), y (m), psi (rad)]
NU = 2  # [v (m/s), omega (rad/s)]

# The chord factor sinc(a) = sin(a)/a uses its Taylor series below this |a|; the truncation
# |a|^6/5040 < 2e-28 is far below double rounding (same choice as the Ackermann reference).
_SINC_SERIES_LIMIT = 1e-4

# Near-zero branch of the literal arc form (step_exact_arc_literal), on |dpsi| = |omega*Ts| (rad).
# The straight-line branch errs by ~|s||dpsi|/2, the division branch by ~|s|*eps/|dpsi|; they balance
# near 2e-8 (Ackermann analysis, measured jump 1.1e-8*|s|). Only the reference uses this form: the
# chord form has no branch and is the one proposed for hardware.
DPSI_EPS = 2e-8


# ----------------------------------------------------------------------------------------------
# Sourced parameter sets
# ----------------------------------------------------------------------------------------------

A300_MANUAL = "https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/husky/a300/user_manual_husky/"
A300_SPEC = "https://clearpathrobotics.com/husky-spec-comparison/"
A300_CONTROL_YAML = ("https://github.com/clearpathrobotics/clearpath_common/blob/jazzy/"
                     "clearpath_control/config/a300/control/diff_4wd.yaml")
A300_TELEOP_YAML = ("https://github.com/clearpathrobotics/clearpath_common/blob/jazzy/"
                    "clearpath_control/config/a300/teleop_joy.yaml")
A300_URDF = ("https://github.com/clearpathrobotics/clearpath_common/blob/jazzy/"
             "clearpath_platform_description/urdf/a300/a300.urdf.xacro")
CLEARPATH_TWIST_MUX = ("https://github.com/clearpathrobotics/clearpath_common/blob/jazzy/"
                       "clearpath_control/config/twist_mux.yaml")
CLEARPATH_DRIVING = "https://docs.clearpathrobotics.com/docs/ros/tutorials/driving"
CLEARPATH_PLATFORM_API = "https://docs.clearpathrobotics.com/docs/ros/api/platform_api"
TB4_FEATURES = "https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html"
CREATE3_SAFETY = "https://iroboteducation.github.io/create3_docs/api/safety/"
CREATE3_ROS2 = "https://iroboteducation.github.io/create3_docs/api/ros2/"
CREATE3_ODOMETRY = "https://iroboteducation.github.io/create3_docs/api/odometry/"
CREATE3_MOVING = "https://iroboteducation.github.io/create3_docs/api/moving-the-robot/"
CREATE3_SIM_CONTROL = ("https://github.com/iRobotEducation/create3_sim/blob/jazzy/irobot_create_common/"
                       "irobot_create_control/config/control.yaml")
CREATE3_SIM_URDF = ("https://github.com/iRobotEducation/create3_sim/blob/jazzy/irobot_create_common/"
                    "irobot_create_description/urdf/create3.urdf.xacro")

PLACEHOLDER_QUERY = "PLACEHOLDER_: query on the robot"
PLACEHOLDER_IDENTIFY = "PLACEHOLDER_: identify by experiment"


@dataclass(frozen=True)
class Sourced:
    """A robot parameter with its unit and source. value None means PLACEHOLDER_ (unknown)."""

    value: float | None
    unit: str
    source: str

    @property
    def known(self) -> bool:
        return self.value is not None


PARAMETER_SETS: dict[str, dict[str, Sourced]] = {
    # All values fetched 2026-10-25. Configuration files are the jazzy-branch defaults; the robot's
    # generated configuration may override them, so each one is also "verify on the robot".
    "husky_a300": {
        "wheelbase": Sourced(0.512, "m", A300_MANUAL + " (Technical Specifications); URDF wheel_base"),
        "track_spec": Sourced(0.566, "m", A300_MANUAL + " (Technical Specifications, Track Width)"),
        "track_kinematic": Sourced(0.562, "m", A300_MANUAL + " (Robot Equations: effective track W, "
                                   "'as a starting point'); diff_4wd.yaml wheel_separation; URDF wheel_track"),
        # Jazzy default of clearpath_common, NOT confirmed on the target robot: gate G0 of the plan
        # (read the robot's generated controller parameters before any physical test).
        "wheel_separation_multiplier": Sourced(1.75, "-", A300_CONTROL_YAML + " (Jazzy default; "
                                               "unconfirmed on the target robot, gate G0)"),
        "tire_radius_effective": Sourced(0.1625, "m", A300_MANUAL + " (Effective Tire Radius); diff_4wd.yaml"),
        "v_max_platform": Sourced(2.0, "m/s", A300_MANUAL + "; " + A300_SPEC),
        "v_max_offroad_no_payload": Sourced(1.5, "m/s", A300_MANUAL + " (Important System Limits)"),
        "v_max_offroad_payload_50kg": Sourced(1.0, "m/s", A300_MANUAL + " (Important System Limits)"),
        "v_teleop_slow_manual": Sourced(0.3, "m/s", A300_MANUAL + " (Connecting and Using the Controller)"),
        "v_teleop_normal_config": Sourced(0.4, "m/s", A300_TELEOP_YAML + " scale_linear.x"),
        "omega_teleop_normal_config": Sourced(0.75, "rad/s", A300_TELEOP_YAML + " scale_angular.yaw"),
        "v_teleop_turbo_config": Sourced(1.5, "m/s", A300_TELEOP_YAML + " scale_linear_turbo.x"),
        "controller_v_max": Sourced(2.0, "m/s", A300_CONTROL_YAML + " linear.x.max_velocity"),
        "controller_accel_max": Sourced(2.0, "m/s^2", A300_CONTROL_YAML + " linear.x.max_acceleration"),
        "controller_decel_max": Sourced(4.0, "m/s^2", A300_CONTROL_YAML + " linear.x.max_deceleration (-4.0)"),
        "controller_omega_max": Sourced(2.0, "rad/s", A300_CONTROL_YAML + " angular.z.max_velocity"),
        "controller_alpha_accel_max": Sourced(4.0, "rad/s^2", A300_CONTROL_YAML + " angular.z.max_acceleration"),
        "controller_alpha_decel_max": Sourced(4.0, "rad/s^2", A300_CONTROL_YAML + " angular.z.max_deceleration"),
        "controller_rate": Sourced(50.0, "Hz", A300_CONTROL_YAML + " update_rate, publish_rate"),
        "cmd_vel_timeout": Sourced(0.5, "s", A300_CONTROL_YAML + " cmd_vel_timeout; " + CLEARPATH_TWIST_MUX
                                   + " (external cmd_vel timeout 0.5, priority 1); " + CLEARPATH_DRIVING),
        "side_speed_max": Sourced(None, "m/s", PLACEHOLDER_IDENTIFY + " (motor/driver limit per side; not documented)"),
        "side_accel_max": Sourced(None, "m/s^2", PLACEHOLDER_IDENTIFY + " (not documented)"),
        "track_effective_icr": Sourced(None, "m", PLACEHOLDER_IDENTIFY + " (surface- and load-dependent)"),
        "actuator_delay": Sourced(None, "s", PLACEHOLDER_IDENTIFY),
    },
    "turtlebot4_create3": {
        "track_kinematic": Sourced(0.233, "m", CREATE3_SIM_CONTROL + " wheel_separation; " + CREATE3_SIM_URDF
                                   + " distance_between_wheels (simulation model). Real robot: "
                                   + PLACEHOLDER_QUERY + " /static_transform wheel_base (" + CREATE3_ROS2 + ")"),
        "wheel_radius": Sourced(0.03575, "m", CREATE3_SIM_CONTROL + " (simulation); TB4 manual: wheel diameter "
                                "72 mm (" + TB4_FEATURES + "). Real robot: " + PLACEHOLDER_QUERY
                                + " /static_transform wheels_radius"),
        "v_max_safety_on": Sourced(0.306, "m/s", CREATE3_SAFETY + " (safety_override none/backup_only)"),
        "v_max_safety_off": Sourced(0.46, "m/s", CREATE3_SAFETY + " (safety_override full; not for MPC use)"),
        "omega_max": Sourced(1.90, "rad/s", TB4_FEATURES + " (Maximum angular velocity)"),
        "wheel_accel_limit": Sourced(0.9, "m/s^2", CREATE3_SAFETY + "; " + CREATE3_ROS2
                                     + " (motion_control wheel_accel_limit, default 900 mm/s^2, range 1..900)"),
        "sim_alpha_max": Sourced(7.725, "rad/s^2", CREATE3_SIM_CONTROL + " angular.z.max_acceleration "
                                 "(= 2*0.9/0.233, simulation)"),
        "odom_rate": Sourced(20.0, "Hz", CREATE3_ODOMETRY),
        "wheel_topics_rate": Sourced(62.5, "Hz", CREATE3_ODOMETRY),
        "cmd_vel_timeout_sim": Sourced(0.5, "s", CREATE3_SIM_CONTROL + " cmd_vel_timeout (simulation)"),
        "cmd_vel_timeout_robot": Sourced(None, "s", PLACEHOLDER_IDENTIFY + " (" + CREATE3_MOVING
                                         + ": 'a fixed timeout period', value not stated)"),
        "side_speed_max": Sourced(None, "m/s", PLACEHOLDER_IDENTIFY + " (whether 0.306 m/s bounds the body or "
                                  "each wheel is not stated)"),
        "actuator_delay": Sourced(None, "s", PLACEHOLDER_IDENTIFY),
    },
}


def parameter(robot: str, name: str) -> Sourced:
    return PARAMETER_SETS[robot][name]


# ----------------------------------------------------------------------------------------------
# Limits used by the MPC (an operating envelope chosen inside the robot's documented limits)
# ----------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UnicycleLimits:
    """Control limits in physical units, all magnitudes positive unless stated.

    v_min, v_max: signed speed bounds (m/s); 0 must be inside (stop is always admissible).
        v_min = 0 means forward only (recommended for the Create 3: rear cliffs, backup limit).
    omega_max: |omega| bound (rad/s).
    accel_max / decel_max: largest rate at which |v| may grow / shrink (m/s^2).
    alpha_accel_max / alpha_decel_max: the same for |omega| (rad/s^2).
    track: W used to map (v, omega) to wheel-side speeds (m). For the A300 this must be the
        controller's wheel_separation * wheel_separation_multiplier if the side limits are meant
        as the commanded wheel speeds (see the design note, section 4).
    side_speed_max: optional |v +- omega W/2| bound (m/s); None = not enforced.
    side_accel_max: optional |d(v +- omega W/2)/dt| bound (m/s^2); None = not enforced.
    """

    v_min: float
    v_max: float
    omega_max: float
    accel_max: float
    decel_max: float
    alpha_accel_max: float
    alpha_decel_max: float
    track: float
    side_speed_max: float | None = None
    side_accel_max: float | None = None

    def __post_init__(self) -> None:
        values = [self.v_min, self.v_max, self.omega_max, self.accel_max, self.decel_max,
                  self.alpha_accel_max, self.alpha_decel_max, self.track]
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
            raise ValueError("limits must be finite numbers (unknowns must be supplied explicitly)")
        if not self.v_min <= 0.0 <= self.v_max or not self.v_min < self.v_max:
            raise ValueError("need v_min <= 0 <= v_max and v_min < v_max")
        for name in ("omega_max", "accel_max", "decel_max", "alpha_accel_max", "alpha_decel_max", "track"):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be positive")
        for name in ("side_speed_max", "side_accel_max"):
            value = getattr(self, name)
            if value is not None and not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"{name} must be positive and finite, or None")

    @property
    def half_track(self) -> float:
        return 0.5 * self.track


def limits_from_parameter_set(robot: str, **envelope: float | None) -> UnicycleLimits:
    """Build limits from the documented controller limits, overridden by an operating envelope.

    Documented values fill what the caller leaves out; anything unknown (None) must be passed
    explicitly, which forces the caller to label it (tests pass PLACEHOLDER_ constants).
    """
    p = PARAMETER_SETS[robot]
    if robot == "husky_a300":
        defaults: dict[str, float | None] = {
            "v_min": -p["controller_v_max"].value,
            "v_max": p["controller_v_max"].value,
            "omega_max": p["controller_omega_max"].value,
            "accel_max": p["controller_accel_max"].value,
            "decel_max": p["controller_decel_max"].value,
            "alpha_accel_max": p["controller_alpha_accel_max"].value,
            "alpha_decel_max": p["controller_alpha_decel_max"].value,
            # The diff_drive_controller converts omega to wheel speeds with the multiplied separation.
            # 1.75 is the unconfirmed Jazzy default; pass track=... once the robot's value is read.
            "track": p["track_kinematic"].value * p["wheel_separation_multiplier"].value,
            "side_speed_max": None,
            "side_accel_max": None,
        }
    elif robot == "turtlebot4_create3":
        accel = p["wheel_accel_limit"].value
        track = p["track_kinematic"].value
        defaults = {
            "v_min": 0.0,  # forward only: the Create 3 cliff sensors face forward (safety page)
            "v_max": p["v_max_safety_on"].value,
            "omega_max": p["omega_max"].value,
            # The Create 3 ramps each wheel at wheel_accel_limit, so the body limits follow from the
            # per-side limit; they are kept as boxes too, and the side limit is the binding one.
            "accel_max": accel,
            "decel_max": accel,
            "alpha_accel_max": 2.0 * accel / track,
            "alpha_decel_max": 2.0 * accel / track,
            "track": track,
            "side_speed_max": None,
            "side_accel_max": accel,
        }
    else:
        raise KeyError(robot)
    defaults.update(envelope)
    missing = [name for name, value in defaults.items()
               if value is None and name not in ("side_speed_max", "side_accel_max")]
    if missing:
        raise ValueError(f"unknown parameters must be supplied: {missing}")
    return UnicycleLimits(**defaults)  # type: ignore[arg-type]


@dataclass(frozen=True)
class CostWeights:
    """Cost weights, each 1/(acceptable error)^2 in the stated unit (cost is dimensionless).

    q_pos (1/m^2), q_psi (1/rad^2): stage pose tracking, k = 1..Nh-1.
    qf_pos (1/m^2), qf_psi (1/rad^2): terminal pose, k = Nh.
    q_v (s^2/m^2), r_omega (s^2/rad^2): speed / yaw-rate deviation from the reference controls.
    s_dv (s^2/m^2), s_domega (s^2/rad^2): per-sample increments, including from u_prev.
    """

    q_pos: float
    q_psi: float
    qf_pos: float
    qf_psi: float
    q_v: float
    r_omega: float
    s_dv: float = 0.0
    s_domega: float = 0.0


# ----------------------------------------------------------------------------------------------
# Angles
# ----------------------------------------------------------------------------------------------


def wrap_angle(angle: Any) -> Any:
    """Wrap to (-pi, pi]; values already in range are returned bit-exactly (idempotent)."""
    values = np.asarray(angle, dtype=np.float64)
    wrapped = np.pi - np.mod(np.pi - values, 2.0 * np.pi)
    wrapped = np.where(wrapped <= -np.pi, wrapped + 2.0 * np.pi, wrapped)  # rounding guard
    result = np.where((values > -np.pi) & (values <= np.pi), values, wrapped)
    return float(result) if np.ndim(result) == 0 else result


def angle_error(psi: Any, psi_ref: Any) -> Any:
    """Heading residual wrap(psi - psi_ref) in (-pi, pi]."""
    return wrap_angle(np.asarray(psi, dtype=np.float64) - np.asarray(psi_ref, dtype=np.float64))


def hls_angle_residual(x_hat: float, x_ref: float) -> float:
    """The residual of System::one_step_error for controlled_state == 2 (commit 8c61f1b).

    e = x_hat - x_ref, then one conditional -/+ 2 pi into (-pi, pi]. Equal to angle_error only
    while |x_hat - x_ref| < 3 pi; the caller must keep inside that contract.
    """
    error = x_hat - x_ref
    if error > math.pi:
        error -= 2.0 * math.pi
    elif error <= -math.pi:
        error += 2.0 * math.pi
    return error


def unwrap_near(angle: Any, center: float) -> Any:
    """Shift angle by a multiple of 2 pi into (center - pi, center + pi]."""
    return center + angle_error(angle, center)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about +z) of a nav_msgs/Odometry orientation, in (-pi, pi]."""
    return wrap_angle(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


# ----------------------------------------------------------------------------------------------
# Continuous model and discrete steps (control held constant over Ts)
# ----------------------------------------------------------------------------------------------


def _as_state_control(state: Any, control: Any) -> tuple[np.ndarray, np.ndarray]:
    s = np.asarray(state, dtype=np.float64)
    u = np.asarray(control, dtype=np.float64)
    if s.shape != (NX,) or u.shape != (NU,):
        raise ValueError(f"state must be ({NX},) and control ({NU},), got {s.shape}, {u.shape}")
    if not (np.all(np.isfinite(s)) and np.all(np.isfinite(u))):
        raise ValueError("state and control must be finite")
    return s, u


def _check_ts(ts: float) -> None:
    if not (math.isfinite(ts) and ts > 0.0):
        raise ValueError("sample time must be finite and positive")


def continuous_dynamics(state: Any, control: Any) -> np.ndarray:
    """[xdot, ydot, psidot] = [v cos psi, v sin psi, omega]."""
    s, u = _as_state_control(state, control)
    return np.array([u[0] * math.cos(s[2]), u[0] * math.sin(s[2]), u[1]])


def sinc(a: float) -> float:
    """Exact sin(a)/a with sinc(0) = 1 (a division above |a| = 1e-4, its series below)."""
    if abs(a) < _SINC_SERIES_LIMIT:
        a2 = a * a
        return 1.0 - a2 / 6.0 + a2 * a2 / 120.0
    return math.sin(a) / a


# Intended range of the hardware approximation: |omega Ts| <= 0.1 rad (A300 controller limit
# 2 rad/s at Ts = 0.05 s; Create 3 1.9 rad/s gives 0.095), i.e. |a| <= 0.05.
SINC_POLY_INTENDED_ABS_DPSI = 0.1


def sinc_poly2(a: float) -> float:
    """Division-free hardware approximation 1 - a^2/6 of sinc(a) (used by HLS variants B1-B3)."""
    return 1.0 - a * a / 6.0


def sinc_poly2_error_bound(a: float) -> float:
    """Bound on sinc(a) - sinc_poly2(a) (alternating Taylor series, valid for |a| <= sqrt(20)).

    The error lies in [0, a^4/120]: the polynomial slightly underestimates the chord length. For |a| <= 0.05 (|omega Ts| <= 0.1) it is <= 5.3e-8 relative to
    the chord length, i.e. <= 5.3e-9 m per step at |v Ts| <= 0.1 m (A300 at 2 m/s). It stays below
    the half-precision spacing near 1 (4.9e-4) for |a| < 0.49.
    """
    return a ** 4 / 120.0


def _finish(x: float, y: float, psi: float, wrap: bool) -> np.ndarray:
    return np.array([x, y, wrap_angle(psi) if wrap else psi])


def step_chord(state: Any, control: Any, ts: float, wrap: bool = True) -> np.ndarray:
    """EXACT zero-order-hold step in chord form (the reference; no branch, continuous at omega = 0).

    a = omega Ts / 2;  x+ = x + v Ts sinc(a) cos(psi + a);  y+ = y + v Ts sinc(a) sin(psi + a);
    psi+ = psi + omega Ts. v = 0 gives an in-place turn; omega = 0 a straight line. sinc is exact
    here (it divides by a); the hardware approximation is step_chord_poly.
    """
    _check_ts(ts)
    s, u = _as_state_control(state, control)
    half = 0.5 * u[1] * ts
    chord = u[0] * ts * sinc(half)
    return _finish(s[0] + chord * math.cos(s[2] + half), s[1] + chord * math.sin(s[2] + half),
                   s[2] + u[1] * ts, wrap)


def step_chord_poly(state: Any, control: Any, ts: float, wrap: bool = True) -> np.ndarray:
    """HARDWARE APPROXIMATION of step_chord in double: sinc(a) replaced by 1 - a^2/6 (no division).

    Position error vs step_chord is |v Ts| (sinc(a) - sinc_poly2(a)) <= |v Ts| a^4/120 per step;
    heading is exact. It isolates the polynomial error from the table / half-precision errors that
    app/husky/hw_estimates.py adds on top.
    """
    _check_ts(ts)
    s, u = _as_state_control(state, control)
    half = 0.5 * u[1] * ts
    chord = u[0] * ts * sinc_poly2(half)
    return _finish(s[0] + chord * math.cos(s[2] + half), s[1] + chord * math.sin(s[2] + half),
                   s[2] + u[1] * ts, wrap)


def step_exact_arc_literal(state: Any, control: Any, ts: float, dpsi_eps: float = DPSI_EPS,
                           wrap: bool = True) -> np.ndarray:
    """The textbook arc form, x+ = x + (v/omega)(sin(psi + dpsi) - sin psi), with a straight-line
    branch for |dpsi| <= dpsi_eps. Reference only: unusable in half precision (design note 5.4)."""
    _check_ts(ts)
    s, u = _as_state_control(state, control)
    v, omega = u
    dpsi = omega * ts
    if abs(dpsi) <= dpsi_eps:
        return _finish(s[0] + v * ts * math.cos(s[2]), s[1] + v * ts * math.sin(s[2]), s[2] + dpsi, wrap)
    radius = v / omega
    return _finish(s[0] + radius * (math.sin(s[2] + dpsi) - math.sin(s[2])),
                   s[1] + radius * (math.cos(s[2]) - math.cos(s[2] + dpsi)), s[2] + dpsi, wrap)


def step_euler(state: Any, control: Any, ts: float, wrap: bool = True) -> np.ndarray:
    _check_ts(ts)
    s, _ = _as_state_control(state, control)
    r = s + ts * continuous_dynamics(s, control)
    return _finish(r[0], r[1], r[2], wrap)


def step_midpoint(state: Any, control: Any, ts: float, wrap: bool = True) -> np.ndarray:
    _check_ts(ts)
    s, _ = _as_state_control(state, control)
    k1 = continuous_dynamics(s, control)
    r = s + ts * continuous_dynamics(s + 0.5 * ts * k1, control)
    return _finish(r[0], r[1], r[2], wrap)


def step_rk4(state: Any, control: Any, ts: float, wrap: bool = True) -> np.ndarray:
    """Classical RK4, the scheme of System::one_step_prediction (hls_system.hpp)."""
    _check_ts(ts)
    s, _ = _as_state_control(state, control)
    k1 = continuous_dynamics(s, control)
    k2 = continuous_dynamics(s + 0.5 * ts * k1, control)
    k3 = continuous_dynamics(s + 0.5 * ts * k2, control)
    k4 = continuous_dynamics(s + ts * k3, control)
    r = s + ts / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    return _finish(r[0], r[1], r[2], wrap)


STEP_METHODS: dict[str, Callable[..., np.ndarray]] = {
    "chord": step_chord,
    "chord_poly": step_chord_poly,
    "arc": step_exact_arc_literal,
    "rk4": step_rk4,
    "midpoint": step_midpoint,
    "euler": step_euler,
}


def step(state: Any, control: Any, ts: float, method: str = "chord", wrap: bool = True) -> np.ndarray:
    if method not in STEP_METHODS:
        raise ValueError(f"unknown method {method!r}; choose from {sorted(STEP_METHODS)}")
    return STEP_METHODS[method](state, control, ts, wrap=wrap)


def expand_horizon(controls: Any, horizon: int) -> np.ndarray:
    """Nu x 2 -> Nh x 2 with the last input held (uu_loop / get_next_uu in hls_system.hpp)."""
    seq = np.asarray(controls, dtype=np.float64).reshape(-1, NU)
    if not 1 <= len(seq) <= horizon:
        raise ValueError("need 1 <= Nu <= Nh")
    return np.vstack([seq, np.repeat(seq[-1:], horizon - len(seq), axis=0)])


def rollout(state0: Any, controls: Any, ts: float, horizon: int | None = None, method: str = "chord",
            wrap: bool = True) -> np.ndarray:
    """Predicted states (Nh + 1) x 3, row 0 = state0."""
    seq = np.asarray(controls, dtype=np.float64).reshape(-1, NU)
    full = expand_horizon(seq, len(seq) if horizon is None else horizon)
    states = np.empty((len(full) + 1, NX))
    states[0] = np.asarray(state0, dtype=np.float64)
    for k, u in enumerate(full):
        states[k + 1] = step(states[k], u, ts, method, wrap)
    return states


def to_local_frame(state0: Any, poses: Any) -> np.ndarray:
    """Express world poses in the frame of state0 (origin at the robot, x forward).

    Headings are unwrapped continuously from 0 along the sequence, so the HLS residual contract
    (|x_hat - ref| < 3 pi) holds whenever the horizon turns by less than about 2 pi. Controls
    [v, omega] are frame-invariant, so a plan solved in this frame is applied unchanged.
    """
    origin = np.asarray(state0, dtype=np.float64)
    p = np.asarray(poses, dtype=np.float64).reshape(-1, NX)
    c, s = math.cos(origin[2]), math.sin(origin[2])
    dx, dy = p[:, 0] - origin[0], p[:, 1] - origin[1]
    local = np.column_stack([c * dx + s * dy, -s * dx + c * dy, np.zeros(len(p))])
    previous = 0.0
    for k in range(len(p)):
        previous = unwrap_near(p[k, 2] - origin[2], previous)
        local[k, 2] = previous
    return local


# ----------------------------------------------------------------------------------------------
# Wheel-side mapping (skid steer and differential drive)
# ----------------------------------------------------------------------------------------------


def body_to_sides(v: Any, omega: Any, track: float) -> tuple[Any, Any]:
    """(v_left, v_right) = (v - omega W/2, v + omega W/2). omega > 0 (CCW) -> right side faster."""
    half = 0.5 * track
    return v - omega * half, v + omega * half


def sides_to_body(v_left: Any, v_right: Any, track: float) -> tuple[Any, Any]:
    """(v, omega) = ((v_r + v_l)/2, (v_r - v_l)/W): the A300 manual's 'Robot Equations'."""
    return 0.5 * (v_left + v_right), (v_right - v_left) / track


def wheel_angular_speeds(v: float, omega: float, track: float, wheel_radius: float) -> tuple[float, float]:
    """Wheel joint speeds (rad/s) per side, for checks against /joint_states."""
    left, right = body_to_sides(v, omega, track)
    return left / wheel_radius, right / wheel_radius


def saturate_sides(v: float, omega: float, track: float, side_speed_max: float,
                   mode: str = "scale") -> tuple[float, float]:
    """Limit |v +- omega W/2| <= side_speed_max.

    mode "scale": shrink both sides by one factor. Keeps the curvature omega/v, the signs of v and
        omega, and therefore the turn direction (the diff_drive_controller's preserve_turning_radius
        does the same for its own limits).
    mode "clip": clip each side independently (changes the curvature; shown for comparison).
    """
    if not side_speed_max > 0.0:
        raise ValueError("side_speed_max must be positive")
    left, right = body_to_sides(v, omega, track)
    if mode == "scale":
        peak = max(abs(left), abs(right))
        if peak <= side_speed_max:
            return v, omega
        factor = side_speed_max / peak
        return v * factor, omega * factor
    if mode == "clip":
        return sides_to_body(float(np.clip(left, -side_speed_max, side_speed_max)),
                             float(np.clip(right, -side_speed_max, side_speed_max)), track)
    raise ValueError("mode must be 'scale' or 'clip'")


# ----------------------------------------------------------------------------------------------
# Constraints: windows, separate violation report, exact projection
# ----------------------------------------------------------------------------------------------


def slew_window(previous: float, grow: float, shrink: float) -> tuple[float, float]:
    """Admissible next value after one sample when |value| may grow by at most `grow` and shrink by
    at most `shrink` (both already multiplied by Ts). Crossing zero is treated conservatively: the
    part past zero counts as growth in the new direction and is limited to `grow`."""
    if previous > 0.0:
        low, high = previous - shrink, previous + grow
    elif previous < 0.0:
        low, high = previous - grow, previous + shrink
    else:
        low, high = -grow, grow
    return max(low, min(previous, 0.0) - grow), min(high, max(previous, 0.0) + grow)


def control_windows(u_prev: Any, limits: UnicycleLimits, ts: float) -> tuple[float, float, float, float]:
    """(v_low, v_high, omega_low, omega_high): box limits intersected with the slew windows."""
    v_prev, w_prev = np.asarray(u_prev, dtype=np.float64).reshape(NU)
    v_lo, v_hi = slew_window(v_prev, limits.accel_max * ts, limits.decel_max * ts)
    w_lo, w_hi = slew_window(w_prev, limits.alpha_accel_max * ts, limits.alpha_decel_max * ts)
    return (max(v_lo, limits.v_min), min(v_hi, limits.v_max),
            max(w_lo, -limits.omega_max), min(w_hi, limits.omega_max))


def check_controls(controls: Any, u_prev: Any, limits: UnicycleLimits, ts: float,
                   tol: float = 1e-9) -> dict[str, Any]:
    """Largest violation of each constraint, reported separately (0 = satisfied); never a cost.

    speed (m/s), omega (rad/s), linear_slew (m/s^2), angular_slew (rad/s^2), side_speed (m/s),
    side_accel (m/s^2); None where the limit is not set. Increments start from u_prev.
    """
    _check_ts(ts)
    seq = np.asarray(controls, dtype=np.float64).reshape(-1, NU)
    previous = np.asarray(u_prev, dtype=np.float64).reshape(NU)
    report: dict[str, Any] = {"finite": bool(np.all(np.isfinite(seq)) and np.all(np.isfinite(previous)))}
    if not report["finite"]:
        report["ok"] = False
        return report
    v, w = seq[:, 0], seq[:, 1]
    report["speed"] = float(max(0.0, np.max(v - limits.v_max), np.max(limits.v_min - v)))
    report["omega"] = float(max(0.0, np.max(np.abs(w) - limits.omega_max)))
    lin, ang = 0.0, 0.0
    prev = previous
    for u in seq:
        v_lo, v_hi = slew_window(prev[0], limits.accel_max * ts, limits.decel_max * ts)
        w_lo, w_hi = slew_window(prev[1], limits.alpha_accel_max * ts, limits.alpha_decel_max * ts)
        lin = max(lin, u[0] - v_hi, v_lo - u[0])
        ang = max(ang, u[1] - w_hi, w_lo - u[1])
        prev = u
    report["linear_slew"] = max(0.0, lin) / ts
    report["angular_slew"] = max(0.0, ang) / ts
    left, right = body_to_sides(v, w, limits.track)
    report["side_speed"] = None if limits.side_speed_max is None else float(
        max(0.0, np.max(np.abs(left)) - limits.side_speed_max, np.max(np.abs(right)) - limits.side_speed_max))
    if limits.side_accel_max is None:
        report["side_accel"] = None
    else:
        sides = np.column_stack([left, right])
        prev_sides = np.array(body_to_sides(previous[0], previous[1], limits.track)).reshape(1, 2)
        increments = np.diff(np.vstack([prev_sides, sides]), axis=0)
        report["side_accel"] = float(max(0.0, np.max(np.abs(increments)) / ts - limits.side_accel_max))
    amounts = [value for key, value in report.items() if key != "finite" and value is not None]
    report["ok"] = all(amount <= tol for amount in amounts)
    return report


def _halfplanes(u_prev: np.ndarray, limits: UnicycleLimits, ts: float) -> list[tuple[float, float, float]]:
    """Feasible set of one stage as half-planes a1 v + a2 omega <= b (it contains u_prev)."""
    v_lo, v_hi, w_lo, w_hi = control_windows(u_prev, limits, ts)
    planes = [(1.0, 0.0, v_hi), (-1.0, 0.0, -v_lo), (0.0, 1.0, w_hi), (0.0, -1.0, -w_lo)]
    h = limits.half_track
    for s1 in (1.0, -1.0):
        for s2 in (1.0, -1.0):
            if limits.side_speed_max is not None:
                planes.append((s1, s2 * h, limits.side_speed_max))
            if limits.side_accel_max is not None:
                planes.append((s1, s2 * h, limits.side_accel_max * ts + s1 * u_prev[0] + s2 * h * u_prev[1]))
    return planes


def project_stage(raw: Any, u_prev: Any, limits: UnicycleLimits, ts: float) -> np.ndarray:
    """Nearest feasible (v, omega) to raw, in the metric normalized by (v_max - v_min)/2 and
    omega_max. The stage set is a convex polygon containing u_prev (if u_prev is feasible), so the
    nearest point is raw itself, a projection onto one edge, or a vertex: all are enumerated.

    Fast path: the box windows are axis-aligned and the metric is diagonal, so clipping to them is
    the nearest point of the box; if that point also satisfies the side diamonds it is the answer.
    Otherwise all edge projections and vertices are evaluated at once (vectorized).
    """
    previous = np.asarray(u_prev, dtype=np.float64).reshape(NU)
    target = np.nan_to_num(np.asarray(raw, dtype=np.float64).reshape(NU), nan=0.0, posinf=1e9, neginf=-1e9)
    v_lo, v_hi, w_lo, w_hi = control_windows(previous, limits, ts)
    clipped = np.array([min(max(target[0], v_lo), v_hi), min(max(target[1], w_lo), w_hi)])
    scale = np.array([0.5 * (limits.v_max - limits.v_min), limits.omega_max])
    planes = np.array(_halfplanes(previous, limits, ts))
    normals, offsets = planes[:, :2] * scale, planes[:, 2]
    tolerance = 1e-12 * (1.0 + np.abs(offsets))

    def feasible(points: np.ndarray) -> np.ndarray:
        return np.all(points @ normals.T <= offsets + tolerance, axis=1)

    if feasible((clipped / scale)[None, :])[0]:
        return clipped
    z_target = target / scale
    norms = np.sum(normals * normals, axis=1)
    edge_points = z_target - ((normals @ z_target - offsets) / norms)[:, None] * normals
    i, j = np.triu_indices(len(normals), k=1)
    a, b = normals[i], normals[j]
    det = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
    keep = np.abs(det) > 1e-14
    a, b, det, oi, oj = a[keep], b[keep], det[keep], offsets[i][keep], offsets[j][keep]
    vertices = np.column_stack([(oi * b[:, 1] - oj * a[:, 1]) / det, (a[:, 0] * oj - b[:, 0] * oi) / det])
    candidates = np.vstack([edge_points, vertices, (previous / scale)[None, :]])
    candidates = candidates[feasible(candidates)]
    best = candidates[np.argmin(np.sum((candidates - z_target) ** 2, axis=1))]
    return best * scale


def project_controls(raw_controls: Any, u_prev: Any, limits: UnicycleLimits, ts: float) -> np.ndarray:
    """Stage-by-stage projection; the result always passes check_controls if u_prev is feasible."""
    _check_ts(ts)
    previous = np.asarray(u_prev, dtype=np.float64).reshape(NU)
    if not check_controls(previous[None, :], previous, limits, ts)["ok"]:
        raise ValueError("u_prev violates the limits; use the stop fallback from a feasible state")
    raw = np.asarray(raw_controls, dtype=np.float64).reshape(-1, NU)
    result = np.empty_like(raw)
    for k, u in enumerate(raw):
        previous = project_stage(u, previous, limits, ts)
        result[k] = previous
    return result


# ----------------------------------------------------------------------------------------------
# Command ownership: tie u_prev to the command actually applied, not to the MPC's last publication
# ----------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class CommandSample:
    """A velocity command observed on the robot side.

    stamp: header.stamp of the message (s); received: host receive time (s); v, omega: the twist.
    """

    stamp: float
    received: float
    v: float
    omega: float


def assess_command_ownership(mux_output: CommandSample | None, limited_output: CommandSample | None,
                             own_commands: dict[float, tuple[float, float]], now: float, max_age: float,
                             tol: float = 1e-9) -> tuple[str, np.ndarray | None, str]:
    """Decide whether the robot is executing the MPC's command and which command was applied.

    mux_output: latest twist_mux output (A300: /<ns>/platform/cmd_vel). The MPC owns the robot only
        if that output carries the stamp of one of the MPC's own recent commands (own_commands:
        stamp -> (v, omega)) with the same twist. twist_mux passing the stamp through unchanged must
        be verified in simulation (plan check V10); if it does not, this test must be redesigned.
    limited_output: latest command after the controller's limiter (diff_drive_controller with
        publish_limited_velocity: true), if observable. It is the best u_prev: it is what the
        controller's slew limiter continues from.

    Returns (status, u_prev, source):
      ("owned", u_prev, "limited" | "mux")  -> solve from u_prev (never from the MPC's own publication);
      ("foreign", None, "mux")              -> another source (teleop, RC, markers) is selected: idle,
                                               stop publishing, reset the warm start, require re-arm;
      ("unknown", None, reason)             -> no fresh mux output: safe stop, then idle.
    """
    if mux_output is None or now - mux_output.received > max_age:
        return "unknown", None, "mux output missing or stale"
    own = own_commands.get(mux_output.stamp)
    if own is None or abs(own[0] - mux_output.v) > tol or abs(own[1] - mux_output.omega) > tol:
        return "foreign", None, "mux"
    if limited_output is not None and now - limited_output.received <= max_age:
        return "owned", np.array([limited_output.v, limited_output.omega]), "limited"
    return "owned", np.array([mux_output.v, mux_output.omega]), "mux"


# ----------------------------------------------------------------------------------------------
# Warm start and known-feasible candidates
# ----------------------------------------------------------------------------------------------


def shift_warm_start(previous_solution: Any) -> np.ndarray:
    """[u1 .. u_{Nu-1}, u_{Nu-1}]: feasible without projection when u0 was the input applied."""
    seq = np.asarray(previous_solution, dtype=np.float64).reshape(-1, NU)
    return np.vstack([seq[1:], seq[-1:]])


def hold_candidate(u_prev: Any, control_horizon: int) -> np.ndarray:
    return np.repeat(np.asarray(u_prev, dtype=np.float64).reshape(1, NU), control_horizon, axis=0)


def stop_candidate(u_prev: Any, control_horizon: int, limits: UnicycleLimits, ts: float) -> np.ndarray:
    """Drive (v, omega) to (0, 0) as fast as every limit allows (projection toward zero each stage).

    A stop manoeuvre, not a collision-free guarantee. With decel 4 m/s^2 (A300 controller default)
    it stops from 2 m/s in 0.5 s = 10 stages of 50 ms.
    """
    return project_controls(np.zeros((control_horizon, NU)), u_prev, limits, ts)


def in_place_turn_candidate(u_prev: Any, heading_error: float, control_horizon: int,
                            limits: UnicycleLimits, ts: float, omega_turn: float | None = None) -> np.ndarray:
    """Stop translating and rotate towards the reference heading (skid-steer / diff-drive only).

    heading_error = wrap(psi_ref - psi) (rad). Stage targets are (0, clip(remaining/Ts, +-omega_turn));
    every stage is projected, so the candidate is feasible. Used when |heading_error| is large.
    """
    omega_cap = limits.omega_max if omega_turn is None else min(abs(omega_turn), limits.omega_max)
    previous = np.asarray(u_prev, dtype=np.float64).reshape(NU)
    remaining = float(heading_error)
    result = np.empty((control_horizon, NU))
    for k in range(control_horizon):
        target = np.array([0.0, float(np.clip(remaining / ts, -omega_cap, omega_cap))])
        previous = project_stage(target, previous, limits, ts)
        result[k] = previous
        remaining -= previous[1] * ts
    return result


def feedforward_from_reference(reference_states: Any, state0: Any, ts: float) -> np.ndarray:
    """Constant-arc controls through consecutive reference poses (inverse of the chord step).

    omega_k = wrap(psi_{k+1} - psi_k)/Ts; v_k = +-|dp| / (Ts sinc(omega_k Ts/2)), the sign from the
    chord direction psi_k + omega_k Ts/2. Exact for arc-consistent references; project before use.
    """
    _check_ts(ts)
    poses = np.vstack([np.asarray(state0, dtype=np.float64).reshape(1, NX),
                       np.asarray(reference_states, dtype=np.float64).reshape(-1, NX)])
    controls = np.zeros((len(poses) - 1, NU))
    for k in range(len(poses) - 1):
        dpsi = angle_error(poses[k + 1, 2], poses[k, 2])
        delta = poses[k + 1, :2] - poses[k, :2]
        chord = float(np.hypot(*delta))
        direction = poses[k, 2] + 0.5 * dpsi
        along = delta[0] * math.cos(direction) + delta[1] * math.sin(direction)
        speed = chord / (ts * sinc(0.5 * dpsi))
        controls[k] = (speed if along >= 0.0 else -speed), dpsi / ts
    return controls


# ----------------------------------------------------------------------------------------------
# Normalized PSO decision variables (the quadrotor ABI uses +-100 per dimension)
# ----------------------------------------------------------------------------------------------


def normalize_controls(controls: Any, limits: UnicycleLimits, scale: float = 100.0) -> np.ndarray:
    seq = np.asarray(controls, dtype=np.float64).reshape(-1, NU)
    center, half = 0.5 * (limits.v_max + limits.v_min), 0.5 * (limits.v_max - limits.v_min)
    return np.column_stack([(seq[:, 0] - center) / half, seq[:, 1] / limits.omega_max]) * scale


def denormalize_controls(normalized: Any, limits: UnicycleLimits, scale: float = 100.0) -> np.ndarray:
    seq = np.asarray(normalized, dtype=np.float64).reshape(-1, NU) / scale
    center, half = 0.5 * (limits.v_max + limits.v_min), 0.5 * (limits.v_max - limits.v_min)
    return np.column_stack([center + half * seq[:, 0], limits.omega_max * seq[:, 1]])


def normalized_slew_limits(limits: UnicycleLimits, ts: float, scale: float = 100.0) -> np.ndarray:
    """Symmetric per-sample du_max for the HLS box (calculate_du_min is symmetric), so each
    dimension uses the smaller of its grow/shrink rates: conservative, never too permissive."""
    half = 0.5 * (limits.v_max - limits.v_min)
    dv = min(limits.accel_max, limits.decel_max) * ts
    dw = min(limits.alpha_accel_max, limits.alpha_decel_max) * ts
    return np.array([dv / half, dw / limits.omega_max]) * scale


def inscribed_box(limits: UnicycleLimits, v_share: float) -> tuple[float, float]:
    """Per-dimension boxes that imply the coupled side-speed limit, for HLS versions that can only
    project boxes: v_box + omega_box W/2 <= side_speed_max with v_box = v_share * side_speed_max."""
    if limits.side_speed_max is None:
        return limits.v_max, limits.omega_max
    if not 0.0 < v_share < 1.0:
        raise ValueError("v_share must be in (0, 1)")
    v_box = min(limits.v_max, v_share * limits.side_speed_max)
    omega_box = min(limits.omega_max, (limits.side_speed_max - v_box) / limits.half_track)
    return v_box, omega_box


# ----------------------------------------------------------------------------------------------
# Cost (violations come from check_controls, never folded in)
# ----------------------------------------------------------------------------------------------


def mpc_cost(states: Any, controls: Any, u_prev: Any, reference_states: Any, weights: CostWeights,
             reference_controls: Any | None = None, hls_residual: bool = False) -> dict[str, float]:
    """Dimensionless cost split into terms.

    states: (Nh + 1) x 3 (row 0 not penalized); controls: Nh x 2; reference_states: Nh x 3 for
    states[1:]. Stages k = 1..Nh-1 use q_*, k = Nh uses qf_* (Qf at i = N-1 in hls_system.hpp).
    hls_residual=True uses the one-conditional residual of commit 8c61f1b instead of a full wrap.
    """
    predicted = np.asarray(states, dtype=np.float64)[1:]
    seq = np.asarray(controls, dtype=np.float64).reshape(-1, NU)
    ref = np.asarray(reference_states, dtype=np.float64).reshape(-1, NX)
    if not len(predicted) == len(seq) == len(ref):
        raise ValueError("states[1:], controls and reference_states must have Nh rows")
    ref_u = np.zeros_like(seq) if reference_controls is None else np.asarray(
        reference_controls, dtype=np.float64).reshape(-1, NU)
    position_sq = np.sum((predicted[:, :2] - ref[:, :2]) ** 2, axis=1)
    if hls_residual:
        heading = np.array([hls_angle_residual(a, b) for a, b in zip(predicted[:, 2], ref[:, 2])])
    else:
        heading = angle_error(predicted[:, 2], ref[:, 2])
    heading_sq = np.asarray(heading) ** 2
    increments = np.diff(np.vstack([np.asarray(u_prev, dtype=np.float64).reshape(1, NU), seq]), axis=0)
    terms = {
        "position": float(weights.q_pos * np.sum(position_sq[:-1])),
        "heading": float(weights.q_psi * np.sum(heading_sq[:-1])),
        "terminal": float(weights.qf_pos * position_sq[-1] + weights.qf_psi * heading_sq[-1]),
        "speed": float(weights.q_v * np.sum((seq[:, 0] - ref_u[:, 0]) ** 2)),
        "yaw_rate": float(weights.r_omega * np.sum((seq[:, 1] - ref_u[:, 1]) ** 2)),
        "increments": float(np.sum(weights.s_dv * increments[:, 0] ** 2 + weights.s_domega * increments[:, 1] ** 2)),
    }
    terms["total"] = float(sum(terms.values()))
    return terms

