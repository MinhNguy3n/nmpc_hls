"""(speed, bicycle steering angle) -> the six Gazebo joint commands of the RC car.

The plant input is the *applied* command: the speed of the rear-axle midpoint v (m/s) and the
bicycle-equivalent steering angle delta (rad, left positive), as in the Ackermann note section 2.
Phase 2's actuator emulation produces it; the bring-up checks send it directly.

Outputs, for gz_ros2_control's JointGroup controllers:
  steering positions (left, right) in rad: ideal Ackermann (note section 6, tested
      `wheel_steering_angles`);
  wheel speeds (FL, FR, RL, RR) in rad/s: each wheel's no-slip ground speed about the ICR divided
      by the wheel radius. Rear: v (1 -+ kappa W_r/2) (`rear_wheel_speeds`). Front: the front
      wheel at (L, y) is at distance sqrt((R - y)^2 + L^2) from the ICR (0, R), so its speed is
      v sqrt((1 - kappa y)^2 + (L kappa)^2), signed like v.

All four wheels are driven, at the speeds a car with ideal differentials would reach without slip.
Real differentials split torque, not speed; the difference only matters when wheels slip, which
the phase 4 speed sweep measures (lateral slip at the rear axle from the Gazebo truth).
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from vehicle_params import CarGeometry, am


@dataclass(frozen=True)
class JointCommands:
    steer_left: float
    steer_right: float
    wheel_fl: float
    wheel_fr: float
    wheel_rl: float
    wheel_rr: float

    @property
    def steering(self) -> list[float]:
        return [self.steer_left, self.steer_right]

    @property
    def wheels(self) -> list[float]:
        return [self.wheel_fl, self.wheel_fr, self.wheel_rl, self.wheel_rr]


def clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def joint_commands(speed: float, delta: float, geometry: CarGeometry, v_max: float) -> JointCommands:
    """Joint commands for an applied (speed, delta); delta and speed are clamped to the limits.

    Non-finite inputs give the safe command: wheels stopped, steering straight.
    """
    if not (math.isfinite(speed) and math.isfinite(delta)):
        return JointCommands(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    v = clamp(speed, -v_max, v_max)
    d = clamp(delta, -geometry.delta_max, geometry.delta_max)
    kappa = am.kappa_from_delta(d, geometry.wheelbase)
    left, right = am.wheel_steering_angles(kappa, geometry.wheelbase, geometry.front_track)
    lk = geometry.wheelbase * kappa
    half_f = 0.5 * geometry.front_track
    v_fl = v * math.hypot(1.0 - kappa * half_f, lk)
    v_fr = v * math.hypot(1.0 + kappa * half_f, lk)
    v_rl, v_rr = am.rear_wheel_speeds(v, kappa, geometry.rear_track)
    r = geometry.wheel_radius
    return JointCommands(left, right, v_fl / r, v_fr / r, v_rl / r, v_rr / r)


STEERING_JOINTS = ("front_left_steer_joint", "front_right_steer_joint")
WHEEL_JOINTS = ("front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint")
