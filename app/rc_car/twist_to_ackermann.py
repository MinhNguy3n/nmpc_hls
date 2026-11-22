"""Twist -> Ackermann command converter of the RC car (our own code, written from the formula).

It replaces the interface of the ROS package `twist_to_ackermann` (Twist in, AckermannDriveStamped
out); no upstream code is used. For the kinematic bicycle about the rear-axle midpoint,
omega = v * tan(delta) / L, so

    delta = atan(L * omega / v), clamped to +-delta_max;   speed = v (unchanged).

Below the speed floor |v| < v_min the steering is set to 0 (delta is undefined at v = 0 and
ill-conditioned near it); `v_min` is a PLACEHOLDER_ design value. Reversing (v < 0) needs no
special case: the formula gives the steering that produces the requested omega while reversing.
Non-finite input gives (0, 0).

The ROS node twist_to_ackermann_node.py wraps convert() (tested in test_twist_to_ackermann.py).
"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Converted:
    speed: float
    steering_angle: float
    clamped: bool       # |atan(L omega / v)| exceeded delta_max
    below_v_min: bool   # |v| < v_min: steering set to 0


def convert(v: float, omega: float, wheelbase: float, delta_max: float, v_min: float) -> Converted:
    if not (wheelbase > 0 and delta_max > 0 and v_min >= 0):
        raise ValueError("wheelbase and delta_max must be > 0, v_min >= 0")
    if not (math.isfinite(v) and math.isfinite(omega)):
        return Converted(0.0, 0.0, False, False)
    if abs(v) < v_min or v == 0.0:
        return Converted(float(v), 0.0, False, True)
    delta = math.atan(wheelbase * omega / v)
    clamped = abs(delta) > delta_max
    return Converted(float(v), max(-delta_max, min(delta_max, delta)), clamped, False)
