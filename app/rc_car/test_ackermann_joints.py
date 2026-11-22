"""Offline tests of the RC car parameter loading and the joint-command geometry (no ROS, no Gazebo).

    /usr/bin/python3 -m pytest app/rc_car -q -p no:cacheprovider
"""

from __future__ import annotations

import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import vehicle_params as vp  # noqa: E402
from ackermann_joints import joint_commands  # noqa: E402

GEOM = vp.load_geometry()
VMAX = 2.0


def rigid_body_wheel_velocity(v: float, kappa: float, x: float, y: float) -> tuple[float, float]:
    """Velocity (body frame) of the point (x, y) of a rigid body whose rear-axle midpoint moves at
    v along x_B with yaw rate v kappa: (v - omega y, omega x)."""
    omega = v * kappa
    return v - omega * y, omega * x


def test_yaml_loads_and_every_entry_is_labelled():
    raw = vp.load_raw()
    assert all(entry["status"] in vp.STATUSES for entry in raw.values())
    assert GEOM.wheelbase == pytest.approx(0.275)
    assert raw["wheelbase"]["status"] == "spec_unverified"
    # the masses are not known yet; the track and the steering travel are approximate measurements
    assert {"mass_battery", "payload_x", "payload_z"} <= set(vp.placeholders())
    assert raw["payload_mass"]["status"] == "measured_approx" and raw["payload_mass"]["value"] == 1.5
    assert {raw[k]["status"] for k in ("front_track", "rear_track", "delta_max")} == {"measured_approx"}


def test_bad_parameter_files_are_rejected(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("wheelbase: {value: 0.275, unit: m, status: guessed, source: x}\n")
    with pytest.raises(ValueError, match="unknown status"):
        vp.load_raw(bad)
    bad.write_text("wheelbase: {value: 0.275, unit: m, status: design}\n")
    with pytest.raises(ValueError, match="needs value"):
        vp.load_raw(bad)
    bad.write_text("wheelbase: {value: 0.275, unit: m, status: design, source: x}\n")
    with pytest.raises(ValueError, match="missing parameters"):
        vp.load_raw(bad)


def test_straight_ahead_all_wheels_equal():
    c = joint_commands(1.0, 0.0, GEOM, VMAX)
    assert c.steering == [0.0, 0.0]
    assert c.wheels == pytest.approx([1.0 / GEOM.wheel_radius] * 4, rel=1e-12)


@pytest.mark.parametrize("v", [0.5, 1.5, -0.4])
@pytest.mark.parametrize("delta_frac", [-1.0, -0.6, -0.1, 0.1, 0.6, 1.0])
def test_each_wheel_rolls_along_its_rigid_body_velocity(v, delta_frac):
    """Steering angles point each front wheel along its contact point's velocity, and every wheel's
    surface speed equals that velocity's magnitude: no wheel slips in the commanded motion."""
    delta = delta_frac * GEOM.delta_max
    kappa = math.tan(delta) / GEOM.wheelbase
    c = joint_commands(v, delta, GEOM, VMAX)
    hf, hr, L, r = 0.5 * GEOM.front_track, 0.5 * GEOM.rear_track, GEOM.wheelbase, GEOM.wheel_radius
    wheels = [(L, hf, c.steer_left, c.wheel_fl), (L, -hf, c.steer_right, c.wheel_fr),
              (0.0, hr, 0.0, c.wheel_rl), (0.0, -hr, 0.0, c.wheel_rr)]
    for x, y, steer, omega_wheel in wheels:
        vx, vy = rigid_body_wheel_velocity(v, kappa, x, y)
        # wheel rolling direction (cos, sin of the steer angle) times signed surface speed
        assert omega_wheel * r * math.cos(steer) == pytest.approx(vx, abs=1e-12)
        assert omega_wheel * r * math.sin(steer) == pytest.approx(vy, abs=1e-12)


def test_left_and_right_turns_mirror():
    a = joint_commands(1.0, 0.3, GEOM, VMAX)
    b = joint_commands(1.0, -0.3, GEOM, VMAX)
    assert (a.steer_left, a.steer_right) == pytest.approx((-b.steer_right, -b.steer_left))
    assert (a.wheel_fl, a.wheel_fr, a.wheel_rl, a.wheel_rr) == pytest.approx((b.wheel_fr, b.wheel_fl, b.wheel_rr, b.wheel_rl))
    # left turn: the left (inner) wheel steers more, and the inner wheels turn slower
    assert a.steer_left > 0.3 > a.steer_right > 0.0
    assert a.wheel_fl < a.wheel_fr and a.wheel_rl < a.wheel_rr


def test_limits_and_non_finite_inputs():
    full = joint_commands(1.0, GEOM.delta_max, GEOM, VMAX)
    assert joint_commands(1.0, 10.0, GEOM, VMAX) == full  # steering clamped to delta_max
    assert full.steer_left == pytest.approx(GEOM.inner_wheel_max)
    assert GEOM.inner_wheel_max > GEOM.delta_max
    fast = joint_commands(50.0, 0.0, GEOM, VMAX)
    assert fast.wheels == pytest.approx([VMAX / GEOM.wheel_radius] * 4)
    for bad in (math.nan, math.inf, -math.inf):
        stop = joint_commands(bad, 0.1, GEOM, VMAX)
        assert stop.steering == [0.0, 0.0] and stop.wheels == [0.0] * 4
        stop = joint_commands(1.0, bad, GEOM, VMAX)
        assert stop.steering == [0.0, 0.0] and stop.wheels == [0.0] * 4


def test_turning_radius_matches_bicycle_model():
    assert GEOM.r_min == pytest.approx(GEOM.wheelbase / math.tan(GEOM.delta_max))
