"""Offline tests of the Twist -> Ackermann converter: sign, clamp, reversing, v ~ 0."""

from __future__ import annotations

import math

import pytest

from twist_to_ackermann import convert

L, DMAX, VMIN = 0.275, 0.45, 0.05


def test_formula_and_sign():
    c = convert(1.0, 0.5, L, DMAX, VMIN)
    assert c.steering_angle == pytest.approx(math.atan(L * 0.5)) and c.speed == 1.0
    assert not c.clamped and not c.below_v_min
    assert convert(1.0, -0.5, L, DMAX, VMIN).steering_angle == pytest.approx(-math.atan(L * 0.5))
    # left turn (omega > 0) -> positive (left) steering when driving forward
    assert convert(0.8, 1.0, L, DMAX, VMIN).steering_angle > 0


def test_round_trip_with_the_bicycle_model():
    for delta in (-0.4, -0.1, 0.0, 0.2, 0.44):
        v = 1.3
        omega = v * math.tan(delta) / L
        assert convert(v, omega, L, DMAX, VMIN).steering_angle == pytest.approx(delta, abs=1e-12)


def test_clamp():
    c = convert(0.5, 10.0, L, DMAX, VMIN)
    assert c.steering_angle == DMAX and c.clamped
    c = convert(0.5, -10.0, L, DMAX, VMIN)
    assert c.steering_angle == -DMAX and c.clamped


def test_reversing():
    """Reversing with steering delta gives omega = v tan(delta) / L < 0 for delta > 0, v < 0."""
    delta = 0.3
    v = -0.6
    omega = v * math.tan(delta) / L
    c = convert(v, omega, L, DMAX, VMIN)
    assert c.speed == v and c.steering_angle == pytest.approx(delta)
    # the same positive omega needs opposite steering forward and in reverse
    assert math.copysign(1, convert(0.6, 0.5, L, DMAX, VMIN).steering_angle) == \
        -math.copysign(1, convert(-0.6, 0.5, L, DMAX, VMIN).steering_angle)


@pytest.mark.parametrize("v", [0.0, 1e-9, -1e-9, 0.049, -0.049])
def test_below_speed_floor_steering_zero(v):
    c = convert(v, 2.0, L, DMAX, VMIN)
    assert c.steering_angle == 0.0 and c.below_v_min and c.speed == v


def test_at_the_speed_floor_formula_applies():
    c = convert(0.05, 0.01, L, DMAX, VMIN)
    assert not c.below_v_min and c.steering_angle == pytest.approx(math.atan(L * 0.01 / 0.05))


def test_non_finite_and_invalid():
    assert convert(math.nan, 0.1, L, DMAX, VMIN).speed == 0.0
    assert convert(1.0, math.inf, L, DMAX, VMIN).steering_angle == 0.0
    with pytest.raises(ValueError):
        convert(1.0, 0.1, 0.0, DMAX, VMIN)
