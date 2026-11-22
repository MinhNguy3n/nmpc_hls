"""Offline tests of the newest-wins pose input."""

from __future__ import annotations

import math

from pose_input import NewestPose, yaw_from_quaternion


def test_newest_capture_stamp_wins():
    p = NewestPose()
    assert p.update(1.00, 0.0, 0.0, 0.0)
    assert p.update(1.02, 0.2, 0.0, 0.0)
    assert not p.update(1.01, 0.1, 0.0, 0.0)  # overtaken frame
    assert not p.update(1.02, 9.0, 0.0, 0.0)  # same stamp
    assert p.pose.x == 0.2 and (p.accepted, p.ignored_old) == (2, 2)
    assert abs(p.age(1.05) - 0.03) < 1e-12 and NewestPose().age(0.0) == math.inf


def test_non_finite_ignored():
    p = NewestPose()
    assert not p.update(1.0, math.nan, 0.0, 0.0) and p.ignored_bad == 1 and p.pose is None


def test_yaw_from_quaternion():
    assert abs(yaw_from_quaternion(0, 0, math.sin(0.4), math.cos(0.4)) - 0.8) < 1e-12
