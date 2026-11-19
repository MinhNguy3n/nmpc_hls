"""Offline tests of estimator_checks.py and the SIL IMU covariance relay (no ROS)."""

from __future__ import annotations

import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import estimator_checks as ec  # noqa: E402
import mux_timing as mt  # noqa: E402
from sil.imu_cov_relay import DRIVER_GYRO_VARIANCE, apply_covariance  # noqa: E402

START = 10.0


def simulate(gain_turn: float, gain_arc: float, dt: float = 0.01):
    """Pose of a unicycle following ESTIMATOR_PROFILE with yaw-rate gains (truth) and without (wheel)."""
    t = np.arange(0.0, START + mt.DURATIONS["estimator"] + 2.0, dt)
    out = {"truth": [], "wheel": []}
    pose = {"truth": np.zeros(3), "wheel": np.zeros(3)}
    for tt in t:
        v, w = mt.profile_command("estimator", tt - START) if tt >= START else (0.0, 0.0)
        gain = gain_arc if v > 0 else gain_turn
        for name, wk in (("truth", w * gain), ("wheel", w)):
            x, y, h = pose[name]
            out[name].append((tt, x, y, h, v, wk))
            pose[name] = np.array([x + v * math.cos(h) * dt, y + v * math.sin(h) * dt, h + wk * dt])
    return out


def rows_from(sim) -> list[dict]:
    rows = []
    for tt, x, y, h, v, w in sim["truth"]:
        rows.append({"source": "truth", "stamp": tt, "x": x, "y": y, "yaw": math.atan2(math.sin(h), math.cos(h)),
                     "z": 0.136, "tilt": 0.0})
        # an estimator that is right, but starts from another origin and heading (odom frame)
        c, s = math.cos(0.7), math.sin(0.7)
        rows.append({"source": "good", "stamp": tt, "x": 3 + c * x - s * y, "y": -1 + s * x + c * y,
                     "yaw": math.atan2(math.sin(h + 0.7), math.cos(h + 0.7)), "omega": w, "cov_yaw": 1e-3,
                     "cov_wz": 2.75e-6})
    for tt, x, y, h, v, w in sim["wheel"]:
        rows.append({"source": "wheel", "stamp": tt, "x": x, "y": y, "yaw": math.atan2(math.sin(h), math.cos(h)),
                     "omega": w, "cov_yaw": 1e-3, "cov_wz": 1e-3})
    return rows


def test_report_separates_a_correct_estimator_from_wheel_odometry(tmp_path):
    rows = rows_from(simulate(0.952, 0.825))
    path = tmp_path / "c.csv"
    ec.write_capture(path, rows)
    rep = ec.estimator_report(ec.read_capture(path), START)
    good, wheel = rep["sources"]["good"], rep["sources"]["wheel"]
    assert good["pos_error_final"] < 0.01 and abs(good["heading_error_final"]) < 0.01
    assert good["segments"]["arc_left"]["yaw_rate_ratio"] == pytest.approx(1.0, abs=0.02)
    # wheel odometry turns 1/0.825 too fast on arcs and 1/0.952 in place
    assert wheel["segments"]["arc_left"]["yaw_rate_ratio"] == pytest.approx(1 / 0.825, rel=0.02)
    assert wheel["segments"]["turn_left"]["yaw_rate_ratio"] == pytest.approx(1 / 0.952, rel=0.02)
    assert abs(wheel["segments"]["arc_left"]["heading_error_end"]) > 0.3 and wheel["pos_error_final"] > 0.2
    assert rep["truth_tilt_max"] == 0.0 and good["cov_wz_median"] == pytest.approx(2.75e-6)


def test_segment_bounds_follow_the_profile():
    bounds = ec.segment_bounds(5.0)
    assert [b[0] for b in bounds] == list(ec.SEGMENTS)
    assert bounds[0][1] == 5.0 and bounds[-1][2] == 5.0 + mt.DURATIONS["estimator"]


def test_imu_relay_sets_only_the_diagonals():
    msg = SimpleNamespace(angular_velocity_covariance=[0.0] * 9, linear_acceleration_covariance=[0.5] * 9)
    apply_covariance(msg, DRIVER_GYRO_VARIANCE, None)
    assert msg.angular_velocity_covariance[0::4] == [pytest.approx(2.749e-6, rel=1e-3)] * 3
    assert msg.angular_velocity_covariance[1] == 0.0 and msg.linear_acceleration_covariance == [0.5] * 9


def test_yaw_map_recovers_a_turn_rate_limit():
    # a body whose yaw rate saturates at 0.25 rad/s and which keeps 90 % of the commanded speed
    dt, rows, sent, pose, t = 0.01, [], [], np.zeros(3), 0.0
    for v, w, dur in ((0.2, 0.1, 5.0), (0.2, 0.6, 5.0)):
        for _ in range(int(dur / dt)):
            wb = math.copysign(min(abs(w), 0.25), w)
            rows.append({"source": "truth", "stamp": t, "x": pose[0], "y": pose[1],
                         "yaw": math.atan2(math.sin(pose[2]), math.cos(pose[2]))})
            if round(t / dt) % 5 == 0:
                sent.append({"received": t, "v": v, "omega": w})
            pose = pose + dt * np.array([0.9 * v * math.cos(pose[2]), 0.9 * v * math.sin(pose[2]), wb])
            t += dt
    steps = ec.yaw_map(rows, sent)
    assert [round(s["w_cmd"], 2) for s in steps] == [0.1, 0.6]
    assert steps[0]["w_ratio"] == pytest.approx(1.0, abs=0.03) and steps[1]["w_body"] == pytest.approx(0.25, abs=0.01)
    assert all(s["v_ratio"] == pytest.approx(0.9, abs=0.02) for s in steps)
