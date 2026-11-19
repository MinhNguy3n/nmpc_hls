"""Offline tests of imu_checks.py and the IMU bridge launch files (no ROS)."""

from __future__ import annotations

import ast
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import imu_checks as ic  # noqa: E402


def write_generated(root: Path, text: str) -> Path:
    path = root / "sensors" / "config"
    path.mkdir(parents=True)
    (path / "imu_0.yaml").write_text(text)
    return root


def test_check_generated_fails_loudly_on_the_empty_bridge(tmp_path, capsys):
    root = write_generated(tmp_path / "a", "[]\n")
    code, message = ic.check_generated(root)
    assert code == 1 and "EMPTY" in message
    assert ic.main(["check-generated", "--setup-path", str(root)]) == 1
    assert "EMPTY" in capsys.readouterr().err


def test_check_generated_detects_a_fixed_generator_and_missing_files(tmp_path):
    fixed = write_generated(tmp_path / "b", yaml.safe_dump([{
        "ros_topic_name": "sensors/imu_0/data", "gz_topic_name": "/a300_00000/sensors/imu_0/data",
        "ros_type_name": "sensor_msgs/msg/Imu", "gz_type_name": "gz.msgs.IMU", "direction": "GZ_TO_ROS"}]))
    assert ic.check_generated(fixed)[0] == 3
    other = write_generated(tmp_path / "c", yaml.safe_dump([{"ros_topic_name": "something/else"}]))
    assert ic.check_generated(other)[0] == 0
    assert ic.check_generated(tmp_path / "missing")[0] == 4


def test_bridge_config_and_launch_file_are_well_formed():
    (entry,) = yaml.safe_load((HERE / "launch" / "a300_imu_bridge.yaml").read_text())
    assert entry == {"ros_topic_name": "/a300_00000/sensors/imu_0/data", "gz_topic_name": "/a300_00000/sensors/imu_0/data",
                     "ros_type_name": "sensor_msgs/msg/Imu", "gz_type_name": "gz.msgs.IMU", "direction": "GZ_TO_ROS"}
    tree = ast.parse((HERE / "launch" / "a300_sim_imu_bridge.launch.py").read_text())
    names = {n.name for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}
    assert "generate_launch_description" in names


def synthetic_rows() -> list[dict]:
    rows = []
    t = np.arange(0.0, 40.0, 0.01)

    def w_true(tt):
        return np.where((tt % 20) < 10, 0.0, 0.2855)  # steps up at 10 s and 30 s, down at 20 s

    for tt in t:
        rows.append({"source": "truth", "stamp": tt, "received": tt + 0.001,
                     "yaw": float(np.sum(w_true(np.arange(0, tt, 0.01))) * 0.01)})
        if not (25.0 <= tt < 35.0):  # the IMU bridge is stopped from 25 to 35 s
            rows.append({"source": "imu", "stamp": tt, "received": tt + 0.002, "frame_id": "imu_0_link",
                         "omega": float(w_true(tt)), "ax": 0.0, "cov_orientation0": -1.0, "cov_gyro_z": 0.0,
                         "cov_accel_x": 0.0})
    for tt in np.arange(0.0, 40.0, 0.02):
        wheel = float(w_true(tt)) / 0.952
        rows.append({"source": "wheel", "stamp": tt, "received": tt + 0.001, "omega": wheel, "v": 0.0})
        imu_ok = not (25.0 <= tt < 35.0)
        filt = float(w_true(tt - 0.1)) if imu_ok else wheel  # 0.1 s lag with IMU, wheel odometry without
        rows.append({"source": "filtered", "stamp": tt, "received": tt + 0.003, "omega": filt, "v": 0.0})
    return rows


def test_imu_report_gap_fallback_and_latency(tmp_path):
    path = tmp_path / "imu.csv"
    ic.write_capture(path, synthetic_rows())
    rep = ic.imu_report(ic.read_capture(path))
    imu = rep["imu"]
    assert imu["rate_hz"] == pytest.approx(100.0, rel=1e-3) and imu["frame_ids"] == ["imu_0_link"]
    assert imu["stamp_clock"].startswith("simulation") and imu["orientation_covariance0"] == [-1.0]
    (gap,) = rep["imu_gaps"]
    assert gap["gap_start"] == pytest.approx(24.992, abs=0.01) and gap["silent_fallback_to_wheel_odometry"]
    assert gap["filtered_minus_truth_during"] > 0.005
    lat = rep["filtered_latency"]
    assert lat["best_lag"] == pytest.approx(0.1, abs=0.02)
    assert any(d == pytest.approx(0.1, abs=0.03) for d in lat["step_delays_50pct"])
    assert ic.main(["report", str(path)]) == 0


def test_gap_spanning_turns_of_both_signs_is_detected_as_fallback():
    # the 2026-11-19 V14 run: the IMU stopped during a +0.3 turn and resumed during a -0.3 turn; without
    # the IMU the EKF followed wheel odometry (5 % high), with a small drift while standing still
    def w_true(tt):
        return 0.2855 if tt < 10 else (0.0 if tt < 14 else -0.2855)

    rows = []
    for tt in np.arange(0.0, 22.0, 0.01):
        rows.append({"source": "truth", "stamp": tt, "received": tt,
                     "yaw": float(sum(w_true(x) for x in np.arange(0, tt, 0.01)) * 0.01)})
        if not 6.0 <= tt < 19.0:
            rows.append({"source": "imu", "stamp": tt, "received": tt, "frame_id": "imu_0_link",
                         "omega": w_true(tt), "ax": 0.0, "cov_orientation0": 0.0, "cov_gyro_z": 0.0, "cov_accel_x": 0.0})
    for tt in np.arange(0.0, 22.0, 0.02):
        wheel = w_true(tt) / 0.952
        filt = wheel + (0.009 if 10 <= tt < 14 else 0.0) if 6.0 <= tt < 19.0 else w_true(tt)
        rows.append({"source": "wheel", "stamp": tt, "received": tt, "omega": wheel, "v": 0.0})
        rows.append({"source": "filtered", "stamp": tt, "received": tt, "omega": filt, "v": 0.0})
    (gap,) = ic.imu_report(rows)["imu_gaps"]
    assert gap["silent_fallback_to_wheel_odometry"]
    assert gap["filtered_minus_truth_during"] > 0.01 > gap["filtered_minus_wheel_during"]
