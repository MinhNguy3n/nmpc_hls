#!/usr/bin/env python3
"""IMU bridge and EKF checks for the A300 IN SIMULATION.

Sub-commands
    check-generated  offline: inspect <setup_path>/sensors/config/imu_0.yaml written by the
                     Clearpath generator. Exit codes (LOUD on stderr):
                       1  EMPTY bridge list: the generator defect is present -> launch the
                          version-controlled bridge (launch/a300_sim_imu_bridge.launch.py)
                       3  the generator now bridges the IMU -> do NOT launch the extra bridge
                       4  file missing or unreadable (simulation not launched yet, wrong path)
                       0  non-empty but without the IMU topic (unexpected; inspect by hand)
    capture          passive (no commands): record the IMU stream (stamp, receive time, frame_id,
                     covariances), platform/odom/filtered, platform/odom and Gazebo ground truth
                     (`gz topic -e`) to a CSV. Drive the robot with `mux_timing.py scenario turn`
                     in another shell, and stop the IMU bridge mid-run for the loss test.
    report           offline: IMU frame/stamp/covariance/rate summary; IMU gaps and whether the EKF
                     silently fell back to wheel odometry; filtered yaw-rate latency against ground
                     truth (lag of best fit and 50 % step-crossing delays)

Runs from the read-only tree; writes only to --out. Offline parts are tested (test_imu_checks.py).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from identify_effective_track import freshness_stats, parse_pose_v_text, yaw_from_quaternion  # noqa: E402

NAMESPACE = "a300_00000"
IMU_TOPIC = f"/{NAMESPACE}/sensors/imu_0/data"
FIELDS = ("source", "stamp", "received", "frame_id", "omega", "v", "ax", "yaw",
          "cov_orientation0", "cov_gyro_z", "cov_accel_x")


# ----------------------------------------------------------------------------------------------
# Generated bridge check
# ----------------------------------------------------------------------------------------------


def check_generated(setup_path: str | Path) -> tuple[int, str]:
    path = Path(setup_path).expanduser() / "sensors" / "config" / "imu_0.yaml"
    try:
        text = path.read_text()
    except OSError as error:
        return 4, f"IMU BRIDGE CHECK: cannot read {path}: {error}. Launch the simulation first."
    try:
        import yaml
        data = yaml.safe_load(text)
    except ImportError:  # minimal fallback: treat '[]' or blank as empty
        data = [] if text.strip() in ("", "[]") else [{"raw": text}]
    except Exception as error:  # noqa: BLE001 - report any parse error loudly
        return 4, f"IMU BRIDGE CHECK: {path} is not valid YAML: {error}"
    entries = data if isinstance(data, list) else ([data] if data else [])
    if not entries:
        return 1, (f"IMU BRIDGE CHECK: {path} is EMPTY ([]). The generator defect is present: the IMU does "
                   f"not reach ROS. Launch launch/a300_sim_imu_bridge.launch.py.")
    if any("sensors/imu_0/data" in json.dumps(e) for e in entries):  # absolute or relative topic names
        return 3, (f"IMU BRIDGE CHECK: {path} now bridges {IMU_TOPIC}. The generator changed: do NOT launch "
                   f"the extra bridge (duplicate publishers); update the plan.")
    return 0, f"IMU BRIDGE CHECK: {path} has {len(entries)} entries but none for {IMU_TOPIC}; inspect it."


# ----------------------------------------------------------------------------------------------
# Offline analysis
# ----------------------------------------------------------------------------------------------


def read_capture(path: str | Path) -> list[dict[str, Any]]:
    rows = []
    with open(path, newline="") as handle:
        for raw in csv.DictReader(handle):
            row: dict[str, Any] = {"source": raw["source"], "frame_id": raw.get("frame_id") or None}
            for key in FIELDS:
                if key in ("source", "frame_id"):
                    continue
                row[key] = float(raw[key]) if raw.get(key) else None
            rows.append(row)
    return rows


def write_capture(path: str | Path, rows: list[dict[str, Any]]) -> None:
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: "" if row.get(k) is None else row[k] for k in FIELDS})


def truth_yaw_rate(rows: list[dict[str, Any]]) -> tuple[np.ndarray, np.ndarray]:
    gt = sorted((r for r in rows if r["source"] == "truth"), key=lambda r: r["stamp"])
    t = np.array([r["stamp"] for r in gt])
    yaw = np.unwrap([r["yaw"] for r in gt])
    if len(t) < 3:
        return t, np.zeros(len(t))
    return t, np.gradient(yaw, t)


def best_lag(t_ref: np.ndarray, y_ref: np.ndarray, t_sig: np.ndarray, y_sig: np.ndarray,
             max_lag: float = 0.5, dt: float = 0.005) -> tuple[float, float]:
    """Lag L >= 0 minimizing the RMS of y_sig(t) - y_ref(t - L); returns (L, rms)."""
    lo, hi = max(t_ref[0], t_sig[0]) + max_lag, min(t_ref[-1], t_sig[-1])
    grid = np.arange(lo, hi, dt)
    if len(grid) < 10:
        return math.nan, math.nan
    sig = np.interp(grid, t_sig, y_sig)
    best = (math.nan, math.inf)
    for lag in np.arange(0.0, max_lag + 1e-12, dt):
        rms = float(np.sqrt(np.mean((sig - np.interp(grid - lag, t_ref, y_ref)) ** 2)))
        if rms < best[1]:
            best = (float(lag), rms)
    return best


def step_delays(t_ref: np.ndarray, y_ref: np.ndarray, t_sig: np.ndarray, y_sig: np.ndarray) -> list[float]:
    """For each rising |y_ref| crossing of half its peak, the delay until |y_sig| crosses it too."""
    level = 0.5 * float(np.max(np.abs(y_ref))) if len(y_ref) else 0.0
    if level <= 0:
        return []
    delays = []
    above_ref = np.abs(y_ref) >= level
    for i in np.flatnonzero(above_ref[1:] & ~above_ref[:-1]) + 1:
        t0 = t_ref[i]
        after = np.flatnonzero((t_sig >= t0 - 0.2) & (np.abs(y_sig) >= level))
        if len(after):
            delays.append(float(t_sig[after[0]] - t0))
    return delays


def imu_report(rows: list[dict[str, Any]], gap: float = 0.2) -> dict[str, Any]:
    imu = sorted((r for r in rows if r["source"] == "imu"), key=lambda r: r["received"])
    filt = sorted((r for r in rows if r["source"] == "filtered"), key=lambda r: r["stamp"])
    wheel = sorted((r for r in rows if r["source"] == "wheel"), key=lambda r: r["stamp"])
    out: dict[str, Any] = {}
    if len(imu) >= 3:
        st = freshness_stats([r["stamp"] for r in imu], [r["received"] for r in imu])
        out["imu"] = {"count": st.count, "rate_hz": st.rate_hz, "dt_max": st.dt_max,
                      "age_median": st.age_median, "age_max": st.age_max, "age_min": st.age_min,
                      "non_monotonic": st.non_monotonic,
                      "frame_ids": sorted({r["frame_id"] for r in imu if r["frame_id"]}),
                      "stamp_clock": ("simulation (age small)" if abs(st.age_median) < 1.0
                                      else "NOT the node clock (age %.1f s): wall vs sim time?" % st.age_median),
                      "orientation_covariance0": sorted({r["cov_orientation0"] for r in imu if r["cov_orientation0"] is not None}),
                      "gyro_z_covariance": sorted({r["cov_gyro_z"] for r in imu if r["cov_gyro_z"] is not None}),
                      "accel_x_covariance": sorted({r["cov_accel_x"] for r in imu if r["cov_accel_x"] is not None})}
    else:
        out["imu"] = {"count": len(imu), "warning": "no or almost no IMU messages: is the bridge running?"}
    # IMU gaps (and an IMU stream that ends early) vs the filtered output
    t_end = max((r["received"] for r in rows), default=0.0)
    gaps = []
    times = [r["received"] for r in imu]
    for a, b in zip(times[:-1], times[1:]):
        if b - a > gap:
            gaps.append((a, b))
    if times and t_end - times[-1] > gap:
        gaps.append((times[-1], t_end))
    t_gt, w_gt = truth_yaw_rate(rows)
    findings = []
    for a, b in gaps:
        during = [r for r in filt if a + 0.3 <= r["stamp"] <= b]  # after the EKF's own settling
        before = [r for r in filt if a - 2.0 <= r["stamp"] < a]
        item: dict[str, Any] = {"gap_start": a, "gap_end": b, "filtered_samples_during": len(during)}
        if during and len(t_gt) >= 3:
            w_f = np.array([r["omega"] for r in during])
            w_g = np.interp([r["stamp"] for r in during], t_gt, w_gt)
            item["filtered_minus_truth_during"] = float(np.mean(w_f - w_g))
            if before:
                wb = np.interp([r["stamp"] for r in before], t_gt, w_gt)
                item["filtered_minus_truth_before"] = float(np.mean(np.array([r["omega"] for r in before]) - wb))
            if wheel:
                w_w = np.interp([r["stamp"] for r in during], [r["stamp"] for r in wheel], [r["omega"] for r in wheel])
                item["filtered_minus_wheel_during"] = float(np.mean(w_f - w_w))
                item["silent_fallback_to_wheel_odometry"] = bool(
                    abs(item["filtered_minus_wheel_during"]) < 0.01 * max(1e-6, float(np.max(np.abs(w_w))))
                    and len(during) > 0)
        findings.append(item)
    out["imu_gaps"] = findings
    if filt and len(t_gt) >= 3:
        t_f = np.array([r["stamp"] for r in filt])
        w_f = np.array([r["omega"] for r in filt])
        lag, rms = best_lag(t_gt, w_gt, t_f, w_f)
        out["filtered_latency"] = {"best_lag": lag, "rms_at_best_lag": rms,
                                   "step_delays_50pct": step_delays(t_gt, w_gt, t_f, w_f)}
    return out


# ----------------------------------------------------------------------------------------------
# ROS capture
# ----------------------------------------------------------------------------------------------


def _capture(args: argparse.Namespace, ros_args: list[str]) -> int:  # pragma: no cover - needs ROS 2
    import subprocess
    import threading
    import time

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu

    rclpy.init(args=["imu_checks"] + ros_args)
    node = Node("imu_checks")
    clock = node.get_clock()
    rows: list[dict[str, Any]] = []

    def now() -> float:
        return clock.now().nanoseconds * 1e-9

    def stamp(m: Any) -> float:
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def on_imu(m: Any) -> None:
        rows.append({"source": "imu", "stamp": stamp(m), "received": now(), "frame_id": m.header.frame_id,
                     "omega": m.angular_velocity.z, "ax": m.linear_acceleration.x,
                     "cov_orientation0": m.orientation_covariance[0], "cov_gyro_z": m.angular_velocity_covariance[8],
                     "cov_accel_x": m.linear_acceleration_covariance[0]})

    def on_odom(source: str):
        def cb(m: Any) -> None:
            q = m.pose.pose.orientation
            rows.append({"source": source, "stamp": stamp(m), "received": now(), "frame_id": m.header.frame_id,
                         "omega": m.twist.twist.angular.z, "v": m.twist.twist.linear.x,
                         "yaw": yaw_from_quaternion(q.x, q.y, q.z, q.w)})
        return cb

    ns = "/" + args.namespace
    node.create_subscription(Imu, f"{ns}/sensors/imu_0/data", on_imu, qos_profile_sensor_data)
    node.create_subscription(Odometry, f"{ns}/platform/odom/filtered", on_odom("filtered"), qos_profile_sensor_data)
    node.create_subscription(Odometry, f"{ns}/platform/odom", on_odom("wheel"), qos_profile_sensor_data)
    proc = subprocess.Popen(["gz", "topic", "-e", "-t", args.gz_topic], stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, text=True, bufsize=1)

    def read_gz() -> None:
        for t_gz, poses in parse_pose_v_text(proc.stdout):
            if args.gt_name in poses:
                rows.append({"source": "truth", "stamp": t_gz, "received": now(), "yaw": poses[args.gt_name][2]})

    threading.Thread(target=read_gz, daemon=True).start()
    start, guard = None, time.monotonic() + args.duration * 4 + 10
    try:
        while rclpy.ok() and time.monotonic() < guard:
            rclpy.spin_once(node, timeout_sec=0.05)
            t = now()
            start = t if (start is None and t > 0) else start
            if start is not None and t - start > args.duration:
                break
    except KeyboardInterrupt:
        pass
    proc.terminate()
    write_capture(args.out, rows)
    print(f"wrote {len(rows)} rows to {args.out}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    ros_args: list[str] = []
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        argv, ros_args = argv[:i], argv[i:]
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("check-generated")
    p.add_argument("--setup-path", default="~/clearpath")
    p = sub.add_parser("capture")
    p.add_argument("--namespace", default=NAMESPACE)
    p.add_argument("--duration", type=float, default=35.0)
    p.add_argument("--gz-topic", default="/world/warehouse/dynamic_pose/info")
    p.add_argument("--gt-name", default="a300_00000/robot")
    p.add_argument("--out", required=True)
    p = sub.add_parser("report")
    p.add_argument("capture_csv")
    p.add_argument("--out", default=None)
    args = parser.parse_args(argv)
    if args.command == "check-generated":
        code, message = check_generated(args.setup_path)
        print(message, file=sys.stderr if code else sys.stdout)
        if code == 1:
            print(message)  # also on stdout so the launch file can log it
        return code
    if args.command == "report":
        text = json.dumps(imu_report(read_capture(args.capture_csv)), indent=2, default=str)
        print(text)
        if args.out:
            Path(args.out).expanduser().write_text(text)
        return 0
    return _capture(args, ros_args)


if __name__ == "__main__":
    sys.exit(main())
