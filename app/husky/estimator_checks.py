#!/usr/bin/env python3
"""Estimator validation for the A300 SIL IN SIMULATION: odometry sources against Gazebo truth.

Sub-commands
    capture  passive (no commands): record any number of nav_msgs/Odometry topics (pose, twist,
             yaw and yaw-rate covariances) and the Gazebo ground truth (`gz topic -e`, with height
             and tilt) to a CSV. Drive the robot with `mux_timing.py scenario estimator --sim`
             in another shell (still, in-place turns both ways, 1 m arcs both ways).
    report   offline: for every source, the pose is aligned to the truth once, at the scenario
             start (translation and heading), and then never corrected. Per profile segment: the
             accumulated heading and position error at its end, the yaw-rate error and the ratio
             of the source's to the true yaw rate while turning; overall: the best-fit yaw-rate
             lag and the median reported covariances.

Runs from the read-only tree; writes only to --out. The analysis is tested offline
(test_estimator_checks.py).
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
from identify_effective_track import parse_pose_v_text, yaw_from_quaternion  # noqa: E402
from imu_checks import best_lag, truth_yaw_rate  # noqa: E402
from mux_timing import ESTIMATOR_PROFILE  # noqa: E402

NAMESPACE = "a300_00000"
FIELDS = ("source", "stamp", "received", "x", "y", "yaw", "v", "omega", "cov_yaw", "cov_wz", "z", "tilt")
SEGMENTS = ("still", "turn_left", "turn_right", "arc_left", "arc_right", "still_end")
DEFAULT_SOURCES = ("stock=platform/odom/filtered", "wheel=platform/odom")


def read_capture(path: str | Path) -> list[dict[str, Any]]:
    with open(path, newline="") as handle:
        return [{k: (v if k == "source" else (float(v) if v else None)) for k, v in raw.items()}
                for raw in csv.DictReader(handle)]


def write_capture(path: str | Path, rows: list[dict[str, Any]]) -> None:
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: "" if row.get(k) is None else row[k] for k in FIELDS})


def _wrap(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi


def _series(rows: list[dict[str, Any]], source: str) -> dict[str, np.ndarray]:
    sel = sorted((r for r in rows if r["source"] == source), key=lambda r: r["stamp"])
    out = {"t": np.array([r["stamp"] for r in sel])}
    for key in ("x", "y", "omega", "cov_yaw", "cov_wz"):
        out[key] = np.array([np.nan if r.get(key) is None else r[key] for r in sel])
    out["yaw"] = np.unwrap([r["yaw"] for r in sel]) if sel else np.array([])
    return out


def segment_bounds(start: float, profile=ESTIMATOR_PROFILE) -> list[tuple[str, float, float]]:
    out, t = [], start
    for name, (duration, _, _) in zip(SEGMENTS, profile):
        out.append((name, t, t + duration))
        t += duration
    return out


def estimator_report(rows: list[dict[str, Any]], start: float, settle: float = 0.5) -> dict[str, Any]:
    """`start`: the scenario start in simulation time (first stamp of the scenario's sent file)."""
    truth = _series(rows, "truth")
    if len(truth["t"]) < 3:
        return {"error": "no ground truth"}
    t_w, w_true = truth_yaw_rate(rows)
    segments = segment_bounds(start)
    report: dict[str, Any] = {"start": start, "segments": [s[0] for s in segments], "sources": {}}
    tilts = [r["tilt"] for r in rows if r["source"] == "truth" and r.get("tilt") is not None]
    report["truth_tilt_max"] = max(tilts) if tilts else None
    for source in sorted({r["source"] for r in rows} - {"truth"}):
        s = _series(rows, source)
        if len(s["t"]) < 3 or s["t"][0] > start or s["t"][-1] < segments[-1][2] - 0.1:
            report["sources"][source] = {"error": "does not cover the scenario"}
            continue
        # align once at the start: the source's pose moved onto the true pose
        x0, y0, h0 = (float(np.interp(start, s["t"], s[k])) for k in ("x", "y", "yaw"))
        gx0, gy0, gh0 = (float(np.interp(start, truth["t"], truth[k])) for k in ("x", "y", "yaw"))
        dh = gh0 - h0
        c, n = math.cos(dh), math.sin(dh)

        def pose_error(t: float) -> tuple[float, float]:
            sx, sy, sh = (float(np.interp(t, s["t"], s[k])) for k in ("x", "y", "yaw"))
            ax = gx0 + c * (sx - x0) - n * (sy - y0)
            ay = gy0 + n * (sx - x0) + c * (sy - y0)
            tx, ty, th = (float(np.interp(t, truth["t"], truth[k])) for k in ("x", "y", "yaw"))
            return math.hypot(ax - tx, ay - ty), float(_wrap(np.array(sh + dh - th)))

        per_segment = {}
        for name, a, b in segments:
            pos_err, yaw_err = pose_error(b)
            m = (s["t"] >= a + settle) & (s["t"] <= b)
            w_t = np.interp(s["t"][m], t_w, w_true)
            w_s = s["omega"][m]
            item = {"pos_error_end": pos_err, "heading_error_end": yaw_err,
                    "yaw_rate_mae": float(np.mean(np.abs(w_s - w_t))) if m.any() else math.nan}
            if m.any() and np.mean(np.abs(w_t)) > 0.05:
                item["yaw_rate_ratio"] = float(np.mean(w_s) / np.mean(w_t))
            per_segment[name] = item
        lag, rms = best_lag(t_w, w_true, s["t"], s["omega"])
        end = segments[-1][2]
        pos_end, yaw_end = pose_error(end)
        report["sources"][source] = {
            "segments": per_segment, "pos_error_final": pos_end, "heading_error_final": yaw_end,
            "heading_error_max": max(abs(pose_error(t)[1]) for t in np.arange(start, end, 0.1)),
            "yaw_rate_lag": lag, "yaw_rate_rms_at_lag": rms,
            "cov_yaw_median": float(np.nanmedian(s["cov_yaw"])), "cov_wz_median": float(np.nanmedian(s["cov_wz"]))}
    return report


def _capture(args: argparse.Namespace, ros_args: list[str]) -> int:  # pragma: no cover - needs ROS 2
    import subprocess
    import threading
    import time

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init(args=["estimator_checks"] + ros_args)
    node = rclpy.create_node("estimator_checks")
    clock = node.get_clock()
    rows: list[dict[str, Any]] = []

    def now() -> float:
        return clock.now().nanoseconds * 1e-9

    def on_odom(source: str):
        def cb(m: Any) -> None:
            q = m.pose.pose.orientation
            rows.append({"source": source, "stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                         "received": now(), "x": m.pose.pose.position.x, "y": m.pose.pose.position.y,
                         "yaw": yaw_from_quaternion(q.x, q.y, q.z, q.w), "v": m.twist.twist.linear.x,
                         "omega": m.twist.twist.angular.z, "cov_yaw": m.pose.covariance[35],
                         "cov_wz": m.twist.covariance[35]})
        return cb

    for item in args.odom or DEFAULT_SOURCES:
        name, _, topic = item.partition("=")
        node.create_subscription(Odometry, f"/{args.namespace}/{topic}", on_odom(name), qos_profile_sensor_data)
    proc = subprocess.Popen(["gz", "topic", "-e", "-t", args.gz_topic], stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, text=True, bufsize=1)

    def read_gz() -> None:
        for t_gz, poses in parse_pose_v_text(proc.stdout, full=True):
            if args.gt_name in poses:
                x, y, yaw, z, tilt = poses[args.gt_name]
                rows.append({"source": "truth", "stamp": t_gz, "received": now(), "x": x, "y": y, "yaw": yaw,
                             "z": z, "tilt": tilt})

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
    rclpy.try_shutdown()
    return 0


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    ros_args: list[str] = []
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        argv, ros_args = argv[:i], argv[i:]
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("capture")
    p.add_argument("--namespace", default=NAMESPACE)
    p.add_argument("--odom", action="append", help="NAME=TOPIC (relative to the namespace); repeatable")
    p.add_argument("--duration", type=float, default=42.0, help="simulation seconds")
    p.add_argument("--gz-topic", default="/world/warehouse/dynamic_pose/info")
    p.add_argument("--gt-name", default="a300_00000/robot")
    p.add_argument("--out", required=True)
    p = sub.add_parser("report")
    p.add_argument("capture_csv")
    p.add_argument("--sent", required=True, help="the scenario's sent CSV (mux_timing.py scenario estimator --out)")
    p.add_argument("--out", default=None)
    args = parser.parse_args(argv)
    if args.command == "capture":
        return _capture(args, ros_args)
    with open(args.sent, newline="") as handle:
        start = min(float(r["received"]) for r in csv.DictReader(handle) if r["topic"] == "sent:cmd_vel")
    result = estimator_report(read_capture(args.capture_csv), start)
    text = json.dumps(result, indent=2)
    print(text)
    if args.out:
        Path(args.out).write_text(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
