#!/usr/bin/env python3
"""Effective-track identification and topic-freshness check for the Husky A300 IN SIMULATION.

Run by the user inside the Jazzy container (plan section 2.1, checks V13 and the identification
step). SIMULATION ONLY: the `record` command refuses to run unless `--sim` is given AND the node
runs with use_sim_time:=true. It is never to be pointed at a real robot.

Sub-commands
    list-gt-frames  print the frame names seen on the ground-truth topic (to choose --gt-frame)
    record          drive a short low-speed script of in-place turns and arcs on /<ns>/cmd_vel and
                    record ground truth, platform/odom, joint_states and cmd_vel_out to a CSV
    freshness       passive (no commands): record stamps and receive times of the topics (V13)
    analyze         offline: effective track and its uncertainty from a `record` CSV
    freshness-report offline: rates, stamp jitter and age statistics from a `freshness` CSV

Ground truth (Gazebo pose), --gt-source:
    gz   (default) read `gz topic -e -t /world/<world>/dynamic_pose/info` in a background process
         and select the pose by its Gazebo name (--gt-frame, in simulation a300_00000/robot; list the
         names with list-gt-frames). Needs no bridge and no Gazebo Python bindings.
    ros  a ros_gz_bridge Pose_V -> tf2_msgs/TFMessage topic. On Jazzy/Harmonic (2026-10-26) that
         bridge leaves frame_id and child_frame_id empty for dynamic_pose/info, so the robot cannot be
         selected by name; kept for setups where the poses carry names.
The world name must match the running world (`gz topic -l | grep dynamic_pose`).

The analysis functions are plain numpy and are tested offline (test_identify_effective_track.py);
rclpy and the ROS messages are imported only inside the ROS sub-commands.

Interpretation (design note 4.4): the simulated effective track validates the measurement
pipeline only. It is not the real robot's track or slip model.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Any, Iterable

import numpy as np

NAMESPACE = "a300_00000"
TRACK_NOMINAL = 0.562  # m, A300 manual / diff_4wd.yaml wheel_separation (multiplier not included)
WHEEL_RADIUS = 0.1625  # m, A300 manual / diff_4wd.yaml; read the sim value with ros2 param get
LEFT_JOINTS = ("front_left_wheel_joint", "rear_left_wheel_joint")  # diff_4wd.yaml
RIGHT_JOINTS = ("front_right_wheel_joint", "rear_right_wheel_joint")
V_CAP, OMEGA_CAP = 0.3, 0.6  # hard caps of the scripted sequence (m/s, rad/s): low speed only

# (duration s, v m/s, omega rad/s): in-place turns both ways at two rates, then arcs (radius 1 m).
SEQUENCE: tuple[tuple[float, float, float], ...] = (
    (2.0, 0.0, 0.0), (5.0, 0.0, 0.3), (2.0, 0.0, 0.0), (5.0, 0.0, -0.3), (2.0, 0.0, 0.0),
    (5.0, 0.0, 0.6), (2.0, 0.0, 0.0), (5.0, 0.0, -0.6), (2.0, 0.0, 0.0),
    (6.0, 0.3, 0.3), (2.0, 0.0, 0.0), (6.0, 0.3, -0.3), (3.0, 0.0, 0.0),
)

FIELDS = ("source", "t", "t_recv", "x", "y", "yaw", "v", "omega", "v_left", "v_right",
          "seg", "v_cmd", "omega_cmd", "t_end")


# ----------------------------------------------------------------------------------------------
# Offline analysis (no ROS)
# ----------------------------------------------------------------------------------------------


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def write_csv(path: str | Path, rows: Iterable[dict[str, Any]]) -> None:
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS, extrasaction="raise")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in FIELDS})


def read_csv(path: str | Path) -> list[dict[str, Any]]:
    rows = []
    with open(path, newline="") as handle:
        for raw in csv.DictReader(handle):
            row: dict[str, Any] = {"source": raw["source"]}
            for key in FIELDS[1:]:
                row[key] = float(raw[key]) if raw.get(key, "") != "" else None
            rows.append(row)
    return rows


def _slope_with_se(t: np.ndarray, y: np.ndarray) -> tuple[float, float]:
    """Least-squares slope of y(t) and its standard error."""
    t0 = t - t.mean()
    sxx = float(np.sum(t0 * t0))
    slope = float(np.sum(t0 * (y - y.mean())) / sxx)
    residual = y - y.mean() - slope * t0
    dof = max(len(t) - 2, 1)
    return slope, float(math.sqrt(float(np.sum(residual ** 2)) / dof / sxx))


@dataclass(frozen=True)
class SegmentResult:
    seg: int
    v_cmd: float
    omega_cmd: float
    omega_gt: float
    omega_gt_se: float
    omega_odom: float | None
    wheel_diff: float  # mean (v_r - v_l), m/s
    wheel_diff_se: float
    track_eff: float | None  # wheel_diff / omega_gt
    track_eff_se: float | None
    ratio_gt_cmd: float  # omega_gt / omega_cmd


@dataclass(frozen=True)
class TrackResult:
    segments: tuple[SegmentResult, ...]
    track_eff: float  # pooled fit of wheel_diff = W_eff * omega_gt through the origin
    track_eff_se: float
    track_eff_min: float
    track_eff_max: float
    ratio_gt_cmd_mean: float
    multiplier_implied: float  # W_eff / TRACK_NOMINAL, compare with wheel_separation_multiplier


def analyze(rows: list[dict[str, Any]], settle: float = 1.0, min_rate: float = 0.05,
            track_nominal: float = TRACK_NOMINAL) -> TrackResult:
    """Effective track from steady parts of the turning segments of a `record` CSV.

    Per segment (window = segment start + settle .. segment end): omega_gt from a line fit of the
    unwrapped ground-truth yaw, wheel_diff from joint_states, omega_odom from platform/odom.
    W_eff = wheel_diff / omega_gt is the track that makes the manual's omega = (v_r - v_l)/W
    reproduce the TRUE yaw rate. Pooled W_eff: least squares through the origin over segments
    with |omega_gt| >= min_rate; its standard error from the fit residuals.
    """
    segs = [r for r in rows if r["source"] == "seg"]
    gt = sorted((r for r in rows if r["source"] == "gt"), key=lambda r: r["t"])
    odom = [r for r in rows if r["source"] == "odom"]
    joints = [r for r in rows if r["source"] == "joints"]
    if not gt or not joints:
        raise ValueError("the CSV needs ground-truth ('gt') and joint ('joints') rows")
    gt_t = np.array([r["t"] for r in gt])
    gt_yaw = np.unwrap(np.array([r["yaw"] for r in gt]))
    results = []
    for seg in segs:
        if abs(seg["omega_cmd"]) < min_rate:
            continue
        lo, hi = seg["t"] + settle, seg["t_end"]
        mask = (gt_t >= lo) & (gt_t <= hi)
        if mask.sum() < 5:
            continue
        omega_gt, omega_se = _slope_with_se(gt_t[mask], gt_yaw[mask])
        diffs = np.array([r["v_right"] - r["v_left"] for r in joints if lo <= r["t"] <= hi])
        if len(diffs) < 3:
            continue
        diff_mean = float(diffs.mean())
        diff_se = float(diffs.std(ddof=1) / math.sqrt(len(diffs)))
        odom_w = [r["omega"] for r in odom if lo <= r["t"] <= hi]
        track = track_se = None
        if abs(omega_gt) >= min_rate:
            track = diff_mean / omega_gt
            track_se = abs(track) * math.hypot(diff_se / abs(diff_mean) if diff_mean else 0.0, omega_se / abs(omega_gt))
        results.append(SegmentResult(int(seg["seg"]), seg["v_cmd"], seg["omega_cmd"], omega_gt, omega_se,
                                     float(np.mean(odom_w)) if odom_w else None, diff_mean, diff_se,
                                     track, track_se, omega_gt / seg["omega_cmd"]))
    usable = [r for r in results if r.track_eff is not None]
    if not usable:
        raise ValueError("no turning segment with enough samples")
    w = np.array([r.omega_gt for r in usable])
    d = np.array([r.wheel_diff for r in usable])
    track = float(np.sum(d * w) / np.sum(w * w))
    if len(usable) >= 2:
        residual = d - track * w
        track_se = float(math.sqrt(float(np.sum(residual ** 2)) / (len(usable) - 1) / float(np.sum(w * w))))
    else:
        track_se = float(usable[0].track_eff_se)
    tracks = [r.track_eff for r in usable]
    return TrackResult(tuple(results), track, track_se, float(min(tracks)), float(max(tracks)),
                       float(np.mean([r.ratio_gt_cmd for r in results])), track / track_nominal)


@dataclass(frozen=True)
class FreshnessStats:
    count: int
    rate_hz: float  # from the median stamp interval
    dt_std: float  # stamp-interval jitter (s)
    dt_max: float
    age_median: float  # receive time - stamp (s)
    age_p95: float
    age_max: float
    age_min: float  # negative = stamp in the future of the receiver clock
    non_monotonic: int  # stamps not strictly increasing (duplicates or reordering)


def freshness_stats(stamps: Iterable[float], received: Iterable[float]) -> FreshnessStats:
    s = np.asarray(list(stamps), dtype=float)
    r = np.asarray(list(received), dtype=float)
    if len(s) < 3 or len(s) != len(r):
        raise ValueError("need at least 3 samples with matching receive times")
    dt = np.diff(s)
    age = r - s
    return FreshnessStats(len(s), float(1.0 / np.median(dt)) if np.median(dt) > 0 else float("nan"),
                          float(np.std(dt)), float(np.max(dt)), float(np.median(age)),
                          float(np.percentile(age, 95)), float(np.max(age)), float(np.min(age)),
                          int(np.sum(dt <= 0)))


def parse_pose_v_text(lines: Iterable[str], full: bool = False) -> Iterable[tuple[float, dict[str, tuple[float, ...]]]]:
    """Parse `gz topic -e` text output of gz.msgs.Pose_V into (stamp s, {name: (x, y, yaw)}) per message.

    With full=True the values are (x, y, yaw, z, tilt), tilt being the angle between the body z
    axis and the vertical (rad): a tipped-over robot keeps moving its wheels and its odometry.

    Protobuf text omits zero-valued fields, so a missing coordinate is 0. A message starts at each
    top-level `header {` block; poses are the top-level `pose {` blocks that follow it.
    """
    stack: list[str] = []
    stamp = {"sec": 0, "nsec": 0}
    poses: dict[str, tuple[float, float, float]] = {}
    pose: dict[str, Any] = {}
    started = False

    def finish_pose() -> None:
        if pose.get("name") is not None:
            q = pose.get("orientation", {})
            p = pose.get("position", {})
            qx, qy, qz, qw = (q.get(k, 0.0) for k in ("x", "y", "z", "w"))
            value: tuple[float, ...] = (p.get("x", 0.0), p.get("y", 0.0), yaw_from_quaternion(qx, qy, qz, qw))
            if full:
                n2 = qx * qx + qy * qy + qz * qz + qw * qw or 1.0
                value += (p.get("z", 0.0), math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (qx * qx + qy * qy) / n2))))
            poses[pose["name"]] = value

    for raw in lines:
        line = raw.strip()
        if not line:
            continue
        if line.endswith("{"):
            block = line[:-1].strip()
            if not stack and block == "header":
                if started:
                    yield stamp["sec"] + stamp["nsec"] * 1e-9, poses
                stamp, poses, started = {"sec": 0, "nsec": 0}, {}, True
            if not stack and block == "pose":
                pose = {}
            stack.append(block)
            continue
        if line == "}":
            closed = stack.pop() if stack else ""
            if not stack and closed == "pose":
                finish_pose()
            continue
        key, _, value = line.partition(":")
        key, value = key.strip(), value.strip()
        if stack == ["header", "stamp"] and key in ("sec", "nsec"):
            stamp[key] = int(value)
        elif stack == ["pose"] and key == "name":
            pose["name"] = value.strip('"')
        elif len(stack) == 2 and stack[0] == "pose" and stack[1] in ("position", "orientation"):
            pose.setdefault(stack[1], {})[key] = float(value)
    if started:
        yield stamp["sec"] + stamp["nsec"] * 1e-9, poses


def freshness_report(rows: list[dict[str, Any]]) -> dict[str, FreshnessStats]:
    report = {}
    for source in sorted({r["source"] for r in rows}):
        sel = [r for r in rows if r["source"] == source]
        if len(sel) >= 3:
            report[source] = freshness_stats([r["t"] for r in sel], [r["t_recv"] for r in sel])
    return report


# ----------------------------------------------------------------------------------------------
# ROS sub-commands (rclpy imported lazily; run inside the Jazzy container only)
# ----------------------------------------------------------------------------------------------


def _ros_main(args: argparse.Namespace, ros_args: list[str]) -> int:  # pragma: no cover - needs ROS
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage

    rclpy.init(args=ros_args)
    node = Node("effective_track_id")
    use_sim_time = bool(node.get_parameter("use_sim_time").value)
    if args.command == "record" and not (args.sim and use_sim_time):
        node.get_logger().error("record is SIMULATION ONLY: pass --sim and --ros-args -p use_sim_time:=true")
        rclpy.shutdown()
        return 2
    if not use_sim_time:
        node.get_logger().warn("use_sim_time is false: stamps and receive times use different clocks")
    ns = "/" + args.namespace
    rows: list[dict[str, Any]] = []
    frames: set[str] = set()

    def now() -> float:
        return node.get_clock().now().nanoseconds * 1e-9

    def stamp(msg: Any) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_gt(msg: Any) -> None:
        for tf in msg.transforms:
            frames.add(f"{tf.header.frame_id} -> {tf.child_frame_id}")
            if tf.child_frame_id == args.gt_frame:
                q = tf.transform.rotation
                rows.append({"source": "gt", "t": stamp(tf), "t_recv": now(), "x": tf.transform.translation.x,
                             "y": tf.transform.translation.y, "yaw": yaw_from_quaternion(q.x, q.y, q.z, q.w)})

    def on_odom(msg: Any) -> None:
        q = msg.pose.pose.orientation
        rows.append({"source": "odom", "t": stamp(msg), "t_recv": now(), "x": msg.pose.pose.position.x,
                     "y": msg.pose.pose.position.y, "yaw": yaw_from_quaternion(q.x, q.y, q.z, q.w),
                     "v": msg.twist.twist.linear.x, "omega": msg.twist.twist.angular.z})

    def on_joints(msg: Any) -> None:
        speed = dict(zip(msg.name, msg.velocity))
        if all(j in speed for j in LEFT_JOINTS + RIGHT_JOINTS):
            rows.append({"source": "joints", "t": stamp(msg), "t_recv": now(),
                         "v_left": args.wheel_radius * float(np.mean([speed[j] for j in LEFT_JOINTS])),
                         "v_right": args.wheel_radius * float(np.mean([speed[j] for j in RIGHT_JOINTS]))})

    def on_twist(source: str) -> Any:
        def callback(msg: Any) -> None:
            rows.append({"source": source, "t": stamp(msg), "t_recv": now(),
                         "v": msg.twist.linear.x, "omega": msg.twist.angular.z})
        return callback

    gz_process = None
    if args.gt_source == "ros":
        node.create_subscription(TFMessage, args.gt_topic, on_gt, qos_profile_sensor_data)
    else:
        import subprocess
        import threading

        gz_process = subprocess.Popen(["gz", "topic", "-e", "-t", args.gz_topic], stdout=subprocess.PIPE,
                                      stderr=subprocess.DEVNULL, text=True, bufsize=1)

        def read_gz() -> None:
            for t_gz, poses in parse_pose_v_text(gz_process.stdout):
                frames.update(poses)
                if args.gt_frame in poses:
                    x, y, yaw = poses[args.gt_frame]
                    rows.append({"source": "gt", "t": t_gz, "t_recv": now(), "x": x, "y": y, "yaw": yaw})

        threading.Thread(target=read_gz, daemon=True).start()
    if args.command != "list-gt-frames":
        node.create_subscription(Odometry, f"{ns}/platform/odom", on_odom, qos_profile_sensor_data)
        node.create_subscription(JointState, args.joint_topic, on_joints, qos_profile_sensor_data)
        node.create_subscription(TwistStamped, f"{ns}/platform_velocity_controller/cmd_vel_out",
                                 on_twist("cmd_vel_out"), qos_profile_sensor_data)
        node.create_subscription(TwistStamped, f"{ns}/platform/cmd_vel", on_twist("mux_out"), qos_profile_sensor_data)

    publisher = node.create_publisher(TwistStamped, f"{ns}/cmd_vel", 10) if args.command == "record" else None
    plan = [(d, max(-V_CAP, min(V_CAP, v)), max(-OMEGA_CAP, min(OMEGA_CAP, w))) for d, v, w in SEQUENCE]
    state = {"t0": None, "seg": -1, "done": False}

    def send(v: float, w: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
        publisher.publish(msg)

    def on_timer() -> None:
        t = now()
        if t <= 0.0:
            return  # wait for /clock
        if state["t0"] is None:
            state["t0"] = t
        elapsed, start = t - state["t0"], 0.0
        for index, (duration, v, w) in enumerate(plan):
            if elapsed < start + duration:
                if index != state["seg"]:
                    state["seg"] = index
                    rows.append({"source": "seg", "t": t, "seg": index, "v_cmd": v, "omega_cmd": w,
                                 "t_end": state["t0"] + start + duration})
                if publisher is not None:
                    send(v, w)
                return
            start += duration
        state["done"] = True

    duration = sum(d for d, _, _ in plan) if args.command == "record" else args.duration
    if args.command == "record":
        node.create_timer(0.05, on_timer)
    end_wall = None
    try:
        import time
        end_wall = time.monotonic() + (3.0 if args.command == "list-gt-frames" else duration * 4 + 30)
        while rclpy.ok() and not state["done"] and time.monotonic() < end_wall:
            rclpy.spin_once(node, timeout_sec=0.05)
            if args.command == "freshness" and rows and rows[-1]["t"] - rows[0]["t"] > duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if publisher is not None:
            for _ in range(20):  # best effort only: the real stop is the controller timeout / lock
                send(0.0, 0.0)
                rclpy.spin_once(node, timeout_sec=0.05)
    if gz_process is not None:
        gz_process.terminate()
    if args.command == "list-gt-frames":
        print("\n".join(sorted(frames)) or "no ground-truth messages: check the world name / bridge")
    else:
        write_csv(args.out, rows)
        print(f"wrote {len(rows)} rows to {args.out}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    ros_args: list[str] = []
    if "--ros-args" in argv:
        index = argv.index("--ros-args")
        argv, ros_args = argv[:index], ["identify_effective_track"] + argv[index:]
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("list-gt-frames", "record", "freshness"):
        p = sub.add_parser(name)
        p.add_argument("--namespace", default=NAMESPACE)
        p.add_argument("--gt-source", choices=("gz", "ros"), default="gz",
                       help="gz: read `gz topic -e` (default); ros: a ros_gz_bridge TFMessage topic")
        p.add_argument("--gz-topic", default="/world/warehouse/dynamic_pose/info")
        p.add_argument("--gt-topic", default="/gt/dynamic_poses", help="ROS topic for --gt-source ros")
        p.add_argument("--gt-frame", default="",
                       help=f"the robot's Gazebo pose name (gz; in simulation {NAMESPACE}/robot) or child_frame_id (ros)")
        p.add_argument("--joint-topic", default=f"/{NAMESPACE}/platform/joint_states")
        p.add_argument("--wheel-radius", type=float, default=WHEEL_RADIUS)
        p.add_argument("--out", default="track_id.csv")
        p.add_argument("--duration", type=float, default=30.0, help="freshness recording length (s)")
        p.add_argument("--sim", action="store_true", help="confirm this is a simulation (required for record)")
    p = sub.add_parser("analyze")
    p.add_argument("csv")
    p.add_argument("--settle", type=float, default=1.0)
    p = sub.add_parser("freshness-report")
    p.add_argument("csv")
    args = parser.parse_args(argv)
    if args.command == "analyze":
        result = analyze(read_csv(args.csv), settle=args.settle)
        for s in result.segments:
            print(f"seg {s.seg:2d} v_cmd {s.v_cmd:+.2f} w_cmd {s.omega_cmd:+.2f}  w_gt {s.omega_gt:+.4f}±{s.omega_gt_se:.4f}"
                  f"  w_odom {s.omega_odom if s.omega_odom is not None else float('nan'):+.4f}"
                  f"  (vr-vl) {s.wheel_diff:+.4f}  W_eff {s.track_eff:.4f}±{s.track_eff_se:.4f}"
                  f"  w_gt/w_cmd {s.ratio_gt_cmd:.4f}")
        print(f"pooled W_eff = {result.track_eff:.4f} ± {result.track_eff_se:.4f} m "
              f"(segment range {result.track_eff_min:.4f}..{result.track_eff_max:.4f}); "
              f"implied multiplier {result.multiplier_implied:.3f} on {TRACK_NOMINAL} m; "
              f"mean w_gt/w_cmd {result.ratio_gt_cmd_mean:.4f}")
        return 0
    if args.command == "freshness-report":
        for source, st in freshness_report(read_csv(args.csv)).items():
            print(f"{source:12s} n={st.count:5d} rate {st.rate_hz:6.2f} Hz  dt std {st.dt_std * 1e3:6.2f} ms "
                  f"max {st.dt_max * 1e3:6.1f} ms  age median {st.age_median * 1e3:6.2f} p95 {st.age_p95 * 1e3:6.2f} "
                  f"max {st.age_max * 1e3:6.2f} min {st.age_min * 1e3:6.2f} ms  non-monotonic {st.non_monotonic}")
        return 0
    if args.command == "record" and not args.gt_frame:
        parser.error("record needs --gt-frame (find it with list-gt-frames)")
    return _ros_main(args, ros_args)


if __name__ == "__main__":
    sys.exit(main())
