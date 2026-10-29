#!/usr/bin/env python3
"""Thin rclpy node around sil.controller.MpcCore: Python-backend SIL for the A300 IN SIMULATION.

SIMULATION ONLY: refuses to start without --sim and use_sim_time:=true. Run from the read-only
source tree; writes only under --out-dir (default ~/logs/sil).

    cd <app/husky>
    python3 -m sil.sil_node --sim --path circle --pso 4x50 --gain plain --seed 1 --auto-arm \
        --ros-args -p use_sim_time:=true

Signals (kept distinct; plan section 3.6):
    feedback   /<ns>/platform/odom/filtered   -> the controller's only state input
    applied    /<ns>/platform_velocity_controller/cmd_vel_out -> u_prev
    ownership  /<ns>/platform/cmd_vel (mux output), locks platform/emergency_stop,
               platform/safety_stop, joy_teleop/bt_quality_stop
    diagnostic /<ns>/platform/odom (wheel odometry), /<ns>/diagnostics (twist_mux, 1 Hz): logged only
    scoring    Gazebo ground truth via `gz topic -e` (sil.gz_truth): logged only, never fed back
Publishes TwistStamped on /<ns>/cmd_vel only in the states owned (plan) and arming (hold probe).
Services: /sil_mpc/arm and /sil_mpc/disarm (std_srvs/Trigger). --auto-arm requests the FIRST
arming automatically (SIL convenience); after any takeover or lock a manual re-arm is required.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict
import datetime
import json
from pathlib import Path
import sys
import time
from typing import Any

if __package__ in (None, ""):  # allow `python3 sil/sil_node.py` as well as `python3 -m sil.sil_node`
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    __package__ = "sil"  # noqa: A001

from sil import _HUSKY  # noqa: F401,E402
from sil import pso as pso_mod  # noqa: E402
from sil.controller import CoreConfig, MpcCore, OdomSample  # noqa: E402
from sil.gz_truth import GzTruthReader  # noqa: E402
from sil.logs import EPISODE_FIELDS, SIGNAL_FIELDS, TICK_FIELDS, TRANSITION_FIELDS, CsvLog, tick_row, write_meta  # noqa: E402
from sil.reference import PathSpec  # noqa: E402
import command_ownership as co  # noqa: E402
import unicycle_model as um  # noqa: E402

LOCKS = ("platform/emergency_stop", "platform/safety_stop", "joy_teleop/bt_quality_stop")


def parse_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    ros_args: list[str] = []
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        argv, ros_args = argv[:i], argv[i:]
    p = argparse.ArgumentParser(description="Python SIL MPC node (simulation only)")
    p.add_argument("--sim", action="store_true", help="confirm this is a simulation (required)")
    p.add_argument("--namespace", default="a300_00000")
    p.add_argument("--path", choices=("line", "circle", "figure8"), default="circle")
    p.add_argument("--speed", type=float, default=0.3)
    p.add_argument("--size", type=float, default=1.0)
    p.add_argument("--pso", default="4x50", help="particles x iterations, e.g. 4x50, 7x30, 9x24")
    p.add_argument("--nu", type=int, default=25)
    p.add_argument("--gain", choices=("plain", "sim-derived"), default="plain")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--v-max", type=float, default=0.3, help="PLACEHOLDER_ SIL envelope (m/s)")
    p.add_argument("--omega-max", type=float, default=0.75, help="PLACEHOLDER_ SIL envelope (rad/s)")
    p.add_argument("--auto-arm", action="store_true")
    p.add_argument("--extra-time", type=float, default=3.0, help="s after the path ends before stopping")
    p.add_argument("--max-wall", type=float, default=1800.0, help="wall-clock safety limit (s)")
    p.add_argument("--rtf", type=float, default=None, help="the Gazebo real-time factor you set (recorded)")
    p.add_argument("--gz-topic", default="/world/warehouse/dynamic_pose/info")
    p.add_argument("--gt-name", default="a300_00000/robot")
    p.add_argument("--no-truth", action="store_true", help="do not start the gz ground-truth reader")
    p.add_argument("--out-dir", default="~/logs/sil")
    return p.parse_args(argv), ros_args


def _stamp(msg: Any) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def main(argv: list[str] | None = None) -> int:  # pragma: no cover - needs ROS 2 and Gazebo
    args, ros_args = parse_args(list(sys.argv[1:] if argv is None else argv))
    import rclpy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger

    rclpy.init(args=["sil_mpc"] + ros_args)
    node = Node("sil_mpc")
    if not (args.sim and node.get_parameter("use_sim_time").value):
        node.get_logger().error("SIMULATION ONLY: pass --sim and --ros-args -p use_sim_time:=true")
        rclpy.shutdown()
        return 2

    limits = um.limits_from_parameter_set("husky_a300", v_min=0.0, v_max=args.v_max, omega_max=args.omega_max)
    spec = PathSpec(args.path, args.speed, args.size)
    cfg = CoreConfig(limits=limits, pso=pso_mod.PsoConfig.parse(args.pso), control_horizon=args.nu,
                     gain_mode=args.gain, path=spec)
    core = MpcCore(cfg, seed=args.seed)
    ns = "/" + args.namespace
    run_id = (f"{args.path}_{args.pso}_nu{args.nu}_{args.gain}_seed{args.seed}_"
              f"{datetime.datetime.now().strftime('%Y%m%dT%H%M%S')}")
    out = Path(args.out_dir).expanduser() / run_id
    out.mkdir(parents=True, exist_ok=False)
    write_meta(out / "meta.json", {
        "run_id": run_id, "args": vars(args), "ts": cfg.ts, "horizon": cfg.horizon,
        "control_horizon": cfg.control_horizon, "pso": args.pso, "gain_mode": args.gain,
        "gain_label": "sim-derived (simulator physics, not robot values)" if args.gain == "sim-derived" else "plain",
        "seed": args.seed, "deadline": cfg.deadline, "limits": asdict(limits), "cost": asdict(cfg.cost),
        "path": {"kind": spec.kind, "speed": spec.speed, "size": spec.size, "duration": spec.duration},
        "ownership": asdict(cfg.ownership), "max_odom_age": cfg.max_odom_age,
        "clocks": "simulation time for t, stamps and C_sim; wall clock for C_wall",
        "rtf_set_by_user": args.rtf, "thresholds": "provisional until V13-active",
    })
    logs = {"ticks": CsvLog(out / "ticks.csv", TICK_FIELDS), "transitions": CsvLog(out / "transitions.csv", TRANSITION_FIELDS),
            "episodes": CsvLog(out / "episodes.csv", EPISODE_FIELDS), "feedback": CsvLog(out / "feedback.csv", SIGNAL_FIELDS),
            "wheel_odom": CsvLog(out / "wheel_odom.csv", SIGNAL_FIELDS), "truth": CsvLog(out / "truth.csv", SIGNAL_FIELDS),
            "applied": CsvLog(out / "applied.csv", SIGNAL_FIELDS),
            "diag": CsvLog(out / "diagnostics.csv", ("received", "current_priority", "raw"))}
    events: deque = deque()  # (kind, payload) appended by subscription threads, drained by the tick
    clock = node.get_clock()

    def now() -> float:
        return clock.now().nanoseconds * 1e-9

    subs = ReentrantCallbackGroup()
    tick_group = MutuallyExclusiveCallbackGroup()

    def odom_row(msg: Any, received: float) -> dict[str, Any]:
        q = msg.pose.pose.orientation
        pc, tc = msg.pose.covariance, msg.twist.covariance
        return {"stamp": _stamp(msg), "received": received, "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y, "yaw": um.yaw_from_quaternion(q.x, q.y, q.z, q.w),
                "v": msg.twist.twist.linear.x, "omega": msg.twist.twist.angular.z,
                "cov_x": pc[0], "cov_y": pc[7], "cov_yaw": pc[35], "cov_vx": tc[0], "cov_wz": tc[35]}

    node.create_subscription(Odometry, f"{ns}/platform/odom/filtered",
                             lambda m: events.append(("feedback", odom_row(m, now()))), qos_profile_sensor_data,
                             callback_group=subs)
    node.create_subscription(Odometry, f"{ns}/platform/odom",
                             lambda m: events.append(("wheel_odom", odom_row(m, now()))), qos_profile_sensor_data,
                             callback_group=subs)
    node.create_subscription(TwistStamped, f"{ns}/platform_velocity_controller/cmd_vel_out",
                             lambda m: events.append(("applied", {"stamp": _stamp(m), "received": now(),
                                                                  "v": m.twist.linear.x, "omega": m.twist.angular.z})),
                             qos_profile_sensor_data, callback_group=subs)
    node.create_subscription(TwistStamped, f"{ns}/platform/cmd_vel",
                             lambda m: events.append(("mux", (co.stamp_ns(m.header.stamp.sec, m.header.stamp.nanosec),
                                                              m.twist.linear.x, m.twist.angular.z, now()))),
                             qos_profile_sensor_data, callback_group=subs)
    for lock in LOCKS:
        node.create_subscription(Bool, f"{ns}/{lock}", lambda m, name=lock: events.append(("lock", (name, m.data, now()))),
                                 qos_profile_sensor_data, callback_group=subs)

    def on_diag(msg: Any) -> None:
        for status in msg.status:
            if "twist_mux" in status.name.lower() or "twist mux" in status.name.lower():
                values = {kv.key: kv.value for kv in status.values}
                events.append(("diag", {"received": now(), "current_priority": values.get("current priority"),
                                        "raw": json.dumps(values)}))
    node.create_subscription(DiagnosticArray, f"{ns}/diagnostics", on_diag, 10, callback_group=subs)

    if not args.no_truth:
        truth = GzTruthReader(args.gz_topic, args.gt_name, now, lambda s, r, x, y, yaw: events.append(
            ("truth", {"stamp": s, "received": r, "x": x, "y": y, "yaw": yaw})))
        truth.start()
    else:
        truth = None

    publisher = node.create_publisher(TwistStamped, f"{ns}/cmd_vel", 10)
    state: dict[str, Any] = {"last_stamp": 0, "transitions": 0, "episode": 0, "origin": None, "end": None,
                             "auto_arm": args.auto_arm, "wall_start": time.monotonic(), "done": False}

    def drain() -> None:
        while events:
            kind, payload = events.popleft()
            if kind == "feedback":
                logs["feedback"].write(payload)
                core.on_odom(OdomSample(payload["stamp"], payload["received"], payload["x"], payload["y"],
                                        payload["yaw"], payload["v"], payload["omega"]))
            elif kind == "applied":
                logs["applied"].write(payload)
                core.on_applied(co.AppliedSample(payload["stamp"], payload["received"], payload["v"], payload["omega"]))
            elif kind == "mux":
                core.on_mux(*payload)
            elif kind == "lock":
                core.on_lock(*payload)
            elif kind in ("wheel_odom", "truth", "diag"):
                logs[kind].write(payload)  # logged only: never fed to the core

    def arm_srv(_req: Any, res: Any) -> Any:
        drain()
        res.success, res.message = core.request_arm(now())
        return res

    def disarm_srv(_req: Any, res: Any) -> Any:
        core.disarm(now())
        res.success, res.message = True, "disarmed"
        return res

    node.create_service(Trigger, "/sil_mpc/arm", arm_srv, callback_group=tick_group)
    node.create_service(Trigger, "/sil_mpc/disarm", disarm_srv, callback_group=tick_group)

    def on_tick() -> None:
        drain()
        stamp = clock.now().nanoseconds  # one reading: the tick time and the published stamp
        t = stamp * 1e-9
        if stamp <= state["last_stamp"]:
            return  # simulation paused: no duplicate stamps
        state["last_stamp"] = stamp
        if state["auto_arm"] and core.sm.state == co.State.DISARMED:
            ok, _ = core.request_arm(t)
            state["auto_arm"] = not ok  # the first successful arming consumes it
        record = core.begin_tick(t, stamp, time.perf_counter)
        if record.publish is not None:
            msg = TwistStamped()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            msg.header.frame_id = "base_link"
            msg.twist.linear.x, msg.twist.angular.z = record.publish
            publisher.publish(msg)
        core.finish_tick(record, now, time.perf_counter)
        logs["ticks"].write(tick_row(record))
        for tr in core.sm.transitions[state["transitions"]:]:
            logs["transitions"].write({"t": tr.time, "old": tr.old.value, "new": tr.new.value, "reason": tr.reason})
        state["transitions"] = len(core.sm.transitions)
        if core.origin is not None and core.origin is not state["origin"]:
            state["episode"] += 1
            state["origin"] = core.origin
            logs["episodes"].write({"episode": state["episode"], "tau0": core.tau0, "origin_x": core.origin[0],
                                    "origin_y": core.origin[1], "origin_yaw": core.origin[2]})
            state["end"] = core.tau0 + spec.duration + args.extra_time
        if (state["end"] is not None and t > state["end"]) or time.monotonic() - state["wall_start"] > args.max_wall:
            core.disarm(t, "run finished")  # stop publishing; the robot stops on its own timeouts
            state["done"] = True

    node.create_timer(cfg.ts, on_tick, callback_group=tick_group)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    node.get_logger().info(f"SIL run {run_id}: logs in {out}")
    try:
        while rclpy.ok() and not state["done"]:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        core.disarm(now(), "shutdown")
        if truth is not None:
            truth.stop()
        drain()
        for log in logs.values():
            log.close()
        node.destroy_node()
        rclpy.shutdown()
    print(f"logs: {out}\nscore: python3 -m sil.metrics {out} --out {out}/report.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
