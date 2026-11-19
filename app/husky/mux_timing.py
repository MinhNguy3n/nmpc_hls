#!/usr/bin/env python3
"""V13-active and the V11 leak investigation: twist_mux timing IN SIMULATION.

Sub-commands
    capture   passive: record receive time and stamp of /<ns>/cmd_vel (input),
              /<ns>/joy_teleop/cmd_vel, /<ns>/platform/cmd_vel (mux output), cmd_vel_out, the three
              lock topics and the twist_mux diagnostics (about 2 Hz) to a CSV: the lock priority and
              the unmasked velocity inputs (see parse_mux_status)
    scenario  SIMULATION ONLY (--sim and use_sim_time:=true): publish a scripted test pattern
                active    cmd_vel at 20 Hz, stamp = sim now, zero twist
                takeover  cmd_vel at 20 Hz + joy_teleop at 10 Hz during two windows (zero twist)
                lock      cmd_vel at 20 Hz + safety_stop and bt_quality_stop engaged/released
                turn      cmd_vel at 20 Hz with in-place turns (for the IMU checks; the robot turns)
                steps     --steps "v:omega:duration,..." (e.g. a yaw-rate map); then still for 3 s
                estimator still, in-place turns both ways, 1 m arcs both ways at 0.3 m/s
                          (ESTIMATOR_PROFILE; for estimator_checks.py; the robot drives about 3 m)
              Sources are tagged in twist.linear.y (cmd_vel 0.001, joy 0.002), which the
              diff-drive controller ignores, so a mux output sample can be attributed even
              without a stamp match. The robot does not move except in `turn` and `estimator`.
    report    offline: echo delay and jitter (input receive -> mux output receive, matched by exact
              stamp), lost and reordered messages, takeover and hand-back timing, leak events (an
              input forwarded while a higher-priority source was active within its 0.5 s timeout),
              lock behaviour, per-topic freshness

Run from the read-only tree; writes only to --out. The analysis is plain Python and is tested
offline (test_mux_timing.py); rclpy is imported only inside the ROS sub-commands.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from identify_effective_track import freshness_stats  # noqa: E402

NAMESPACE = "a300_00000"
INPUT, JOY, MUX, APPLIED = "cmd_vel", "joy_teleop/cmd_vel", "platform/cmd_vel", "platform_velocity_controller/cmd_vel_out"
LOCKS = ("platform/emergency_stop", "platform/safety_stop", "joy_teleop/bt_quality_stop")
TAGS = {INPUT: 0.001, JOY: 0.002}
MUX_TIMEOUT = 0.5  # s, every twist_mux input (V12)
FIELDS = ("topic", "stamp_ns", "received", "received_wall", "v", "omega", "tag", "data", "inputs")

# scenario timings (s of simulation time from the scenario start); the robot stays still except `turn`
TAKEOVER_WINDOWS = ((10.0, 20.0), (25.0, 30.0))
LOCK_EVENTS = ((10.0, "platform/safety_stop", True), (15.0, "platform/safety_stop", False),
               (20.0, "joy_teleop/bt_quality_stop", True), (25.0, "joy_teleop/bt_quality_stop", False))
TURN_PROFILE = ((3.0, 0.0), (8.0, 0.3), (4.0, 0.0), (8.0, -0.3), (3.0, 0.0))  # (duration, omega)
# (duration, v, omega): still, in-place left and right, arcs of radius 1 m left and right, still
ESTIMATOR_PROFILE = ((3.0, 0.0, 0.0), (6.0, 0.0, 0.3), (6.0, 0.0, -0.3), (8.0, 0.3, 0.3), (8.0, 0.3, -0.3),
                     (3.0, 0.0, 0.0))
DURATIONS = {"active": 30.0, "takeover": 40.0, "lock": 30.0, "turn": sum(d for d, _ in TURN_PROFILE),
             "estimator": sum(d for d, _, _ in ESTIMATOR_PROFILE), "steps": 0.0}
V_CAP, OMEGA_CAP = 0.3, 0.75  # the SIL envelope (sil_node --v-max/--omega-max defaults); simulation only


def parse_steps(text: str) -> tuple[tuple[float, float, float], ...]:
    """'v:omega:duration,...' -> ((duration, v, omega), ...) followed by 3 s still."""
    steps = []
    for item in text.split(","):
        v, w, d = (float(x) for x in item.split(":"))
        if not (0.0 <= v <= V_CAP and abs(w) <= OMEGA_CAP and d > 0):
            raise ValueError(f"step '{item}' outside v 0..{V_CAP}, |omega| <= {OMEGA_CAP}, duration > 0")
        steps.append((d, v, w))
    return tuple(steps) + ((3.0, 0.0, 0.0),)


def profile_command(profile: str, elapsed: float, custom: tuple = ()) -> tuple[float, float]:
    """(v, omega) of a scripted profile at `elapsed` s after its start; (0, 0) outside the motion."""
    acc = 0.0
    steps = [(d, 0.0, w) for d, w in TURN_PROFILE] if profile == "turn" else \
        list(ESTIMATOR_PROFILE) if profile == "estimator" else list(custom) if profile == "steps" else []
    for duration, v, w in steps:
        if elapsed < acc + duration:
            return v, w
        acc += duration
    return 0.0, 0.0


# ----------------------------------------------------------------------------------------------
# Offline analysis
# ----------------------------------------------------------------------------------------------


def read_capture(path: str | Path) -> list[dict[str, Any]]:
    rows = []
    with open(path, newline="") as handle:
        for raw in csv.DictReader(handle):
            row: dict[str, Any] = {"topic": raw["topic"]}
            row["stamp_ns"] = int(raw["stamp_ns"]) if raw.get("stamp_ns") else None
            for key in ("received", "received_wall", "v", "omega", "tag"):
                row[key] = float(raw[key]) if raw.get(key) else None
            row["data"] = None if not raw.get("data") else raw["data"] in ("1", "True", "true")
            row["inputs"] = raw.get("inputs") or None
            rows.append(row)
    return rows


def write_capture(path: str | Path, rows: Iterable[dict[str, Any]]) -> None:
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: ("" if row.get(k) is None else (int(row[k]) if k == "data" else row[k])) for k in FIELDS})


def parse_mux_status(values: dict[str, str]) -> tuple[float, str] | None:
    """(lock priority, ';'-joined unmasked velocity inputs) from a twist_mux status, else None.

    `current priority` is the highest engaged LOCK priority (0 = no lock), not the selected input
    (twist_mux 4.5.0, twist_mux_diagnostics_status.hpp). An input is reported `unmasked` while it is
    fresh (within its timeout) and its priority is not below the lock priority (topic_handle.hpp
    isMasked), so the selected source is the highest-priority unmasked input.
    """
    if "current priority" not in values:
        return None
    unmasked = sorted(k.split(".", 1)[1] for k, v in values.items()
                      if k.startswith("velocity topics.") and v.strip().startswith("unmasked"))
    return float(values["current priority"]), ";".join(unmasked)


def mux_status_transitions(rows: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    """The diagnostics samples at which the lock priority or the unmasked set changed."""
    out: list[dict[str, Any]] = []
    last = None
    for r in rows:
        if r["topic"] != "diag_mux":
            continue
        key = (r["v"], r.get("inputs") or "")
        if key != last:
            out.append({"received": r["received"], "lock_priority": key[0], "unmasked": key[1]})
            last = key
    return out


def _stats(values: list[float]) -> dict[str, float]:
    if not values:
        return {"n": 0}
    a = np.asarray(values)
    return {"n": len(a), "median": float(np.median(a)), "p95": float(np.percentile(a, 95)),
            "max": float(a.max()), "min": float(a.min()), "std": float(a.std())}


def _bursts(times: list[float], gap: float) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    for t in sorted(times):
        if out and t - out[-1][1] <= gap:
            out[-1] = (out[-1][0], t)
        else:
            out.append((t, t))
    return out


def _lock_intervals(rows: list[dict[str, Any]], name: str, t_end: float) -> list[tuple[float, float]]:
    """Engaged intervals of a lock from its Bool samples (receive times)."""
    intervals, start = [], None
    for r in sorted((r for r in rows if r["topic"] == name and r["data"] is not None), key=lambda r: r["received"]):
        if r["data"] and start is None:
            start = r["received"]
        elif not r["data"] and start is not None:
            intervals.append((start, r["received"]))
            start = None
    if start is not None:
        intervals.append((start, t_end))
    return intervals


def mux_report(rows: list[dict[str, Any]]) -> dict[str, Any]:
    by = {topic: sorted((r for r in rows if r["topic"] == topic), key=lambda r: r["received"])
          for topic in {r["topic"] for r in rows}}
    inputs = {topic: {r["stamp_ns"]: r for r in by.get(topic, []) if r["stamp_ns"] is not None} for topic in (INPUT, JOY)}
    mux = by.get(MUX, [])
    t_end = max((r["received"] for r in rows), default=0.0)

    def source(r: dict[str, Any]) -> str:
        # the tag first: the scenario sends cmd_vel and joy in the same tick with the same stamp
        for topic, tag in TAGS.items():
            if r["tag"] is not None and abs(r["tag"] - tag) < 1e-9:
                return topic if r["stamp_ns"] in inputs[topic] else topic + " (tag only)"
        for topic in (INPUT, JOY):
            if r["stamp_ns"] in inputs[topic]:
                return topic
        return "unknown"

    labelled = [(r, source(r)) for r in mux]
    result: dict[str, Any] = {"echo_delay": {}, "lost": {}, "reordered": {}, "duplicates": {}}
    echoed: dict[str, set[int]] = {INPUT: set(), JOY: set()}
    for topic in (INPUT, JOY):
        delays = []
        last = None
        reordered = duplicates = 0
        for r, src in labelled:
            if src != topic:
                continue
            delays.append(r["received"] - inputs[topic][r["stamp_ns"]]["received"])
            if r["stamp_ns"] in echoed[topic]:
                duplicates += 1
            elif last is not None and r["stamp_ns"] < last:
                reordered += 1
            echoed[topic].add(r["stamp_ns"])
            last = r["stamp_ns"] if last is None else max(last, r["stamp_ns"])
        result["echo_delay"][topic] = _stats(delays)
        result["reordered"][topic] = reordered
        result["duplicates"][topic] = duplicates
    # what was active when each cmd_vel input arrived
    joy_times = [r["received"] for r in by.get(JOY, [])]
    joy_walls = [r["received_wall"] for r in by.get(JOY, [])]
    locks = {name: _lock_intervals(rows, name, t_end) for name in LOCKS}
    lock_walls = {r["received"]: r["received_wall"] for r in rows if r["topic"] in LOCKS and r["data"]}

    def joy_active(t: float, wall: float | None = None) -> float | None:
        """Age of the last joy input at time t if within the mux timeout, else None.

        Simulation time has 1 ms steps, so an input and a joy message can share t; `wall` (the
        input's wall-clock receive time) then decides which arrived first."""
        prior = [j for j, w in zip(joy_times, joy_walls)
                 if j < t or (j == t and (wall is None or w is None or w < wall))]
        return t - prior[-1] if prior and t - prior[-1] <= MUX_TIMEOUT else None

    def locked(t: float, wall: float | None = None) -> list[str]:
        """Locks engaged at t; on a tie with the engage time, `wall` decides as in joy_active."""
        def engaged(a: float, b: float) -> bool:
            if t == a and wall is not None and lock_walls.get(a) is not None:
                return lock_walls[a] < wall
            return a <= t <= b
        return [n for n, ivs in locks.items() if any(engaged(a, b) for a, b in ivs)]

    unexpected, suppressed = [], 0
    for stamp, r in inputs[INPUT].items():
        if stamp in echoed[INPUT]:
            continue
        if joy_active(r["received"]) is not None or locked(r["received"]):
            suppressed += 1
        else:
            unexpected.append(r["received"])
    result["lost"] = {INPUT: {"unexpected": len(unexpected), "times": unexpected[:20],
                              "suppressed_by_higher_priority_or_lock": suppressed}}
    # takeovers and hand-backs
    takeovers = []
    for start, end in _bursts(joy_times, MUX_TIMEOUT):
        first_joy_out = next((r["received"] for r, s in labelled if s == JOY and r["received"] >= start), None)
        back = next((r["received"] for r, s in labelled if s == INPUT and r["received"] > end), None)
        takeovers.append({"joy_first": start, "joy_last": end,
                          "takeover_latency": None if first_joy_out is None else first_joy_out - start,
                          "handback_after_last_joy": None if back is None else back - end})
    result["takeovers"] = takeovers
    # leak events: a cmd_vel sample forwarded while joy was active (within its timeout) or a lock engaged
    leaks = []
    for i, (r, src) in enumerate(labelled):
        if not src.startswith(INPUT):
            continue
        # judged at the time the INPUT arrived (a sample already in flight when joy starts is a race,
        # not a leak); falls back to the forwarding time if the input was not captured
        r_in = inputs[INPUT].get(r["stamp_ns"], r)
        t_in = r_in["received"]
        age = joy_active(t_in, r_in.get("received_wall"))
        engaged = locked(t_in, r_in.get("received_wall"))
        if age is None and not engaged:
            continue
        nxt = [j for j in joy_times if j > r["received"]]
        leaks.append({"received": r["received"], "input_received": t_in, "stamp_ns": r["stamp_ns"],
                      "last_joy_age": age, "locks": engaged,
                      "next_joy_in": nxt[0] - r["received"] if nxt else None,
                      "input_age_at_forward": (r["received"] - inputs[INPUT][r["stamp_ns"]]["received"])
                      if r["stamp_ns"] in inputs[INPUT] else None,
                      "previous_output": labelled[i - 1][1] if i else None,
                      "next_output": labelled[i + 1][1] if i + 1 < len(labelled) else None})
    result["leaks"] = leaks
    result["locks"] = {}
    for name, ivs in locks.items():
        events = []
        for a, b in ivs:
            during = sum(1 for r, s in labelled if a < r["received"] < b)
            first_after = next((r["received"] for r, s in labelled if r["received"] > b), None)
            events.append({"engaged": a, "released": b, "mux_outputs_while_engaged": during,
                           "first_output_after_release": None if first_after is None else first_after - b})
        result["locks"][name] = events
    result["unattributed_mux_outputs"] = sum(1 for _, s in labelled if s == "unknown")
    freshness = {}
    for topic in (INPUT, JOY, MUX, APPLIED):
        sel = [r for r in by.get(topic, []) if r["stamp_ns"] is not None]
        if len(sel) >= 3:
            st = freshness_stats([r["stamp_ns"] * 1e-9 for r in sel], [r["received"] for r in sel])
            freshness[topic] = {"rate_hz": st.rate_hz, "dt_std": st.dt_std, "dt_max": st.dt_max,
                                "age_median": st.age_median, "age_p95": st.age_p95, "age_max": st.age_max,
                                "age_min": st.age_min, "non_monotonic": st.non_monotonic}
    result["freshness"] = freshness
    result["diag_mux_samples"] = len(by.get("diag_mux", []))
    result["diag_mux_transitions"] = mux_status_transitions(by.get("diag_mux", []))
    return result


# ----------------------------------------------------------------------------------------------
# ROS sub-commands
# ----------------------------------------------------------------------------------------------


def _ros(args: argparse.Namespace, ros_args: list[str]) -> int:  # pragma: no cover - needs ROS 2
    import time

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import Bool

    rclpy.init(args=["mux_timing"] + ros_args)
    node = Node("mux_timing")
    sim = bool(node.get_parameter("use_sim_time").value)
    if args.command == "scenario" and not (args.sim and sim):
        node.get_logger().error("scenario is SIMULATION ONLY: pass --sim and --ros-args -p use_sim_time:=true")
        rclpy.shutdown()
        return 2
    ns = "/" + args.namespace
    clock = node.get_clock()
    rows: list[dict[str, Any]] = []

    def now() -> float:
        return clock.now().nanoseconds * 1e-9

    if args.command == "capture":
        def twist_cb(topic: str):
            def cb(m: Any) -> None:
                rows.append({"topic": topic, "stamp_ns": m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec,
                             "received": now(), "received_wall": time.monotonic(), "v": m.twist.linear.x,
                             "omega": m.twist.angular.z, "tag": m.twist.linear.y})
            return cb
        for topic in (INPUT, JOY, MUX, APPLIED):
            node.create_subscription(TwistStamped, f"{ns}/{topic}", twist_cb(topic), qos_profile_sensor_data)
        for lock in LOCKS:
            node.create_subscription(Bool, f"{ns}/{lock}", lambda m, n=lock: rows.append(
                {"topic": n, "received": now(), "received_wall": time.monotonic(), "data": bool(m.data)}),
                qos_profile_sensor_data)

        def diag(m: Any) -> None:
            for st in m.status:
                status = parse_mux_status({kv.key: kv.value for kv in st.values})
                if status is not None:
                    rows.append({"topic": "diag_mux", "received": now(), "received_wall": time.monotonic(),
                                 "v": status[0], "inputs": status[1]})
        node.create_subscription(DiagnosticArray, f"{ns}/diagnostics", diag, 10)
        end = time.monotonic() + args.duration * 4 + 10  # wall-clock guard
        start_sim = None
        try:
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
                t = now()
                if t > 0 and start_sim is None:
                    start_sim = t
                if start_sim is not None and t - start_sim > args.duration:
                    break
        except KeyboardInterrupt:
            pass
        write_capture(args.out, rows)
        print(f"wrote {len(rows)} rows to {args.out}")
    else:
        pubs = {INPUT: node.create_publisher(TwistStamped, f"{ns}/{INPUT}", 10),
                JOY: node.create_publisher(TwistStamped, f"{ns}/{JOY}", 10)}
        lock_pubs = {n: node.create_publisher(Bool, f"{ns}/{n}", 10) for n in LOCKS[1:]}  # never emergency_stop
        sent: list[dict[str, Any]] = []
        state = {"t0": None, "next_joy": 0.0, "events": list(LOCK_EVENTS), "done": False}

        def send(topic: str, omega: float = 0.0, v: float = 0.0) -> None:
            msg = TwistStamped()
            ns_now = clock.now().nanoseconds
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns_now, 1_000_000_000)
            msg.header.frame_id = "base_link"
            msg.twist.linear.y = TAGS[topic]
            msg.twist.angular.z = float(max(-OMEGA_CAP, min(OMEGA_CAP, omega)))  # hard caps
            msg.twist.linear.x = float(max(0.0, min(V_CAP, v)))
            pubs[topic].publish(msg)
            sent.append({"topic": "sent:" + topic, "stamp_ns": ns_now, "received": ns_now * 1e-9,
                         "received_wall": time.monotonic(), "v": msg.twist.linear.x, "omega": msg.twist.angular.z,
                         "tag": TAGS[topic]})

        custom = parse_steps(args.steps) if args.profile == "steps" else ()

        def tick() -> None:
            t = now()
            if t <= 0:
                return
            if state["t0"] is None:
                state["t0"] = t
            el = t - state["t0"]
            if el > (sum(d for d, _, _ in custom) if args.profile == "steps" else DURATIONS[args.profile]):
                state["done"] = True
                return
            v, omega = profile_command(args.profile, el, custom)
            send(INPUT, omega, v)
            if args.profile == "takeover" and any(a <= el < b for a, b in TAKEOVER_WINDOWS) and el >= state["next_joy"]:
                send(JOY)
                state["next_joy"] = el + 0.1  # 10 Hz
            while args.profile == "lock" and state["events"] and el >= state["events"][0][0]:
                _, name, engaged = state["events"].pop(0)
                lock_pubs[name].publish(Bool(data=engaged))
                sent.append({"topic": "sent:" + name, "received": t, "received_wall": time.monotonic(), "data": engaged})

        node.create_timer(0.05, tick)  # 20 Hz in simulation time
        try:
            while rclpy.ok() and not state["done"]:
                rclpy.spin_once(node, timeout_sec=0.05)
        except KeyboardInterrupt:
            pass
        finally:
            for n, p in lock_pubs.items():  # never leave a lock engaged by this script
                p.publish(Bool(data=False))
        write_capture(args.out, sent)
        print(f"scenario {args.profile} done; sends logged to {args.out}")
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
    p = sub.add_parser("capture")
    p.add_argument("--namespace", default=NAMESPACE)
    p.add_argument("--duration", type=float, default=45.0, help="simulation seconds")
    p.add_argument("--out", required=True)
    p = sub.add_parser("scenario")
    p.add_argument("profile", choices=sorted(DURATIONS))
    p.add_argument("--steps", default="", help='profile "steps": "v:omega:duration,..."')
    p.add_argument("--namespace", default=NAMESPACE)
    p.add_argument("--sim", action="store_true")
    p.add_argument("--out", required=True)
    p = sub.add_parser("report")
    p.add_argument("capture_csv")
    p.add_argument("--out", default=None)
    args = parser.parse_args(argv)
    if args.command == "report":
        result = mux_report(read_capture(args.capture_csv))
        text = json.dumps(result, indent=2, default=str)
        print(text)
        if args.out:
            Path(args.out).expanduser().write_text(text)
        return 0
    return _ros(args, ros_args)


if __name__ == "__main__":
    sys.exit(main())
