"""Path-tracking runs of the RC car in Gazebo (SIMULATION ONLY; assumed chain parameters,
PLACEHOLDER_ vehicle values; never a vehicle result).

Run inside the container while app/rc_car/launch/rc_car_stack.launch.py and the arm's launch
(phase 3: launch/baseline_rpp.launch.py) are up. For every episode (path x speed x seed x grid
point) the harness:
  1. disarms the mux, cancels any goal, puts the car on the path start (Gazebo set_pose);
  2. sets the arm's parameters (baseline: RPP desired_linear_vel = speed, plus the grid values);
  3. publishes `episode/start` {t_start, seed} 0.3 s ahead (paired seeds: the mocap streams and
     PWM hold phases restart from the seed at t_start), and waits for t_start;
  4. starts the arm (baseline: FollowPath goals, see SEGMENTS below), waits for its command
     stream and arms the mux;
  5. records until the arm reports the end, a timeout, or the mux leaves `controller` (a safety
     event: the run ends there and is REPORTED, never dropped or repeated);
  6. disarms the mux at the end (an operator disarm, not counted as a safety event) and scores
     the Gazebo truth with tracking_metrics (scoring stops SCORE_END_MARGIN before the end).

    python3 app/rc_car/checks/track_run.py --arm baseline --paths line circle --speeds 1.0 --seeds 1 \
        --profile realistic_guess [--grid lookahead_time=0.4,0.6 min_lookahead_dist=0.2,0.35] [--out DIR]

SEGMENTS (baseline): Nav2's SimpleGoalChecker tests only the distance to the path's LAST pose,
so on a closed or multi-lap path it fires the first time the car passes that point (found
2026-11-22: the two-lap circle "ended" after one lap). The path, trimmed by PATH_TRIM at its end,
is therefore sent as overlapping segments of SEG_LEN, each replacing the previous goal (a Nav2
preemption: the plan is updated without stopping) when the car's progress passes the next
segment's start, SEG_STEP after the previous one. While a segment is active its end lies at least
SEG_LEN - SEG_STEP ahead along the path, far beyond the lookahead. Progress for this switching
comes from the mocap pose (the controller's own view), never from the Gazebo truth.

Writes DIR/<episode>.json (metrics, safety counts, raw series) and DIR/summary.json.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import time

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parents[1] / "husky"))
import bringup_analysis as ba  # noqa: E402
import episode as ep_msg  # noqa: E402
import paths as P  # noqa: E402
import tracking_metrics as tm  # noqa: E402
import vehicle_params as vp  # noqa: E402
from sil.gz_truth import GzTruthReader  # noqa: E402

WORLD, MODEL = "lab_floor", "reely_new1"
PATH_TRIM = 0.3          # m cut from the end of the path sent to the arm (closed paths end at their start)
SCORE_END_MARGIN = 0.5   # m before the end where scoring stops (> PATH_TRIM + goal tolerance 0.15)
EPISODE_LEAD = 0.3       # s between the episode message and t_start
SEG_LEN, SEG_STEP = 2.0, 1.0  # m, baseline goal segments (see SEGMENTS above)


def segments(length: float) -> list[tuple[float, float]]:
    """(s0, s1) of the goal segments covering [0, length]."""
    out, s0 = [], 0.0
    while True:
        s1 = min(s0 + SEG_LEN, length)
        out.append((s0, s1))
        if s1 >= length:
            return out
        s0 += SEG_STEP


def parse_grid(items: list[str]) -> list[dict]:
    keys, values = [], []
    for item in items:
        key, _, vals = item.partition("=")
        keys.append(key)
        values.append([float(v) for v in vals.split(",")])
    return [dict(zip(keys, combo)) for combo in itertools.product(*values)] or [{}]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--arm", choices=["baseline", "nmpc"], default="baseline")
    parser.add_argument("--rtf", type=float, default=None,
                        help="the real-time factor the stack was launched with (rc_car_stack.launch.py rtf:=); recorded "
                             "and checked against the measured one. --set-physics also sets it at run time (diagnostic only)")
    parser.add_argument("--set-physics", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--paths", nargs="+", default=list(P.EVALUATION))
    parser.add_argument("--speeds", type=float, nargs="+", default=[1.0])
    parser.add_argument("--seeds", type=int, nargs="+", default=[1])
    parser.add_argument("--profile", required=True, help="actuator profile the stack runs (recorded)")
    parser.add_argument("--mocap-profile", default="realistic_guess")
    parser.add_argument("--grid", nargs="*", default=[],
                        help="arm parameters, key=v1,v2 (tuning only): baseline lookahead_time, min_lookahead_dist; nmpc q_pos, q_psi")
    parser.add_argument("--label", default="")
    parser.add_argument("--out", default=str(Path.home() / "logs" / time.strftime("rc_car_track_%Y%m%d_%H%M%S")))
    args = parser.parse_args()

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import PoseStamped
    from pose_input import NewestPose, yaw_from_quaternion
    from nav2_msgs.action import FollowPath
    from nav_msgs.msg import Path as PathMsg
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState, GetState
    from rcl_interfaces.srv import SetParameters
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    geometry = vp.load_geometry()
    area = P.load_area()
    import actuator_model as am_act
    act_limits = vars(am_act.load_profile(args.profile))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    def secs(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    class Harness(Node):
        def __init__(self) -> None:
            super().__init__("rc_car_track_run",
                             parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
            self.lock = threading.Lock()
            self.sim = 0.0
            self.mode = None
            self.state: dict = {}
            self.events: list = []
            self.stats: dict[str, dict] = {}
            self.truth: list = []
            self.requested: list = []
            self.applied: list = []
            self.pub_episode = self.create_publisher(String, "episode/start", 10)
            self.pub_nmpc = self.create_publisher(String, "nmpc/command", 10)
            self.nmpc_status: dict = {}
            self.create_subscription(String, "nmpc/status", lambda m: setattr(self, "nmpc_status", json.loads(m.data)), 10)
            self.clock_rows: list = []
            self.arm_cli = self.create_client(Trigger, "mux/arm")
            self.disarm_cli = self.create_client(Trigger, "mux/disarm")
            self.param_cli = self.create_client(SetParameters, "/controller_server/set_parameters")
            self.follow = ActionClient(self, FollowPath, "follow_path")
            self.create_subscription(Clock, "/clock", self.on_clock, 50)
            self.create_subscription(String, "mux/state", self.on_state, 50)
            for topic in ("mocap/stats", "converter/stats", "mocap_bridge/stats"):
                self.create_subscription(String, topic, lambda m, k=topic: self.stats.__setitem__(k, json.loads(m.data)), 10)
            self.create_subscription(AckermannDriveStamped, "controller/ackermann_cmd", self.on_req, 50)
            self.create_subscription(AckermannDriveStamped, "plant/ackermann_cmd", self.on_applied, 100)
            self.mocap = NewestPose()
            self.create_subscription(PoseStamped, f"/mocap/{MODEL}/pose", self.on_mocap, 50)

        def on_mocap(self, msg) -> None:
            q = msg.pose.orientation
            self.mocap.update(secs(msg.header.stamp), msg.pose.position.x, msg.pose.position.y,
                              yaw_from_quaternion(q.x, q.y, q.z, q.w))

        def now(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def on_clock(self, m) -> None:
            self.sim = secs(m.clock)
            self.clock_rows.append((self.sim, time.monotonic()))
            if len(self.clock_rows) > 400_000:
                del self.clock_rows[:200_000]

        def on_state(self, msg) -> None:
            s = json.loads(msg.data)
            self.mode, self.state = s["mode"], s
            e = s.get("last_event")
            if e and (not self.events or self.events[-1] != e):
                with self.lock:
                    self.events.append(e)

        def on_req(self, msg) -> None:
            with self.lock:
                self.requested.append((self.now(), secs(msg.header.stamp), msg.drive.speed, msg.drive.steering_angle))

        def on_applied(self, msg) -> None:
            with self.lock:
                self.applied.append((secs(msg.header.stamp), msg.drive.speed, msg.drive.steering_angle))

        def on_truth(self, stamp, _received, x, y, yaw, *_rest) -> None:
            with self.lock:
                self.truth.append((stamp, x, y, yaw))

    rclpy.init()
    node = Harness()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    truth = GzTruthReader(f"/world/{WORLD}/dynamic_pose/info", MODEL, node.now, node.on_truth)
    truth.start()

    def wait_sim(seconds: float) -> None:
        start, t0 = time.monotonic(), node.sim
        while node.sim - t0 < seconds:
            if time.monotonic() - start > 120:
                raise TimeoutError("simulation clock stalled")
            time.sleep(0.001)

    def wait_future(future, timeout: float = 10.0):
        start = time.monotonic()
        while not future.done():
            if time.monotonic() - start > timeout:
                return None
            time.sleep(0.002)
        return future.result()

    def trigger(client) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=10):
            return False, "no service"
        r = wait_future(client.call_async(Trigger.Request()))
        return (bool(r.success), r.message) if r else (False, "no reply")

    def ensure_active(timeout: float = 40.0) -> str:
        """Wait for controller_server to be active; if the lifecycle manager got stuck (seen
        2026-11-22: a DDS response timeout on change_state), drive the transitions directly."""
        get = node.create_client(GetState, "/controller_server/get_state")
        change = node.create_client(ChangeState, "/controller_server/change_state")
        start, note = time.monotonic(), "lifecycle manager"
        while time.monotonic() - start < timeout:
            if get.wait_for_service(timeout_sec=2):
                r = wait_future(get.call_async(GetState.Request()), 5)
                label = r.current_state.label if r else "?"
                if label == "active":
                    return note
                if time.monotonic() - start > 15 and label in ("unconfigured", "inactive"):
                    tid = Transition.TRANSITION_CONFIGURE if label == "unconfigured" else Transition.TRANSITION_ACTIVATE
                    req = ChangeState.Request()
                    req.transition.id = tid
                    wait_future(change.call_async(req), 10)
                    note = f"activated by the harness (was {label})"
                    continue
            time.sleep(0.5)
        raise RuntimeError("controller_server did not become active")

    def set_params(values: dict) -> None:
        if not node.param_cli.wait_for_service(timeout_sec=20):
            raise RuntimeError("controller_server parameters unavailable")
        req = SetParameters.Request()
        req.parameters = [Parameter(f"FollowPath.{k}", Parameter.Type.DOUBLE, float(v)).to_parameter_msg()
                          for k, v in values.items()]
        r = wait_future(node.param_cli.call_async(req))
        bad = [p.reason for p in (r.results if r else []) if not p.successful]
        if r is None or bad:
            raise RuntimeError(f"setting {values} failed: {bad or 'no reply'}")

    def set_pose(x: float, y: float, yaw: float) -> None:
        req = (f'name: "{MODEL}", position: {{x: {x}, y: {y}, z: {geometry.wheel_radius + 0.005}}}, '
               f"orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}")
        subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/set_pose", "--reqtype", "gz.msgs.Pose",
                        "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
                       check=True, capture_output=True)

    def path_msg(path: P.Path2D, s0: float, s1: float) -> PathMsg:
        keep = (path.s >= s0 - 1e-9) & (path.s <= s1 + 1e-9)
        step = max(1, int(round(0.05 / (path.s[1] - path.s[0]))))
        msg = PathMsg()
        msg.header.frame_id = "map"
        msg.header.stamp = node.get_clock().now().to_msg()
        for x, y, yaw in zip(path.x[keep][::step], path.y[keep][::step], path.yaw[keep][::step]):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            msg.poses.append(ps)
        return msg

    def rtf_window(n0: int) -> float | None:
        rows = np.array(node.clock_rows[n0:])
        return ba.real_time_factor(rows[:, 0], rows[:, 1]) if len(rows) > 100 else None

    def set_rtf(rtf: float) -> None:
        req = f"max_step_size: 0.001, real_time_factor: {rtf}"
        r = subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/set_physics", "--reqtype", "gz.msgs.Physics",
                            "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
                           capture_output=True, text=True)
        print(f"set real-time factor {rtf}: {r.stdout.strip() or r.stderr.strip()}", flush=True)

    def snapshot() -> dict:
        return json.loads(json.dumps({"stats": node.stats, "counters": node.state.get("counters", {})}))

    def delta(after: dict, before: dict) -> dict:
        d = {}
        for group in ("stats", "counters"):
            for key, val in after.get(group, {}).items():
                if isinstance(val, dict):
                    for k2, v2 in val.items():
                        b = before.get(group, {}).get(key, {}).get(k2, 0)
                        if isinstance(v2, (int, float)) and isinstance(b, (int, float)):
                            d[f"{key}.{k2}"] = v2 - b
                elif isinstance(val, (int, float)):
                    d[key] = val - before.get(group, {}).get(key, 0)
        return d

    def wait_stream(n_req0: int, t0: float) -> None:
        while True:  # the arm's stream must flow before the mux can be armed
            with node.lock:
                fresh = [r for r in node.requested[n_req0:] if r[0] > node.now() - 0.1]
            if len(fresh) >= 2:
                return
            if node.now() - t0 > 5.0:
                raise RuntimeError("no controller command stream")
            time.sleep(0.002)

    def drive_baseline(path, speed, seed, grid, n_req0):
        if not node.follow.wait_for_server(timeout_sec=20):
            raise RuntimeError("no follow_path action server")
        segs = segments(path.length - PATH_TRIM)

        def send(k: int):
            goal = FollowPath.Goal(path=path_msg(path, *segs[k]), controller_id="FollowPath",
                                   goal_checker_id="goal_checker", progress_checker_id="progress_checker")
            h = wait_future(node.follow.send_goal_async(goal))
            if h is None or not h.accepted:
                raise RuntimeError(f"FollowPath goal (segment {k}) rejected")
            return h

        handle = send(0)
        active, progress_idx = 0, 0
        ds = float(path.s[1] - path.s[0])
        result_future = handle.get_result_async()
        wait_stream(n_req0, node.now())
        ok, why = trigger(node.arm_cli)
        t_arm = node.now()
        end_reason = "armed" if ok else f"arm_failed: {why}"
        timeout = path.length / speed * 1.6 + 5.0
        switches = []
        while ok:
            pose = node.mocap.pose
            if pose is not None and active + 1 < len(segs):
                lo, hi = max(0, progress_idx - 10), min(len(path.s), progress_idx + 101)
                progress_idx = lo + int(np.argmin((path.x[lo:hi] - pose.x) ** 2 + (path.y[lo:hi] - pose.y) ** 2))
                if progress_idx * ds >= segs[active + 1][0]:
                    active += 1
                    handle = send(active)
                    result_future = handle.get_result_async()
                    switches.append((node.now(), active, progress_idx * ds))
            if result_future.done() and active + 1 == len(segs):
                end_reason = "arm_reported_end"
                break
            if result_future.done() and active + 1 < len(segs):
                end_reason = f"arm_ended_early (segment {active} of {len(segs)})"
                break
            if node.mode not in (None, "controller"):
                end_reason = f"safety_event: mode {node.mode}"
                break
            if node.now() - t_arm > timeout:
                end_reason = "timeout"
                break
            time.sleep(0.002)
        t_end = node.now()
        trigger(node.disarm_cli)
        if not result_future.done():
            wait_future(handle.cancel_goal_async(), 5.0)
        result = result_future.result() if result_future.done() else None
        return t_arm, t_end, end_reason, {
            "arm_result": None if result is None else {"status": int(result.status),
                                                       "error_code": int(getattr(result.result, "error_code", 0))},
            "segments": len(segs), "segment_switches": switches}

    def drive_nmpc(path, speed, seed, grid, n_req0):
        node.nmpc_status = {}
        node.pub_nmpc.publish(String(data=json.dumps({"cmd": "start", "path": path.name, "speed": speed,
                                                      "seed": seed, **grid})))
        wait_stream(n_req0, node.now())
        ok, why = trigger(node.arm_cli)
        node.pub_nmpc.publish(String(data=json.dumps({"cmd": "go"})))
        t_arm = node.now()
        end_reason = "armed" if ok else f"arm_failed: {why}"
        timeout = path.length / speed * 1.6 + 5.0
        while ok:
            if node.nmpc_status.get("state") == "done":
                end_reason = "arm_reported_end"
                break
            if node.mode not in (None, "controller"):
                end_reason = f"safety_event: mode {node.mode}"
                break
            if node.now() - t_arm > timeout:
                end_reason = "timeout"
                break
            time.sleep(0.002)
        t_end = node.now()
        trigger(node.disarm_cli)
        node.pub_nmpc.publish(String(data=json.dumps({"cmd": "stop"})))
        wait_sim(0.3)
        return t_arm, t_end, end_reason, {"nmpc": dict(node.nmpc_status)}

    def episode(path: P.Path2D, speed: float, seed: int, grid: dict) -> dict:
        trigger(node.disarm_cli)
        wait_sim(0.3)
        set_pose(*path.start)
        wait_sim(1.0)
        if args.arm == "baseline":
            set_params({"desired_linear_vel": speed, **grid})
        t_start = node.now() + EPISODE_LEAD
        node.pub_episode.publish(String(data=ep_msg.Episode(t_start, seed).to_json()))
        while node.now() < t_start:
            time.sleep(0.001)
        before = snapshot()
        with node.lock:
            n_req0, n_ev0 = len(node.requested), len(node.events)
            n_clock0 = len(node.clock_rows)
        drive = drive_baseline if args.arm == "baseline" else drive_nmpc
        t_arm, t_end, end_reason, extra = drive(path, speed, seed, grid, n_req0)
        wait_sim(1.2)  # one more stats message
        after = snapshot()
        with node.lock:
            tr = np.array([r for r in node.truth if t_arm <= r[0] <= t_end])
            req = np.array([r for r in node.requested[n_req0:] if t_arm <= r[0] <= t_end])
            app = np.array([r for r in node.applied if t_arm <= r[0] <= t_end])
            events = [e for e in node.events[n_ev0:] if t_arm - 1e-6 <= e[0] <= t_end]
        safety, operator = {}, 0
        for _t, _from, to, reason in events:
            if reason == "operator":  # the harness's own end-of-episode disarm (clocks differ by ~1 ms)
                operator += 1
            elif to in ("hold", "teleop", "locked"):
                safety[reason] = safety.get(reason, 0) + 1
        rec: dict = {"note": "SIMULATION ONLY; assumed chain parameters, PLACEHOLDER_ vehicle values",
                     "arm": args.arm, "path": path.name, "speed": speed, "seed": seed, "grid": grid,
                     "profile": args.profile, "mocap_profile": args.mocap_profile, "label": args.label,
                     "t_start": t_start, "t_arm": t_arm, "t_end": t_end, "end_reason": end_reason, **extra,
                     "rtf": rtf_window(n_clock0),
                     "safety_events": safety, "safety_event_total": int(sum(safety.values())),
                     "operator_disarms_in_window": operator,
                     "mux_events": events, "counter_deltas": delta(after, before)}
        if len(tr) > 10:
            rec["tracking"] = tm.track(path, tr[:, 0], tr[:, 1], tr[:, 2], tr[:, 3], end_margin=SCORE_END_MARGIN)
            _, v_lat, _ = ba.body_velocities(tr[:, 0], tr[:, 1], tr[:, 2], tr[:, 3])
            rec["slip_rear_lateral_mps"] = {"rms": float(np.sqrt(np.mean(v_lat ** 2))), "max": float(np.max(np.abs(v_lat)))}
        if len(req) > 2:
            d = req[:, 3]
            rec["requested"] = {"n": int(len(req)), "rate_hz": float((len(req) - 1) / (req[-1, 0] - req[0, 0])),
                                "steer_at_limit_frac": float(np.mean(np.abs(d) >= geometry.delta_max - 1e-9)),
                                "steer_step_max_rad": float(np.max(np.abs(np.diff(d)))),
                                "speed_max": float(np.max(np.abs(req[:, 2])))}
        if len(app) > 2:
            dd = np.diff(app[:, 2]) / np.maximum(np.diff(app[:, 0]), 1e-6)
            rate_lim = float(act_limits["delta_rate_max"] or math.inf)
            rec["applied"] = {"steer_max_abs_rad": float(np.max(np.abs(app[:, 2]))),
                              "steer_rate_max_radps": float(np.max(np.abs(dd))),
                              "steer_at_limit_frac": float(np.mean(np.abs(app[:, 2]) >= geometry.delta_max - 1e-6)),
                              "steer_rate_at_limit_frac": float(np.mean(np.abs(dd) >= 0.98 * rate_lim)),
                              "speed_max": float(np.max(np.abs(app[:, 1])))}
        rec["series"] = {"truth": tr[::2].round(5).tolist() if len(tr) else [],
                         "requested": req[:, [1, 2, 3]].round(5).tolist() if len(req) else [],
                         "applied": app[::10].round(5).tolist() if len(app) else []}
        return rec

    summary = []
    exit_code = 0
    try:
        start = time.monotonic()
        while not (node.truth and node.mode and node.stats.get("mocap/stats")):
            if time.monotonic() - start > 90:
                raise TimeoutError(f"stack not ready: truth {bool(node.truth)}, mux {node.mode}, stats {list(node.stats)}")
            time.sleep(0.1)
        if args.rtf is not None and args.set_physics:
            set_rtf(args.rtf)  # diagnostic only: a runtime set_physics call broke the plant (2026-11-22)
        if args.arm == "baseline":
            activation = ensure_active()
            print(f"controller_server active ({activation})", flush=True)
        grid = parse_grid(args.grid)
        for name, speed, seed, g in itertools.product(args.paths, args.speeds, args.seeds, grid):
            path = P.build(name, area, kappa_max=geometry.kappa_max)
            rec = episode(path, speed, seed, g)
            tag = f"{args.arm}_{name}_v{speed:g}_seed{seed}" + "".join(f"_{k}{v:g}" for k, v in g.items())
            if "nmpc" in rec:
                n = rec["nmpc"]
                var = "" if n.get("variant", "phase4") == "phase4" else f"{n.get('variant')}_"
                tag = f"nmpc{n.get('size')}_{var}{'comp' if n.get('compensation') else 'nocomp'}_{n.get('arith')}_" + tag[len('nmpc_'):]
            (out / f"{tag}.json").write_text(json.dumps(rec, default=str))
            tr = rec.get("tracking", {})
            row = {"episode": tag, "end_reason": rec["end_reason"], "safety_event_total": rec["safety_event_total"],
                   "safety_events": rec["safety_events"], **{k: tr.get(k) for k in (
                       "cross_track_rmse_m", "cross_track_max_m", "heading_rmse_rad", "heading_max_rad",
                       "completion", "reached_end", "mean_progress_speed_mps")},
                   "slip_rms": rec.get("slip_rear_lateral_mps", {}).get("rms"),
                   "steer_at_limit_frac": rec.get("requested", {}).get("steer_at_limit_frac"),
                   "mocap_spikes": rec["counter_deltas"].get("mocap/stats.spikes"),
                   "mocap_dropped": rec["counter_deltas"].get("mocap/stats.dropped"),
                   "grid": g, "path": name, "speed": speed, "seed": seed, "rtf": rec.get("rtf"),
                   **({f"nmpc_{k}": rec["nmpc"].get(k) for k in ("deadline_misses", "stale_pose", "ticks", "no_pose", "fallbacks",
                                                               "overruns", "python_busy", "cost_final", "solve_wall_ms",
                                                               "size", "compensation", "arith")} if "nmpc" in rec else {})}
            summary.append(row)
            print(f"{tag}: {rec['end_reason']}, e_y rmse {1e3 * (tr.get('cross_track_rmse_m') or math.nan):.1f} mm "
                  f"max {1e3 * (tr.get('cross_track_max_m') or math.nan):.1f} mm, heading rmse "
                  f"{1e3 * (tr.get('heading_rmse_rad') or math.nan):.1f} mrad, completion {tr.get('completion')}, "
                  f"safety {rec['safety_events']}, rtf {rec.get('rtf')}"
                  + (f", nmpc misses {rec['nmpc'].get('deadline_misses')}/{rec['nmpc'].get('ticks')} stale poses "
                     f"{rec['nmpc'].get('stale_pose')} overruns "
                     f"{rec['nmpc'].get('overruns')} busy {rec['nmpc'].get('python_busy')} fallbacks {rec['nmpc'].get('fallbacks')} "
                     f"solve wall {rec['nmpc'].get('solve_wall_ms')}" if "nmpc" in rec else ""), flush=True)
    except Exception:
        import traceback
        traceback.print_exc()
        exit_code = 1
    finally:
        try:
            trigger(node.disarm_cli)
        except Exception:
            pass
        truth.stop()
        (out / "summary.json").write_text(json.dumps({"note": "SIMULATION ONLY", "profile": args.profile,
                                                      "mocap_profile": args.mocap_profile, "label": args.label,
                                                      "episodes": summary}, indent=2, default=str))
        sys.stdout.flush()
        import os
        os._exit(exit_code)  # skip the rclpy/action-client teardown, which aborts with a spinning thread


if __name__ == "__main__":
    sys.exit(main())
