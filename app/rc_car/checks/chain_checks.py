"""Phase 2 chain checks in Gazebo (SIMULATION ONLY, assumed chain parameters; never a vehicle result).

Run inside the container while app/rc_car/launch/rc_car_stack.launch.py is up. The script acts as
a stand-in controller on `controller/ackermann_cmd` (50 Hz stream, header stamp = send time) and
as the operator (mux/arm, teleop/ackermann_cmd, mux/lock). It measures:

  D1 command delay: steering steps (+-0.25 rad, at random times, car at rest), from the command's
     header stamp to the front-left steering joint leaving its previous angle by 2 mrad
     (/joint_states header stamps, 200 Hz). Next to it, the chain-only delay of the offline model
     (actuator_model.servo_start_delays) for the same profile.
  D2 pose delay: for every /mocap/<body>/pose, its age at arrival here (receive time - capture
     stamp), i.e. at the controller's input.
  D3 speed step 0 -> 0.5 m/s straight: start delay and 10-90 % rise of the truth speed.
  S  live mux checks: setpoint timeout -> hold (and the car stops), no automatic resume, explicit
     re-arm, teleop takeover and release, lock and release.

    python3 app/rc_car/checks/chain_checks.py --profile realistic_guess [--steps 30] [--out DIR]

`--profile` must name the actuator profile the stack was launched with (for the offline
comparison and the report). Writes DIR/report.json and prints a summary.
"""

from __future__ import annotations

import argparse
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
import actuator_model as am  # noqa: E402
import bringup_analysis as ba  # noqa: E402
import chain_analysis as ca  # noqa: E402
import command_mux as cm  # noqa: E402
import vehicle_params as vp  # noqa: E402
from sil.gz_truth import GzTruthReader  # noqa: E402

WORLD, MODEL = "lab_floor", "reely_new1"
STEER_JOINT = "front_left_steer_joint"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--mocap-profile", default="realistic_guess")
    parser.add_argument("--steps", type=int, default=30)
    parser.add_argument("--step-angle", type=float, default=0.25)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--label", default="")
    parser.add_argument("--out", default=str(Path.home() / "logs" / time.strftime("rc_car_chain_%Y%m%d_%H%M%S")))
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import PoseStamped
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger

    rng = np.random.default_rng(args.seed)
    geometry = vp.load_geometry()
    mux_cfg = cm.load_mux_config()

    def secs(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    class Checker(Node):
        def __init__(self) -> None:
            super().__init__("rc_car_chain_checks",
                             parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
            self.lock = threading.Lock()
            self.sim = 0.0
            self.clock_rows: list[tuple[float, float]] = []
            self.cmd = (0.0, 0.0)
            self.streaming = False
            self.joints: list[tuple[float, float]] = []
            self.pose_age: list[tuple[float, float]] = []  # (receive time, age)
            self.mocap_stats: dict = {}
            self.events: list[tuple[float, str, str]] = []
            self.mode = None
            self.truth: list[tuple[float, float, float, float]] = []
            self.pub = self.create_publisher(AckermannDriveStamped, "controller/ackermann_cmd", 20)
            self.pub_teleop = self.create_publisher(AckermannDriveStamped, "teleop/ackermann_cmd", 20)
            self.pub_lock = self.create_publisher(Bool, "mux/lock", 10)
            self.arm_client = self.create_client(Trigger, "mux/arm")
            self.create_subscription(Clock, "/clock", self.on_clock, 50)
            self.create_subscription(JointState, "/joint_states", self.on_joints, 50)
            self.create_subscription(PoseStamped, f"/mocap/{MODEL}/pose", self.on_pose, 50)
            self.create_subscription(String, "mux/state", self.on_state, 50)
            self.create_subscription(String, "mocap/stats", self.on_mocap_stats, 10)
            self.create_timer(0.02, self.stream)

        def on_clock(self, msg) -> None:
            self.sim = secs(msg.clock)
            self.clock_rows.append((self.sim, time.monotonic()))

        def now(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def on_joints(self, msg) -> None:
            if STEER_JOINT in msg.name:
                with self.lock:
                    self.joints.append((secs(msg.header.stamp), msg.position[msg.name.index(STEER_JOINT)]))

        def on_pose(self, msg) -> None:
            with self.lock:
                now = self.now()
                self.pose_age.append((now, now - secs(msg.header.stamp)))

        def on_mocap_stats(self, msg) -> None:
            self.mocap_stats = json.loads(msg.data)

        def on_state(self, msg) -> None:
            s = json.loads(msg.data)
            self.mode = s["mode"]
            e = s.get("last_event")
            if e and (not self.events or tuple(self.events[-1]) != (e[0], e[2], e[3])):
                with self.lock:
                    self.events.append((e[0], e[2], e[3]))

        def on_truth(self, stamp, _received, x, y, yaw, *_rest) -> None:
            with self.lock:
                self.truth.append((stamp, x, y, yaw))

        def send(self, publisher, speed: float, angle: float) -> float:
            msg = AckermannDriveStamped()
            stamp = self.get_clock().now()
            msg.header.stamp = stamp.to_msg()
            msg.drive.speed, msg.drive.steering_angle = speed, angle
            publisher.publish(msg)
            return stamp.nanoseconds * 1e-9

        def stream(self) -> None:
            if self.streaming:
                self.send(self.pub, *self.cmd)

        def step(self, speed: float, angle: float) -> float:
            """Change the command and publish it at once (not on the 50 Hz grid)."""
            self.cmd = (speed, angle)
            return self.send(self.pub, speed, angle)

    rclpy.init()
    node = Checker()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    truth = GzTruthReader(f"/world/{WORLD}/dynamic_pose/info", MODEL, node.now, node.on_truth)
    truth.start()

    def wait_sim(seconds: float) -> None:
        start, t0 = time.monotonic(), node.sim
        while node.sim - t0 < seconds:
            if time.monotonic() - start > 120:
                raise TimeoutError("simulation clock stalled")
            time.sleep(0.0005)

    def arm() -> tuple[bool, str]:
        if not node.arm_client.wait_for_service(timeout_sec=10):
            return False, "no mux/arm service"
        future = node.arm_client.call_async(Trigger.Request())
        start = time.monotonic()
        while not future.done() and time.monotonic() - start < 10:
            time.sleep(0.005)
        r = future.result()
        return (bool(r.success), r.message) if r else (False, "no reply")

    def reset() -> None:
        req = (f'name: "{MODEL}", position: {{x: 0, y: 0, z: {geometry.wheel_radius + 0.005}}}, '
               "orientation: {z: 0, w: 1}")
        subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/set_pose", "--reqtype", "gz.msgs.Pose",
                        "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
                       check=True, capture_output=True)

    def truth_speed(t0: float, t1: float):
        with node.lock:
            a = np.array([r for r in node.truth if t0 <= r[0] <= t1])
        v, _, _ = ba.body_velocities(a[:, 0], a[:, 1], a[:, 2], a[:, 3])
        return a[:, 0], v

    report: dict = {"note": "SIMULATION ONLY, ASSUMED chain parameters (params/command_chain.yaml); not a vehicle result",
                    "label": args.label,
                    "profile": args.profile, "mocap_profile": args.mocap_profile,
                    "actuator_profile_values": vars(am.load_profile(args.profile)), "mux": vars(mux_cfg)}
    safety: dict = {}
    try:
        start = time.monotonic()
        while not (node.joints and node.pose_age and node.mode and node.truth):
            if time.monotonic() - start > 90:
                raise TimeoutError(f"missing inputs: joints {bool(node.joints)}, mocap {bool(node.pose_age)}, "
                                   f"mux {node.mode}, truth {bool(node.truth)}")
            time.sleep(0.1)
        t_reset = node.now()
        reset()
        wait_sim(1.0)
        node.streaming = True
        wait_sim(0.3)
        ok, why = arm()
        if not ok:
            raise RuntimeError(f"arming failed: {why}")
        t_armed = node.now()

        # D1 steering steps at random times (not on the 50 Hz stream grid)
        n_clock0 = len(node.clock_rows)
        cmd_times, angle = [], args.step_angle
        node.step(0.0, -angle)
        wait_sim(0.6)
        for _ in range(args.steps):
            wait_sim(float(rng.uniform(0.45, 0.65)))
            cmd_times.append(node.step(0.0, angle))
            angle = -angle
        wait_sim(0.5)
        clock = np.array(node.clock_rows[n_clock0:])
        report["rtf_during_D1"] = ba.real_time_factor(clock[:, 0], clock[:, 1])
        with node.lock:
            j = np.array(node.joints)
        delays = [ca.motion_start(j[:, 0], j[:, 1], t, threshold=2e-3) for t in cmd_times]
        offline = am.servo_start_delays(am.load_profile(args.profile), 2000, np.random.default_rng(args.seed),
                                        step=2 * args.step_angle, delta_max=geometry.delta_max)
        report["D1_command_to_steering_motion"] = {
            "gazebo": ca.summary(delays), "gazebo_hist_2ms": ca.histogram(delays), "gazebo_samples_s": delays,
            "offline_chain_only": ca.summary(offline), "offline_hist_2ms": ca.histogram(offline),
            "what": "controller header stamp -> front-left steering joint 2 mrad from its previous angle; "
                    "Gazebo adds the 2 ms actuator tick, the 200 Hz controller manager and 200 Hz joint_states"}

        # D3 speed step, straight
        node.step(0.0, 0.0)
        wait_sim(0.6)
        t_v = node.step(0.5, 0.0)
        wait_sim(1.5)
        node.step(0.0, 0.0)
        wait_sim(1.0)
        t, v = truth_speed(t_v - 0.2, t_v + 1.5)
        report["D3_speed_step"] = {"cmd_mps": 0.5, "start_delay_s": ca.motion_start(t, v, t_v, threshold=0.01),
                                   "rise_10_90_s": ba.rise_time(t[t >= t_v], v[t >= t_v], 0.0, 0.5),
                                   "final_mps": float(np.mean(v[t >= t_v + 1.2]))}

        # S1 setpoint timeout -> hold, car stops; S2 no automatic resume
        node.step(0.3, 0.0)
        wait_sim(0.8)
        node.streaming = False
        t_stop = node.now()
        wait_sim(0.8)
        trip = ca.first_transition(node.events, t_stop, "hold")
        t_s, v_s = truth_speed(t_stop, t_stop + 0.8)
        safety["S1_setpoint_timeout"] = {
            "transition": trip, "after_last_setpoint_s": None if trip is None else trip[0] - t_stop,
            "speed_after_hold_mps": None if trip is None else float(np.max(np.abs(v_s[t_s >= trip[0] + 0.3]))),
            "pass": bool(trip and trip[1] == "setpoint_timeout"
                         and mux_cfg.T_wd - 0.03 <= trip[0] - t_stop <= mux_cfg.T_wd + 0.05)}
        node.streaming = True
        wait_sim(0.6)
        safety["S2_no_auto_resume"] = {"mode_after_0.6s_of_new_setpoints": node.mode, "pass": node.mode == "hold"}
        ok, why = arm()
        safety["S3_explicit_rearm"] = {"ok": ok, "message": why, "pass": ok}
        node.step(0.0, 0.0)

        # S4 teleop takeover and release
        wait_sim(0.3)
        t_tel = node.now()
        for _ in range(10):
            node.send(node.pub_teleop, 0.0, 0.1)
            wait_sim(0.05)
        t_last_tel = node.now()
        wait_sim(0.6)
        take = ca.first_transition(node.events, t_tel - 0.01, "teleop")
        rel = ca.first_transition(node.events, t_tel, "hold")
        safety["S4_teleop"] = {"takeover": take, "release": rel, "mode_after": node.mode,
                               "pass": bool(take and rel and rel[1] == "teleop_released" and node.mode == "hold"
                                            and mux_cfg.T_teleop - 0.05 <= rel[0] - t_last_tel <= mux_cfg.T_teleop + 0.05)}

        # S5 lock and release
        ok, _ = arm()
        wait_sim(0.2)
        t_lock = node.now()
        node.pub_lock.publish(Bool(data=True))
        wait_sim(0.3)
        locked = node.mode
        ok_locked, why_locked = arm()
        node.pub_lock.publish(Bool(data=False))
        wait_sim(0.5)
        safety["S5_lock"] = {"rearmed_before": ok, "mode_while_locked": locked, "arm_while_locked": [ok_locked, why_locked],
                             "transitions": [e for e in node.events if e[0] >= t_lock], "mode_after_release": node.mode,
                             "pass": locked == "locked" and not ok_locked and node.mode == "hold"}
        node.streaming = False
        report["S_live_mux"] = safety
        report["events"] = node.events

        # D2 pose age at the controller input (from arming on; the start-up is excluded)
        wait_sim(1.2)  # one more mocap/stats message
        with node.lock:
            rows = [(t, a) for t, a in node.pose_age if t >= t_armed]
        ages = [a for _, a in rows]
        report["D2_pose_capture_to_controller_input"] = {
            "summary": ca.summary(ages), "hist_2ms": ca.histogram(ages),
            "over_100ms_at": [[round(t, 3), round(a * 1e3, 1)] for t, a in rows if a > 0.1][:60],
            "mocap_stats": node.mocap_stats, "t_armed": t_armed, "t_reset": t_reset,
            "what": "receive time (sim) - capture stamp of /mocap pose; source_lag and late_frames in mocap_stats "
                    "are Gazebo-pipeline artifacts, not part of the emulated delay"}
    finally:
        node.streaming = False
        truth.stop()
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        (out / "report.json").write_text(json.dumps(report, indent=2, default=str))
        node.destroy_node()
        rclpy.try_shutdown()

    print_summary(report)
    print(f"report: {Path(args.out) / 'report.json'}")
    return 0 if all(s["pass"] for s in report["S_live_mux"].values()) else 1


def print_summary(r: dict) -> None:
    f = lambda s: (f"median {s['median_ms']:.1f} ms, p5-p95 {s['p5_ms']:.1f}-{s['p95_ms']:.1f}, "  # noqa: E731
                   f"min-max {s['min_ms']:.1f}-{s['max_ms']:.1f} (n={s['n']}, nan {s['n_nan']})") if s.get("n") else str(s)
    print(f"SIMULATION ONLY, ASSUMED chain parameters - actuator {r['profile']}, mocap {r['mocap_profile']}")
    d1 = r["D1_command_to_steering_motion"]
    print(f"D1 command -> steering motion, Gazebo:   {f(d1['gazebo'])}")
    print(f"D1 chain only, offline model:           {f(d1['offline_chain_only'])}")
    print(f"   RTF during D1: {r.get('rtf_during_D1', float('nan')):.3f}")
    d2 = r["D2_pose_capture_to_controller_input"]
    print(f"D2 pose capture -> controller input:     {f(d2['summary'])}")
    ms = d2["mocap_stats"]
    if ms:
        print(f"   mocap: frames {ms['frames']}, dropped {ms['dropped']}, spikes {ms['spikes']}; source lag (sim artifact) "
              f"median {ms['source_lag_ms']['median']:.1f} ms, max {ms['source_lag_ms']['max']:.1f} ms, "
              f">20 ms: {ms['source_lag_ms']['n_over_20ms']}; late frames {ms['late_frames']['n']} "
              f"(max {ms['late_frames']['max_ms']:.0f} ms) at {ms['late_frames']['times'][:6]}")
    print(f"   ages > 100 ms at (t, ms): {d2['over_100ms_at'][:8]}")
    d3 = r["D3_speed_step"]
    print(f"D3 speed step 0.5 m/s: start {d3['start_delay_s'] * 1e3:.0f} ms, rise 10-90 {d3['rise_10_90_s'] * 1e3:.0f} ms, "
          f"final {d3['final_mps']:.3f} m/s")
    for k, s in r["S_live_mux"].items():
        print(f"{k}: {'PASS' if s['pass'] else 'FAIL'}  {json.dumps({x: y for x, y in s.items() if x != 'pass'}, default=str)[:220]}")


if __name__ == "__main__":
    sys.exit(main())
