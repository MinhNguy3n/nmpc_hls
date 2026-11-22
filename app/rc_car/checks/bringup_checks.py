"""Bring-up checks C1-C6 of the Reely NEW1 simulation (SIMULATION ONLY: never a vehicle result).

Run inside the container while app/rc_car/launch/rc_car_sim.launch.py is up. The script publishes
applied commands straight to the plant interface (`plant/ackermann_cmd`; no actuator emulation,
no controller), resets the car with Gazebo's set_pose between scenarios, and scores everything
against the Gazebo truth of the model (the rear-axle midpoint):

  C1 rest: no drift with a zero command
  C2 straight-line speed tracking at 3 speeds
  C3 constant delta -> circle radius vs L / tan(delta), left and right, at 3 speeds (+ full lock)
  C4 steering: per-wheel Ackermann angles at standstill, steady-state error, 10-90 % rise time
  C5 topic rates, header stamps vs /clock, use_sim_time of the plant interface
  C6 real-time factor

    python3 app/rc_car/checks/bringup_checks.py [--out DIR] [--speeds 0.5 1.0 1.5]
    python3 app/rc_car/checks/bringup_checks.py --only-full-lock --label rwd   # C3 scrub diagnostic

C3 reports, from 2026-11-22 on, the rear-axle speed error and the lateral speed next to the radius.

Writes DIR/report.json and prints a PASS/FAIL table with the thresholds.
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
import bringup_analysis as ba  # noqa: E402
import vehicle_params as vp  # noqa: E402
from ackermann_joints import STEERING_JOINTS, joint_commands  # noqa: E402
from sil.gz_truth import GzTruthReader  # noqa: E402

WORLD = "lab_floor"
MODEL = "reely_new1"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default=str(Path.home() / "logs" / time.strftime("rc_car_bringup_%Y%m%d_%H%M%S")))
    parser.add_argument("--params", default=str(vp.DEFAULT_PARAMS))
    parser.add_argument("--speeds", type=float, nargs="+", default=[0.5, 1.0, 1.5])
    parser.add_argument("--delta-frac", type=float, default=0.6, help="C3 steering as a fraction of delta_max")
    parser.add_argument("--only-full-lock", action="store_true",
                        help="diagnostic: run only the C3 full-lock circles (both signs) at the lowest speed")
    parser.add_argument("--label", default="", help="free text stored in the report (e.g. the model variant)")
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from ackermann_msgs.msg import AckermannDriveStamped
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray

    geometry = vp.load_geometry(args.params)
    raw = vp.load_raw(args.params)
    v_max = float(raw["v_max_sim"]["value"])

    class Recorder(Node):
        def __init__(self) -> None:
            super().__init__("rc_car_bringup_checks",
                             parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
            self.lock = threading.Lock()
            self.cmd = (0.0, 0.0)
            self.clock: list[tuple[float, float]] = []      # (sim, wall)
            self.joints: list[tuple[float, float, dict]] = []  # (header stamp, latest clock, positions)
            self.truth: list[tuple[float, float, float, float]] = []  # (gz stamp, x, y, yaw)
            self.wheel_cmd_wall: list[float] = []
            self.pub = self.create_publisher(AckermannDriveStamped, "plant/ackermann_cmd", 10)
            self.create_subscription(Clock, "/clock", self.on_clock, 50)
            self.create_subscription(JointState, "/joint_states", self.on_joints, 50)
            self.create_subscription(Float64MultiArray, "/wheel_velocity_controller/commands", self.on_wheel_cmd, 50)
            self.create_timer(0.02, self.tick)  # 50 Hz sim time

        def sim_now(self) -> float:
            with self.lock:
                return self.clock[-1][0] if self.clock else 0.0

        def on_clock(self, msg) -> None:
            with self.lock:
                self.clock.append((msg.clock.sec + msg.clock.nanosec * 1e-9, time.monotonic()))

        def on_joints(self, msg) -> None:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            with self.lock:
                self.joints.append((stamp, self.clock[-1][0] if self.clock else float("nan"),
                                    dict(zip(msg.name, msg.position))))

        def on_wheel_cmd(self, _msg) -> None:
            with self.lock:
                self.wheel_cmd_wall.append(time.monotonic())

        def on_truth(self, stamp, _received, x, y, yaw, *_rest) -> None:
            with self.lock:
                self.truth.append((stamp, x, y, yaw))

        def tick(self) -> None:
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.drive.speed, msg.drive.steering_angle = self.cmd
            self.pub.publish(msg)

    rclpy.init()
    node = Recorder()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    truth = GzTruthReader(f"/world/{WORLD}/dynamic_pose/info", MODEL, node.sim_now, node.on_truth)
    truth.start()

    def wait_sim(seconds: float, wall_timeout: float = 600.0) -> None:
        start, t0 = time.monotonic(), node.sim_now()
        while node.sim_now() - t0 < seconds:
            if time.monotonic() - start > wall_timeout:
                raise TimeoutError("simulation clock stalled")
            time.sleep(0.01)

    def truth_window(t0: float, t1: float) -> tuple[np.ndarray, ...]:
        with node.lock:
            rows = [r for r in node.truth if t0 <= r[0] <= t1]
        a = np.array(rows, dtype=float)
        if len(a) < 10:
            raise RuntimeError(f"only {len(a)} truth samples in [{t0:.2f}, {t1:.2f}]")
        return a[:, 0], a[:, 1], a[:, 2], a[:, 3]

    def joints_window(t0: float, t1: float) -> list[tuple[float, dict]]:
        with node.lock:
            return [(s, p) for s, _c, p in node.joints if t0 <= s <= t1]

    def reset(x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        node.cmd = (0.0, 0.0)
        wait_sim(1.0)
        req = (f'name: "{MODEL}", position: {{x: {x}, y: {y}, z: {geometry.wheel_radius + 0.005}}}, '
               f"orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}")
        subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/set_pose", "--reqtype", "gz.msgs.Pose",
                        "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
                       check=True, capture_output=True)
        wait_sim(1.5)

    def run(speed: float, delta: float, seconds: float) -> tuple[float, float]:
        t0 = node.sim_now()
        node.cmd = (speed, delta)
        wait_sim(seconds)
        t1 = node.sim_now()
        node.cmd = (0.0, 0.0)
        return t0, t1

    report: dict = {"note": "SIMULATION ONLY (Gazebo, plant interface, no actuator emulation); not a vehicle result",
                    "label": args.label, "only_full_lock": args.only_full_lock,
                    "params_file": args.params, "placeholders": vp.placeholders(args.params),
                    "geometry": {"wheelbase": geometry.wheelbase, "front_track": geometry.front_track,
                                 "rear_track": geometry.rear_track, "wheel_radius": geometry.wheel_radius,
                                 "delta_max": geometry.delta_max, "r_min": geometry.r_min,
                                 "inner_wheel_max": geometry.inner_wheel_max},
                    "thresholds": {k: getattr(ba, k) for k in dir(ba) if k.startswith("C") and k[1].isdigit()}}
    wall_start = time.monotonic()
    try:
        start = time.monotonic()
        while not (node.truth and node.joints):
            if time.monotonic() - start > 60:
                raise TimeoutError("no truth or joint_states: is the simulation running (GZ_IP=127.0.0.1)?")
            time.sleep(0.1)

        if args.only_full_lock:
            report["C3"] = []
            for delta in (geometry.delta_max, -geometry.delta_max):
                v = min(args.speeds)
                reset()
                radius = geometry.wheelbase / math.tan(abs(delta))
                t0, t1 = run(v, delta, 1.0 + 1.25 * 2 * math.pi * radius / v)
                r = ba.circle(*truth_window(t0 + 1.0, t1), geometry.wheelbase, delta, speed_cmd=v)
                r.update(speed_cmd=v, delta_cmd=delta, lateral_accel=v * v / radius)
                r["pass"] = abs(r["radius_rel_err"]) <= ba.C3_MAX_RADIUS_ERR and r["turn_sign_ok"]
                report["C3"].append(r)
            print(f"SIMULATION ONLY - C3 full-lock diagnostic, variant {args.label or '(none)'}")
            print_c3(report)
            return 0  # the finally block writes report.json

        # C1 rest
        reset()
        t0, t1 = run(0.0, 0.0, 10.0)
        c1 = ba.drift(*truth_window(t0 + 0.5, t1))
        c1["pass"] = c1["max_xy_m"] <= ba.C1_MAX_DRIFT_M and c1["max_yaw_rad"] <= ba.C1_MAX_YAW_RAD
        report["C1"] = c1

        # C2 straight line
        report["C2"] = []
        for v in args.speeds:
            reset()
            t0, t1 = run(v, 0.0, 6.0)
            r = ba.straight_line(*truth_window(t0 + 1.0, t1))
            r.update(speed_cmd=v, speed_rel_err=r["speed_mps"] / v - 1.0)
            r["pass"] = abs(r["speed_rel_err"]) <= ba.C2_MAX_SPEED_ERR and abs(r["heading_change_rad"]) <= ba.C2_MAX_HEADING_RAD
            report["C2"].append(r)

        # C3 circles: +- delta_frac * delta_max at every speed, + full lock at the lowest speed
        report["C3"] = []
        cases = [(v, s * args.delta_frac * geometry.delta_max) for s in (1, -1) for v in args.speeds]
        cases += [(min(args.speeds), s * geometry.delta_max) for s in (1, -1)]
        for v, delta in cases:
            reset()
            radius = geometry.wheelbase / math.tan(abs(delta))
            t0, t1 = run(v, delta, 1.0 + 1.25 * 2 * math.pi * radius / v)
            r = ba.circle(*truth_window(t0 + 1.0, t1), geometry.wheelbase, delta, speed_cmd=v)
            r.update(speed_cmd=v, delta_cmd=delta, lateral_accel=v * v / radius)
            r["pass"] = abs(r["radius_rel_err"]) <= ba.C3_MAX_RADIUS_ERR and r["turn_sign_ok"]
            report["C3"].append(r)

        # C4 steering at standstill: steps 0 -> +max -> -max -> +0.5 max -> -0.5 max -> 0
        reset()
        report["C4"] = []
        previous = 0.0
        for frac in (1.0, -1.0, 0.5, -0.5, 0.0):
            delta = frac * geometry.delta_max
            expected = joint_commands(0.0, delta, geometry, v_max).steering
            before = joint_commands(0.0, previous, geometry, v_max).steering
            t0, t1 = run(0.0, delta, 1.0)
            rows = joints_window(t0, t1)
            t = np.array([s for s, _ in rows])
            angles = np.array([[p[j] for j in STEERING_JOINTS] for _, p in rows])
            steady = angles[t >= t1 - 0.2].mean(axis=0)
            err = steady - np.array(expected)
            rise = [ba.rise_time(t, angles[:, i], before[i], expected[i]) if expected[i] != before[i] else 0.0
                    for i in range(2)]
            r = {"delta_cmd": delta, "expected_left_right": expected, "steady_left_right": steady.tolist(),
                 "steady_err_rad": err.tolist(), "rise_10_90_s": rise}
            r["pass"] = bool(np.all(np.abs(err) <= ba.C4_MAX_STEER_ERR_RAD))
            report["C4"].append(r)
            previous = delta

        # C5 rates and stamps (whole run)
        with node.lock:
            clock = np.array(node.clock)
            joints = list(node.joints)
            truth_t = [r[0] for r in node.truth]
            wheel_wall = list(node.wheel_cmd_wall)
        js = ba.rate([s for s, _c, _p in joints])
        lag = np.array([c - s for s, c, _p in joints if math.isfinite(c)])
        sim_span = clock[-1, 0] - clock[0, 0]
        wall_span = clock[-1, 1] - clock[0, 1]
        plant = ba.rate(wheel_wall)
        plant["rate_hz_sim"] = plant["rate_hz"] * wall_span / sim_span  # published on a sim-time timer
        use_sim = subprocess.run(["ros2", "param", "get", "/plant_interface", "use_sim_time"],
                                 capture_output=True, text=True, timeout=30).stdout.strip()
        c5 = {"joint_states": js, "joint_states_stamp_lag_s": {"mean": float(lag.mean()), "max": float(np.abs(lag).max())},
              "truth": ba.rate(truth_t), "clock_wall": ba.rate(clock[:, 1]), "plant_commands": plant,
              "plant_interface_use_sim_time": use_sim}
        c5["pass"] = (js["rate_hz"] >= ba.C5_MIN_RATE_FRAC * 200 and c5["truth"]["rate_hz"] >= ba.C5_MIN_RATE_FRAC * 100
                      and plant["rate_hz_sim"] >= ba.C5_MIN_RATE_FRAC * 100
                      and c5["joint_states_stamp_lag_s"]["max"] <= ba.C5_MAX_STAMP_LAG_S and "True" in use_sim)
        report["C5"] = c5

        # C6 real-time factor (no pass threshold: the checks run in sim time; RTF sets wall time only)
        report["C6"] = {"rtf": ba.real_time_factor(clock[:, 0], clock[:, 1]), "sim_s": float(sim_span),
                        "wall_s": float(wall_span)}
    finally:
        node.cmd = (0.0, 0.0)
        time.sleep(0.2)
        truth.stop()
        report["wall_total_s"] = time.monotonic() - wall_start
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        (out / "report.json").write_text(json.dumps(report, indent=2))
        node.destroy_node()
        rclpy.try_shutdown()

    print_summary(report)
    print(f"report: {Path(args.out) / 'report.json'}")
    return 0 if all(_passed(report[k]) for k in ("C1", "C2", "C3", "C4", "C5")) else 1


def _passed(entry) -> bool:
    return all(e["pass"] for e in entry) if isinstance(entry, list) else bool(entry["pass"])


def print_c3(r: dict) -> None:
    for c in r["C3"]:
        print(f"C3 v={c['speed_cmd']:.2f} delta={c['delta_cmd']:+.3f}: R fit {c['radius_fit_m']:.4f} vs {c['radius_expected_m']:.4f} m "
              f"({c['radius_rel_err'] * 100:+.2f} %), rear-axle speed {c['speed_rel_err'] * 100:+.2f} %, a_y {c['lateral_accel']:.2f} m/s^2, "
              f"v_lat mean {c['lateral_speed_mean'] * 1e3:+.1f} mm/s, turn sign {'ok' if c['turn_sign_ok'] else 'WRONG'}  "
              f"{'PASS' if c['pass'] else 'FAIL'}")


def print_summary(r: dict) -> None:
    ok = lambda p: "PASS" if p else "FAIL"  # noqa: E731
    print("SIMULATION ONLY - bring-up checks of the Reely NEW1 model (not a vehicle result)")
    c1 = r["C1"]
    print(f"C1 rest {c1['duration_s']:.1f} s: max xy {c1['max_xy_m'] * 1e3:.3f} mm, max yaw {c1['max_yaw_rad'] * 1e3:.3f} mrad  {ok(c1['pass'])}")
    for c in r["C2"]:
        print(f"C2 v={c['speed_cmd']:.2f}: measured {c['speed_mps']:.4f} m/s ({c['speed_rel_err'] * 100:+.2f} %), "
              f"heading change {c['heading_change_rad'] * 1e3:+.2f} mrad, lateral {c['lateral_max_m'] * 1e3:.1f} mm  {ok(c['pass'])}")
    print_c3(r)
    for c in r["C4"]:
        print(f"C4 delta={c['delta_cmd']:+.3f}: L/R expected {c['expected_left_right'][0]:+.4f}/{c['expected_left_right'][1]:+.4f}, "
              f"steady {c['steady_left_right'][0]:+.4f}/{c['steady_left_right'][1]:+.4f} rad, "
              f"rise {c['rise_10_90_s'][0] * 1e3:.0f}/{c['rise_10_90_s'][1] * 1e3:.0f} ms  {ok(c['pass'])}")
    c5 = r["C5"]
    print(f"C5 joint_states {c5['joint_states']['rate_hz']:.1f} Hz (gap {c5['joint_states']['max_gap_s'] * 1e3:.1f} ms), "
          f"truth {c5['truth']['rate_hz']:.1f} Hz, plant commands {c5['plant_commands']['rate_hz_sim']:.1f} Hz sim, "
          f"stamp lag max {c5['joint_states_stamp_lag_s']['max'] * 1e3:.1f} ms, plant use_sim_time: {c5['plant_interface_use_sim_time']}  {ok(c5['pass'])}")
    print(f"C6 RTF {r['C6']['rtf']:.3f} ({r['C6']['sim_s']:.0f} s sim in {r['C6']['wall_s']:.0f} s wall)")


if __name__ == "__main__":
    sys.exit(main())
