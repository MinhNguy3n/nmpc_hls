"""NMPC arm node of the RC car simulation (SIMULATION ONLY; the logic is nmpc_controller.py).

Input:  `/mocap/<body>/pose` (PoseStamped, header = capture time) through pose_input.NewestPose;
        `nmpc/command` (std_msgs/String JSON): {"cmd": "start", "path", "speed", "seed"
        [, "q_pos", "q_psi"]} -> HOLD (streams v = 0, kappa = 0 so that the mux can be armed);
        {"cmd": "go"} -> RUNNING; {"cmd": "stop"} -> IDLE.
Output: `controller/ackermann_cmd` (AckermannDriveStamped): speed and bicycle steering angle,
        published at the timing model's t_apply (simulation time), header stamp = t_apply;
        `nmpc/status` (std_msgs/String JSON, 5 Hz and at the end): state, progress, ticks,
        deadline misses, stale poses at the tick, missing poses, fallbacks, Python overruns and busy ticks, solve wall
        times, costs.

The PSO runs in a worker process (its RNG seeded from the episode seed), so the node keeps
publishing on time. Python's speed never enters the timing model: a result that arrives after
its t_apply is an OVERRUN (counted; lower the real-time factor so that there are none), and a
tick that finds the worker still busy is skipped and counted (python_busy).

Parameters: `size` (3x24 | 10x40 | 10x60), `profile` (actuator profile of the run: the chain term
of tau), `compensation` (true), `arith` (half | float), `variant` (phase4 = the frozen phase 4
controller | redesign = the frozen redesign of params/nmpc.yaml | redesign_smooth = the variant
NOT selected by the rule, run for steering smoothness), `body`.

    python3 app/rc_car/nmpc_node.py --ros-args -p use_sim_time:=true -p size:=10x40 -p profile:=realistic_guess
"""

from __future__ import annotations

import json
import math
import multiprocessing as mp
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def worker(conn) -> None:
    """Solve loop of the worker process: ('seed', s) re-seeds; ('solve', problem, pso, w, arith)."""
    import numpy as np
    import nmpc_engine as ne
    rng = np.random.default_rng(0)
    while True:
        msg = conn.recv()
        if msg[0] == "seed":
            rng = np.random.default_rng([int(msg[1]), 7_919])  # as NmpcController.start_episode
        elif msg[0] == "solve":
            t0 = time.perf_counter()
            sol = ne.solve(msg[1], msg[2], msg[3], rng, msg[4])
            conn.send((sol, time.perf_counter() - t0))
        elif msg[0] == "quit":
            return


def main() -> None:
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    import nmpc_controller as nc
    import paths as P
    from pose_input import NewestPose, yaw_from_quaternion

    class NmpcNode(Node):
        def __init__(self, conn) -> None:
            super().__init__("nmpc")
            self.size = self.declare_parameter("size", "10x40").value
            self.profile = self.declare_parameter("profile", "realistic_guess").value
            self.compensation = bool(self.declare_parameter("compensation", True).value)
            self.arith = self.declare_parameter("arith", "half").value
            self.variant = self.declare_parameter("variant", "phase4").value  # phase4 | redesign | redesign_smooth
            if self.variant not in ("phase4", "redesign", "redesign_smooth"):
                raise ValueError(f"variant must be phase4, redesign or redesign_smooth, not {self.variant!r}")
            body = self.declare_parameter("body", "reely_new1").value
            self.cfg = nc.load_config()
            self.area = P.load_area()
            self.conn = conn
            self.pose_in = NewestPose()
            self.state = "idle"
            self.ctrl = None
            self.pending_plan = None
            self.sched = nc.Scheduler()
            self.busy_skips = 0
            self.solve_wall: list[float] = []
            self.last_cmd = None
            self.pub = self.create_publisher(AckermannDriveStamped, "controller/ackermann_cmd", 20)
            self.pub_status = self.create_publisher(String, "nmpc/status", 10)
            self.create_subscription(PoseStamped, f"/mocap/{body}/pose", self.on_pose, 50)
            self.create_subscription(String, "nmpc/command", self.on_command, 10)
            self.create_timer(self.cfg.ts, self.tick)
            self.create_timer(0.001, self.poll)
            self.create_timer(0.2, self.status)
            self.get_logger().info(f"size {self.size} profile {self.profile} compensation {self.compensation} "
                                   f"arith {self.arith}; limits {self.cfg.limits}; weights {self.cfg.weights} "
                                   f"({self.cfg.weights_status})")

        def now(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def on_pose(self, msg) -> None:
            q = msg.pose.orientation
            self.pose_in.update(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg.pose.position.x,
                                msg.pose.position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

        def on_command(self, msg) -> None:
            d = json.loads(msg.data)
            if d["cmd"] == "start":
                weights = None
                if "q_pos" in d or "q_psi" in d:
                    import yaml
                    w = yaml.safe_load((HERE / "params" / "nmpc.yaml").read_text())["weights"]
                    weights = nc.weights_from(w, d.get("q_pos"), d.get("q_psi"))
                rd = None if self.variant == "phase4" else nc.load_redesign(section=self.variant)
                self.ctrl = nc.NmpcController(self.cfg, self.size, self.profile, self.compensation, self.arith, weights,
                                              redesign=rd)
                path = P.build(d["path"], self.area, kappa_max=self.cfg.limits.kappa_max)
                self.ctrl.start_episode(path, float(d["speed"]), int(d["seed"]))
                self.conn.send(("seed", int(d["seed"])))
                self.sched, self.busy_skips, self.solve_wall, self.pending_plan = nc.Scheduler(), 0, [], None
                self.state = "hold"
                self.get_logger().info(f"episode {d}: weights {self.ctrl.weights}")
            elif d["cmd"] == "go" and self.state == "hold":
                self.state = "running"
            elif d["cmd"] == "stop":
                self.state = "idle"
            self.status()

        def send(self, t_stamp: float, v: float, delta: float) -> None:
            msg = AckermannDriveStamped()
            sec = int(math.floor(t_stamp))
            msg.header.stamp.sec, msg.header.stamp.nanosec = sec, int(round((t_stamp - sec) * 1e9))
            msg.header.frame_id = "nmpc"
            msg.drive.speed, msg.drive.steering_angle = float(v), float(delta)
            self.pub.publish(msg)

        def tick(self) -> None:
            t = self.now()
            if self.state == "hold":
                self.send(t, 0.0, 0.0)
            elif self.state == "running":
                if self.pending_plan is not None:
                    self.busy_skips += 1
                    return
                plan = self.ctrl.prepare(t, self.pose_in.pose)
                if plan is not None:
                    self.pending_plan = plan
                    self.conn.send(("solve", plan.problem, self.ctrl.pso, self.ctrl.weights, self.arith))

        def poll(self) -> None:
            t = self.now()
            if self.pending_plan is not None and self.conn.poll():
                sol, wall = self.conn.recv()
                self.solve_wall.append(wall)
                cmd = self.ctrl.finish(self.pending_plan, sol)
                self.pending_plan = None
                if self.state == "running":
                    self.sched.submit(t, cmd)
                    self.last_cmd = cmd
                    if cmd.done:
                        self.state = "done"
                        self.status()
            for cmd in self.sched.due(t):
                self.send(cmd.t_apply, cmd.v, cmd.delta)

        def status(self) -> None:
            st = self.ctrl.stats if self.ctrl else None
            wall = np.array(self.solve_wall) if self.solve_wall else np.zeros(1)
            d = {"t": self.now(), "state": self.state, "size": self.size, "profile": self.profile,
                 "compensation": self.compensation, "arith": self.arith, "variant": self.variant,
                 "redesign": self.ctrl.redesign.label if self.ctrl else None,
                 "ticks": st.ticks if st else 0, "deadline_misses": st.deadline_misses if st else 0,
                 "stale_pose": st.stale_pose if st else 0,
                 "no_pose": st.no_pose if st else 0, "fallbacks": st.fallbacks if st else 0,
                 "overruns": self.sched.overruns, "overrun_max_s": self.sched.overrun_max,
                 "python_busy": self.busy_skips,
                 "solve_wall_ms": {"n": len(self.solve_wall), "median": float(np.median(wall) * 1e3),
                                   "max": float(wall.max() * 1e3)},
                 "cost_final": st.costs[-1] if st and st.costs else None,
                 "cost_mean": float(np.mean(st.costs)) if st and st.costs else None,
                 "progress_m": self.last_cmd.s_measured if self.last_cmd else 0.0,
                 "solve_time_model_s": self.ctrl.solve_time if self.ctrl else None,
                 "tau_eff_s": self.ctrl.tau_eff if self.ctrl else None,
                 "weights": vars(self.ctrl.weights) if self.ctrl else None}
            self.pub_status.publish(String(data=json.dumps(d)))

    ctx = mp.get_context("spawn")
    parent, child = ctx.Pipe()
    proc = ctx.Process(target=worker, args=(child,), daemon=True)
    proc.start()
    rclpy.init()
    node = NmpcNode(parent)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            parent.send(("quit",))
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
