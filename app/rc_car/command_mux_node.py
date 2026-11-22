"""Command mux node of the RC car simulation (the rules are in command_mux.py).

Inputs:  `teleop/ackermann_cmd` (AckermannDriveStamped; the RC transmitter in the vehicle chain,
         top priority), `controller/ackermann_cmd` (the controller under test; header stamp =
         the time the command was computed), `mux/lock` (std_msgs/Bool, kill lock),
         `/mocap/<body>/pose` (PoseStamped, for the freshness gate; header stamp = capture time).
Services: `mux/arm`, `mux/disarm` (std_srvs/Trigger). Arming is explicit only.
Outputs: `actuator/ackermann_cmd` (AckermannDriveStamped): forwarded at once with the source's
         header stamp, plus a keep-alive every 1/publish_rate s (the hold, or the last command);
         `mux/state` (std_msgs/String, JSON) on every mode change and at 2 Hz.

Parameters: `chain_file` (mux and freshness_gate sections), `body` (default reely_new1),
`use_gate` (default true).

    python3 app/rc_car/command_mux_node.py --ros-args -p use_sim_time:=true
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import command_mux as cm  # noqa: E402


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger

    def stamp_s(header) -> float:
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    class MuxNode(Node):
        def __init__(self) -> None:
            super().__init__("command_mux")
            chain = self.declare_parameter("chain_file", str(cm.DEFAULT_CHAIN)).value
            body = self.declare_parameter("body", "reely_new1").value
            use_gate = bool(self.declare_parameter("use_gate", True).value)
            import yaml
            rate = float(yaml.safe_load(Path(chain).read_text())["mux"]["publish_rate"])
            self.gate = cm.FreshnessGate(cm.load_gate_threshold(chain)) if use_gate else None
            self.mux = cm.CommandMux(cm.load_mux_config(chain), self.gate)
            self.n_events = 0
            self.pub = self.create_publisher(AckermannDriveStamped, "actuator/ackermann_cmd", 20)
            self.pub_state = self.create_publisher(String, "mux/state", 10)
            self.create_subscription(AckermannDriveStamped, "teleop/ackermann_cmd", self.on_teleop, 20)
            self.create_subscription(AckermannDriveStamped, "controller/ackermann_cmd", self.on_ctrl, 20)
            self.create_subscription(Bool, "mux/lock", self.on_lock, 10)
            self.create_subscription(PoseStamped, f"/mocap/{body}/pose", self.on_pose, 20)
            self.create_service(Trigger, "mux/arm", self.on_arm)
            self.create_service(Trigger, "mux/disarm", self.on_disarm)
            self.create_timer(1.0 / rate, self.tick)
            self.create_timer(0.5, self.publish_state)
            self.get_logger().info(f"{self.mux.config}; gate {'T_fresh=%g s' % self.gate.T_fresh if self.gate else 'off'}"
                                   " (ASSUMED values)")

        def now(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def send(self, out: cm.Output | None) -> None:
            if out is not None:
                msg = AckermannDriveStamped()
                if out.stamp is not None:
                    sec = int(out.stamp)
                    msg.header.stamp.sec, msg.header.stamp.nanosec = sec, int(round((out.stamp - sec) * 1e9))
                else:
                    msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = out.source
                msg.drive.speed, msg.drive.steering_angle = out.speed, out.steering_angle
                self.pub.publish(msg)
            if len(self.mux.events) != self.n_events:
                for e in self.mux.events[self.n_events:]:
                    self.get_logger().warn(f"mode {e[1]} -> {e[2]} ({e[3]}) at {e[0]:.3f}")
                self.n_events = len(self.mux.events)
                self.publish_state()

        def on_teleop(self, msg) -> None:
            self.send(self.mux.on_teleop(self.now(), msg.drive.speed, msg.drive.steering_angle, stamp_s(msg.header)))

        def on_ctrl(self, msg) -> None:
            self.send(self.mux.on_controller(self.now(), msg.drive.speed, msg.drive.steering_angle, stamp_s(msg.header)))

        def on_lock(self, msg) -> None:
            self.mux.on_lock(self.now(), bool(msg.data))
            self.send(self.mux.output())

        def on_pose(self, msg) -> None:
            if self.gate is not None:
                self.gate.on_pose(stamp_s(msg.header))

        def on_arm(self, _req, res):
            res.success, res.message = self.mux.request_arm(self.now())
            self.send(None)
            return res

        def on_disarm(self, _req, res):
            self.mux.disarm(self.now(), "operator")
            res.success, res.message = True, self.mux.mode.value
            self.send(self.mux.output())
            return res

        def tick(self) -> None:
            self.send(self.mux.tick(self.now()))

        def publish_state(self) -> None:
            now = self.now()
            state = {"t": now, "mode": self.mux.mode.value, "lock": self.mux.lock_engaged,
                     "pose_age": None if self.gate is None or self.gate.newest is None else now - self.gate.newest,
                     "counters": vars(self.mux.counters), "last_event": self.mux.events[-1] if self.mux.events else None}
            self.pub_state.publish(String(data=json.dumps(state)))

    rclpy.init()
    node = MuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
