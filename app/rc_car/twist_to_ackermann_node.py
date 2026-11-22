"""Twist -> AckermannDriveStamped node (the converter is twist_to_ackermann.convert).

Input:  `cmd_vel_in` (geometry_msgs/TwistStamped if `stamped` is true, the default, else Twist).
Output: `controller/ackermann_cmd` (AckermannDriveStamped). The header stamp is the input's stamp
        (the time the controller computed the command); for an unstamped or zero-stamped input,
        the receive time.
        `converter/stats` (std_msgs/String, JSON, 1 Hz): messages, clamped, below v_min.

Parameters: `params_file` (wheelbase, delta_max), `v_min` (m/s, PLACEHOLDER_ default 0.05), `stamped`.

    python3 app/rc_car/twist_to_ackermann_node.py --ros-args -p use_sim_time:=true -r cmd_vel_in:=/baseline/cmd_vel
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import vehicle_params as vp  # noqa: E402
from twist_to_ackermann import convert  # noqa: E402


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import Twist, TwistStamped
    from std_msgs.msg import String

    class Converter(Node):
        def __init__(self) -> None:
            super().__init__("twist_to_ackermann")
            g = vp.load_geometry(self.declare_parameter("params_file", str(vp.DEFAULT_PARAMS)).value)
            self.L, self.dmax = g.wheelbase, g.delta_max
            self.v_min = float(self.declare_parameter("v_min", 0.05).value)
            stamped = bool(self.declare_parameter("stamped", True).value)
            self.counts = {"messages": 0, "clamped": 0, "below_v_min": 0}
            self.pub = self.create_publisher(AckermannDriveStamped, "controller/ackermann_cmd", 20)
            self.pub_stats = self.create_publisher(String, "converter/stats", 10)
            if stamped:
                self.create_subscription(TwistStamped, "cmd_vel_in", lambda m: self.on_twist(m.twist, m.header.stamp), 20)
            else:
                self.create_subscription(Twist, "cmd_vel_in", lambda m: self.on_twist(m, None), 20)
            self.create_timer(1.0, lambda: self.pub_stats.publish(String(data=json.dumps(self.counts))))
            self.get_logger().info(f"L={self.L} delta_max={self.dmax} v_min={self.v_min} (PLACEHOLDER_) stamped={stamped}")

        def on_twist(self, twist, stamp) -> None:
            c = convert(twist.linear.x, twist.angular.z, self.L, self.dmax, self.v_min)
            self.counts["messages"] += 1
            self.counts["clamped"] += int(c.clamped)
            self.counts["below_v_min"] += int(c.below_v_min)
            msg = AckermannDriveStamped()
            if stamp is None or (stamp.sec == 0 and stamp.nanosec == 0):
                msg.header.stamp = self.get_clock().now().to_msg()
            else:
                msg.header.stamp = stamp
            msg.drive.speed, msg.drive.steering_angle = c.speed, c.steering_angle
            self.pub.publish(msg)

    rclpy.init()
    node = Converter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
