"""Mocap pose -> TF and odometry for the Nav2 baseline (the controller's view of the car).

Input:  `/mocap/<body>/pose` (PoseStamped; header stamp = capture time) through
        pose_input.NewestPose (out-of-order poses ignored); `actuator/speed` (TwistStamped, the
        emulated VESC speed telemetry).
Output: TF `map -> base_link` stamped with the CAPTURE time (so Nav2 sees the pose's real age),
        and `odom` (nav_msgs/Odometry, frame map, child base_link): the same pose, with
        twist.linear.x = the latest VESC speed (angular left 0: RPP needs only the forward speed,
        for its speed-scaled lookahead).
        `mocap_bridge/stats` (std_msgs/String, JSON, 1 Hz): accepted, ignored_old.

The Gazebo truth never reaches the controller; scoring uses it separately.

    python3 app/rc_car/mocap_bridge_node.py --ros-args -p use_sim_time:=true
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pose_input import NewestPose, yaw_from_quaternion  # noqa: E402


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster

    class Bridge(Node):
        def __init__(self) -> None:
            super().__init__("mocap_bridge")
            body = self.declare_parameter("body", "reely_new1").value
            self.frame = self.declare_parameter("frame", "map").value
            self.input = NewestPose()
            self.speed = 0.0
            self.tf = TransformBroadcaster(self)
            self.pub_odom = self.create_publisher(Odometry, "odom", 20)
            self.pub_stats = self.create_publisher(String, "mocap_bridge/stats", 10)
            self.create_subscription(PoseStamped, f"/mocap/{body}/pose", self.on_pose, 20)
            self.create_subscription(TwistStamped, "actuator/speed", self.on_speed, 20)
            self.create_timer(1.0, self.stats)

        def on_speed(self, msg) -> None:
            self.speed = float(msg.twist.linear.x)

        def on_pose(self, msg) -> None:
            q = msg.pose.orientation
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            if not self.input.update(stamp, msg.pose.position.x, msg.pose.position.y, yaw):
                return
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id, t.child_frame_id = self.frame, "base_link"
            t.transform.translation.x, t.transform.translation.y = msg.pose.position.x, msg.pose.position.y
            t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.tf.sendTransform(t)
            o = Odometry()
            o.header.stamp = msg.header.stamp
            o.header.frame_id, o.child_frame_id = self.frame, "base_link"
            o.pose.pose.position.x, o.pose.pose.position.y = msg.pose.position.x, msg.pose.position.y
            o.pose.pose.orientation = t.transform.rotation
            o.twist.twist.linear.x = self.speed
            self.pub_odom.publish(o)

        def stats(self) -> None:
            self.pub_stats.publish(String(data=json.dumps(
                {"accepted": self.input.accepted, "ignored_old": self.input.ignored_old, "ignored_bad": self.input.ignored_bad})))

    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
