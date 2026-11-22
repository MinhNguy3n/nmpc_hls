"""Plant interface of the RC car simulation: applied (speed, steering angle) -> Gazebo joint commands.

Input:  `plant/ackermann_cmd` (ackermann_msgs/AckermannDriveStamped): `drive.speed` is the
        rear-axle-midpoint speed (m/s) and `drive.steering_angle` the bicycle-equivalent angle
        (rad, left positive), i.e. what the actuator has *applied*. In phase 2 only the actuator
        emulation (`ackermann_actuator`) publishes here; the bring-up checks publish directly.
Output: `/steering_position_controller/commands` [left, right] (rad) and
        `/wheel_velocity_controller/commands` [FL, FR, RL, RR] (rad/s), std_msgs/Float64MultiArray,
        computed by the tested ackermann_joints.joint_commands at once on every input (so the
        interface adds no hold of its own) and at a fixed rate (`rate`, Hz) for the guard.

Parameter `driven` (diagnostic): `4wd` (default) publishes all four wheel speeds; `rwd` publishes
only [RL, RR], for the rear-drive variant of the C3 scrub diagnostic (config/controllers_rwd.yaml).

Sim-only guard (not the vehicle watchdog T_wd): if no input arrives for `plant_timeout` (from the
parameter file), the wheel speeds go to zero and the steering holds its last angle.

    python3 app/rc_car/plant_interface_node.py --ros-args -p use_sim_time:=true
"""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import vehicle_params as vp  # noqa: E402
from ackermann_joints import JointCommands, joint_commands  # noqa: E402


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from ackermann_msgs.msg import AckermannDriveStamped
    from std_msgs.msg import Float64MultiArray

    class PlantInterface(Node):
        def __init__(self) -> None:
            super().__init__("plant_interface")
            path = self.declare_parameter("params_file", str(vp.DEFAULT_PARAMS)).value
            rate = float(self.declare_parameter("rate", 100.0).value)
            driven = str(self.declare_parameter("driven", "4wd").value)
            if driven not in ("4wd", "rwd"):
                raise ValueError(f"driven must be 4wd or rwd, not {driven!r}")
            self.wheel_slice = slice(0, 4) if driven == "4wd" else slice(2, 4)
            raw = vp.load_raw(path)
            self.geometry = vp.load_geometry(path)
            self.v_max = float(raw["v_max_sim"]["value"])
            self.timeout = float(raw["plant_timeout"]["value"])
            self.speed = 0.0
            self.delta = 0.0
            self.last_input = None  # node-clock time of the last input
            self.timed_out = True
            self.pub_steer = self.create_publisher(Float64MultiArray, "/steering_position_controller/commands", 10)
            self.pub_wheel = self.create_publisher(Float64MultiArray, "/wheel_velocity_controller/commands", 10)
            self.create_subscription(AckermannDriveStamped, "plant/ackermann_cmd", self.on_cmd, 10)
            self.create_timer(1.0 / rate, self.tick)
            self.get_logger().info(
                f"L={self.geometry.wheelbase} Wf={self.geometry.front_track} delta_max={self.geometry.delta_max} "
                f"v_max={self.v_max} timeout={self.timeout} s driven={driven}; PLACEHOLDER_ entries: {vp.placeholders(path)}")

        def on_cmd(self, msg: AckermannDriveStamped) -> None:
            self.speed = float(msg.drive.speed)
            self.delta = float(msg.drive.steering_angle)
            self.last_input = self.get_clock().now()
            self.publish(joint_commands(self.speed, self.delta, self.geometry, self.v_max))

        def tick(self) -> None:
            now = self.get_clock().now()
            stale = self.last_input is None or (now - self.last_input).nanoseconds * 1e-9 > self.timeout
            if stale != self.timed_out:
                self.get_logger().warn("input stale: wheels stopped" if stale else "input fresh")
                self.timed_out = stale
            c = joint_commands(0.0 if stale else self.speed, self.delta, self.geometry, self.v_max)
            self.publish(c)

        def publish(self, c: JointCommands) -> None:
            self.pub_steer.publish(Float64MultiArray(data=c.steering))
            self.pub_wheel.publish(Float64MultiArray(data=list(c.wheels)[self.wheel_slice]))

    rclpy.init()
    node = PlantInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:  # leave the plant stopped with the steering straight
            node.publish(JointCommands(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        except Exception:  # the context may already be shut down by the signal
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
