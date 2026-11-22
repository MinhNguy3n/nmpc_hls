"""Actuator emulation node of the RC car simulation: the sole command path into the plant.

Input:  `actuator/ackermann_cmd` (AckermannDriveStamped, from the mux): requested speed (m/s,
        rear-axle midpoint) and bicycle steering angle (rad, left positive).
Output: `plant/ackermann_cmd` (AckermannDriveStamped): the APPLIED speed and angle after the
        emulated command link, output hold, VESC and servo (actuator_model.ActuatorModel),
        sampled every `period` s of sim time (default 2 ms; the model itself handles events at
        their exact times).
        `actuator/speed` (TwistStamped, linear.x): the emulated VESC speed telemetry, from the
        rear wheel joint speeds x wheel radius, at `telemetry_rate` Hz. The controller's speed
        feedback; scoring uses the Gazebo truth.

`episode/start` (std_msgs/String, JSON {"t_start", "seed"}, episode.py): the PWM hold phases are
drawn from the seed and placed relative to t_start (paired seeds across runs).

Parameters: `profile` (ideal | realistic_guess | fast_servo, params/command_chain.yaml), `seed`
(hold phases), `chain_file`, `params_file`. ALL PROFILE VALUES ARE ASSUMED.

    python3 app/rc_car/actuator_node.py --ros-args -p use_sim_time:=true -p profile:=realistic_guess
"""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import actuator_model as am  # noqa: E402
import episode  # noqa: E402
import vehicle_params as vp  # noqa: E402
from ackermann_joints import WHEEL_JOINTS  # noqa: E402


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import TwistStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    class ActuatorNode(Node):
        def __init__(self) -> None:
            super().__init__("ackermann_actuator")
            name = self.declare_parameter("profile", "realistic_guess").value
            seed = int(self.declare_parameter("seed", 0).value)
            chain = self.declare_parameter("chain_file", str(am.DEFAULT_CHAIN)).value
            params = self.declare_parameter("params_file", str(vp.DEFAULT_PARAMS)).value
            period = float(self.declare_parameter("period", 0.002).value)
            telemetry_rate = float(self.declare_parameter("telemetry_rate", 50.0).value)
            raw = vp.load_raw(params)
            geometry = vp.load_geometry(params)
            self.radius = geometry.wheel_radius
            self.profile = am.load_profile(name, chain)
            self.model = None  # created at the first tick, on the sim clock
            self.model_args = (self.profile, geometry.delta_max, float(raw["v_max_sim"]["value"]),
                               np.random.default_rng(seed))
            self.pending: list[tuple[float, float, float]] = []
            self.rear_speed = None
            self.pub = self.create_publisher(AckermannDriveStamped, "plant/ackermann_cmd", 10)
            self.pub_speed = self.create_publisher(TwistStamped, "actuator/speed", 10)
            self.create_subscription(AckermannDriveStamped, "actuator/ackermann_cmd", self.on_cmd, 20)
            self.create_subscription(JointState, "/joint_states", self.on_joints, 20)
            self.create_subscription(String, "episode/start", self.on_episode, 10)
            self.create_timer(period, self.tick)
            self.create_timer(1.0 / telemetry_rate, self.telemetry)
            self.get_logger().info(f"profile {name} ({self.profile.status}, ASSUMED values): {self.profile}; seed {seed}")

        def now(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def on_cmd(self, msg: AckermannDriveStamped) -> None:
            t = self.now()
            if self.model is None:
                self.pending.append((t, msg.drive.speed, msg.drive.steering_angle))
            else:
                self.model.command(t, float(msg.drive.speed), float(msg.drive.steering_angle))

        def tick(self) -> None:
            t = self.now()
            if t <= 0.0:
                return  # no /clock yet
            if self.model is None:
                self.model = am.ActuatorModel(*self.model_args, t0=t)
                self.get_logger().info(f"hold phases: esc {self.model.esc_phase * 1e3:.2f} ms, "
                                       f"servo {self.model.servo_phase * 1e3:.2f} ms")
                for args in self.pending:
                    self.model.command(*args)
                self.pending.clear()
            speed, delta = self.model.advance(t)
            out = AckermannDriveStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.drive.speed, out.drive.steering_angle = float(speed), float(delta)
            self.pub.publish(out)

        def on_episode(self, msg) -> None:
            ep = episode.parse(msg.data)
            if self.model is None:
                self.model = am.ActuatorModel(*self.model_args, t0=self.now())
            self.model.advance(self.now())
            t_start = ep.t_start
            if t_start < self.model.t:
                self.get_logger().error(f"episode t_start {t_start:.3f} already passed ({self.model.t:.3f}): "
                                        "phases placed from now, NOT paired")
                t_start = self.model.t
            self.model.rephase(t_start, ep.seed)
            self.get_logger().info(f"episode: t_start {t_start:.3f}, seed {ep.seed}; hold phases esc "
                                   f"{self.model.esc_phase * 1e3:.2f} ms, servo {self.model.servo_phase * 1e3:.2f} ms")

        def on_joints(self, msg: JointState) -> None:
            v = dict(zip(msg.name, msg.velocity))
            rear = [v.get(j) for j in WHEEL_JOINTS[2:]]
            if all(w is not None for w in rear):
                self.rear_speed = 0.5 * (rear[0] + rear[1]) * self.radius

        def telemetry(self) -> None:
            if self.rear_speed is None:
                return
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = float(self.rear_speed)
            self.pub_speed.publish(msg)

    rclpy.init()
    node = ActuatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
