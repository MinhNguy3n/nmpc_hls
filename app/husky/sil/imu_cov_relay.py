#!/usr/bin/env python3
"""Republish the simulated IMU with a chosen covariance (SIMULATION ONLY; for the SIL estimator).

Gazebo's IMU reports all covariances as 0; robot_localization raises a variance below 1e-9 to
1e-9 (ekf.cpp, Ekf::correct), so the simulated gyro is trusted far more than a real one. This
relay copies every message (stamp, frame, data unchanged) and sets the diagonal of
angular_velocity_covariance and linear_acceleration_covariance:

    gyro_variance   default 2.75e-6 (rad/s)^2: the phidgets_spatial driver default
                    (0.095 deg/s white-noise sigma "as per manual"), not a calibration of a unit
    accel_variance  default 7.54e-6 (m/s^2)^2: driver default 280 ug sigma
    "native"        for either: keep Gazebo's value (the 1e-9 floor applies in the EKF)

    python3 imu_cov_relay.py --ros-args -r __ns:=/a300_00000 -p out_topic:=sil/imu_driver/data \
        -p gyro_variance:=2.75e-6 -p use_sim_time:=true
"""

from __future__ import annotations

import sys

DRIVER_GYRO_VARIANCE = (0.095 * 3.141592653589793 / 180.0) ** 2  # 2.75e-6
DRIVER_ACCEL_VARIANCE = (280e-6 * 9.80665) ** 2  # 7.54e-6


def apply_covariance(msg, gyro: float | None, accel: float | None):
    """Set the diagonals in place (None keeps the incoming values); returns msg."""
    for cov, value in ((msg.angular_velocity_covariance, gyro), (msg.linear_acceleration_covariance, accel)):
        if value is not None:
            for i in (0, 4, 8):
                cov[i] = value
    return msg


def _value(text: str) -> float | None:
    return None if text == "native" else float(text)


def main() -> int:  # pragma: no cover - needs ROS 2
    from rcl_interfaces.msg import ParameterDescriptor
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    rclpy.init(args=sys.argv)
    node = rclpy.create_node("sil_imu_cov_relay")
    in_topic = node.declare_parameter("in_topic", "sensors/imu_0/data").value
    out_topic = node.declare_parameter("out_topic", "sil/imu/data").value
    either = ParameterDescriptor(dynamic_typing=True)  # a number or "native"
    gyro = _value(str(node.declare_parameter("gyro_variance", DRIVER_GYRO_VARIANCE, either).value))
    accel = _value(str(node.declare_parameter("accel_variance", DRIVER_ACCEL_VARIANCE, either).value))
    pub = node.create_publisher(Imu, out_topic, qos_profile_sensor_data)
    node.create_subscription(Imu, in_topic, lambda m: pub.publish(apply_covariance(m, gyro, accel)),
                             qos_profile_sensor_data)
    node.get_logger().info(f"IMU relay {in_topic} -> {out_topic}: gyro variance {'native' if gyro is None else gyro}, "
                           f"accel variance {'native' if accel is None else accel}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
