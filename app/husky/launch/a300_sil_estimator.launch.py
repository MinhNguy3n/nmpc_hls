"""SIMULATION-ONLY estimator for the A300 SIL (GPT-6 review 2026-11-19, option b).

The generated EKF (`ekf_node`, platform/odom/filtered) fuses the wheel odometry's ABSOLUTE x, y
and yaw, so its pose follows wheel odometry (first SIL run, 2026-11-19: 1.13 m / 1.09 rad drift
per lap). This launch adds separate EKF instances, one per variant, that fuse:

    wheel odometry (platform/odom): v_x only            -- velocity, no pose
    IMU (via sil/imu_cov_relay.py): yaw rate only        -- with a stated gyro variance

The lateral velocity is not fused (the diff-drive odometry reports 0, it is not a measurement)
and stays at its initial 0. The IMU's a_x, which the generated EKF fuses, is left out: it only
feeds the acceleration state of the prediction, v_x is measured at 50 Hz, and it was not
validated here (sensor offset 0.059 m from base_link, vibration).

Each variant publishes /<ns>/sil/<name>/odom/filtered, with publish_tf false; the generated EKF,
its TF and the generated configuration are untouched. Run it next to the simulation and the IMU
bridge launch:

    ros2 launch <this file>                                    # variant driver:2.75e-6
    ros2 launch <this file> variants:=native:native,driver:2.75e-6,s1e-4:1e-4,s1e-2:1e-2

`native` keeps Gazebo's zero covariance (raised to 1e-9 by robot_localization). 2.75e-6
(rad/s)^2 is the phidgets_spatial driver default (0.095 deg/s datasheet sigma), not a
calibration; other values are labelled simulator assumptions.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RELAY = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "sil", "imu_cov_relay.py")
VELOCITY_ONLY = [False] * 6 + [True] + [False] * 8  # v_x
YAW_RATE_ONLY = [False] * 11 + [True] + [False] * 3  # v_yaw


def parse_variants(text: str) -> list[tuple[str, str]]:
    out = []
    for item in text.split(","):
        name, _, variance = item.strip().partition(":")
        if not name.replace("_", "").replace("-", "").isalnum() or not variance:
            raise ValueError(f"variant '{item}': expected name:variance")
        out.append((name.replace("-", "_"), variance))
    return out


def _setup(context, *args, **kwargs):
    ns = LaunchConfiguration("namespace").perform(context)
    sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    actions = []
    for name, variance in parse_variants(LaunchConfiguration("variants").perform(context)):
        imu_topic = f"sil/{name}/imu"
        actions.append(ExecuteProcess(
            cmd=["python3", RELAY, "--ros-args", "-r", f"__ns:=/{ns}", "-r", f"__node:=sil_imu_relay_{name}",
                 "-p", f"out_topic:={imu_topic}", "-p", f"gyro_variance:={variance}",
                 "-p", f"use_sim_time:={str(sim_time).lower()}"],
            output="screen"))
        actions.append(Node(
            package="robot_localization", executable="ekf_node", name=f"sil_ekf_{name}", namespace=ns,
            output="screen",
            parameters=[{"use_sim_time": sim_time, "frequency": 50.0, "two_d_mode": True, "publish_tf": False,
                         "print_diagnostics": True, "odom_frame": "odom", "base_link_frame": "base_link",
                         "world_frame": "odom",
                         "odom0": "platform/odom", "odom0_config": VELOCITY_ONLY, "odom0_differential": False,
                         "odom0_queue_size": 10,
                         "imu0": imu_topic, "imu0_config": YAW_RATE_ONLY, "imu0_differential": False,
                         "imu0_queue_size": 10}],
            # Clearpath namespaces TF (/<ns>/tf); without base_link -> imu_0_link the EKF drops every IMU message
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static"), ("odometry/filtered", f"sil/{name}/odom/filtered"),
                        ("set_pose", f"sil/{name}/set_pose"), ("enable", f"sil/{name}/enable"),
                        ("toggle", f"sil/{name}/toggle"), ("/diagnostics", f"/{ns}/sil/{name}/diagnostics")]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="a300_00000"),
        DeclareLaunchArgument("variants", default_value="driver:2.75e-6"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
