"""Bridge the A300 IMU from Gazebo to ROS 2 (simulation workaround, version-controlled).

Launch IN ADDITION to `ros2 launch clearpath_gz simulation.launch.py ...`, in a second shell:

    ros2 launch ~/nmpc_hls/.claude/worktrees/agent-a7f53c7be12d59b73/app/husky/launch/a300_sim_imu_bridge.launch.py

It first runs the generator check (imu_checks.py check-generated); unless `check:=false`, the
bridge is started only when the generated imu_0.yaml is empty (the observed generator defect), so
that a fixed generator never gets a duplicate publisher. Works from a read-only source tree.

Arguments: setup_path (default ~/clearpath), check (true|false), use_sim_time (default true).
"""

import os
import subprocess
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "a300_imu_bridge.yaml")
CHECK = os.path.join(os.path.dirname(HERE), "imu_checks.py")


def _setup(context, *args, **kwargs):
    setup_path = os.path.expanduser(LaunchConfiguration("setup_path").perform(context))
    sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    actions = []
    if LaunchConfiguration("check").perform(context).lower() == "true":
        result = subprocess.run([sys.executable, CHECK, "check-generated", "--setup-path", setup_path],
                                capture_output=True, text=True)
        actions.append(LogInfo(msg=result.stdout.strip() or result.stderr.strip()))
        if result.returncode != 1:  # 1 = generated bridge list empty -> the workaround is needed
            actions.append(LogInfo(msg="a300_sim_imu_bridge: NOT starting the extra IMU bridge "
                                       f"(check-generated exit code {result.returncode})"))
            return actions
    actions.append(Node(package="ros_gz_bridge", executable="parameter_bridge", name="a300_imu_0_bridge_workaround",
                        output="screen", parameters=[{"config_file": CONFIG, "use_sim_time": sim_time}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("setup_path", default_value="~/clearpath"),
        DeclareLaunchArgument("check", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
