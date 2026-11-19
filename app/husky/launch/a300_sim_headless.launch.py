"""Headless A300 simulation: Clearpath's simulation.launch.py with the Gazebo server only (no GUI).

clearpath_gz/gz_sim.launch.py always starts the GUI (gz_args has no -s), which needs a display.
This launch keeps everything else the same: the resource path, the /clock bridge and
clearpath_gz/robot_spawn.launch.py with the same arguments. Use it for SSH sessions and automated
runs. A GUI can be attached to the running server at any time, and closed without stopping it:

    gz sim -g -v 3 --gui-config $(ros2 pkg prefix clearpath_gz)/share/clearpath_gz/config/gui.config

The A300 example configuration has an IMU only, so the server needs no rendering. Add
--headless-rendering to gz_args before adding cameras or lidars.

    ros2 launch <this file> setup_path:=$HOME/clearpath/ world:=warehouse
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

POSE = ("x", "y", "yaw")


def generate_launch_description():
    pkg_clearpath_gz = get_package_share_directory("clearpath_gz")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    packages_paths = [os.path.join(p, "share") for p in os.getenv("AMENT_PREFIX_PATH").split(":")]

    arguments = [
        DeclareLaunchArgument("world", default_value="warehouse"),
        DeclareLaunchArgument("setup_path", default_value=[EnvironmentVariable("HOME"), "/clearpath/"]),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("verbosity", default_value="3"),
        DeclareLaunchArgument("z", default_value="0.3"),
    ] + [DeclareLaunchArgument(p, default_value="0.0") for p in POSE]

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments=[("gz_args", [LaunchConfiguration("world"), ".sdf -r -s -v ",
                                       LaunchConfiguration("verbosity")]),
                          ("on_exit_shutdown", "true")])

    clock_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", name="clock_bridge",
                        output="screen", arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"])

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_clearpath_gz, "launch", "robot_spawn.launch.py")),
        launch_arguments=[(name, LaunchConfiguration(name))
                          for name in ("use_sim_time", "setup_path", "world", "rviz", "z") + POSE])

    return LaunchDescription(arguments + [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH",
                               os.path.join(pkg_clearpath_gz, "worlds") + ":"
                               + os.path.join(pkg_clearpath_gz, "meshes") + "::" + ":".join(packages_paths)),
        gz_server, clock_bridge, robot_spawn])
