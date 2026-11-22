"""Baseline arm: Nav2 RPP (minimal controller_server, empty costmap) -> own Twist->Ackermann converter.

    controller_server (FollowPath action, RPP) --/baseline/cmd_vel (TwistStamped)--> twist_to_ackermann
        --> controller/ackermann_cmd --> command_mux --> actuator --> plant
    /mocap/<body>/pose --> mocap_bridge (TF map->base_link at the capture stamp, odom with VESC speed)

Run next to rc_car_stack.launch.py (same ROS domain). Paths are sent by checks/track_run.py.

    ros2 launch app/rc_car/launch/baseline_rpp.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # app/rc_car
CONFIG = os.path.join(HERE, "config", "baseline_rpp.yaml")


def script(name, *extra):
    return ExecuteProcess(cmd=["python3", os.path.join(HERE, name), "--ros-args", "-p", "use_sim_time:=true", *extra],
                          output="screen")


def generate_launch_description():
    return LaunchDescription([
        script("mocap_bridge_node.py"),
        script("twist_to_ackermann_node.py", "-r", "cmd_vel_in:=/baseline/cmd_vel"),
        Node(package="nav2_controller", executable="controller_server", output="screen",
             parameters=[CONFIG], remappings=[("cmd_vel", "/baseline/cmd_vel")]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_baseline",
             output="screen", parameters=[{"use_sim_time": True, "autostart": True,
                                           "node_names": ["controller_server"], "bond_timeout": 0.0}]),
    ])
