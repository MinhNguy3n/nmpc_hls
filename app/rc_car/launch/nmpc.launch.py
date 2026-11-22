"""NMPC arm: nmpc_node.py (emulated engine) -> controller/ackermann_cmd -> mux -> actuator -> plant.

Run next to rc_car_stack.launch.py (same ROS domain). Episodes are driven by checks/track_run.py
--arm nmpc. SIMULATION ONLY.

    ros2 launch app/rc_car/launch/nmpc.launch.py size:=10x40 profile:=realistic_guess compensation:=true arith:=half
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # app/rc_car


def generate_launch_description():
    names = ("size", "profile", "compensation", "arith", "variant")
    defaults = ("10x40", "realistic_guess", "true", "half", "phase4")
    args = [DeclareLaunchArgument(n, default_value=d) for n, d in zip(names, defaults)]
    cmd = ["python3", os.path.join(HERE, "nmpc_node.py"), "--ros-args", "-p", "use_sim_time:=true"]
    for n in names:
        cmd += ["-p", [n, ":=", LaunchConfiguration(n)]]
    return LaunchDescription(args + [ExecuteProcess(cmd=cmd, output="screen")])
