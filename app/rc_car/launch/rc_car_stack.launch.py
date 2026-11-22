"""Reely NEW1 simulation with the phase 2 command chain:

    teleop/ackermann_cmd ─┐
    controller/ackermann_cmd ─> command_mux ─> actuator/ackermann_cmd ─> ackermann_actuator
    mux/lock, mux/arm ────┘        ^                                          │
                                   │ /mocap/<body>/pose (freshness gate)       v
    Gazebo truth ─> mocap_emulator ┘                       plant/ackermann_cmd ─> plant interface

Includes rc_car_sim.launch.py (Gazebo, the car, the plant interface) and starts the mux, the
actuator emulation and the OptiTrack emulator. ALL CHAIN VALUES ARE ASSUMED
(params/command_chain.yaml).

    ros2 launch app/rc_car/launch/rc_car_stack.launch.py profile:=realistic_guess mocap_profile:=realistic_guess seed:=1
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # app/rc_car


def node(script, *params):
    cmd = ["python3", os.path.join(HERE, script), "--ros-args", "-p", "use_sim_time:=true"]
    for key, value in params:
        cmd += ["-p", [key, ":=", value]]
    return ExecuteProcess(cmd=cmd, output="screen")


def generate_launch_description():
    args = [
        DeclareLaunchArgument("profile", default_value="realistic_guess", description="actuator profile"),
        DeclareLaunchArgument("mocap_profile", default_value="realistic_guess"),
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("actuator_period", default_value="0.002", description="actuator node tick, s"),
        DeclareLaunchArgument("world", default_value=os.path.join(HERE, "worlds", "lab_floor.sdf")),
        DeclareLaunchArgument("gz_extra", default_value=""),
        DeclareLaunchArgument("rtf", default_value="1.0", description="real-time factor (in the world file)"),
        DeclareLaunchArgument("contact_width", default_value="", description="wheel collision width, m (sensitivity point)"),
    ]
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(HERE, "launch", "rc_car_sim.launch.py")),
        launch_arguments=[("gui", LaunchConfiguration("gui")), ("world", LaunchConfiguration("world")),
                          ("gz_extra", LaunchConfiguration("gz_extra")), ("rtf", LaunchConfiguration("rtf")),
                          ("contact_width", LaunchConfiguration("contact_width"))])
    seed = LaunchConfiguration("seed")
    return LaunchDescription(args + [
        sim,
        node("command_mux_node.py"),
        node("actuator_node.py", ("profile", LaunchConfiguration("profile")), ("seed", seed),
             ("period", LaunchConfiguration("actuator_period"))),
        node("mocap_emulator_node.py", ("profile", LaunchConfiguration("mocap_profile")), ("seed", seed)),
    ])
