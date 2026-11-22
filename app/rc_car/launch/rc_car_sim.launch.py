"""Reely NEW1 simulation: Gazebo (lab floor) + the car via gz_ros2_control + the plant interface.

Starts, in the container (GZ_IP=127.0.0.1, ROS_DOMAIN_ID as in the Husky setup):
  - gz sim with worlds/lab_floor.sdf (server only by default; gui:=true for the GUI);
  - the /clock bridge;
  - robot_state_publisher with the xacro (parameters from params/reely_new1.yaml), which is where
    gz_ros2_control reads the robot description;
  - the spawn of model `reely_new1` (rear-axle midpoint at x, y, yaw);
  - controller spawners: joint_state_broadcaster, steering_position_controller,
    wheel_velocity_controller (config/controllers.yaml);
  - plant_interface_node.py (input plant/ackermann_cmd).

    ros2 launch app/rc_car/launch/rc_car_sim.launch.py            # headless
    ros2 launch app/rc_car/launch/rc_car_sim.launch.py gui:=true

C3 scrub diagnostic only (defaults give the normal 4WD model):
    ... drive:=rwd controllers_file:=<app/rc_car/config/controllers_rwd.yaml>   # rear drive only
    ... contact_width:=0.01                                                      # narrow contacts
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetLaunchConfiguration)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # app/rc_car
XACRO = os.path.join(HERE, "description", "reely_new1.urdf.xacro")
PARAMS = os.path.join(HERE, "params", "reely_new1.yaml")
CONTROLLERS = os.path.join(HERE, "config", "controllers.yaml")
WORLD = os.path.join(HERE, "worlds", "lab_floor.sdf")
PLANT = os.path.join(HERE, "plant_interface_node.py")


def generate_launch_description():
    gz_launch = os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    sim_time = {"use_sim_time": True}
    arguments = [
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=PARAMS),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("verbosity", default_value="3"),
        DeclareLaunchArgument("world", default_value=WORLD, description="world SDF (figures: a screenshot variant)"),
        DeclareLaunchArgument("gz_extra", default_value="", description="extra gz sim arguments, e.g. --headless-rendering"),
        DeclareLaunchArgument("rtf", default_value="1.0",
                              description="real-time factor, written into a copy of the world (NOT set at run time: "
                                          "a runtime set_physics call broke the plant, 2026-11-22)"),
        DeclareLaunchArgument("controllers_file", default_value=CONTROLLERS),
        DeclareLaunchArgument("drive", default_value="4wd", description="4wd | rwd (diagnostic)"),
        DeclareLaunchArgument("contact_width", default_value="", description="wheel collision width, m (diagnostic)"),
    ]
    params_file = LaunchConfiguration("params_file")

    def world_with_rtf(context):
        rtf = float(LaunchConfiguration("rtf").perform(context))
        world = LaunchConfiguration("world").perform(context)
        if rtf == 1.0:
            return []
        import re
        text = open(world).read()
        text, n = re.subn(r"<real_time_factor>[^<]*</real_time_factor>", f"<real_time_factor>{rtf}</real_time_factor>", text)
        if n != 1:
            raise RuntimeError(f"{world}: expected one <real_time_factor>, found {n}")
        out = f"/tmp/rc_car_world_rtf{rtf:g}.sdf"
        open(out, "w").write(text)
        return [SetLaunchConfiguration("world", out)]

    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch), condition=UnlessCondition(LaunchConfiguration("gui")),
        launch_arguments=[("gz_args", [LaunchConfiguration("world"), " -r -s -v ", LaunchConfiguration("verbosity"),
                                       " ", LaunchConfiguration("gz_extra")]),
                          ("on_exit_shutdown", "true")])
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch), condition=IfCondition(LaunchConfiguration("gui")),
        launch_arguments=[("gz_args", [LaunchConfiguration("world"), " -r -v ", LaunchConfiguration("verbosity")]),
                          ("on_exit_shutdown", "true")])

    clock_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", name="clock_bridge",
                        output="screen", arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"])

    description = ParameterValue(
        Command(["xacro ", XACRO, " params_file:=", params_file,
                 " controllers_file:=", LaunchConfiguration("controllers_file"),
                 " drive:=", LaunchConfiguration("drive"),
                 " contact_width:='", LaunchConfiguration("contact_width"), "'"]),
        value_type=str)
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
               parameters=[{"robot_description": description}, sim_time])

    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-world", "lab_floor", "-name", "reely_new1", "-topic", "robot_description",
                            "-x", LaunchConfiguration("x"), "-y", LaunchConfiguration("y"),
                            "-z", "0.07", "-Y", LaunchConfiguration("yaw")])

    spawners = Node(package="controller_manager", executable="spawner", output="screen",
                    arguments=["joint_state_broadcaster", "steering_position_controller",
                               "wheel_velocity_controller", "--controller-manager", "/controller_manager",
                               "--controller-manager-timeout", "60"],
                    parameters=[sim_time])

    plant = ExecuteProcess(cmd=["python3", PLANT, "--ros-args", "-p", "use_sim_time:=true",
                                "-p", ["params_file:=", params_file],
                                "-p", ["driven:=", LaunchConfiguration("drive")]], output="screen")

    after_spawn = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[spawners]))
    after_controllers = RegisterEventHandler(OnProcessExit(target_action=spawners, on_exit=[plant]))

    return LaunchDescription(arguments + [OpaqueFunction(function=world_with_rtf), gz_headless, gz_gui, clock_bridge, rsp, spawn,
                                          after_spawn, after_controllers])
