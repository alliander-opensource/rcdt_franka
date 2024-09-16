# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node, SetParameter
from moveit_configs_utils import MoveItConfigsBuilder

from rcdt_utilities.launch_utils import (
    get_file_path,
    get_yaml,
    get_robot_description,
    LaunchArguments,
)

ARGS = LaunchArguments()
ARGS.add_value("simulation", "true", ["true", "false"])
ARGS.add_value("rviz", "true", ["true", "false"])
ARGS.add_value("moveit", "off", ["classic", "servo", "off"])
ARGS.add_value("gamepad", "virtual", ["xbox", "virtual"])
ARGS.update_from_sys()

xacro_path = get_file_path("franka_description", ["robots", "fr3"], "fr3.urdf.xacro")
xacro_arguments = {"ros2_control": "true"}
if ARGS.get_value("simulation") == "true":
    xacro_arguments["gazebo"] = "true"
else:
    xacro_arguments["robot_ip"] = "172.16.0.2"
robot_description = get_robot_description(xacro_path, xacro_arguments)

controllers_config = get_file_path("rcdt_franka", ["config"], "ros_controller.yaml")
moveit_config = MoveItConfigsBuilder("fr3", package_name="rcdt_franka_moveit_config")
servo_config = get_yaml(get_file_path("rcdt_franka", ["config"], "servo_params.yaml"))
# /configs

# robot_description to topic
robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[robot_description],
)
# /robot_description to topic

# start robot
if ARGS.get_value("simulation") == "true":
    robot = IncludeLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "simulation.launch.py")
    )
else:
    robot = IncludeLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "robot.launch.py")
    )
# /start robot

# joint_state_broadcaster
joint_state_broadcaster = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster"],
)
# /joint_state_broadcaster

# rviz
display_config = get_file_path("rcdt_franka", ["rviz"], "general.rviz")
if ARGS.get_value("moveit") == "classic":
    display_config = get_file_path("rcdt_franka", ["rviz"], "moveit.rviz")
rviz = Node(
    package="rviz2",
    executable="rviz2",
    arguments=["--display-config", display_config, "-f" "fr3_link0"],
    parameters=[moveit_config.to_dict()],
)
# /rviz

# fr3_arm_controller
fr3_arm_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["fr3_arm_controller", "-p", controllers_config],
)
# /fr3_arm_controller

# gripper_controller
gripper_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["gripper_action_controller", "-p", controllers_config],
)
# /gripper_controller

# moveit
moveit = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    parameters=[moveit_config.to_dict()],
)
# /moveit

# moveit_servo
servo_params = {"moveit_servo": servo_config}
moveit_servo = Node(
    package="moveit_servo",
    executable="servo_node_main",
    parameters=[servo_params, moveit_config.to_dict()],
)
# /moveit_servo

# gamepad
gamepad = Node(package="rcdt_utilities", executable="gamepad_node.py")
# /gamepad


def generate_launch_description() -> LaunchDescription:
    skip = LaunchDescriptionEntity()
    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True)
            if ARGS.get_value("simulation") == "true"
            else skip,
            robot_state_publisher,
            robot,
            joint_state_broadcaster,
            fr3_arm_controller,
            gripper_controller,
            rviz if ARGS.get_value("rviz") == "true" else skip,
            moveit if ARGS.get_value("moveit") == "classic" else skip,
            moveit_servo if ARGS.get_value("moveit") == "servo" else skip,
            gamepad if ARGS.get_value("moveit") == "servo" else skip,
        ]
    )
