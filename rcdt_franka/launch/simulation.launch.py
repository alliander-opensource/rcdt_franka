# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

from rcdt_utilities.launch_utils import get_file_path

gazebo = IncludeLaunchDescription(
    get_file_path("ros_gz_sim", ["launch"], "gz_sim.launch.py"),
    launch_arguments={
        "gz_args": "empty.sdf -r",
    }.items(),
)

spawn_robot = Node(
    package="ros_gz_sim",
    executable="create",
    arguments=["-topic", "/robot_description"],
    output="screen",
)

sync_clock = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    name="sync_clock",
    arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
)

fr3_gripper = Node(
    package="rcdt_franka",
    executable="simulated_gripper_node.py",
    output="screen",
)

controllers_config = get_file_path(
    "rcdt_franka", ["config"], "simulation_controllers.yaml"
)
gripper_action_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["gripper_action_controller", "-p", controllers_config],
)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            gazebo,
            spawn_robot,
            sync_clock,
            fr3_gripper,
            gripper_action_controller,
        ]
    )
