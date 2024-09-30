# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

from rcdt_utilities.launch_utils import get_file_path

gazebo_robot = IncludeLaunchDescription(
    get_file_path("rcdt_utilities", ["launch"], "gazebo_robot.launch.py")
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
            gazebo_robot,
            fr3_gripper,
            gripper_action_controller,
        ]
    )
