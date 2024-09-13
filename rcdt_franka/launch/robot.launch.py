# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node

from rcdt_utilities.launch_utils import get_file_path

# Opstarten robot
franka_controllers = param_file = get_file_path(
    "franka_bringup", ["config"], "controllers.yaml"
)

ros2_control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
        franka_controllers,
        {"arm_id": "fr3"},
    ],
    remappings=[("/controller_manager/robot_description", "/robot_description")],
    on_exit=Shutdown(),
)
# /Opstarten robot


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            ros2_control_node,
        ]
    )
