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
    remappings=[
        ("/controller_manager/robot_description", "/robot_description"),
        ("joint_states", "franka/joint_states"),
    ],
    on_exit=Shutdown(),
)

gripper_config = get_file_path("franka_gripper", ["config"], "franka_gripper_node.yaml")
franka_gripper = Node(
    package="franka_gripper",
    executable="franka_gripper_node",
    parameters=[
        {
            "robot_ip": "172.16.0.2",
            "joint_names": ["fr3_finger_joint1", "fr3_finger_joint2"],
        },
        gripper_config,
    ],
)

joint_state_publisher = Node(
    package="joint_state_publisher",
    executable="joint_state_publisher",
    name="joint_state_publisher",
    parameters=[
        {
            "source_list": ["/franka/joint_states", "franka_gripper_node/joint_states"],
            "rate": 30,
        }
    ],
)

# /Opstarten robot


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            ros2_control_node,
            franka_gripper,
            joint_state_publisher,
        ]
    )
