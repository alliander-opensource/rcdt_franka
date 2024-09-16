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

static_transform_publisher = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="static_tf_world_base",
    arguments=["--frame-id", "world", "--child-frame-id", "base"],
)

gripper_action_server = Node(
    package="rcdt_franka",
    executable="gripper_action_server.py",
    output="screen",
)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            gazebo,
            spawn_robot,
            sync_clock,
            static_transform_publisher,
            gripper_action_server,
        ]
    )
