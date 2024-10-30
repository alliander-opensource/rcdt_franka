# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription, LaunchContext, LaunchDescriptionEntity
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node, SetParameter

from rcdt_utilities_py.launch_utils import (
    get_file_path,
    get_robot_description,
    LaunchArgument,
)

sim_launch_arg = LaunchArgument("simulation", True, [True, False])
moveit_launch_arg = LaunchArgument("moveit", "off", ["node", "rviz", "servo", "off"])
rviz_launch_arg = LaunchArgument("rviz", False, [True, False])


def launch_setup(context: LaunchContext) -> None:
    use_sim = sim_launch_arg.value(context)
    moveit_arg = moveit_launch_arg.value(context)
    use_rviz = rviz_launch_arg.value(context)

    xacro_path = get_file_path(
        "franka_description", ["robots", "fr3"], "fr3.urdf.xacro"
    )
    xacro_arguments = {"ros2_control": "true"}
    if use_sim:
        xacro_arguments["gazebo"] = "true"
    else:
        xacro_arguments["robot_ip"] = "172.16.0.2"
    robot_description = get_robot_description(xacro_path, xacro_arguments)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    if use_sim:
        robot = IncludeLaunchDescription(
            get_file_path("rcdt_utilities", ["launch"], "gazebo_robot.launch.py")
        )
    else:
        robot = IncludeLaunchDescription(
            get_file_path("rcdt_franka", ["launch"], "robot.launch.py")
        )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    controllers = IncludeLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "controllers.launch.py"),
        launch_arguments={
            "simulation": str(use_sim),
            "arm_controller": "fr3_arm_controller",
            "gripper_controller": "fr3_gripper",
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py"),
        launch_arguments={"rviz_frame": "fr3_link0"}.items(),
    )
    load_rviz = (use_rviz or not use_sim) and moveit_arg != "rviz"

    moveit = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "moveit.launch.py"),
        launch_arguments={"moveit_config_package": "rcdt_franka_moveit_config"}.items(),
    )

    joy = Node(
        package="joy",
        executable="game_controller_node",
        parameters=[
            {"sticky_buttons": True},
        ],
    )

    joy_to_twist = Node(
        package="rcdt_utilities",
        executable="joy_to_twist_node.py",
        parameters=[
            {"pub_topic": "/servo_node/delta_twist_cmds"},
            {"config_pkg": "rcdt_franka"},
            {"pub_frame": "fr3_hand"},
        ],
    )

    joy_to_gripper = Node(
        package="rcdt_franka",
        executable="joy_to_gripper_node.py",
    )

    skip = LaunchDescriptionEntity()
    return [
        SetParameter(name="use_sim_time", value=use_sim),
        robot_state_publisher,
        robot,
        joint_state_broadcaster,
        controllers,
        rviz if load_rviz else skip,
        moveit if moveit_arg != "off" else skip,
        joy,
        joy_to_twist if moveit_arg == "servo" else skip,
        joy_to_gripper,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            sim_launch_arg.declaration,
            rviz_launch_arg.declaration,
            moveit_launch_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
