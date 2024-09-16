#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from franka_msgs.action import Grasp, Homing, Move
from control_msgs.action import GripperCommand

MAX = 0.039
MIN = 0.001


class GripperActionServer(Node):
    def __init__(self):
        super().__init__("fr3_gripper")
        self.action_client = ActionClient(
            self, GripperCommand, "/gripper_action_controller/gripper_cmd"
        )
        ActionServer(self, Grasp, "~/grasp", self.grasp_action)
        ActionServer(self, Homing, "~/homing", self.homing_action)
        ActionServer(self, Move, "~/move", self.move_action)

    def grasp_action(self, goal_handle: ServerGoalHandle) -> Grasp.Result:
        request: Grasp.Goal = goal_handle.request
        result = Grasp.Result()
        if self.move(0, request.force):
            goal_handle.succeed()
        return result

    def homing_action(self, goal_handle: ServerGoalHandle) -> Homing.Result:
        result = Homing.Result()
        if self.move(MAX):
            goal_handle.succeed()
        return result

    def move_action(self, goal_handle: ServerGoalHandle) -> Move.Result:
        request: Move.Goal = goal_handle.request
        result = Move.Result()
        if self.move(request.width):
            goal_handle.succeed()
        return result

    def move(self, width: float, max_effort: float = 0.0) -> bool:
        goal = GripperCommand.Goal()
        goal.command.position = self.respect_limits(width)
        goal.command.max_effort = max_effort
        self.action_client.wait_for_server()
        return self.action_client.send_goal_async(goal)

    def respect_limits(self, width: float) -> float:
        return min(MAX, max(MIN, width))


def main(args: str = None) -> None:
    rclpy.init(args=args)

    gripper_action_server = GripperActionServer()

    rclpy.spin(gripper_action_server)


if __name__ == "__main__":
    main()