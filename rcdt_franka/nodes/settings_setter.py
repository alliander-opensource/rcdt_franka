#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from franka_msgs.srv import SetForceTorqueCollisionBehavior


class SettingsSetter(Node):
    def __init__(self):
        super().__init__("settings_setter")
        self.set_thresholds()

    def set_thresholds(self) -> None:
        client = self.create_client(
            SetForceTorqueCollisionBehavior,
            "/service_server/set_force_torque_collision_behavior",
        )
        request = SetForceTorqueCollisionBehavior.Request()
        percentage = 1.0
        torques = [100.0, 100.0, 100.0, 80.0, 80.0, 40.0, 40.0]
        forces = [100.0, 100.0, 100.0, 30.0, 30.0, 30.0]
        request.upper_torque_thresholds_nominal = [
            percentage * torque for torque in torques
        ]
        request.upper_force_thresholds_nominal = [
            percentage * force for force in forces
        ]
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        client.call_async(request)


def main(args: str = None) -> None:
    rclpy.init(args=args)

    settings_setter = SettingsSetter()

    rclpy.spin(settings_setter)


if __name__ == "__main__":
    main()
