#!/usr/bin/env python3
# -*- coding: utf8 -*-

"""

Robko 01 - ROS2 Python Control Software

Copyright (C) [2024] [Orlin Dimitrov]

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

class Robko01FollowJointTrajectoryClient(Node):

    def __init__(self):

        super().__init__('robko01_follow_joint_trajectory_client')

        self.__logger = self.get_logger()
        self.__logger.info("HOI")

        # Create the action client
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            'robko01_follow_joint_trajectory')

        # Wait for the server to be ready
        self._action_client.wait_for_server()

        # Create and send the goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['base', 'shoulder', 'elbow', 'ld', 'rd', 'gripper']

        # Define trajectory points
        point = JointTrajectoryPoint()
        point.positions = [200.0, 200.0, 200.0, 200.0, 200.0, 200.0]
        point.velocities = [-5.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 2

        goal_msg.trajectory.points.append(point)

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.__logger.info('Goal rejected.')
            return

        self.__logger.info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.__logger.info(f'Result: {result.error_code}')
        rclpy.shutdown()
        # self.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    client = Robko01FollowJointTrajectoryClient()

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        # rclpy.shutdown()

if __name__ == '__main__':
    main()
