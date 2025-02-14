#!/usr/bin/env python
# -*- coding: utf8 -*-

"""

Robko 01 - ROS2 Control Software

Copyright (C) [2025] [Orlin Dimitrov]

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

import sys
import os
import threading
import queue
import traceback
import argparse

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
# from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory

from robko01.controllers.controller_factory import ControllerFactory
from robko01.utils.thread_timer import ThreadTimer
from robko01.utils.actions import Actions

import serial

class JointStatesListener(Node):

#region Constructor
    def __init__(self):
        """Constructor
        """

        super().__init__('joint_states_listener')

        self.__conversion_table_rad = [1125, 1125, 672, 241, 241, 1]
        """Conversion tables from radians to steps.
        """

        self.__topic = "/joint_states"
        """Subscription topic.
        """

        self.__rate = 1
        """Update rate.
        """        

        # Subscription
        self.__subscription = self.create_subscription(
            JointState,
            self.__topic,
            self._listener_callback,
            self.__rate)

        # prevent unused variable warning
        self.__subscription
#endregion

#region Protected Methods
    def _radians_to_steps(self, radians_list):
        result = []
        for key, value in enumerate(radians_list):
            result.append(int(self.__conversion_table_rad[key]*value))
        return result

    def _listener_callback(self, msg):
        angles = msg.position[0:6]
        steps = self._radians_to_steps(angles)
        self.get_logger().info(f'{steps}')
#endregion

def main(args=None):
    """Main function.
    """

    joint_states_listener = None

    try:
        rclpy.init(args=args)
        joint_states_listener = JointStatesListener()
        rclpy.spin(joint_states_listener)
    except KeyboardInterrupt:
        pass
    finally:
        if joint_states_listener is not None:
            joint_states_listener.destroy_node()
            # rclpy.shutdown()

if __name__ == '__main__':
    main()
