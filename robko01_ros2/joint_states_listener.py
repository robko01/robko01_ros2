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

        super().__init__('robko01_joint_states_listener')

        self.__logger = self.get_logger()
        self.__logger.info("HOI -> Human Oral Interaction")

        # self.__conversion_table_rad = [1125, 1125, 672, 241, 241, 1] # Original
        self.__conversion_table_rad = [544, 544, 325, 140, 140, 150] # Compensated because of the motors controllers settings.
        """Conversion tables from radians to steps.
        """

        self.__controller = None
        """Controller instance.
        """        

        self.__set_position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        """Set position.
        """        

        self.__current_position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        """Current position.
        """

        self.__current_speed = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        """Axis speeds.
        """

        self.__axis_states = 0
        """Axis action states.
        """

        self.__port_a_inputs = 0
        """Port A inputs.
        """

        self.__port_a_outputs = 0
        """Port A outputs.
        """

        self.__robot_ready = False
        """Robot ready flag.
        """

        self.__topic = "/joint_states"
        """Subscription topic.
        """

        self.__rate = 1
        """Update rate.
        """        

        self.__subscription = None
        """Subscription instance.
        """        

        self.__speed = 70
        """Default constant speed.
        """        

        self.__angles = None
        """Angles of the robot.
        """

        self.__actions_queue = queue.Queue()
        """Actions queue.
        """

        self.__action_update_timer = ThreadTimer()
        """Action update timer.
        """

    def __del__(self):

        self.destroy_node()

#endregion

#region Private Methods (Controller)

    def __init_controller(self):
        # Declare parameters
        self.declare_parameter('host', 'localhost')  # Default value is 'localhost'
        self.declare_parameter('port', 8000)        # Default value is 8000
        self.declare_parameter('cname', "orlin369")        # Default value is orlin369

        # Get parameter values
        host = self.get_parameter('host').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        cname = self.get_parameter('cname').get_parameter_value().string_value

        # Manual convert to string.
        port = str(port)

        # Create the robot controller.
        self.__controller = ControllerFactory.create(host=host, port=port, cname=cname)
        self.__controller.connect()
        self.__controller.enable()

#endregion

#region Private Methods (Robot Action Handler)

    def __put_action(self, action):

        self.__actions_queue.put(action)

    def __do_action(self, action):

        if self.__controller is None:
            return

        if action == Actions.NONE:
            pass

        if action == Actions.UpdateAbsolutePositions:
            self.__controller.move_absolute(self.__set_position)
            self.__logger.info(f'{self.__set_position}')


        elif action == Actions.UpdateOutputs:
            self.__controller.set_outputs(self.__port_a_outputs)

        elif action == Actions.ClearController:
            self.__controller.clear()

        elif action == Actions.ResetController:
            pass

    def __action_timer_cb(self):

        try:
            self.__axis_states = self.__controller.is_moving()
            self.__current_position = self.__controller.current_position()
            self.__port_a_inputs = self.__controller.get_inputs()

            if not self.__actions_queue.empty():
                action = self.__actions_queue.get()
                self.__do_action(action)

            self.__robot_ready = True

        except serial.serialutil.SerialException as exc:
            self.__robot_ready = False
            self.__logger.error(exc)

        except Exception as exc:
            self.__robot_ready = False
            self.__logger.error(traceback.format_exc())

    def __init_action_timer(self):
        self.__action_update_timer.update_rate = 1
        self.__action_update_timer.set_cb(self.__action_timer_cb)
        self.__action_update_timer.start()

#endregion

#region Private Methods (Listener)

    def __radians_to_steps(self, radians_list):
        result = []
        for key, value in enumerate(radians_list):
            result.append(int(self.__conversion_table_rad[key]*value))
        return result

    def __listener_callback(self, msg):

        angles = msg.position[0:6]
        if self.__angles != angles:
            self.__angles = angles

            # Elbow compensation.
            angles[2] = angles[2] + angles[1]

            # P compensation.
            angles[3] = angles[3] + angles[2]

            # Differentials inverse model.
            q4 = angles[4] + angles[3]
            q5 = angles[4] - angles[3]
            angles[3] = q4
            angles[4] = q5

            # Convert to steps.
            steps = self.__radians_to_steps(angles)

            # Gripper compensation.
            # In steps is essayer because
            # ration between elbow and gripper is 1:1.
            steps[5] = steps[5] - steps[2]

            # Apply the position.
            self.__set_position[0:12:2] = steps
            self.__set_position[1:12:2] = [self.__speed]*6

            # Go to position.
            self.__put_action(Actions.UpdateAbsolutePositions)

    def __init_joint_listener(self):
        # Subscription
        self.__subscription = self.create_subscription(
            JointState,
            self.__topic,
            self.__listener_callback,
            self.__rate)

        # prevent unused variable warning
        self.__subscription

#endregion

#region Public Methods

    def init(self):
        self.__init_controller()
        self.__init_action_timer()
        self.__init_joint_listener()

    def destroy_node(self):

        if self.__action_update_timer is not None:
            self.__action_update_timer.stop()

        # Release the robot resource.
        if self.__controller is not None:
            self.__controller.disconnect()

        # Call the base class method to perform the default destruction process
        super().destroy_node()
#endregion

def main(args=None):
    """Main function.
    """

    joint_states_listener = None

    try:
        rclpy.init(args=args)
        joint_states_listener = JointStatesListener()
        joint_states_listener.init()
        rclpy.spin(joint_states_listener)
    except KeyboardInterrupt:
        pass
    finally:
        if joint_states_listener is not None:
            joint_states_listener.destroy_node()
            # rclpy.shutdown()

if __name__ == '__main__':
    main()
