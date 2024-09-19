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

class Robko01Service(Node):
#0889487321 - Manol

#region Attributes

    __current_position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    """Current end-effector position.
    """

    __current_speed = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    """Axis speeds.
    """

    __axis_states = 0
    """Axis action states.
    """

    __port_a_inputs = 0
    """Port A inputs.
    """

    __port_a_outputs = 0
    """Port A outputs.
    """

    __robot_ready = False
    """Robot ready flag.
    """

    __logger = None
    """Logger.
    """

    __joint_state_msg = None
    """Joints staes.
    """

#endregion

#region Constructor

    def __init__(self, **kwargs):

        super().__init__('robko01_ros2')

        self.__logger = self.get_logger()
        self.__logger.info("HOI")

        # Create the robot controller.
        port = "/dev/ttyUSB0"
        cname = "orlin369"
        self.__controller = ControllerFactory.create(port=port, cname=cname)
        """Controller
        """

        # Create the action server.
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'robko01_follow_joint_trajectory',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Create action queue.
        self.__actions_queue = queue.Queue()
        """Actions queue.
        """

        # Create action update timer.
        self.__action_update_timer = ThreadTimer()
        """Action update timer.
        """
        self.__action_update_timer.update_rate = 0.5
        self.__action_update_timer.set_cb(self.__action_timer_cb)
        self.__action_update_timer.start()

        # Create joint state message.
        self.__joint_state_msg = JointState()
        """Joint messages.
        """

        # Create publisher for joint state message.
        self.__publisher = self.create_publisher(
            JointState,
            'robko01_joint_states',
            10)
        """Publisher
        """

        # Create publisher timer.
        self.__publisher_timer = ThreadTimer()
        """Publisher timer.
        """
        self.__publisher_timer.update_rate = 0.1
        self.__publisher_timer.set_cb(self.__pub_worker)
        self.__publisher_timer.start()

    def __del__(self):

        if self.__publisher_timer is not None:
            self.__publisher_timer.stop()

        if self.__action_update_timer is not None:
            self.__action_update_timer.stop()

        self.__logger.info("Double HOI")

#endregion

#region Private Methods (Service Handler)

    def goal_callback(self, goal_request):
        # Accept all goals for now
        self.__logger.info('Received request to follow a trajectory.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Allow all cancellations
        self.__logger.info('Received request to cancel trajectory.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        feedback_msg = FollowJointTrajectory.Feedback()

        for point in goal_handle.request.trajectory.points:

            # TODO: Scale values to some unit in angle speed.
            # Set velocities and ask controller to made it.
            int_velocities = [int(x) for x in point.velocities]
            self.__current_speed[1:12:2] = int_velocities
            self.__put_action(Actions.UpdateSpeeds)

            # Here we just log the points and assume execution happens instantly.
            feedback_msg.actual.positions = point.positions
            feedback_msg.actual.velocities = point.velocities
            goal_handle.publish_feedback(feedback_msg)

            # Simulate some time delay (in a real robot, you'd move joints here)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=point.time_from_start.sec))

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

#endregion

#region Private Methods (Robot Action Handler)

    def __put_action(self, action):

        self.__actions_queue.put(action)

    def __do_action(self, action):

        if self.__controller is None:
            return

        if action == Actions.NONE:
            pass

        if action == Actions.UpdateSpeeds:
            self.__controller.enable()
            self.__controller.move_speed(self.__current_speed)

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

#endregion

#region Private Methods (State Pulisher)

    def __pub_worker(self):
        # Free-wheeling process: runs in a separate thread
        while rclpy.ok():
            # Perform some task periodically (e.g., every 1 second)

            if self.__logger is not None and self.__robot_ready == True:
                self.__logger.info(f"Current position: {self.__current_position}")
                self.__logger.info(f"Axis states: {self.__axis_states}")
                self.__logger.info(f"Port A inputs: {self.__port_a_inputs}")

                # Update time.
                self.__joint_state_msg.header.stamp = self.get_clock().now().to_msg()

                # Joint names
                self.__joint_state_msg.name = ['base', 'shoulder', 'elbow', 'ld', 'rd', 'gripper']

                # Joint positions - simple oscillation using sine function
                self.__joint_state_msg.position = self.__current_position[0::2]

                # Optional: Joint velocity and effort can also be specified
                self.__joint_state_msg.velocity = self.__current_position[1::2]


                self.__publisher.publish(self.__joint_state_msg)

            # Sleep to simulate work and synchronize with ROS2 spin time
            rclpy.spin_once(self, timeout_sec=1.0)  # Sync with ROS2 spin time

#endregion

def main(args=None):
    rclpy.init(args=args)

    robko01_service = Robko01Service()

    try:
        rclpy.spin(robko01_service)
    except KeyboardInterrupt:
        pass
    finally:
        robko01_service.destroy_node()
        # rclpy.shutdown()

if __name__ == '__main__':
    main()
