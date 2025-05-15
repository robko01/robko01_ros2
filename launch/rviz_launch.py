#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():

    # Path to the package.
    package_name = "robko01_ros2"
    package_path = FindPackageShare(package=package_name).find(package_name)

    # Path to the URDF model.
    # urdf_model_path = os.path.join(package_path, "urdf", "robko01.urdf")
    robot_model_path = os.path.join(package_path, "description", "robko01.urdf.xacro")

    # Process xacro to URDF
    robot_description = xacro.process_file(robot_model_path).toxml()

    # Write to temporary file
    # tmp_urdf = tempfile.NamedTemporaryFile(delete=False, suffix=".urdf")
    # tmp_urdf.write(robot_description.encode("utf-8"))
    # tmp_urdf.close()

    # Load the URDF model.
    # urdf_model_content = None
    # with open(tmp_urdf, "r") as urdf_file:
    #     urdf_model_content = urdf_file.read()

    # Joint state publisher GUI
    joint_state_publisher_gui = TimerAction(
        period=0.0,  # Delay to ensure robot_description is published
        actions=[
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[robot_model_path],
                condition=IfCondition(LaunchConfiguration("gui"))
            )
        ]
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[robot_model_path],
        condition=UnlessCondition(LaunchConfiguration("gui"))
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description
            }]
    )

    # RViz
    rviz_config_path = os.path.join(package_path, "rviz", "default.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name="gui",
            default_value="True",
            description="Flag to enable joint_state_publisher_gui"),
        joint_state_publisher_gui,
        joint_state_publisher,
        robot_state_publisher,
        rviz
    ])
