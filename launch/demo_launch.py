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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Path to the package.
    package_name = 'robko01_ros2'
    package_path = FindPackageShare(package=package_name).find(package_name)

    # Path to the URDF model.
    urdf_model_path = os.path.join(package_path, 'urdf', 'robko01.urdf')

    # Path to the RViz configuration.
    rviz_config_path = os.path.join(package_path, 'rviz', 'default.rviz')

    # Load the URDF model.
    with open(urdf_model_path, 'r') as urdf_file:
        urdf_model_content = urdf_file.read()

    # Parameters for nodes
    params = {'robot_description': urdf_model_content}

    # Nodes

    joint_state_publisher_gui_node = TimerAction(
        period=0.0,  # Delay to ensure robot_description is published
        actions=[
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                parameters=[urdf_model_path],
                condition=IfCondition(LaunchConfiguration('gui'))
            )
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[urdf_model_path],
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    joint_states_listener_node = Node(
        package=package_name,
        executable='joint_states_listener',  # No .py extension
        name='joint_states_listener',
        arguments=['--ros-args', '--param', 'host:=1.robko01.loc', '--param', 'port:=10182'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='True',
                              description='Flag to enable joint_state_publisher_gui'),
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        joint_states_listener_node
    ])
