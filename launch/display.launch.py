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

import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():

    # Path to the package.
    package_path = launch_ros.substitutions.FindPackageShare(package='robko01_ros2').find('robko01_ros2')

    # Path to the URDF model.
    urdf_model_path       = os.path.join(package_path, 'urdf/robko01.urdf')

    # Path to the RViz configuration.
    rviz_config_path = os.path.join(package_path, 'rviz', 'default.rviz')

    # Load the URFT model.
    urdf_model_content = b""
    with open(urdf_model_path,'r') as urdf_model_file:
        urdf_model_content = urdf_model_file.read()

    # Parameters (robot model)
    params = {
        'robot_description': urdf_model_content
    }

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    #
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params]
        # condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    #
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[params]
        # condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    #
    robot_state_publisher_node =launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params])

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='This is a flag for joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=urdf_model_path,
                                            description='Path to the urdf model file'),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        robot_state_publisher_node
    ])