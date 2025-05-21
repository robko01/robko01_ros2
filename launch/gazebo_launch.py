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

import os
import tempfile

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():

    # Path to the package.
    package_name = 'robko01_ros2'
    package_path = FindPackageShare(package=package_name).find(package_name)

    # Path to the URDF model.
    # urdf_model_path = os.path.join(package_path, 'urdf', 'robko01.urdf')
    robot_model_path = os.path.join(package_path, 'description', 'robko01.urdf.xacro')

    # Process xacro to URDF
    robot_description = xacro.process_file(robot_model_path).toxml()

    # Write to temporary file
    tmp_urdf = tempfile.NamedTemporaryFile(delete=False, suffix=".urdf")
    tmp_urdf.write(robot_description.encode("utf-8"))
    tmp_urdf.close()

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_ros"),
            "/launch/gazebo.launch.py"
        ]),
        launch_arguments={
            "world": os.path.join(package_path, "worlds", "robko01.world")
            }.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "robko01",
            "-file", tmp_urdf.name,
            "-timeout", "120",
            # "-topic", "robot_description",
            # "-x", "0",
            # "-y", "0",
            # "-z", "0.1"
        ],
        output="screen"
    )

    # RViz
    rviz_config_path = os.path.join(package_path, 'rviz', 'default.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_robot,
        rviz
    ])