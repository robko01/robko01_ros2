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

from setuptools import find_packages, setup

package_name = 'robko01_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='orlin369@gmail.com',
    description='ROS2 python package for supproting Robko01',
    license='GPL License',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = robko01_ros2.service:main',
            'client = robko01_ros2.client:main',
        ],
    },
    # Use the new-style development mode
    # cmdclass={
    #     'develop': 'setuptools.command.develop.develop',
    #     'install': 'setuptools.command.install.install',
    # },
)
