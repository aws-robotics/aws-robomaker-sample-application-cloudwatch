#!/usr/bin/env python
# Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.

# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED,INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name='x_pos', default_value='-3.5'),
            launch.actions.DeclareLaunchArgument(
                name='y_pos', default_value='5.5'),
            launch.actions.DeclareLaunchArgument(
                name='z_pos', default_value='0.30'),
            launch.actions.DeclareLaunchArgument(
                name='gui', default_value='false'),
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory(
                            'aws_robomaker_bookstore_world'),
                        'launch',
                        'bookstore.launch.py',
                    )
                ),
                launch_arguments={
                    'gui': launch.substitutions.LaunchConfiguration('gui')
                }.items(),
            ),
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory(
                            'turtlebot3_description_reduced_mesh'
                        ),
                        'launch',
                        'spawn_turtlebot.launch.py',
                    )
                ),
                launch_arguments={
                    'x_pos': launch.substitutions.LaunchConfiguration('x_pos'),
                    'y_pos': launch.substitutions.LaunchConfiguration('y_pos'),
                    'z_pos': launch.substitutions.LaunchConfiguration('z_pos'),
                }.items(),
            ),
        ]
    )
    return ld


if __name__ == '__main__':
    generate_launch_description()
