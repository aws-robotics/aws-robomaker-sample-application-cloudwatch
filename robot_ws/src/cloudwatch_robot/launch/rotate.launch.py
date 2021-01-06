#!/usr/bin/env python
"""
Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software
is furnished to do so.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED,INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')


def generate_launch_description():
    turtlebot_urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    turtlebot_urdf_file_path = os.path.join(
        get_package_share_directory('turtlebot3_description_reduced_mesh'),
        'urdf',
        turtlebot_urdf_file_name,
    )

    launch_actions = [
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time', default_value='true'),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('cloudwatch_robot'),
                    'launch',
                    'monitoring.launch.py',
                )
            )
        ),
        launch_ros.actions.Node(
            package='cloudwatch_robot',
            node_executable='rotate',
            node_name='rotate',
            output='screen',
            parameters=[
                {
                    'use_sim_time': launch.substitutions.LaunchConfiguration(
                        'use_sim_time'
                    )
                }
            ],
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': launch.substitutions.LaunchConfiguration(
                        'use_sim_time'
                    )
                }
            ],
            arguments=[turtlebot_urdf_file_path],
        ),
    ]
    ld = launch.LaunchDescription(launch_actions)
    return ld


if __name__ == '__main__':
    generate_launch_description()
