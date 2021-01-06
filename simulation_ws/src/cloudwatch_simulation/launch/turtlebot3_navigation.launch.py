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

# *******************************************************************************/
# This launch file takes two non-optional arguments:
# - map_file            : Full path to map file to load
# - params_file         : Full path to params file to load
# *******************************************************************************/
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch configurations
    map_file = LaunchConfiguration('map_file')
    print('Map File: {}'.format(map_file))

    params_file = LaunchConfiguration('params_file')
    print('Param File: {}'.format(params_file))

    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch'
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory(
            'nav2_bringup'), 'launch', 'nav2_default_view.rviz'
    )
    print('Rviz config: {}'.format(rviz_config_dir))

    # Launch arguments
    declare_map_file_arg = DeclareLaunchArgument(
        name='map_file', description='Full path to map file to load'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_arg = DeclareLaunchArgument(
        name='params_file', description='Full path to params file to load'
    )

    declare_open_rviz_arg = DeclareLaunchArgument(
        name='open_rviz',
        default_value='true',
        description='Open rviz on launch if true',
    )

    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'nav2_bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params': LaunchConfiguration('params_file'),
        }.items(),
    )

    start_rviz_cmd = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('open_rviz')),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_map_file_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_params_file_arg)
    ld.add_action(declare_open_rviz_arg)

    ld.add_action(start_nav2_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
