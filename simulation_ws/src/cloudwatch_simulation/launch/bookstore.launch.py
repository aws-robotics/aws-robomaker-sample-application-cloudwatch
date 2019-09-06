import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='false'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [get_package_share_directory(
                    'aws_robomaker_bookstore_world'), '/launch/bookstore.launch.py']
            ),
            launch_arguments={
                'gui': launch.substitutions.LaunchConfiguration('gui')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [get_package_share_directory(
                    'turtlebot3_description_reduced_mesh'), '/launch/spawn_turtlebot.launch.py']
            ),
            launch_arguments={
                'x_pos': '-3.5',
                'y_pos': '5.5',
                'z_pos': '0.3'
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
