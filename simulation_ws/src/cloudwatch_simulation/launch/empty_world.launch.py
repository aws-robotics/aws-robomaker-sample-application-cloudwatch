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
                    'gazebo_ros'), '/launch/empty_world.launch.py']
            ),
            launch_arguments={
                'world_name': get_package_share_directory('cloudwatch_simulation') + '/worlds/empty.world',
                'paused': 'false',
                'use_sim_time': 'true',
                'gui': launch.substitutions.LaunchConfiguration('gui'),
                'headless': 'false',
                'debug': 'false',
                'verbose': 'false'
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [get_package_share_directory(
                    'turtlebot3_description_reduced_mesh'), '/launch/spawn_turtlebot.launch.py']
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
