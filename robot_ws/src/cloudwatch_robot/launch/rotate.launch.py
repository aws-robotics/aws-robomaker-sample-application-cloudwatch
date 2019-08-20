import os
import sys

import launch
import launch_ros.actions
from monitoring.launch import get_launch_actions as get_monitoring_launch_actions


def get_launch_actions():
    launch_actions = [
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        launch_ros.actions.Node(
            package='cloudwatch_robot',
            node_executable='rotate',
            node_name='rotate',
            output='screen',
            parameters=[
                {
                    'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
                }
            ]
        )
    ]
    launch_actions += get_monitoring_launch_actions()
    return launch_actions


def generate_launch_description():
    launch_actions = get_launch_actions()
    ld = launch.LaunchDescription(launch_actions)
    return ld


if __name__ == '__main__':
    generate_launch_description()
