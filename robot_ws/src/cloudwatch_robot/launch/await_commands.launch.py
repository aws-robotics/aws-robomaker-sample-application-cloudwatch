import os
import sys

import launch
import launch_ros.actions
from .monitoring.launch import get_launch_actions as get_monitoring_launch_actions


def get_launch_actions():
    launch_actions = [
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        )
    ]
    launch_actions += get_monitoring_launch_actions()
    return launch_actions

def generate_launch_description():
    ld = launch.LaunchDescription(get_launch_actions())
    return ld


if __name__ == '__main__':
    generate_launch_description()
