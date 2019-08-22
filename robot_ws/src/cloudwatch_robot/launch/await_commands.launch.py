import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch_actions = [
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [get_package_share_directory('cloudwatch_robot'), '/launch/monitoring.launch.py']
            )
        )
    ]
    ld = launch.LaunchDescription(launch_actions)
    return ld

if __name__ == '__main__':
    generate_launch_description()
