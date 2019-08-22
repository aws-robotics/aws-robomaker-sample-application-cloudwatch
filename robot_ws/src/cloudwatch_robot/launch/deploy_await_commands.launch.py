import os
import sys

import launch
import launch_rpos.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_actions = [
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [get_package_share_directory('turtlebot3_bringup'), '/launch/turtlebot3_robot.launch.py']
            )
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [get_package_share_directory('cloudwatch_robot'), '/launch/await_commands.launch.py']
                # TODO: Pass use_sim_time arg here. 
            )
        ),
    ]
    ld = launch.LaunchDescription(launch_actions)
    return ld


if __name__ == '__main__':
    generate_launch_description()
