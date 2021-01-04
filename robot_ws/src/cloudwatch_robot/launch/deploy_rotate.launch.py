import os

import launch
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_actions = [
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('turtlebot3_bringup'),
                    'launch',
                    'robot.launch.py',
                )
            )
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('cloudwatch_robot'),
                    'launch',
                    'rotate.launch.py',
                )
            ),
            launch_arguments={'use_sim_time': 'false'}.items(),
        ),
    ]
    ld = launch.LaunchDescription(launch_actions)
    return ld


if __name__ == '__main__':
    generate_launch_description()
