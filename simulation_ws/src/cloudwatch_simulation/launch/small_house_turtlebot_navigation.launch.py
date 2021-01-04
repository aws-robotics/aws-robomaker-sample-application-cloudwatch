import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name='x_pos', default_value='3.5'),
            launch.actions.DeclareLaunchArgument(
                name='y_pos', default_value='1.0'),
            launch.actions.DeclareLaunchArgument(
                name='z_pos', default_value='0.0'),
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('cloudwatch_simulation'),
                        'launch',
                        'sample_world_turtlebot_navigation.launch.py',
                    )
                ),
                launch_arguments={
                    'world_launch_file': 'small_house_world.launch.py',
                    'world_package': get_package_share_directory(
                        'aws_robomaker_small_house_world'
                    ),
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
