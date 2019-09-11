import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    world = os.path.join(get_package_share_directory('cloudwatch_simulation'), 'worlds', 'empty.world')
    env = {
        'GAZEBO_MODEL_PATH': os.path.split(get_package_share_directory('turtlebot3_description_reduced_mesh'))[0],
    }
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='false'
        ),
        launch.actions.ExecuteProcess(
            cmd=['gzserver', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
            cwd=get_package_share_directory('cloudwatch_simulation'),
            additional_env=env,
            output='screen'
        ),
        launch.actions.ExecuteProcess(
            cmd=['gzclient'],
            additional_env=env,
            output='screen',
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('gui'))
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'turtlebot3_description_reduced_mesh'), 'launch', 'spawn_turtlebot.launch.py')
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
