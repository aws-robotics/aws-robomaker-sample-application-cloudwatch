import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=launch.substitutions.EnvironmentVariable(
                'TURTLEBOT3_MODEL'),
            description='model type [burger, waffle, waffle_pi]'
        ),
        launch.actions.DeclareLaunchArgument(
            name='map_file',
            default_value=os.path.join(get_package_share_directory(
                'turtlebot3_navigation2') + 'map', 'map.yaml')
        ),
        launch.actions.DeclareLaunchArgument(
            name='open_rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='initial_pose_x',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='initial_pose_y',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='initial_pose_a',
            default_value='0.0'
        ),
        launch_ros.actions.Node(
            package='map_server',
            node_executable='map_server',
            node_name='map_server',
            arguments=[launch.substitutions.LaunchConfiguration('map_file')]
        ),
        launch_ros.actions.Node(
            package='rviz',
            node_executable='rviz',
            node_name='rviz',
            on_exit=launch.actions.Shutdown(),
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('open_rviz'))
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'turtlebot3_navigation2'), 'launch', 'amcl.launch.py')
            ),
            launch_arguments={
                'initial_pose_x': launch.substitutions.LaunchConfiguration('initial_pose_x'),
                'initial_pose_y': launch.substitutions.LaunchConfiguration('initial_pose_y'),
                'initial_pose_a': launch.substitutions.LaunchConfiguration('initial_pose_a')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'turtlebot3_navigation'), 'launch', 'move_base.launch.py')
            ),
            launch_arguments={
                'model': launch.substitutions.LaunchConfiguration('model')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
