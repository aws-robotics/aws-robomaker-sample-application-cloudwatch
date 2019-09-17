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
            name='x_pos',
            default_value='-2.5'
        ),
        launch.actions.DeclareLaunchArgument(
            name='y_pos',
            default_value='6.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='yaw',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='z_pos',
            default_value='0.30'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'aws_robomaker_bookstore_world'), 'launch', 'bookstore.launch.py')
            ),
            launch_arguments={
                'gui': launch.substitutions.LaunchConfiguration('gui'),
                'gazebo_model_path': os.path.split(get_package_share_directory('turtlebot3_description_reduced_mesh'))[0],
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'turtlebot3_description_reduced_mesh'), 'launch', 'spawn_turtlebot.launch.py')
            ),
            launch_arguments={
                'x_pos': launch.substitutions.LaunchConfiguration('x_pos'),
                'y_pos': launch.substitutions.LaunchConfiguration('y_pos'),
                'z_pos': launch.substitutions.LaunchConfiguration('z_pos'),
                'yaw': launch.substitutions.LaunchConfiguration('yaw')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('turtlebot3_bringup'), 'launch', 'turtlebot3_state_publisher.launch.py')
            ),
            launch_arguments={
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'cloudwatch_simulation'), 'launch', 'turtlebot3_nav2.launch.py')
            ),
            launch_arguments={
                'map_file': os.path.join(get_package_share_directory('aws_robomaker_bookstore_world'), 'maps', f"turtlebot3_{os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')}", 'map.yaml'),
                'initial_pose_x': launch.substitutions.LaunchConfiguration('x_pos'),
                'initial_pose_y': launch.substitutions.LaunchConfiguration('y_pos'),
                'initial_pose_a': launch.substitutions.LaunchConfiguration('yaw'),
                'open_rviz': 'false'
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
