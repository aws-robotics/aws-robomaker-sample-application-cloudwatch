import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value='$(optenv TURTLEBOT3_MODEL waffle_pi)',
            description='model type [burger, waffle, waffle_pi]'
        ),
        launch.actions.DeclareLaunchArgument(
            name='x_pos',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='y_pos',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='z_pos',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='roll',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pitch',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='yaw',
            default_value='0.0'
        ),
        launch_ros.actions.Node(
            package='gazebo_ros',
            node_executable='spawn_model',
            node_name='spawn_urdf',
            parameters=[
                {
                    'robot_description': None
                }
            ]
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': None
                }
            ]
        ),
        launch_ros.actions.Node(
            package='joint_state_publisher',
            node_executable='joint_state_publisher',
            node_name='joint_state_publisher',
            parameters=[
                {
                    'robot_description': None
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
