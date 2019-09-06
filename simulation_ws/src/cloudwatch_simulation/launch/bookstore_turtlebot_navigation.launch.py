import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='follow_route',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='false'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [get_package_share_directory(
                    'cloudwatch_simulation'), '/launch/bookstore_turtlebot3_navigation.launch.py']
            ),
            launch_arguments={
                'gui': launch.substitutions.LaunchConfiguration('gui'),
                'x_pos': '-3.5',
                'y_pos': '5.5',
                'yaw': '0.0'
            }.items()
        ),
        launch_ros.actions.Node(
            package='aws_robomaker_simulation_common',
            node_executable='route_manager',
            node_name='route_manager',
            screen='screen',
            parameters={
                'route_file': get_package_share_directory('aws_robomaker_bookstore_world') + '/routes/route.yaml'
            }.items(),
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('follow_route'))
        ),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
