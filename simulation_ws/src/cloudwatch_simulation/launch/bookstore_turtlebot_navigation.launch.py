import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')

def generate_launch_description():
    turtlebot_urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    turtlebot_urdf_file_path = os.path.join(get_package_share_directory('turtlebot3_description_reduced_mesh'), 'urdf', turtlebot_urdf_file_name)

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='x_pos',
            default_value='-3.5'
        ),
        launch.actions.DeclareLaunchArgument(
            name='y_pos',
            default_value='5.5'
        ),
        launch.actions.DeclareLaunchArgument(
            name='z_pos',
            default_value='0.30'
        ),
        launch.actions.DeclareLaunchArgument(
            name='yaw',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='follow_route',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('cloudwatch_simulation'), 
                    'launch', 
                    'bookstore.launch.py'
                )
            ),
            launch_arguments={
                'gui': launch.substitutions.LaunchConfiguration('gui'),
                'gazebo_model_path': os.path.split(get_package_share_directory('turtlebot3_description_reduced_mesh'))[0],
            }.items()
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
            }],
            arguments=[
                turtlebot_urdf_file_path
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('cloudwatch_simulation'), 
                    'launch', 
                    'turtlebot3_navigation.launch.py'
                )
            ),
            launch_arguments={
                'map_file': os.path.join(get_package_share_directory('cloudwatch_simulation'), 'maps', 'map.yaml'),
                'params_file': os.path.join(get_package_share_directory('cloudwatch_simulation'), 'param', TURTLEBOT3_MODEL + ".yaml"),
                'open_rviz': 'false',
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
            }.items()
        ),
        launch_ros.actions.Node(
            package='aws_robomaker_simulation_common',
            node_executable='route_manager',
            node_name='route_manager',
            output='screen',
            parameters=[{
                # Route file is passed as "<package_name>.<relative path in install space>" due to limitations on string parameter size.
                'route_file': '.'.join(['aws_robomaker_bookstore_world', os.path.join('routes', 'route.yaml')])
            }],
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('follow_route'))
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
