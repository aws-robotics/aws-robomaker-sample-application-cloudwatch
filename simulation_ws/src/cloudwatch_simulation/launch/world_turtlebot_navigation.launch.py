# *******************************************************************************/
# This launch file serves as a template for navigation launch files.
# It takes two key argument:
# - world_launch_file, which specifies the world launch file name
# - world_package, which specifies the actual package name that stores the world
# *******************************************************************************/
import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')

def generate_launch_description():
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
        launch.actions.DeclareLaunchArgument(
            name='world_launch_file',
            default_value='bookstore.launch.py'
        ),
        launch.actions.DeclareLaunchArgument(
            name='world_package',
            default_value='aws_robomaker_bookstore_world'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [
                    get_package_share_directory('cloudwatch_simulation'),
                    '/launch/',
                    launch.substitutions.LaunchConfiguration('world_launch_file')
                ]
            ),
            launch_arguments={
                'gui': launch.substitutions.LaunchConfiguration('gui'),
                'gazebo_model_path': os.path.split(get_package_share_directory('turtlebot3_description_reduced_mesh'))[0],
            }.items()
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
                'route_file': [launch.substitutions.LaunchConfiguration('world_package'), '.', os.path.join('routes', 'route.yaml')]
            }],
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('follow_route'))
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
