# *******************************************************************************/
# This launch file serves as a template for navigation launch files.
# It takes five non-optional arguments:
# - x_pos            : The x coordinate of the robot initial pose
# - y_pos            : The y coordinate of the robot initial pose
# - z_pos            : The z coordinate of the robot initial pose
# - world_launch_file: It specifies the world launch file name
# - world_package    : It specifies the absolute path of the package that stores the world
#
# To start with a different initial pose, besides the x_pos, y_pos and z_pos in this launch file,
# also modify the param.yaml file in the world repo accordingly.
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
            description='The x coordinate of the robot initial pose'
        ),
        launch.actions.DeclareLaunchArgument(
            name='y_pos',
            description='The y coordinate of the robot initial pose'
        ),
        launch.actions.DeclareLaunchArgument(
            name='z_pos',
            description='The z coordinate of the robot initial pose'
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
            description='The world launch file'
        ),
        launch.actions.DeclareLaunchArgument(
            name='world_package',
            description='The absolute path of the package that stores the world launch file'
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
                'map_file': [launch.substitutions.LaunchConfiguration('world_package'), '/maps', '/turtlebot3_', TURTLEBOT3_MODEL, '/map.yaml'],
                'params_file': [launch.substitutions.LaunchConfiguration('world_package'), '/param/', TURTLEBOT3_MODEL + ".yaml"],
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
