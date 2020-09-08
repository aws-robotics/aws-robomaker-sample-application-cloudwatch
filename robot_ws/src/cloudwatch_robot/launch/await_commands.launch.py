import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')

def generate_launch_description():
    turtlebot_urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    turtlebot_urdf_file_path = os.path.join(get_package_share_directory('turtlebot3_description_reduced_mesh'), 'urdf', turtlebot_urdf_file_name)

    launch_actions = [
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('cloudwatch_robot'), 'launch', 'monitoring.launch.py')
            )
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
            ])
    ]
    ld = launch.LaunchDescription(launch_actions)
    return ld

if __name__ == '__main__':
    generate_launch_description()
