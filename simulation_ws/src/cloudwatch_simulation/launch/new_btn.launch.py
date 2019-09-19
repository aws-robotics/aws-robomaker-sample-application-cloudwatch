import os
import sys

import launch
import launch_ros.actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')


def generate_launch_description():
    use_gui = launch.actions.DeclareLaunchArgument(
        name='gui',
        default_value='false'
    )

    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description_reduced_mesh'),
        'urdf',
        urdf_file_name)

    bookstore_sim_cmd = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('cloudwatch_simulation'),
                'launch',
                'bookstore.launch.py')
        ),
        launch_arguments={
            'gui': 'true'
        }.items()
    )

    state_publisher_cmd = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': 'true'}],
        arguments=[urdf]
    )

    nav2_start_cmd = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('cloudwatch_simulation'),
                'launch',
                'turtlebot3_nav2.launch.py')
        )
    )

    ld = launch.LaunchDescription()

    ld.add_action(bookstore_sim_cmd)
    ld.add_action(state_publisher_cmd)
    ld.add_action(nav2_start_cmd)

    return ld
