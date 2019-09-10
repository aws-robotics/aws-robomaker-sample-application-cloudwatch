import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    world = os.path.join(get_package_share_directory('cloudwatch_simulation'), 'worlds', 'empty.world')
    env = {
        'GAZEBO_MODEL_PATH': ":".join([
            '/usr/share/gazebo-9/models', 
            os.path.join(get_package_share_directory('turtlebot3_description_reduced_mesh'), 'models'),
            os.path.split(get_package_share_directory('turtlebot3_description_reduced_mesh'))[0],
        ]),
        'GAZEBO_RESOURCE_PATH': ":".join([
            '/usr/share/gazebo-9', os.path.join(get_package_share_directory('turtlebot3_description_reduced_mesh'))])
    }
    # print(f"env: {env}")
    # print(f"Reduced mesh dir: {get_package_share_directory('turtlebot3_description_reduced_mesh')}")
    # print(f"Reduced mesh dir: {os.path.split(get_package_share_directory('turtlebot3_description_reduced_mesh'))[0]}")
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
            output='screen',
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('gui'))
        ),
        # launch.actions.DeclareLaunchArgument(
        #     name='gui',
        #     default_value='false'
        # ),
        # launch.actions.IncludeLaunchDescription(
        #     launch.launch_description_sources.PythonLaunchDescriptionSource(
        #         os.path.join(get_package_share_directory(
        #             'gazebo_ros'), 'launch', 'empty_world.launch.py')
        #     ),
        #     launch_arguments={
        #         'world_name': get_package_share_directory('cloudwatch_simulation') + '/worlds/empty.world',
        #         'paused': 'false',
        #         'use_sim_time': 'true',
        #         'gui': launch.substitutions.LaunchConfiguration('gui'),
        #         'headless': 'false',
        #         'debug': 'false',
        #         'verbose': 'false'
        #     }.items()
        # ),
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
