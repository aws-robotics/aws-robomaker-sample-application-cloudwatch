import os
import sys

import launch
import launch_ros.actions


def get_launch_actions():
    launch_actions = [
        launch.actions.DeclareLaunchArgument(
            name='aws_region',
            description='AWS region override, defaults to config .yaml if unset'
        ),
        launch.actions.DeclareLaunchArgument(
            name='launch_id',
            description='Used for resource name suffix if specified'
        ),
        launch.actions.DeclareLaunchArgument(
            name='metrics_node_name'
        ),
        launch.actions.DeclareLaunchArgument(
            name='aws_metrics_namespace',
            default_value='robomaker_cloudwatch_monitoring_example'
        ),
        launch.actions.DeclareLaunchArgument(
            name='logger_node_name'
        ),
        launch.actions.DeclareLaunchArgument(
            name='log_group_name',
            default_value='robomaker_cloudwatch_monitoring_example'
        ),
        launch_ros.actions.Node(
            package='cloudwatch_robot',
            node_executable='monitor_speed',
            node_name='monitor_speed',
            output='log'
        ),
        launch_ros.actions.Node(
            package='cloudwatch_robot',
            node_executable='monitor_obstacle_distance',
            node_name='monitor_obstacle_distance',
            output='log'
        ),
        launch_ros.actions.Node(
            package='cloudwatch_robot',
            node_executable='monitor_distance_to_goal',
            node_name='monitor_distance_to_goal',
            output='log'
        )
    ]
    return launch_actions

def generate_launch_description():
    launch_actions = get_launch_actions()
    ld = launch.LaunchDescription(launch_actions)
    return ld


if __name__ == '__main__':
    generate_launch_description()
