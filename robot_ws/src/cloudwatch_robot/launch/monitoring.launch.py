import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    launch_actions = [
        launch.actions.DeclareLaunchArgument(
            name='aws_region',
            description='AWS region override, defaults to config .yaml if unset',
            default_value='us-west-2'
        ),
        launch.actions.DeclareLaunchArgument(
            name='launch_id',
            description='Used for resource name suffix if specified',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='metrics_node_name',
            default_value="metrics-ros2"
        ),
        launch.actions.DeclareLaunchArgument(
            name='aws_metrics_namespace',
            default_value='robomaker_cloudwatch_monitoring_example'
        ),
        launch.actions.DeclareLaunchArgument(
            name='logger_node_name',
            default_value='logger-ros2'
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
    ld = launch.LaunchDescription(launch_actions)
    return ld


if __name__ == '__main__':
    generate_launch_description()
