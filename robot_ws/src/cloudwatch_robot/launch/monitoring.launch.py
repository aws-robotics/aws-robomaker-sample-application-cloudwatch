import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_actions = [
        launch.actions.DeclareLaunchArgument(
            name='aws_region',
            description='AWS region override, defaults to config .yaml if unset',
            default_value=launch.substitutions.EnvironmentVariable('ROS_AWS_REGION')
        ),
        launch.actions.DeclareLaunchArgument(
            name='launch_id',
            description='Used for resource name suffix if specified',
            default_value=launch.substitutions.EnvironmentVariable('LAUNCH_ID')
        ),
        launch.actions.DeclareLaunchArgument(
            name='metrics_node_name',
            default_value="cloudwatch_metrics_collector"
        ),
        launch.actions.DeclareLaunchArgument(
            name='aws_metrics_namespace',
            default_value='robomaker_cloudwatch_monitoring_example'
        ),
        launch.actions.DeclareLaunchArgument(
            name='logger_node_name',
            default_value='cloudwatch_logger'
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
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [get_package_share_directory('health_metric_collector_node'), '/launch/health_metric_collector.launch.py']
            ),
            launch_arguments={
                'config_file': os.path.join(get_package_share_directory('cloudwatch_robot'), '/config/health_metrics_config.yaml')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [get_package_share_directory('cloudwatch_metrics_collector'), '/launch/cloudwatch_metrics_collector_launch.py']
            ),
            launch_arguments={
                'node_name': launch.substitutions.LaunchConfiguration('metrics_node_name'),
                'config_file': os.path.join(get_package_share_directory('cloudwatch_robot'), '/config/cloudwatch_metrics_config.yaml'),
                'aws_region': launch.substitutions.LaunchConfiguration('aws_region'),
                'launch_id': launch.substitutions.LaunchConfiguration('launch_id')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [get_package_share_directory('cloudwatch_logger'), '/launch/cloudwatch_logger.launch.py']
            ),
            launch_arguments={
                'node_name': launch.substitutions.LaunchConfiguration('logger_node_name'),
                'config_file': os.path.join(get_package_share_directory('cloudwatch_robot'), '/config/cloudwatch_logs_config.yaml'),
                'aws_region': launch.substitutions.LaunchConfiguration('aws_region'),
                'launch_id': launch.substitutions.LaunchConfiguration('launch_id')
            }.items()
        ),
    ]
    ld = launch.LaunchDescription(launch_actions)
    return ld


if __name__ == '__main__':
    generate_launch_description()
