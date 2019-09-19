import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')


def generate_launch_description():
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    default_map_dir = os.path.join(
        get_package_share_directory('cloudwatch_simulation'),
        'maps',
        'map.yaml')
    map_dir = LaunchConfiguration('map', default=default_map_dir)
    print('Map File: {}'.format(default_map_dir))

    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    default_param_dir = os.path.join(
        get_package_share_directory('cloudwatch_simulation'),
        'param',
        param_file_name)
    param_dir = LaunchConfiguration('params', default=default_param_dir)
    print('Param File: {}'.format(default_param_dir))

    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'nav2_default_view.rviz')
    print('Rviz config: {}'.format(rviz_config_dir))

    # Launch arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_dir,
        description='Full path to map file to load'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params',
        default_value=param_dir,
        description='Full path to param file to load'
    )

    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'nav2_bringup_launch.py')),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params': param_dir}.items(),
    )

    start_rviz_cmd = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)

    ld.add_action(start_nav2_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
