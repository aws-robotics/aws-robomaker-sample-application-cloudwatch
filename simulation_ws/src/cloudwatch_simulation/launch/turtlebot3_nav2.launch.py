import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')


def generate_launch_description():
    # Launch configurations
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description_reduced_mesh'),
        'urdf',
        urdf_file_name)
    print('URDF File: {}'.format(urdf))

    world_file_name = TURTLEBOT3_MODEL + '.model'
    world = os.path.join(
        get_package_share_directory('turtlebot3_description_reduced_mesh'),
        'worlds',
        world_file_name)
    print('World File: {}'.format(world))

    use_gazebo_gui = LaunchConfiguration('gui', default='true')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    default_map_dir = os.path.join(
        get_package_share_directory('turtlebot3_description_reduced_mesh'),
        'map',
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

    # Nodes and launch files
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', world, '-s', 'libgazebo_ros_init.so'],
        output='screen',
        shell=True
    )

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        shell=True,
        condition=IfCondition(use_gazebo_gui)
    )

    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf]
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

    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_robot_state_publisher)
    ld.add_action(start_nav2_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
