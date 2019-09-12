# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

"""Launch turtlebot3_description_reduced_mesh and a rotate node."""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch
import tempfile
import launch_ros.actions
from launch_ros import get_default_launch_description

from ament_index_python.packages import get_package_share_directory
import xacro
import lifecycle_msgs.msg
import subprocess

def generate_launch_description():
    turtlebot3_model = "turtlebot3_" + os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi') + ".urdf"

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='x_pos',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='y_pos',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='z_pos',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='roll',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pitch',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='yaw',
            default_value='0.0'
        ),
        launch_ros.actions.Node(
            package='gazebo_ros',
            node_executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity',
                'robot',
                '-file', 
                os.path.join(get_package_share_directory('turtlebot3_description_reduced_mesh'), 'urdf', turtlebot3_model),
                '-x',
                launch.substitutions.LaunchConfiguration('x_pos'),
                '-y',
                launch.substitutions.LaunchConfiguration('y_pos'),
                '-z',
                launch.substitutions.LaunchConfiguration('z_pos'),
                '-R',
                launch.substitutions.LaunchConfiguration('roll'),
                '-P',
                launch.substitutions.LaunchConfiguration('pitch'),
                '-Y',
                launch.substitutions.LaunchConfiguration('yaw')
            ]
        ),
    ])
    return ld 


if __name__ == '__main__':
    generate_launch_description()
