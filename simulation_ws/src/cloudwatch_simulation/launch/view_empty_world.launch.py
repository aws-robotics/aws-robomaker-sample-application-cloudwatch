import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('cloudwatch_simulation'),
                        'launch',
                        'empty_world.launch.py',
                    )),
                launch_arguments={
                    'gui': 'true'}.items(),
            )])
    return ld


if __name__ == '__main__':
    generate_launch_description()
