# Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
from setuptools import setup

package_name = 'aws_robomaker_simulation_common'

setup(
    name=package_name,
    version='2.0.0',
    python_requires='>=3.5.0',
    packages=[package_name],
    data_files=[
        ('lib/' + package_name, ['src/' + package_name + '/route_manager.py']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=[
        'setuptools',
        'rospkg==1.1.7',
        'pyyaml'
    ],
    zip_safe=True,
    author='AWS RoboMaker',
    author_email='ros-contributions@amazon.com',
    maintainer='AWS RoboMaker',
    maintainer_email='ros-contributions@amazon.com',
    keywords=['ROS'],
    entry_points={
        'console_scripts': [
            'route_manager = aws_robomaker_simulation_common.route_manager:main'
        ],
    },
)
