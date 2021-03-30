from setuptools import find_packages, setup
from ament_index_python.packages import get_package_share_directory
from setuptools.command.install import install
from shutil import copyfile
import os

package_name = 'cloudwatch_robot'

class CopyRvizModel(install):
    def run(self):
        src = get_package_share_directory('turtlebot3_navigation2')+'/rviz/tb3_navigation2.rviz'
        dest_dir = '../../install/cloudwatch_robot/rviz'
        os.mkdir(dest_dir)
        copyfile(src, dest_dir+'/turtlebot3_navigation.rviz')
        install.run(self)

setup(
    name=package_name,
    version='2.0.0',
    package_dir={'': 'src'},
    packages=find_packages(where='src'),
    cmdclass={
        'install': CopyRvizModel,
    },
    data_files=[
        ('share/' + package_name + '/launch',
         ['launch/await_commands.launch.py']),
        ('share/' + package_name + '/launch',
         ['launch/deploy_rotate.launch.py']),
        ('share/' + package_name + '/launch', ['launch/monitoring.launch.py']),
        ('share/' + package_name + '/launch', ['launch/rotate.launch.py']),
        (
            'share/' + package_name + '/launch',
            ['launch/deploy_await_commands.launch.py'],
        ),
        ('share/' + package_name + '/config',
         ['config/cloudwatch_logs_config.yaml']),
        (
            'share/' + package_name + '/config',
            ['config/cloudwatch_metrics_config.yaml'],
        ),
        ('share/' + package_name + '/config',
         ['config/health_metrics_config.yaml']),
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='AWS RoboMaker',
    author_email='ros-contributions@amazon.com',
    maintainer='AWS RoboMaker',
    maintainer_email='ros-contributions@amazon.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'AWS RoboMaker robot package that sends robot metrics to CloudWatch'),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor_distance_to_goal = '
            'cloudwatch_robot.monitor_distance_to_goal:main',
            'monitor_obstacle_distance = '
            'cloudwatch_robot.monitor_obstacle_distance:main',
            'monitor_speed = cloudwatch_robot.monitor_speed:main',
            'rotate = cloudwatch_robot.rotate:main',
        ]
    },
)
