from setuptools import setup

package_name = 'cloudwatch_simulation'

setup(
    name=package_name,
    version='2.0.0',
    data_files=[
        ('share/' + package_name + '/launch', ['launch/bookstore_turtlebot_navigation.launch.py']),
        ('share/' + package_name + '/launch', ['launch/bookstore_turtlebot3_navigation.launch.py']),
        ('share/' + package_name + '/launch', ['launch/bookstore.launch.py']),
        ('share/' + package_name + '/launch', ['launch/empty_world.launch.py']),
        ('share/' + package_name + '/launch', ['launch/turtlebot3_navigation.launch.py']),
        ('share/' + package_name + '/launch', ['launch/view_empty_world.launch.py']),
        ('share/' + package_name + '/launch', ['launch/turtlebot3_nav2.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/empty.world']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        'AWS RoboMaker robot package that sends robot metrics to CloudWatch'
    ),
    license='Apache License, Version 2.0',
)

