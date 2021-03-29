cmake_minimum_required(VERSION 2.8.3)
project(aws_robomaker_simulation_common)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

catkin_package()
catkin_python_setup()

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS 
  nodes/route_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
