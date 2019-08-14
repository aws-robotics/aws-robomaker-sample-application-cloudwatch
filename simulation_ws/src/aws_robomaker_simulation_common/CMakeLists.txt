cmake_minimum_required(VERSION 3.5)
project(aws_robomaker_simulation_common)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()



set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${gazebo_ros_INCLUDE_DIRS}
  ${turtlebot3_description_INCLUDE_DIRS} ${turtlebot3_navigation_INCLUDE_DIRS})
find_package(ament_cmake REQUIRED)
find_package(REQUIRED REQUIRED)
find_package(turtlebot3_description REQUIRED)
find_package(turtlebot3_navigation REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ament_cmake REQUIRED)
set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${REQUIRED_INCLUDE_DIRS}
  ${turtlebot3_description_INCLUDE_DIRS} ${turtlebot3_navigation_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})
set(LIBRARY_DIRS ${ament_cmake_LIBRARIES} ${REQUIRED_LIBRARIES}
  ${turtlebot3_description_LIBRARIES} ${turtlebot3_navigation_LIBRARIES}
  ${gazebo_ros_LIBRARIES} ${ament_cmake_LIBRARIES})
set(LIBS ${ament_cmake_LIBRARIES} ${REQUIRED_LIBRARIES}
  ${turtlebot3_description_LIBRARIES} ${turtlebot3_navigation_LIBRARIES}
  ${gazebo_ros_LIBRARIES} ${ament_cmake_LIBRARIES})
set(LIBRARY_DIRS ${ament_cmake_LIBRARIES} ${gazebo_ros_LIBRARIES}
  ${turtlebot3_description_LIBRARIES} ${turtlebot3_navigation_LIBRARIES})
set(LIBS ${ament_cmake_LIBRARIES} ${gazebo_ros_LIBRARIES}
  ${turtlebot3_description_LIBRARIES} ${turtlebot3_navigation_LIBRARIES})

install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

catkin_install_python(PROGRAMS nodes/route_manager DESTINATION bin)

# Copy the rviz models for easier access in AWS RoboMaker RViz
install(FILES ${turtlebot3_navigation_DIR}/../rviz/turtlebot3_navigation.rviz
  DESTINATION share/${PROJECT_NAME}/rviz
  RENAME turtlebot3_navigation.rviz)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(turtlebot3_description)
ament_export_dependencies(turtlebot3_navigation)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()

ament_export_dependencies(ament_cmake)
ament_export_dependencies(REQUIRED)
ament_export_dependencies(turtlebot3_description)
ament_export_dependencies(turtlebot3_navigation)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(ament_cmake)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
