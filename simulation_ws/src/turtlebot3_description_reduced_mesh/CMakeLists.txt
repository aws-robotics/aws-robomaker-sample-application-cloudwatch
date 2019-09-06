cmake_minimum_required(VERSION 3.5)
project(turtlebot3_description_reduced_mesh)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${gazebo_ros_INCLUDE_DIRS})

include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARIES} ${ament_cmake_LIBRARIES}
  ${gazebo_ros_LIBRARIES})

install(DIRECTORY launch meshes urdf DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(gazebo_ros)

ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
