cmake_minimum_required(VERSION 3.5)
project(lidar_checker)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(monitor_obstacle_distance monitor_obstacle_distance.cpp)
ament_target_dependencies(monitor_obstacle_distance rclcpp std_msgs sensor_msgs)

install(TARGETS
  monitor_obstacle_distance
  DESTINATION lib/${PROJECT_NAME})

ament_package()
