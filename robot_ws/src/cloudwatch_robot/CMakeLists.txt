################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(cloudwatch_robot)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################

set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${cloudwatch_logger_INCLUDE_DIRS}
  ${cloudwatch_metrics_collector_INCLUDE_DIRS} ${rclpy_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}
  ${health_metric_collector_INCLUDE_DIRS}
  ${rosidl_default_generators_INCLUDE_DIRS})
find_package(ament_cmake REQUIRED)
find_package(REQUIRED REQUIRED)
find_package(cloudwatch_logger REQUIRED)
find_package(health_metric_collector REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cloudwatch_metrics_collector REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${REQUIRED_INCLUDE_DIRS}
  ${cloudwatch_logger_INCLUDE_DIRS} ${health_metric_collector_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS} ${cloudwatch_metrics_collector_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS} ${rclpy_INCLUDE_DIRS}
  ${rosidl_default_generators_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})
set(LIBRARY_DIRS ${ament_cmake_LIBRARIES} ${REQUIRED_LIBRARIES}
  ${cloudwatch_logger_LIBRARIES} ${health_metric_collector_LIBRARIES}
  ${geometry_msgs_LIBRARIES} ${ament_cmake_LIBRARIES} ${sensor_msgs_LIBRARIES}
  ${cloudwatch_metrics_collector_LIBRARIES} ${std_msgs_LIBRARIES}
  ${rclpy_LIBRARIES} ${rosidl_default_generators_LIBRARIES} ${nav_msgs_LIBRARIES})
set(LIBS ${ament_cmake_LIBRARIES} ${REQUIRED_LIBRARIES}
  ${cloudwatch_logger_LIBRARIES} ${health_metric_collector_LIBRARIES}
  ${geometry_msgs_LIBRARIES} ${ament_cmake_LIBRARIES} ${sensor_msgs_LIBRARIES}
  ${cloudwatch_metrics_collector_LIBRARIES} ${std_msgs_LIBRARIES}
  ${rclpy_LIBRARIES} ${rosidl_default_generators_LIBRARIES} ${nav_msgs_LIBRARIES})
set(LIBRARY_DIRS ${ament_cmake_LIBRARIES} ${cloudwatch_logger_LIBRARIES}
  ${cloudwatch_metrics_collector_LIBRARIES} ${rclpy_LIBRARIES}
  ${nav_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES} ${health_metric_collector_LIBRARIES}
  ${rosidl_default_generators_LIBRARIES})
set(LIBS ${ament_cmake_LIBRARIES} ${cloudwatch_logger_LIBRARIES}
  ${cloudwatch_metrics_collector_LIBRARIES} ${rclpy_LIBRARIES}
  ${nav_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES} ${health_metric_collector_LIBRARIES}
  ${rosidl_default_generators_LIBRARIES})
################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
#add_action_files(
#  FILES
#  Turtlebot3.action
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#  nav_msgs
#)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS nodes/rotate nodes/monitor_speed
  nodes/monitor_obstacle_distance nodes/monitor_distance_to_goal
  DESTINATION bin)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY deploymentScripts DESTINATION share)

################################################################################
# Test
################################################################################


ament_export_dependencies(ament_cmake)
ament_export_dependencies(cloudwatch_logger)
ament_export_dependencies(cloudwatch_metrics_collector)
ament_export_dependencies(rclpy)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(health_metric_collector)
ament_export_dependencies(rosidl_default_generators)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()

ament_export_dependencies(ament_cmake)
ament_export_dependencies(REQUIRED)
ament_export_dependencies(cloudwatch_logger)
ament_export_dependencies(health_metric_collector)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(cloudwatch_metrics_collector)
ament_export_dependencies(std_msgs)
ament_export_dependencies(rclpy)
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(nav_msgs)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
