################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(cloudwatch_robot)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  cloudwatch_metrics_collector
  cloudwatch_logger
  health_metric_collector
  turtlebot3_description # required to install .rviz model
)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

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
catkin_package(
  DEPENDS
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    message_runtime
    cloudwatch_metrics_collector
    cloudwatch_logger
    health_metric_collector
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS 
  nodes/rotate.py
  nodes/monitor_speed.py
  nodes/monitor_obstacle_distance.py
  nodes/monitor_distance_to_goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY deploymentScripts
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}
)

# Copy the rviz model for easier access in AWS RoboMaker RViz 
install(FILES ${turtlebot3_description_DIR}/../rviz/model.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  RENAME turtlebot3_model.rviz
)
################################################################################
# Test
################################################################################

