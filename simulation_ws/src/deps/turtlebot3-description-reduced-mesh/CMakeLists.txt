cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_description_reduced_mesh)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/turtlebot3_description_reduced_mesh
)
