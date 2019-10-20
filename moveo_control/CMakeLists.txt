################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(moveo_control)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  #controller_manager
  #gazebo_ros
  joint_state_controller
  robot_state_publisher
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#catkin_package(
#  CATKIN_DEPENDS controller_manager gazebo_ros_pkgs gazebo_ros
#)

catkin_package(
  CATKIN_DEPENDS controller_manager joint_state_controller robot_state_publisher
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
