cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3-nav-noetic)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  move_base_msgs
  actionlib
  tf
  gazebo_ros
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs nav_msgs move_base_msgs actionlib tf gazebo_ros
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/obstacle_avoidance.py
  scripts/patrol.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch and config files
install(DIRECTORY launch config rviz worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)