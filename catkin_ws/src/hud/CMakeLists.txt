cmake_minimum_required(VERSION 2.8.3)
project(hud)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  roscpp
)
