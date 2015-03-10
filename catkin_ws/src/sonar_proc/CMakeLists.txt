cmake_minimum_required(VERSION 2.8.3)
project(sonar_proc)

find_package(catkin REQUIRED COMPONENTS
  rospy
  genmsg
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
)
