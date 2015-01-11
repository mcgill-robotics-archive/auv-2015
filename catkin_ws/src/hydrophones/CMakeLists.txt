cmake_minimum_required(VERSION 2.8.3)
project(hydrophones)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Pinger.msg
  TDOA.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  hydrophones
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  roscpp
)
