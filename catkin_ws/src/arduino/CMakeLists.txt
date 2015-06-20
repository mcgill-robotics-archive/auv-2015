cmake_minimum_required(VERSION 2.8.3)
project(arduino)

find_package(catkin REQUIRED COMPONENTS
  rosserial_client
  rosserial_arduino
)

catkin_package(
  DEPENDS auv_msgs
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

# The ros_lib has to be deleted before generating a new one.
add_custom_target(
  delete_${PROJECT_NAME}_ros_lib
  COMMAND rm -rf ${PROJECT_BINARY_DIR}/ros_lib
)

add_dependencies(
  ${PROJECT_NAME}_ros_lib
  delete_${PROJECT_NAME}_ros_lib 
  auv_msgs_genpy
)

