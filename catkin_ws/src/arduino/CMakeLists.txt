cmake_minimum_required(VERSION 2.8.3)
project(arduino)

find_package(catkin REQUIRED COMPONENTS
  rosserial_client
  rosserial_arduino
)

catkin_package()

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)
