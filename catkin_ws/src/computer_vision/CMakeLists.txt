cmake_minimum_required(VERSION 2.8.3)
project(catkin_ws)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  message_generation
  tf
)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
##add_message_files(
  ##FILES
  ##VisibleObjectData.msg
##)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES test_zone
  CATKIN_DEPENDS roscpp std_msgs message_runtime cv_bridge image_transport
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Testing ##
#############

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(downsample_a src/downsample_a.cpp)

target_link_libraries(downsample_a
			${catkin_LIBRARIES}
			${OpenCV_LIBRARIES}
)


# This makes sure message headers are generated before being used
##add_dependencies(distanceCalculator ${catkin_EXPORTED_TARGETS})
