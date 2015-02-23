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
  message_generation
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

add_message_files(
	FILES
	ObjectImageLocation.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES test_zone
  CATKIN_DEPENDS message_runtime roscpp std_msgs message_runtime cv_bridge image_transport
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

add_executable(computer_vision_edge src/computer_vision_edge.cpp)
add_executable(computer_vision_color src/computer_vision_color.cpp)
add_executable(image_publish src/image_publish.cpp)

add_library(computer_vision src/threshold.cpp src/histogram.cpp src/edge.cpp src/object_finder.cpp)

target_link_libraries(computer_vision_edge computer_vision
			${catkin_LIBRARIES}
			${OpenCV_LIBRARIES}
)
target_link_libraries(computer_vision_color computer_vision 
			${catkin_LIBRARIES}
			${OpenCV_LIBRARIES}
)
target_link_libraries(image_publish
			${catkin_LIBRARIES}
			${OpenCV_LIBRARIES}
)

# This makes sure message headers are generated before being used
##add_dependencies(distanceCalculator ${catkin_EXPORTED_TARGETS})
