cmake_minimum_required(VERSION 2.8.3)
project(computer_vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  tf
)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
##add_message_files(
  ##FILES
  ##VisibleObjectData.msg
##)

# Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
# )

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

add_executable(computer_vision_a src/computer_vision_a.cpp)

target_link_libraries(computer_vision_a
			${catkin_LIBRARIES}
			${OpenCV_LIBRARIES}
)

add_executable(computer_vision_COLOR src/computer_vision_COLOR.cpp)

add_library(computer_vision src/threshold.cpp src/histogram.cpp src/object_finder.cpp)

target_link_libraries(computer_vision_COLOR computer_vision 
			${catkin_LIBRARIES}
			${OpenCV_LIBRARIES}
)

add_executable(computer_vision_OBJECT src/computer_vision_OBJECT.cpp)

target_link_libraries(computer_vision_OBJECT
			${catkin_LIBRARIES}
			${OpenCV_LIBRARIES}
)

add_executable(image_publish src/image_publish.cpp)

target_link_libraries(image_publish
			${catkin_LIBRARIES}
			${OpenCV_LIBRARIES}
)

# This makes sure message headers are generated before being used
##add_dependencies(distanceCalculator ${catkin_EXPORTED_TARGETS})
