cmake_minimum_required(VERSION 2.8.3)
project(computer_vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  tf
  message_generation
)

## Python/c++ integration starts here
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 2.7 REQUIRED)
## | 
## V this requires setup.py exists
catkin_python_setup()
## "This macro ensures modules and global scripts declared therein get installed" -ROS Tutorials
## Python/c++ integration ends

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


## second and thired are python/c++ libs
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

#############
## Testing ##
#############

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(computer_vision_lib src/threshold.cpp src/histogram.cpp src/object_finder.cpp src/buoy.cpp)

### Python/c++ interface starts here
add_library(pyterface src/pyterface.cpp)
target_link_libraries(pyterface
			${catkin_LIBRARIES}
			${Boost_LIBRARIES} )
# Don't prepend wrapper library name with lib and add to Python libs.
set_target_properties(pyterface PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
### Python/c++ ends

add_executable(computer_vision_a src/computer_vision_a.cpp)
add_executable(computer_vision_color src/computer_vision_COLOR.cpp)
add_executable(image_publish src/image_publish.cpp)
add_executable(imshow src/imshow.cpp)
add_executable(filters src/filters.cpp)

target_link_libraries(computer_vision_a computer_vision_lib
			${catkin_LIBRARIES}
			${OpenCV_LIBRARIES}
)

target_link_libraries(computer_vision_color computer_vision_lib
			${catkin_LIBRARIES}
			${OpenCV_LIBRARIES}
)

target_link_libraries(image_publish
			${catkin_LIBRARIES}
			${OpenCV_LIBRARIES}
)

target_link_libraries(imshow computer_vision_lib
			${catkin_LIBRARIES}
			${OpenCV_LIBRARIES}
)

target_link_libraries(filters computer_vision_lib
			${catkin_LIBRARIES}
			${OpenCV_LIBRARIES}
)

# This makes sure message headers are generated before being used
##add_dependencies(distanceCalculator ${catkin_EXPORTED_TARGETS})
