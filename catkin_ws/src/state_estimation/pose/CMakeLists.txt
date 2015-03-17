include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(XimuPublisher src/XimuPublisher.cpp src/XimuReceiver.cpp)
target_link_libraries(XimuPublisher ${catkin_LIBRARIES})
add_executable(bumpSensor src/bumpSensor.cpp)
target_link_libraries(bumpSensor  ${catkin_LIBRARIES})



