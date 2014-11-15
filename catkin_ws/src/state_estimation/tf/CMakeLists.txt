include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
add_executable(horizon_broadcaster src/horizon_broadcaster.cpp)
target_link_libraries(horizon_broadcaster ${catkin_LIBRARIES})

