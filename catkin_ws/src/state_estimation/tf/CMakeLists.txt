include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(sim_slam_input src/sim_slam_input.cpp)
target_link_libraries(sim_slam_input ${catkin_LIBRARIES})

