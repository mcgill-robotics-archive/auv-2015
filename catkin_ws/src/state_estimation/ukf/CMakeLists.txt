## Declare a cpp executable
add_executable(pose_ukf
  src/ros_pose.cpp 
  src/ukf.cpp
  src/matrix_utils.cpp
  src/rotation_vector_utils.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(pose_ukf ${catkin_LIBRARIES})




#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
include_directories(src)
catkin_add_gtest(${PROJECT_NAME}-test 
  test/test_ukf.cpp
  src/matrix_utils.cpp
  src/ukf.cpp
  src/rotation_vector_utils.cpp
)

## Simulates visual odometry and CV/sonar inputs ##
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(sim_slam_input test/sim_slam_input.cpp)
target_link_libraries(sim_slam_input ${catkin_LIBRARIES})
