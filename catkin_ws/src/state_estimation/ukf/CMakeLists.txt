## Declare a cpp executable
add_executable(pose_ukf
  src/ros_pose.cpp 
  src/ukf.cpp
  src/ukf_pose.cpp
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
  src/ukf.cpp
)
