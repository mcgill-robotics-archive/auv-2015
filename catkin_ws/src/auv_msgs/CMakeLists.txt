cmake_minimum_required(VERSION 2.8.3)
project(auv_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib_msgs
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  #added by Controls
  SetPosition.msg
  SetVelocity.msg
  MotorCommands.msg
  SolenoidCommands.msg

  #added by State Estimation
  RangeBearingElevation.msg
  SlamEstimate.msg

  #added by CV
  CVTarget.msg

  #added by IO
  RadioCommands.msg
)

add_service_files(FILES
  ConfidenceRadius.srv
  XYConfidenceRadius.srv
  Covariance.srv
  RcMode.srv
)

add_action_files(
  DIRECTORY action
  FILES SetVelocity.action
  SetPosition.action
  SetCVTarget.action
)


generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime actionlib_msgs)

