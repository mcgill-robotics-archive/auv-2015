cmake_minimum_required(VERSION 2.8.3)
project(auv_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(FILES
                #added by Elec Interface
                  Solenoid.msg
                  Motor.msg

                #added by Controls
                  SetPosition.msg
                  SetVelocity.msg
                  MotorCommands.msg

                #added by Autonomy
                  CVTarget.msg
                  SonarTarget.msg

		#added by State Estimation
		  SlamTarget.msg
		  SlamEstimate.msg
                 )

#add_service_files(FILES
#                  RequestParam.srv
#                  RequestMessageInfo.srv
#                  RequestServiceInfo.srv
#                 )

generate_messages()
catkin_package(CATKIN_DEPENDS message_runtime)

