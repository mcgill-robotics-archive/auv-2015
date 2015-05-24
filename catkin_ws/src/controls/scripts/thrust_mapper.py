#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64, String
from std_msgs.msg import String
from auv_msgs.msg import MotorCommands
from numpy import *
from numpy.linalg import *
from thrust_character import Force_PWM

#set length from axis for each thruster ex:l_1z is distance from T1 to z-axis
#z-axis is down, x-axis is towards bow
#T1 = surge port         \surges positive thrust towards stern
#T2 = surge starboard     \surges positive thrust towards stern
#T3 = sway bow        \positive thrust towards starboard
#T4 = sway stern    \positive thrust towards port
#T5 = heave port bow        \heaves positive thrust towards bottom
#T6 = heave starboard bow    \heaves positive thrust towards bottom
#T7 = heave starboard stern    \heaves positive thrust towards bottom
#T8 = heave port stern        \heaves positive thrust towards bottom
l_1z = 8.176*.0254
l_2z = 8.176*.0254
l_3z = 18.216*.0254
l_4z = 12.003*.0254
l_5x = 8.297*.0254
l_6x = 8.297*.0254
l_7x = 8.297*.0254
l_8x = 8.297*.0254
l_5y = 13.169*.0254
l_6y = 13.169*.0254
l_7y = 6.331*.0254
l_8y = 6.331*.0254

A1 = matrix([(-1,1,0,0),(0,0,1,-1),(-l_1z,l_2z,l_3z,l_4z)])
# What about the mass and MOI????
# Do we want to calculate pinv or do we want to specify it ourselves?
Fx_Fy_Mz_to_T1234 = pinv(A1, rcond = 1e-15)

A2 = asarray([(1,1,1,1),(-l_5x,l_6x,l_7x,-l_8x),(-l_5y,-l_6y,l_7y,l_8y)])
Fz_Mx_My_to_T5678 = pinv(A2, rcond= 1e-15)


def Wrench_to_thrust_callback(data):
    #repackage
    fx = data.force.x
    fy = data.force.y
    fz = data.force.z
    mx = data.torque.x
    my = data.torque.y
    mz = data.torque.z

    B1 = matrix([fx, fy, mz]).T
    B2 = matrix([fz, mx, my]).T
    
    T1234 = Fx_Fy_Mz_to_T1234*B1
    T5678 = Fz_Mx_My_to_T5678*B2

    out_temp = array([ 0, 0, 0, 0, 0, 0, 0 ,0])
    #match forces with PWM values from thrusterchar for T1234. Why don't the different thrusters have different characterizations
    for i in range(0,4):
        if T1234[i] == 0:
            out_temp[i] = 1500
        elif T1234[i] >=24.643147775:
            out_temp[i] = 1900
        elif T1234[i] <= -17.3035821:
            out_temp[i] = 1100
        elif T1234[i] < 0:
            for find in range(39,-1,-1): #80 is length of thrust char - match to negative force assign PWM    
                if T1234[i] > Force_PWM[find,0]:
                    out_temp[i] = -int((Force_PWM[find,0]-T1234[i])/(Force_PWM[find,0]-Force_PWM[(find-1),0])*10) + Force_PWM[(find),1]
                    break
        else: 
            for find in range(40,80): #80 is length of thrust char - match to positive force assign PWM
                if T1234[i] < Force_PWM[find,0]:
                    # Why is it *10?
                    out_temp[i] = -int((Force_PWM[find,0]-T1234[i])/(Force_PWM[find,0]-Force_PWM[(find-1),0])*10) + Force_PWM[(find),1]
                    break
    #match forces with PWM values from thrusterchar for T5678
    for i in range(0, 4):
        if T5678[i] == 0:
            out_temp[i] = 1500
        elif T5678[i] >=24.643147775:
            out_temp[i] = 1900
        elif T5678[i] <= -17.3035821:
            out_temp[i] = 1100
        elif T5678[i] < 0:
            for find in range(39,-1,-1): #80 is length of thrust char - match to negative force assign PWM    
                if T5678[i] > Force_PWM[find,0]:
                    out_temp[i] = -int((Force_PWM[find,0]-T5678[i])/(Force_PWM[find,0]-Force_PWM[(find-1),0])*10) + Force_PWM[(find),1]
                    break
        else: 
            for find in range(40,80): #80 is length of thrust char    - match to positive force assign PWM
                if T5678[i] < Force_PWM[find,0]:
                    out_temp[i] = -int((Force_PWM[find,0]-T5678[i])/(Force_PWM[find,0]-Force_PWM[(find-1),0])*10) + Force_PWM[(find),1]
                    break
    # Do we want this?
    out_temp -= 1500


    #T1 = surge port         \surges positive thrust towards stern
    #T2 = surge starboard     \surges positive thrust towards stern
    #T3 = sway bow        \positive thrust towards starboard
    #T4 = sway stern    \positive thrust towards port
    #T5 = heave port bow        \heaves positive thrust towards bottom
    #T6 = heave starboard bow    \heaves positive thrust towards bottom
    #T7 = heave starboard stern    \heaves positive thrust towards bottom
    #T8 = heave port stern        \heaves positive thrust towards bottom
    cmds_msg = MotorCommands()
    cmds_msg.portSurge= out_temp[0]
    cmds_msg.starboardSurge = out_temp[1]
    cmds_msg.bowSway = out_temp[2]
    cmds_msg.sternSway = out_temp[3]
    cmds_msg.portBowHeave = out_temp[4]
    cmds_msg.starboardBowHeave = out_temp[5]
    cmds_msg.starboardSternHeave = out_temp[6]
    cmds_msg.portSternHeave = out_temp[7]
    thrust_pub.publish(cmds_msg)


if __name__ == '__main__':
    global thrust_pub 
    rospy.init_node('thrust_mapper')
    thrust_pub = rospy.Publisher('thrust_cmds', MotorCommands, queue_size = 5)
    sub = rospy.Subscriber("controls/wrench", Wrench, Wrench_to_thrust_callback)
    rospy.spin()

