#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64
from std_msgs.msg import String
from auv_msgs.msg import SetPosition
from auv_msgs.msg import SetVelocity

wrenchPublisher = None

xPos = 0.0
yPos = 0.0

surgeSpeed = 0.0
swaySpeed = 0.0

depth = 0.0
roll = 0.0
pitch = 0.0
yaw = 0.0

isSettingPosition = 0

estimated_depth = 0.0

def setPosition_callback(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard setPosition xPos = %f, yPos = %f, depth = %f, roll = %f, pitch = %f, yaw = %f", data.xPos, data.yPos, data.depth, data.roll, data.pitch, data.yaw)

    global xPos, yPos, depth, isSettingPosition
    xPos = data.xPos
    yPos = data.yPos
    depth = data.depth
    isSettingPosition = 1


def setVelocity_callback(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard setVelocity surgeSpeed = %f, swaySpeed = %f, depth = %f, roll = %f, pitch = %f, yaw = %f", data.surgeSpeed, data.swaySpeed, data.depth, data.roll, data.pitch, data.yaw)

    global surgeSpeed, swaySpeed, depth, isSettingPosition
    surgeSpeed = data.surgeSpeed
    swaySpeed = data.swaySpeed
    depth = data.depth
    isSettingPosition = 0
    
def getDepth_callback(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard estimated_depth = %f", data.data)

    global estimated_depth
    estimated_depth = data.data


def rosInit():
    rospy.init_node('controls', anonymous=True)

    global wrenchPublisher
    rospy.Subscriber("autonomy/set_position", SetPosition, setPosition_callback)
    rospy.Subscriber("autonomy/set_velocity", SetVelocity, setVelocity_callback)
    rospy.Subscriber("state_estimation/filteredDepth", Float64, getDepth_callback)

    wrenchPublisher = rospy.Publisher("controls/wrench", Wrench, queue_size=100)

    #rospy.spin()

if __name__ == '__main__':
    rosInit()

    r = rospy.Rate(1)


    ep_depth = 0.0
    ei_depth = 0.0
    ed_depth = 0.0

    prev_ep_depth = 0.0

    kp_depth = 1.0
    ki_depth = 1.0
    kd_depth = 1.0   

    fx = 0.0
    fy = 0.0
    fz = 0.0

    dt = 0.1
    r = rospy.Rate(1/dt)

    while not rospy.is_shutdown():
        #rospy.loginfo("isSettingPosition: %d, xPos is: %f, yPos is: %f, depth is: %f, surgeSpeed is: %f, swaySpeed is: %f", isSettingPosition, xPos, yPos, depth, surgeSpeed, swaySpeed)
        rospy.loginfo("estimated_depth is: %f", estimated_depth)

        prev_ep_depth = ep_depth
        ep_depth = depth - estimated_depth
        ei_depth += ep_depth*dt
        ed_depth = (ep_depth - prev_ep_depth)/dt
        fz = kp_depth*ep_depth + ki_depth*ei_depth + kd_depth*ed_depth
        
        wrenchMsg = Wrench()

        wrenchMsg.force.x = 0;
        wrenchMsg.force.y = 0;
        wrenchMsg.force.z = fz;
        wrenchMsg.torque.x = 0;
        wrenchMsg.torque.y = 0;
        wrenchMsg.torque.z = 0;

        wrenchPublisher.publish(wrenchMsg)

        r.sleep()


