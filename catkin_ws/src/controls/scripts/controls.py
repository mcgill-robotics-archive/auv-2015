#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from auv_msgs.msg import SetPosition
from auv_msgs.msg import SetVelocity

xPos = 0.0
yPos = 0.0

surgeSpeed = 0.0
swaySpeed = 0.0

depth = 0.0
roll = 0.0
pitch = 0.0
yaw = 0.0

isSettingPosition = 0

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
    


def listener():
    rospy.init_node('controls', anonymous=True)

    rospy.Subscriber("autonomy/setPosition", SetPosition, setPosition_callback)
    rospy.Subscriber("autonomy/setVelocity", SetVelocity, setVelocity_callback)

    #rospy.spin()

if __name__ == '__main__':
    listener()

    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        rospy.loginfo("isSettingPosition: %d, xPos is: %f, yPos is: %f, depth is: %f, surgeSpeed is: %f, swaySpeed is: %f", isSettingPosition, xPos, yPos, depth, surgeSpeed, swaySpeed)


        r.sleep()


