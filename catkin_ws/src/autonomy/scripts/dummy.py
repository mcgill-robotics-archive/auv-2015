#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import Int8

def rosInit():

    global wrenchPublisher
    rospy.Subscriber("autonomy/set_position", SetPosition, setPosition_callback)
    rospy.Subscriber("autonomy/set_velocity", SetVelocity, setVelocity_callback)
    rospy.Subscriber("state_estimation/filteredDepth", Float64, getDepth_callback)

    #rospy.spin()

if __name__ == '__main__':
    rospy.init_node('dummy', anonymous=True)

    wrenchPublisher = rospy.Publisher("state_estimation/filtered_depth", Int8, queue_size=100)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        wrenchMsg = Int8()

        wrenchMsg.data = random.uniform(1,100)
        wrenchPublisher.publish(wrenchMsg)

        r.sleep()


