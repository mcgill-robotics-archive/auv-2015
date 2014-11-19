#!/usr/bin/env python
from math import pi
import rospy
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64
from std_msgs.msg import String
from auv_msgs.msg import SetPosition
from auv_msgs.msg import SetVelocity
import tf
from tf.transformations import euler_from_quaternion

wrenchPublisher = None
listener = None

#error relative to object
xPos = 0.0
yPos = 0.0

#error relative to floating frame
depth = 0.0

#error relative to horizon
roll = 0.0

#absolute yaw relative to initial_frame
desired_yaw = 0.0
desired_pitch = 0.0

#absolute value
surgeSpeed = 0.0
swaySpeed = 0.0

isSettingPosition = 0


def setPosition_callback(data):
    global xPos, yPos, depth, roll, pitch, yaw, isSettingPosition
    xPos = data.xPos
    yPos = data.yPos
    depth = data.depth
    roll = data.roll
    desired_pitch = data.pitch
    desired_yaw = data.yaw

    isSettingPosition = 1


def setVelocity_callback(data):
    global surgeSpeed, swaySpeed, depth, roll, pitch, yaw, isSettingPosition
    surgeSpeed = data.surgeSpeed
    swaySpeed = data.swaySpeed
    depth = data.depth
    roll = data.roll
    desired_pitch = data.pitch
    desired_yaw = data.yaw

    isSettingPosition = 0


def get_transform(origin_frame, target_frame):
    global listener
    if not listener:
        listener = tf.TransformListener()
    t = rospy.Time(0)
    found = False
    while not found:
        try:
            (trans, rot) = listener.lookupTransform(
                # FROM
                origin_frame,
                # TO
                target_frame,
                # NOW
                rospy.Time(0)
            )
            return (trans, rot)
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            continue


def rosInit():
    rospy.init_node('controls', anonymous=True)

    global wrenchPublisher
    wrenchPublisher = rospy.Publisher("controls/wrench", Wrench, queue_size=100)


if __name__ == '__main__':
    rosInit()

    ei_xPos = 0.0
    ed_xPos = 0.0

    ei_yPos = 0.0
    ed_yPos = 0.0

    ei_depth = 0.0
    ed_depth = 0.0

    ei_roll = 0.0
    ed_roll = 0.0

    ep_pitch = 0.0
    ei_pitch = 0.0
    ed_pitch = 0.0

    ep_yaw = 0.0
    ei_yaw = 0.0
    ed_yaw = 0.0

    kp_xPos = rospy.get_param("/kp_xPos")
    ki_xPos = rospy.get_param("/ki_xPos")
    kd_xPos = rospy.get_param("/kd_xPos")

    kp_yPos = rospy.get_param("/kp_yPos")
    ki_yPos = rospy.get_param("/ki_yPos")
    kd_yPos = rospy.get_param("/kd_yPos")

    kp_depth = rospy.get_param("/kp_depth")
    ki_depth = rospy.get_param("/ki_depth")
    kd_depth = rospy.get_param("/kd_depth")

    kp_pitch = rospy.get_param("/kp_pitch")
    ki_pitch = rospy.get_param("/ki_pitch")
    kd_pitch = rospy.get_param("/kd_pitch")  

    kp_roll = rospy.get_param("/kp_roll")
    ki_roll = rospy.get_param("/ki_roll")
    kd_roll = rospy.get_param("/kd_roll")

    kp_yaw = rospy.get_param("/kp_yaw")
    ki_yaw = rospy.get_param("/ki_yaw")
    kd_yaw = rospy.get_param("/kd_yaw")

    surge_coeff = rospy.get_param("/surge_coeff")
    sway_coeff = rospy.get_param("/sway_coeff")


    fx = 0.0
    fy = 0.0
    fz = 0.0

    tx = 0.0
    ty = 0.0
    tz = 0.0


    dt = 0.1
    r = rospy.Rate(1/dt)


    rospy.Subscriber("autonomy/set_position", SetPosition, setPosition_callback)
    rospy.Subscriber("autonomy/set_velocity", SetVelocity, setVelocity_callback)


    while not rospy.is_shutdown():

    	rospy.loginfo("pitch gain is: %f, desired_yaw is: %f, surgeSpeed is: %f, swaySpeed is: %f", kp_pitch, desired_yaw, surgeSpeed, swaySpeed,)

    	if (isSettingPosition == 1):
    		pass

      	  #X position PID control: Taken out for testing with Asimov
        	# ei_xPos += xPos*dt
        	# ed_xPos = (xPos - prev_xPos)/dt
        	# prev_xPos = xPos
        	# fx = kp_xPos*xPos + ki_xPos*ei_xPos + kd_xPos*ed_xPos

        	#Y position PID control: Taken out for testing with Asimov
       		# ei_yPos += yPos*dt
        	# ed_yPos = (yPos - prev_yaw)/dt
        	# prev_yPos = yPos
        	# fy = kp_yPos*yPos + ki_yPos*ei_xPos + kd_yPos*ed_yPos

        else:
        	fx = surge_coeff*surgeSpeed	#This is based on Nick's code, he simply multiplied the the speed command
        	fy = sway_coeff*swaySpeed	#by 5 and published it as a force

        #Depth PID control: Taken out while the dpeth sensor is broken
        # ei_depth += depth*dt
        # ed_depth = (depth - prev_depth)/dt
        # prev_depth = depth
        # fz = kp_depth*depth + ki_depth*ei_depth + kd_depth*ed_depth

        # Roll PID control: Taken out for test on Asimov who can't control roll
        # ei_roll += roll*dt
        # ed_roll = (roll - prev_roll)/dt
        # prev_roll = roll
        # tx = kp_roll*roll + ki_roll*ei_roll + kd_roll*ed_roll

        ep_yaw_prev = ep_yaw
        ep_pitch_prev = ep_pitch

        before_transform = rospy.Time.now()

        (trans, rot) = get_transform("/initial_horizon", "/robot")

        if (rot):
            (estimated_roll, estimated_pitch, estimated_yaw) = euler_from_quaternion(rot)

        after_transform = rospy.Time.now()
        #dt = after_transform.to_sec() - before_transform.to_sec()

        rospy.loginfo("Duration: %f", dt)

        #imu/initial to imu

        ep_yaw = desired_yaw - estimated_yaw
        ep_pitch = desired_pitch - estimated_pitch

        #Correct angle error for wrap aroud
        if (ep_yaw > pi):
        	ep_yaw -= 2*pi

        elif (ep_yaw < -pi):
        	ep_yaw += 2*pi

        #Pitch PID control
        ei_pitch += ep_pitch*dt
        ed_pitch = (ep_pitch - ep_pitch_prev)/dt
        ty = kp_pitch*ep_pitch + ki_pitch*ei_pitch + kd_pitch*ed_pitch

        #Yaw PID control
        ei_yaw += ep_yaw*dt
        ed_yaw = (ep_yaw - ep_yaw_prev)/dt
        tz = kp_yaw*ep_yaw + ki_yaw*ei_yaw + kd_yaw*ed_yaw
        
        wrenchMsg = Wrench()

        wrenchMsg.force.x = fx;
        wrenchMsg.force.y = fy;
        wrenchMsg.force.z = fz;
        wrenchMsg.torque.x = tx;
        wrenchMsg.torque.y = ty;
        wrenchMsg.torque.z = tz;

        wrenchPublisher.publish(wrenchMsg)

        r.sleep()
