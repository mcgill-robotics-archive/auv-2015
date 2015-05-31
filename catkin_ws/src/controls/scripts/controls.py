#!/usr/bin/env python
from math import pi
import numpy as np
import rospy
from geometry_msgs.msg import Wrench
from auv_msgs.msg import SetVelocity, SetVelocityAction, SetVelocityFeedback
import tf
from tf.transformations import euler_from_quaternion
from actionlib import SimpleActionServer

depth_desired = 0.0
depth_estimated = 0.0
angles_desired = np.zeros(3)
angles_estimated = np.zeros(4)
proportional_error = np.zeros(4)
integral_error = np.zeros(4)


# Absolute value
surgeSpeed = 0.0
swaySpeed = 0.0

def pid(errors, gains):
        integral_error += proportional_error*dt
        return (proportional_gain * proportional_error + 
                integral_gain * integral_error + 
                derivative_gain * proportional_error_rate, integral_error)

def normalize_angle(angle, max_angle=pi):
    # Returns angle between -max_angle and max_angle
    angle = angle % (2 * pi)
    if angle > pi:
        angle -= 2*pi
    return angle

def set_velocity_callback():
    global surgeSpeed, swaySpeed, depth_desired, roll, desired_pitch, desired_yaw, isSettingPosition
    print 'cmd recieved'
    cmd = server.accept_new_goal().cmd
    surgeSpeed = cmd.surgeSpeed
    swaySpeed = cmd.swaySpeed
    depth_desired = cmd.depth
    angles_desired[0] = cmd.roll
    angles_desired[1] = cmd.pitch
    angles_desired[2] = cmd.yaw

    isSettingPosition = 0
    
def set_velocity_preempt():
    surgeSpeed = 0
    swaySpeed = 0
    print 'preempted'
    server.set_preempted()


def get_transform(origin_frame, target_frame):
    global listener
    if not listener:
        listener = tf.TransformListener()
    (trans, rot) = listener.lookupTransform(
                # FROM
                origin_frame,
                # TO
                target_frame,
                # NOW
                rospy.Time()
            )
    return (trans, rot)



def rosInit():
    rospy.init_node('controls')

    global wrenchPublisher, listener, server
    listener = tf.TransformListener()
    #rospy.Subscriber("autonomy/set_velocity", SetVelocity, setVelocity_callback)
    server = SimpleActionServer('controls', SetVelocityAction, auto_start=False)
    server.register_goal_callback(set_velocity_callback)
    server.register_preempt_callback(set_velocity_preempt)
    server.start()
    wrenchPublisher = rospy.Publisher('controls/wrench', Wrench, queue_size=100)
    # Maybe we can use rospy.wait_for_message instead of this?
    t = rospy.Time.now() + rospy.Duration.from_sec(1)
    while rospy.Time.now() < t :
        try:
	    get_transform("initial_horizon", "robot")
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

if __name__ == '__main__':
    rosInit()
    integral_gains = np.array([rospy.get_param("~ki_roll"),
            rospy.get_param("~ki_pitch"),
            rospy.get_param("~ki_yaw"),
            rospy.get_param("~ki_depth")])

    proportional_gains = np.array([rospy.get_param("~kp_roll"),
            rospy.get_param("~kp_pitch"),
            rospy.get_param("~kp_yaw"),
            rospy.get_param("~kp_depth")])
    
    derivative_gains = np.array([rospy.get_param("~kd_roll"),
            rospy.get_param("~kd_pitch"),
            rospy.get_param("~kd_yaw"),
            rospy.get_param("~kd_depth")])

    surge_coeff = rospy.get_param("~surge_coeff")
    sway_coeff = rospy.get_param("~sway_coeff")

    dt = 0.1
    r = rospy.Rate(1/dt)

    while not rospy.is_shutdown():
        # Velocity control is open-loop
        fx = surge_coeff*surgeSpeed
        fy = sway_coeff*swaySpeed

        # TODO: Error handling
        (trans, rot) = get_transform("/initial_horizon", "/robot")
        angles_estimated = euler_from_quaternion(rot)

        proportional_error_prev = proportional_error
        proportional_error[0:3] =  angles_desired - angles_estimated
        proportional_error[3] = depth_desired - depth_estimated
        integral_error += proportional_error * dt
        derivative_error = (proportional_error - proportional_error_prev) / dt

        # Correct yaw angle error for wrap around. The other angles are
        # not corrected only small roll and pitch are supported.
        proportional_error[2] = normalize_angle(proportional_error[2])
        derivative_error[2] = normalize_angle(derivative_error[2])

        # Send feedback on yaw error
        if server.is_active():
            feedback = SetVelocityFeedback()
            feedback.yaw_error = proportional_error[2]
            server.publish_feedback(feedback)
            print 'feedback'

        output = integral_error * integral_gains \
                + proportional_error * proportional_gains \
                + derivative_error * derivative_gains

        wrenchMsg = Wrench()
        wrenchMsg.force.x = fx;
        wrenchMsg.force.y = fy;
        wrenchMsg.force.z = output[3];
        wrenchMsg.torque.x = output[0];
        wrenchMsg.torque.y = output[1];
        wrenchMsg.torque.z = output[2];
        wrenchPublisher.publish(wrenchMsg)
        r.sleep()
