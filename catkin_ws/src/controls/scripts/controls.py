#!/usr/bin/env python
from math import pi
import numpy as np
import rospy
from geometry_msgs.msg import Wrench
from auv_msgs.msg import (SetVelocityAction, SetVelocityFeedback,
                          SetPositionAction, SetPositionFeedback)
import tf
from tf.transformations import euler_from_quaternion
from actionlib import SimpleActionServer

# flake8 wants these declared up here
pos_server = None
vel_server = None

depth_desired = 0.0
depth_estimated = 0.0
angles_desired = np.zeros(3)
proportional_error = np.zeros(4)
integral_error = np.array([2.58233408,-6.77174055,1.12110974,0.])

# Position control
target_frame_id = ''
pos_desired = np.zeros(2)
pos_integral_error = np.zeros(2)
pos_proportional_error = np.zeros(2)

# Absolute value
surge_speed = 0.0
sway_speed = 0.0

server = None
listener = None
wrenchPublisher = None
last_heading = 0.0


def normalize_angle(angle, max_angle=pi):
    # Returns angle between -max_angle and max_angle
    angle = angle % (2 * pi)
    if angle > pi:
        angle -= 2*pi
    return angle


def set_velocity_callback():
    global angles_desired, surge_speed, sway_speed, depth_desired, last_heading
    set_position_preempt()
    cmd = vel_server.accept_new_goal().cmd
    angles_desired[0] = cmd.roll
    angles_desired[1] = cmd.pitch
    angles_desired[2] = cmd.yaw
    surge_speed = cmd.surgeSpeed
    sway_speed = cmd.swaySpeed
    depth_desired = cmd.depth
    last_heading = cmd.yaw


def set_position_callback():
    global angles_desired, pos_desired, depth_desired, target_frame_id
    set_velocity_preempt()
    cmd = pos_server.accept_new_goal().cmd
    target_frame_id = cmd.target_frame_id
    angles_desired[0] = cmd.roll
    angles_desired[1] = cmd.pitch
    angles_desired[2] = cmd.yaw
    pos_desired[0] = cmd.xPos
    pos_desired[1] = cmd.yPos
    depth_desired = cmd.depth


def set_velocity_preempt():
    global surge_speed, sway_speed, mode
    surge_speed = 0.0
    sway_speed = 0.0
    angles_desired[0] = 0.0
    angles_desired[1] = 0.0
    if vel_server.is_active():
        vel_server.set_preempted()


def set_position_preempt():
    global surge_speed, sway_speed, mode
    surge_speed = 0.0
    sway_speed = 0.0
    angles_desired[0] = 0.0
    angles_desired[1] = 0.0
    angles_desired[2] = last_heading
    if pos_server.is_active():
        pos_server.set_preempted()


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
        rospy.Time())
    return (trans, rot)


def rosInit():
    global wrenchPublisher, listener, pos_server, vel_server
    rospy.init_node('controls')

    listener = tf.TransformListener()
    vel_server = SimpleActionServer(
        'controls_velocity',
        SetVelocityAction,
        auto_start=False)
    vel_server.register_goal_callback(set_velocity_callback)
    vel_server.register_preempt_callback(set_velocity_preempt)
    vel_server.start()
    pos_server = SimpleActionServer(
        'controls_position',
        SetPositionAction,
        auto_start=False)
    pos_server.register_goal_callback(set_position_callback)
    pos_server.register_preempt_callback(set_position_preempt)
    pos_server.start()
    wrenchPublisher = rospy.Publisher(
        'controls/wrench',
        Wrench,
        queue_size=100)
    # Maybe we can use rospy.wait_for_message instead of this?
    t = rospy.Time.now() + rospy.Duration.from_sec(1)
    while rospy.Time.now() < t:
        try:
            get_transform("initial_horizon", "robot")
            break
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            pass


def loop(event):
    global pos_proportional_error, pos_integral_error, proportional_error
    global integral_error
    # Velocity control is open-loop
    fx = surge_coeff*surge_speed
    fy = sway_coeff*sway_speed

    # TODO: Error handling
    (trans, rot) = get_transform("/initial_horizon", "/robot")
    angles_estimated = list(euler_from_quaternion(rot))

    if pos_server.is_active():
        (x_err, y_err, _), quaternion = \
            get_transform(target_frame_id, '/horizon')
        yaw_err = euler_from_quaternion(quaternion)[2]
        print yaw_err
        pos_proportional_error_prev = pos_proportional_error
        pos_proportional_error = pos_desired - np.array([x_err, y_err])
        pos_integral_error = pos_proportional_error * dt
        pos_derivative_error = (pos_proportional_error
                                - pos_proportional_error_prev)/dt
        fx, fy = (pos_integral_error * pos_integral_gains
                  + pos_proportional_error * pos_proportional_gains
                  + pos_derivative_error * pos_derivative_gains)
        print (fx, fy)
        angles_estimated[2] = yaw_err

    proportional_error_prev = proportional_error.copy()
    proportional_error[0:3] = angles_desired - angles_estimated
    proportional_error[3] = depth_desired - depth_estimated
    integral_error += proportional_error * dt
    derivative_error = (proportional_error - proportional_error_prev) / dt

    # Correct yaw angle error for wrap around. The other angles are
    # not corrected only small roll and pitch are supported.
    proportional_error[2] = normalize_angle(proportional_error[2])
    derivative_error[2] = normalize_angle(derivative_error[2])

    # Send feedback on yaw error
    if vel_server.is_active():
        feedback = SetVelocityFeedback()
        feedback.yaw_error = proportional_error[2]
        feedback.depth_error = proportional_error[3]
        vel_server.publish_feedback(feedback)
    elif pos_server.is_active():
        feedback = SetPositionFeedback()
        feedback.yaw_error = proportional_error[2]
        feedback.depth_error = proportional_error[3]
        feedback.x_error = pos_proportional_error[0]
        feedback.y_error = pos_proportional_error[1]
        pos_server.publish_feedback(feedback)

    output = (integral_error * integral_gains
              + proportional_error * proportional_gains
              + derivative_error * derivative_gains)

    print "Integral Error {}".format(integral_error)

    wrenchMsg = Wrench()
    wrenchMsg.force.x = fx
    wrenchMsg.force.y = fy
    wrenchMsg.force.z = output[3]
    wrenchMsg.torque.x = output[0]
    wrenchMsg.torque.y = output[1]
    wrenchMsg.torque.z = output[2]
    wrenchPublisher.publish(wrenchMsg)


if __name__ == '__main__':
    global integral_gains, proportional_gains, derivative_gains, dt
    rosInit()
    integral_gains = np.array([
        rospy.get_param("~ki_roll"),
        rospy.get_param("~ki_pitch"),
        rospy.get_param("~ki_yaw"),
        rospy.get_param("~ki_depth")])
    proportional_gains = np.array([
        rospy.get_param("~kp_roll"),
        rospy.get_param("~kp_pitch"),
        rospy.get_param("~kp_yaw"),
        rospy.get_param("~kp_depth")])
    derivative_gains = np.array([
        rospy.get_param("~kd_roll"),
        rospy.get_param("~kd_pitch"),
        rospy.get_param("~kd_yaw"),
        rospy.get_param("~kd_depth")])
    pos_integral_gains = np.array([
        rospy.get_param('~ki_xPos'),
        rospy.get_param('~ki_yPos')])
    pos_proportional_gains = np.array([
        rospy.get_param('~kp_xPos'),
        rospy.get_param('~kp_yPos')])
    pos_derivative_gains = np.array([
        rospy.get_param('~kd_xPos'),
        rospy.get_param('~kd_yPos')])

    surge_coeff = rospy.get_param("~surge_coeff")
    sway_coeff = rospy.get_param("~sway_coeff")

    dt = 0.1
    timer = rospy.Timer(rospy.Duration(dt), loop)
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()
