#!/usr/bin/env python

#notes fo me:
#shouldn't need getters as modules will let globals be accessible

import rospy
from multiprocessing import Process

from std_msgs.msg import Int8
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from auv_msgs.msg import SetVelocity
from auv_msgs.msg import SetPosition
from auv_msgs.msg import CVTarget
from auv_msgs.msg import SonarTarget

velocityPublisher = None
positionPublisher = None
CVtargetPublisher = None
SonarTargetPublisher = None

robot_pose = {'point_x': -1, 'point_y': -1, 'point_z': -1, 'quat_x': -1, 'quat_y': -1, 'quat_z': -1, 'quat_w': -1}
robot_twist = {'linear_x': -1, 'linear_y': -1, 'linear_z': -1, 'angular_x': -1, 'angular_y': -1, 'angular_z': -1}
target_pose = {'point_x': -1, 'point_y': -1, 'point_z': -1, 'quat_x': -1, 'quat_y': -1, 'quat_z': -1, 'quat_w': -1}

filtered_depth = -1
target_info = ""

def filtered_depth_callback(msg):
	global filtered_depth
	filtered_depth = msg.data

def robot_pose_callback(msg):
	global robot_pose
	robot_pose['point_x'] = msg.point.x
	robot_pose['point_y'] = msg.point.y
	robot_pose['point_z'] = msg.point.z
	robot_pose['quat_x'] = msg.quaternion.x
	robot_pose['quat_y'] = msg.quaternion.y
	robot_pose['quat_z'] = msg.quaternion.z
	robot_pose['quat_w'] = msg.quaternion.w

def robot_twist_callback(msg):
	global robot_twist
	robot_twist['linear_x'] = msg.linear.x
	robot_twist['linear_y'] = msg.linear.y
	robot_twist['linear_z'] = msg.linear.z
	robot_twist['angular_x'] = msg.angular.x
	robot_twist['angular_y'] = msg.angular.y
	robot_twist['angular_z'] = msg.angular.z

def target_pose_callback(msg):
	global target_pose
	target_pose['point_x'] = msg.point.x
	target_pose['point_y'] = msg.point.y
	target_pose['point_z'] = msg.point.z
	target_pose['quat_x'] = msg.quaternion.x
	target_pose['quat_y'] = msg.quaternion.y
	target_pose['quat_z'] = msg.quaternion.z
	target_pose['quat_w'] = msg.quaternion.w

def target_info_callback(msg):
	global target_info
	target_info = msg.data	

def rosInit():
	global velocityPublisher, positionPublisher, CVTargetPublisher, SonarTargetPublisher
	rospy.init_node('autonomy', anonymous=False)

	rospy.Subscriber("state_estimation/filtered_depth", Int8, filtered_depth_callback)
	rospy.Subscriber("state_estimation/robot_pose", Pose, robot_pose_callback)
	rospy.Subscriber("state_estimation/robot_twist", Twist, robot_twist_callback)
	rospy.Subscriber("state_estimation/target_pose", Pose, target_pose_callback)
	rospy.Subscriber("front_cv/target_info", String, target_info_callback)

	velocityPublisher = rospy.Publisher("autonomy/set_velocity", SetVelocity, queue_size = 1000)
	positionPublisher = rospy.Publisher("autonomy/set_position", SetPosition, queue_size = 1000)
	CVTargetPublisher = rospy.Publisher("autonomy/cv_target", CVTarget, queue_size = 1000)	
	SonarTargetPublisher = rospy.Publisher("autonomy/sonar_target", SonarTarget, queue_size = 1000)

if __name__ == '__main__':
	rosInit()
#test block should not be kept in actual code
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		CVmsg = CVTarget()
		Sonarmsg = SonarTarget()
		velocitymsg = SetVelocity()
		positionmsg = SetPosition()

		CVmsg.CVTarget = CVmsg.NOTHING
		Sonarmsg.SonarTarget = Sonarmsg.NOTHING

		velocitymsg.surgeSpeed = 1
		velocitymsg.swaySpeed = 2
		velocitymsg.depth = 3
		velocitymsg.roll = 4
		velocitymsg.pitch = 5
		velocitymsg.yaw = 6

		positionmsg.xPos = -1
		positionmsg.yPos = -2
		positionmsg.depth = -3
		positionmsg.roll = -4
		positionmsg.pitch = -5
		positionmsg.yaw = -6

		velocityPublisher.publish(velocitymsg)
		positionPublisher.publish(positionmsg)
		CVTargetPublisher.publish(CVmsg)
		SonarTargetPublisher.publish(Sonarmsg)

		r.sleep()