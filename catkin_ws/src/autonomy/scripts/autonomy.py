#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import Int8
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from auv_msgs.msg import SetVelocity
from auv_msgs.msg import SetPosition
from auv_msgs.msg import CVTarget
from auv_msgs.msg import SonarTarget

import task_controller

velocity_publisher = None
position_publisher = None
CV_target_publisher = None
sonar_target_publisher = None
filtered_depth = -1
target_info = ""

def filtered_depth_callback(msg):
	global filtered_depth
	print filtered_depth
	filtered_depth = msg.data

def get_depth():
	global filtered_depth
	return filtered_depth	

def target_info_callback(msg):
	global target_info
	target_info = msg.data	

def get_transform(target_frame):
	listener = tf.TransformListener()
	t = rospy.Time(0)
	if listener.frameExists(target_frame) and listener.frameExists("/horizon"):
		t = listener.getLatestCommonTime(target_frame, "/horizon")
		print t
        position, quaternion = listener.lookupTransform(target_frame, "/horizon", t)
        print position, quaternion
        return (position, quaternion)

def set_velocity(desired):
	#velocity_publisher = rospy.Publisher("autonomy/set_velocity", SetVelocity, queue_size = 1000)	
#	global velocity_publisher
	velocitymsg = SetVelocity()
	velocitymsg.surgeSpeed = desired[0]
	velocitymsg.swaySpeed = desired[1]
	velocitymsg.depth = desired[2]
	velocitymsg.roll = desired[3]
	velocitymsg.pitch = desired[4]
	velocitymsg.yaw = desired[5]
	velocity_publisher.publish(velocitymsg)

def set_position(desired):
	global position_publisher
	positionmsg = SetPosition()
	positionmsg.xPos = desired[0]
	positionmsg.yPos = desired[1]
	positionmsg.depth = desired[2]
	positionmsg.roll = desired[3]
	positionmsg.pitch = desired[4]
	positionmsg.yaw = desired[5]
#	positionmsg.frame = desired[6]
	position_publisher.publish(positionmsg)

def set_cv_target(new_target):
	CVmsg = CVTarget()
	CVmsg.CVTarget = new_target
	CV_target_publissonar_target_publisher

def set_sonar_target(new_target):
	Sonarmsg = SonarTarget()
	Sonarmsg.SonarTarget = new_target
	sonar_target_publisher.publish(Sonarmsg)

def rosInit():
	global velocity_publisher, position_publisher, CV_target_publisher, sonar_target_publisher
	rospy.init_node('autonomy', anonymous=False)
	rospy.Subscriber("state_estimation/filtered_depth", Int8, filtered_depth_callback)
	rospy.Subscriber("front_cv/target_info", String, target_info_callback)

	velocity_publisher = rospy.Publisher("autonomy/set_velocity", SetVelocity, queue_size = 1000)
	position_publisher = rospy.Publisher("autonomy/set_position", SetPosition, queue_size = 1000)
	CV_target_publisher = rospy.Publisher("autonomy/cv_target", CVTarget, queue_size = 1000)	
	sonar_target_publisher = rospy.Publisher("autonomy/sonar_target", SonarTarget, queue_size = 1000)
	
def phase1():
	global filtered_depth
	desired = [3, 0, 0, 0, 0, 0]
	while(True):
		#print autonomy.filtered_depth
		desired[0] = filtered_depth
		set_velocity(desired)
		rospy.sleep(2)

if __name__ == '__main__':
	rosInit()
#	task_controller.populate_tasks()
#	task_controller.populate_routine()
	get_transform("/horizon")

	r = rospy.Rate(10)
#	task_controller.temp()
#	task_controller.next_task('maneuver')
	while not rospy.is_shutdown():
		#task_controller.temp()
		phase1()