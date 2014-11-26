import rosbag
import math
bag = rosbag.Bag('half_hour_pose.bag')
n=0
initial_angle = 0
final_angle = 0
for topic, msg, t in bag.read_messages(topics=["/state_estimation/pose"]):
        if n == 0:
        	initial_angle = 2*math.atan2(math.sqrt(math.pow(msg.pose.orientation.x, 2) + math.pow(msg.pose.orientation.y, 2) + math.pow(msg.pose.orientation.z, 2)), msg.pose.orientation.w)
		n+=1
	elif n == 5000:
        	final_angle = 2*math.atan2(math.sqrt(math.pow(msg.pose.orientation.x, 2) + math.pow(msg.pose.orientation.y, 2) + math.pow(msg.pose.orientation.z, 2)), msg.pose.orientation.w)
        	print "change in angle is " + repr(final_angle - initial_angle) + " from time x to y.(settakes change in yaw every 5000 messages at the moment)" 
		n=0
	else:
		n+=1
bag.close()

