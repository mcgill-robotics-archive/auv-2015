import rosbag
import math
bag = rosbag.Bag('half_hour_pose.bag')
n=0
initial_angle = 0
final_angle = 0
initial_time = 0
final_time = 0

for topic, msg, t in bag.read_messages(topics=["/state_estimation/pose"]):

        if n == 0:
        	initial_angle = 2*math.atan2(math.sqrt(math.pow(msg.pose.orientation.x, 2) + math.pow(msg.pose.orientation.y, 2) + math.pow(msg.pose.orientation.z, 2)), msg.pose.orientation.w)
		initial_time = t.secs
		n+=1
	elif n == 10000:
        	final_angle = 2*math.atan2(math.sqrt(math.pow(msg.pose.orientation.x, 2) + math.pow(msg.pose.orientation.y, 2) + math.pow(msg.pose.orientation.z, 2)), msg.pose.orientation.w)
		final_time = t.secs
        	print repr(final_angle - initial_angle) + " in " + repr(final_time - initial_time) + " seconds"
		n=0
	else:
		n+=1
bag.close()

