import rosbag
import math
import numpy
bag = rosbag.Bag('half_hour_pose.bag')
n=0
initial_angle = 0
final_angle = 0
initial_time = 0
final_time = 0
data_points = []
number_of_terms = 0
for topic, msg, t in bag.read_messages(topics=["/state_estimation/pose"]):

        if n == 0:
        	initial_angle = 2*math.atan2(math.sqrt(math.pow(msg.pose.orientation.x, 2) + math.pow(msg.pose.orientation.y, 2) + math.pow(msg.pose.orientation.z, 2)), msg.pose.orientation.w) 
		initial_time = t.secs
		n+=1
	elif n == 10000:
        	final_angle = 2*math.atan2(math.sqrt(math.pow(msg.pose.orientation.x, 2) + math.pow(msg.pose.orientation.y, 2) + math.pow(msg.pose.orientation.z, 2)), msg.pose.orientation.w)
		final_time = t.secs
		yaw_drift = final_angle - initial_angle
		time_interval = final_time - initial_time
		data_points.append(yaw_drift)
		number_of_terms+=1
        	print repr(yaw_drift) + " in " + repr(time_interval) + " seconds"
		n=0
	else:
		n+=1
bag.close()

print "mean is " + repr(numpy.mean(data_points))
print "standard deviation is " + repr(numpy.std(data_points))


