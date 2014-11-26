import rosbag
import math
bag = rosbag.Bag('half_hour_pose.bag')
msg=0
angle = 0
for topic, msg, t in bag.read_messages(topics=["/state_estimation/pose"]):
        angle = 2*math.atan2(math.sqrt(math.pow(msg.pose.orientation.x, 2) + math.pow(msg.pose.orientation.y, 2) + math.pow(msg.pose.orientation.z, 2)), msg.pose.orientation.w)
        print angle
bag.close()

