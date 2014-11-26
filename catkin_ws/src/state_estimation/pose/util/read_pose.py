import rosbag
bag = rosbag.Bag('half_hour_pose.bag')
for topic, msg, t in bag.read_messages(topics=["/state_estimation/pose"]):
	print msg
bag.close()
