import rosbag
import numpy
sum = 0.
bag = rosbag.Bag("ground_2_minutes.bag")
for topic, msg, t in bag.read_messages(topics=["pressure"]):
    print msg
    sum += msg.float
bag.close()
print sum

