import rosbag
import numpy

sum = 0.
num = 0
fileName = input("rosbag file name: (eg \"name.bag\") ")
bag = rosbag.Bag(fileName)
values = []
for topic, msg, t in bag.read_messages(topics=["pressure"]):
    values.append(msg.data)
bag.close()
numpy_values = numpy.array(values)
print numpy.std(numpy_values)


