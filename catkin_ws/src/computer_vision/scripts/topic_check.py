#!/usr/bin/env python2.7

import numpy as np
import rosgraph.masterapi

smalllist = ['/rosout_agg', 'rosgraph_msgs/Log', 'potato']

master = rosgraph.masterapi.Master('/rostopic')

temp = np.array(master.getPublishedTopics('/'))
flattemp = temp.flatten()

print temp
print flattemp

for x in smalllist:
	if x not in flattemp:
		print x

