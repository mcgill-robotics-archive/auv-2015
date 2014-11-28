#!/usr/bin/env python

import task
import rospy
import autonomy
import task_controller
from abc import ABCMeta, abstractmethod

class Gate_Task(task.Task):
	def __init__(self, phase, my_autonomy):
		self.phase = phase
		self.my_autonomy = my_autonomy


	def execute(self):
		if(self.phase == 1):
			self.phase1()

	def phase1(self):
		desired = [3, 0, 0, 0, 0, 0]
		while(True):
			#print autonomy.filtered_depth
			desired[0] = self.my_autonomy.filtered_depth
			self.my_autonomy.set_velocity(desired)
			rospy.sleep(2)